package nl.wur.mavdrone.communication.components.impl

import java.lang.ref.WeakReference
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

import dji.sdk.keyvalue.value.flightcontroller.FlightCoordinateSystem
import dji.sdk.keyvalue.value.flightcontroller.FlightMode
import dji.sdk.keyvalue.value.flightcontroller.RollPitchControlMode
import dji.sdk.keyvalue.value.flightcontroller.VerticalControlMode
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam
import dji.sdk.keyvalue.value.flightcontroller.YawControlMode
import dji.v5.common.callback.CommonCallbacks.CompletionCallback
import dji.v5.common.error.IDJIError
import dji.v5.manager.aircraft.virtualstick.VirtualStickState
import dji.v5.manager.aircraft.virtualstick.VirtualStickStateListener
import dji.v5.manager.aircraft.virtualstick.VirtualStickManager as DJIVirtualStickManager

import io.reactivex.rxjava3.core.Completable
import io.reactivex.rxjava3.core.Observable
import io.reactivex.rxjava3.disposables.Disposable
import io.reactivex.rxjava3.schedulers.Schedulers

import io.dronefleet.mavlink.common.MavSeverity
import io.dronefleet.mavlink.common.Statustext

import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.Drone
import nl.wur.mavdrone.utils.Logger
import nl.wur.mavdrone.utils.MavlinkTexts

/**
 * Climbs to a requested height after the DJI auto takeoff by engaging the virtual
 * stick in vertical position mode. DJI auto takeoff always only hovers at ~1.2 m
 * (the minimum height the firmware accepts for a takeoff), so any higher
 * NAV_TAKEOFF target needs this extra climb.
 *
 * Uses the official DJI VirtualStickManager (not the raw keys) because it also
 * performs the control authority takeover; without that the FC keeps ignoring the
 * stick data and the remote controller stays in command.
 */
class VirtualStickManager(
    connectionWR: WeakReference<MavConnection>,
    private val droneWR: WeakReference<Drone>,
    private val headingDegreesProvider: () -> Double?
) {
    private val log = Logger("VirtualStickManager")
    private var climbDisposable: Disposable? = null

    @Volatile
    private var velocityDisposable: Disposable? = null

    @Volatile
    private var lastVelocityParam: VirtualStickFlightControlParam? = null

    @Volatile
    private var lastSetpointMillis = 0L

    init {
        if (SDK_INITIALIZED.compareAndSet(false, true)) {
            DJIVirtualStickManager.getInstance().init()
        }

        DJIVirtualStickManager.getInstance().setVirtualStickStateListener(object : VirtualStickStateListener {
            override fun onVirtualStickStateUpdate(state: VirtualStickState) {
                log.i("Virtual stick state: enable=${state.isVirtualStickEnable}, authority=${state.currentFlightControlAuthorityOwner}")
            }

            override fun onChangeReasonUpdate(reason: dji.sdk.keyvalue.value.flightcontroller.FlightControlAuthorityChangeReason) {
                log.i("Virtual stick authority change reason: $reason")
            }
        })
    }

    val isClimbActive: Boolean
        get() = climbDisposable != null

    /**
     * Climb to [targetHeight] (meters above the takeoff location) once the ongoing
     * auto takeoff finished. Fire and forget; the result is reported as STATUSTEXT.
     */
    fun climbToHeight(targetHeight: Float) {
        if (climbDisposable != null) {
            log.w("Climb already in progress, ignoring request for ${targetHeight}m")
            return
        }

        log.i("Start virtual stick climb to ${targetHeight}m")
        climbDisposable = waitForTakeoffToComplete()
            .andThen(enableVirtualStick())
            .andThen(runClimbLoop(targetHeight))
            .subscribe(
                {
                    log.i("Climb height of ${targetHeight}m reached")
                    sendStatus(MavSeverity.MAV_SEVERITY_INFO, "Takeoff: reached ${targetHeight}m")
                    releaseVirtualStick()
                },
                { error ->
                    log.e("Climb to ${targetHeight}m failed: ${error.message}")
                    sendStatus(MavSeverity.MAV_SEVERITY_ERROR, "Takeoff: climb failed: ${error.message}")
                    releaseVirtualStick()
                }
            )
    }

    /** Stop an active climb and release the virtual stick (e.g. on command cancel). */
    fun stopClimb() {
        if (climbDisposable != null) {
            log.i("Virtual stick climb canceled")
            sendStatus(MavSeverity.MAV_SEVERITY_WARNING, "Takeoff: climb canceled")
            releaseVirtualStick()
        }
    }

    /**
     * Feed a velocity setpoint (body frame: forward/right/down positive, yaw rate
     * clockwise) from the offboard stream. Engages the virtual stick on the first
     * setpoint, streams the latest setpoint at the required rate and auto-releases
     * the stick once the setpoints stop coming.
     */
    fun feedVelocitySetpoint(forward: Double, right: Double, down: Double, yawRateDeg: Double) {
        // The tested firmware ignores the requested BODY coordinate and interprets the
        // horizontal velocity in DJI's GROUND frame (X east, Y north), so rotate the
        // body-frame setpoint into that frame ourselves; requesting GROUND explicitly
        // makes this correct whether or not the firmware honors the field
        val heading = headingDegreesProvider()
        if (heading == null) {
            log.w("Velocity setpoint dropped: no heading available")
            return
        }
        val yawRad = Math.toRadians(heading)
        val east = forward * sin(yawRad) + right * cos(yawRad)
        val north = forward * cos(yawRad) - right * sin(yawRad)

        lastVelocityParam = VirtualStickFlightControlParam(
            east, north, yawRateDeg, -down,
            VerticalControlMode.VELOCITY,
            RollPitchControlMode.VELOCITY,
            YawControlMode.ANGULAR_VELOCITY,
            FlightCoordinateSystem.GROUND
        )
        lastSetpointMillis = System.currentTimeMillis()

        if (velocityDisposable != null) {
            // Session already running, just refresh the setpoint
            return
        }

        // A running climb owns the virtual stick; hand control over to the velocity loop
        cancelClimbSession()

        startVelocitySession()
    }

    private fun startVelocitySession() {
        log.i("Start virtual stick velocity control session")
        velocityDisposable = enableVirtualStick()
            .andThen(
                Observable.interval(0, COMMAND_INTERVAL_MS, TimeUnit.MILLISECONDS)
                    .subscribeOn(Schedulers.newThread())
                    .takeWhile { System.currentTimeMillis() - lastSetpointMillis <= SETPOINT_TIMEOUT_MS }
            )
            .subscribe(
                {
                    val drone = droneWR.get()
                    if (drone != null && isDroneFlying(drone)) {
                        lastVelocityParam?.let {
                            DJIVirtualStickManager.getInstance().sendVirtualStickAdvancedParam(it)
                        }
                    } else {
                        releaseVirtualStick()
                    }
                },
                { error ->
                    log.e("Velocity control session failed: ${error.message}")
                    releaseVirtualStick()
                },
                {
                    // Setpoints stopped coming (offboard stop / GCS lost), hand control back
                    log.i("Velocity setpoints stopped, releasing virtual stick")
                    releaseVirtualStick()
                }
            )
    }

    /** Release the virtual stick and end every control session. */
    fun stopAll() {
        releaseVirtualStick()
    }

    // The virtual stick cannot be engaged while the DJI auto takeoff is still running,
    // so wait until the aircraft is flying and left the takeoff mode
    private fun waitForTakeoffToComplete(): Completable =
        Completable.create { emitter ->
            emitter.setDisposable(
                Observable.interval(TAKEOFF_CHECK_INTERVAL_MS, TimeUnit.MILLISECONDS)
                    .subscribeOn(Schedulers.newThread())
                    .subscribe({
                        val drone = droneWR.get() ?: return@subscribe

                        // isFlying may not be pushed on some aircraft; the takeoff altitude
                        // then reports that the climb is done
                        val takeoffDone = isDroneFlying(drone)
                            && drone.flightMode != FlightMode.AUTO_TAKE_OFF

                        if (takeoffDone) {
                            emitter.onComplete()
                        }
                    }, { emitter.tryOnError(it) })
            )
        }.timeout(TAKEOFF_COMPLETE_TIMEOUT_MS, TimeUnit.MILLISECONDS, Schedulers.newThread())

    // The DJI manager performs the control authority takeover here; without it the FC
    // silently ignores all stick data. The FC may refuse right after takeoff, retry
    // until it accepts.
    private fun enableVirtualStick(): Completable =
        Completable.create { emitter ->
            emitter.setDisposable(
                Observable.interval(0, STICK_ENABLE_RETRY_INTERVAL_MS, TimeUnit.MILLISECONDS)
                    .subscribeOn(Schedulers.newThread())
                    .subscribe({
                        DJIVirtualStickManager.getInstance().enableVirtualStick(object : CompletionCallback {
                            override fun onSuccess() {
                                // The advanced param carries the vertical position mode; the plain
                                // stick interface only streams velocity commands
                                DJIVirtualStickManager.getInstance().setVirtualStickAdvancedModeEnabled(true)
                                log.i("Virtual stick control enabled")
                                emitter.onComplete()
                            }

                            override fun onFailure(error: IDJIError) {
                                log.w("Enabling virtual stick failed, retrying: ${error.description()}")
                            }
                        })
                    }, { emitter.tryOnError(it) })
            )
        }.timeout(STICK_ENABLE_TIMEOUT_MS, TimeUnit.MILLISECONDS, Schedulers.newThread())

    private fun runClimbLoop(targetHeight: Float): Completable =
        Completable.create { emitter ->
            var consecutiveOnTarget = 0
            val startTime = System.currentTimeMillis()
            val timeoutMs = climbTimeoutMs(targetHeight)

            emitter.setDisposable(
                Observable.interval(0, COMMAND_INTERVAL_MS, TimeUnit.MILLISECONDS)
                    .subscribeOn(Schedulers.newThread())
                    .subscribe({
                        val drone = droneWR.get()
                        if (drone == null || !isDroneFlying(drone)) {
                            emitter.tryOnError(IllegalStateException("Drone is not flying anymore"))
                            return@subscribe
                        }

                        if (System.currentTimeMillis() - startTime > timeoutMs) {
                            emitter.tryOnError(IllegalStateException("Climb timed out"))
                            return@subscribe
                        }

                        // sendVirtualStickAdvancedParam has no callback; failures show up
                        // in the state listener and the timeout aborts stuck climbs
                        DJIVirtualStickManager.getInstance()
                            .sendVirtualStickAdvancedParam(createHoldParam(targetHeight))

                        val altitude = drone.aircraftLocation?.altitude
                        if (altitude != null && abs(altitude - targetHeight) <= ARRIVAL_TOLERANCE) {
                            consecutiveOnTarget++
                            if (consecutiveOnTarget >= CONSECUTIVE_ARRIVAL_SAMPLES) {
                                emitter.onComplete()
                            }
                        } else {
                            consecutiveOnTarget = 0
                        }
                    }, { emitter.tryOnError(it) })
            )
        }

    private fun isDroneFlying(drone: Drone): Boolean =
        drone.isFlying || (drone.aircraftLocation?.altitude ?: 0.0) > MIN_TAKEOFF_HEIGHT

    // In vertical position mode the vertical value is the target height above the
    // takeoff location; pitch/roll/yaw are held at standstill (frame irrelevant for
    // zero velocities, GROUND kept for consistency with the velocity session)
    private fun createHoldParam(targetHeight: Float): VirtualStickFlightControlParam =
        VirtualStickFlightControlParam(
            0.0, 0.0, 0.0, targetHeight.toDouble(),
            VerticalControlMode.POSITION,
            RollPitchControlMode.VELOCITY,
            YawControlMode.ANGULAR_VELOCITY,
            FlightCoordinateSystem.GROUND
        )

    private fun climbTimeoutMs(targetHeight: Float): Long =
        (CLIMB_TIMEOUT_BASE_MS + targetHeight * CLIMB_TIMEOUT_PER_METER_MS)
            .toLong()
            .coerceAtMost(CLIMB_TIMEOUT_MAX_MS)

    private fun cancelClimbSession() {
        climbDisposable?.dispose()
        climbDisposable = null
    }

    private fun cancelVelocitySession() {
        velocityDisposable?.dispose()
        velocityDisposable = null
    }

    private fun releaseVirtualStick() {
        cancelClimbSession()
        cancelVelocitySession()

        DJIVirtualStickManager.getInstance().setVirtualStickAdvancedModeEnabled(false)
        DJIVirtualStickManager.getInstance().disableVirtualStick(object : CompletionCallback {
            override fun onSuccess() {
                log.i("Virtual stick control disabled")
            }

            override fun onFailure(error: IDJIError) {
                log.w("Disabling virtual stick failed: ${error.description()}")
            }
        })
    }

    private fun sendStatus(severity: MavSeverity, text: String) {
        droneWR.get()?.sendMessage(
            Statustext.builder()
                .severity(severity)
                .text(MavlinkTexts.truncateStatustext(text))
                .build()
        )
    }

    companion object {
        /** DJI auto takeoff hover height, also the firmware minimum for a takeoff. */
        const val MIN_TAKEOFF_HEIGHT = 1.2f

        // DJI requires virtual stick data at 5 Hz or higher
        private const val COMMAND_INTERVAL_MS = 200L
        private const val TAKEOFF_CHECK_INTERVAL_MS = 500L
        private const val TAKEOFF_COMPLETE_TIMEOUT_MS = 30000L
        private const val STICK_ENABLE_RETRY_INTERVAL_MS = 1000L
        private const val STICK_ENABLE_TIMEOUT_MS = 20000L
        private const val ARRIVAL_TOLERANCE = 0.2f
        private const val CONSECUTIVE_ARRIVAL_SAMPLES = 3
        private const val CLIMB_TIMEOUT_BASE_MS = 20000f
        private const val CLIMB_TIMEOUT_PER_METER_MS = 2000f
        private const val CLIMB_TIMEOUT_MAX_MS = 120000L

        // Release the virtual stick when no offboard setpoint arrives for this long
        private const val SETPOINT_TIMEOUT_MS = 2000L

        private val SDK_INITIALIZED = AtomicBoolean(false)
    }
}
