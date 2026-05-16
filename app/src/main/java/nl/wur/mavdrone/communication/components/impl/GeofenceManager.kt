package nl.wur.mavdrone.communication.components.impl

import java.lang.ref.WeakReference
import kotlin.math.roundToInt

import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.LocationCoordinate3D
import dji.v5.manager.KeyManager
import dji.v5.manager.aircraft.waypoint3.WaypointMissionManager
import dji.v5.manager.aircraft.waypoint3.model.WaypointMissionExecuteState
import dji.v5.utils.common.ContextUtil

import io.reactivex.rxjava3.core.Completable
import io.reactivex.rxjava3.core.Single
import io.reactivex.rxjava3.schedulers.Schedulers

import org.locationtech.jts.geom.Coordinate

import io.dronefleet.mavlink.MavlinkMessage
import io.dronefleet.mavlink.common.CommandLong
import io.dronefleet.mavlink.common.FenceAction
import io.dronefleet.mavlink.common.FenceBreach
import io.dronefleet.mavlink.common.FenceMitigate
import io.dronefleet.mavlink.common.FenceStatus
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.util.EnumValue

import nl.wur.mavdrone.MavManager.MavlinkManagerCommunicationModel.ShouldShowWarning
import nl.wur.mavdrone.R
import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.BaseComponent
import nl.wur.mavdrone.communication.components.Drone
import nl.wur.mavdrone.communication.components.impl.mission.WaypointMission
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createWaypointItem
import nl.wur.mavdrone.communication.utils.KMZ.Companion.create
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandAccepted
import nl.wur.mavdrone.utils.RXUtils
import nl.wur.mavdrone.utils.RXUtils.Companion.djiCompletableFunction
import nl.wur.mavdrone.utils.RXUtils.Companion.djiPerformAction
import nl.wur.mavdrone.utils.RXUtils.Companion.waitForVariable

class GeofenceManager(
    connectionWR: WeakReference<MavConnection>,
    private val droneWR: WeakReference<Drone>,
    private val missionManagerWR: WeakReference<MissionManager>
) : BaseComponent(connectionWR, "MavGeofenceManager") {
    var autoEnable: Boolean = false
    private var fenceAction: FenceAction? = null

    private var breachCount = 0
    private var breachTypeLast = FenceBreach.FENCE_BREACH_NONE
    private var breachTimeLast: Long = 0

    var isEnabled: Boolean = false
        private set
    private var currentBreachHandled = false

    private var droneComponentIdValue: Int = 1

    override fun create() {
        KeyManager.getInstance().listen(KeyTools.createKey(FlightControllerKey.KeyAircraftLocation3D), this)
        { _: LocationCoordinate3D?, newValue: LocationCoordinate3D? -> newValue?.let { checkGeofence(it) } }

        KeyManager.getInstance().listen(KeyTools.createKey(FlightControllerKey.KeyIsNearDistanceLimit), this)
        { oldValue: Boolean?, newValue: Boolean? -> oldValue?.let { ov -> newValue?.let { nv -> nearDJILimit(ov, nv, FenceBreach.FENCE_BREACH_BOUNDARY) } } }

        KeyManager.getInstance().listen(KeyTools.createKey(FlightControllerKey.KeyIsNearHeightLimit), this)
        { oldValue: Boolean?, newValue: Boolean? -> oldValue?.let { ov -> newValue?.let { nv -> nearDJILimit(ov, nv, FenceBreach.FENCE_BREACH_MAXALT) } } }

        droneComponentIdValue = EnumValue.create(droneWR.get()?.componentID).value()

        connectionWR.get()?.let {
            val defaultFenceActionValue = EnumValue.of(FenceAction.FENCE_ACTION_HOLD).value()
            fenceAction = EnumValue.create(FenceAction::class.java, it.androidPreferences.getInt("FENCE_ACTION", defaultFenceActionValue)).entry()
            autoEnable = it.androidPreferences.getBoolean("FENCE_AUTO_ENBL", true)
        }
    }

    override fun destroy() {
        KeyManager.getInstance().cancelListen(KeyTools.createKey(FlightControllerKey.KeyAircraftLocation3D), this)
        KeyManager.getInstance().cancelListen(KeyTools.createKey(FlightControllerKey.KeyIsNearDistanceLimit), this)
        KeyManager.getInstance().cancelListen(KeyTools.createKey(FlightControllerKey.KeyIsNearHeightLimit), this)

        super.destroy()
    }

    fun onCommandLong(msg: MavlinkMessage<*>): Boolean {
        val payload = msg.payload as CommandLong

        if (payload.targetSystem() == systemID && (payload.targetComponent() == droneComponentIdValue || payload.targetComponent() == 0)
        ) {
            log.i("Received command: ${payload.command().entry()}")

            if (payload.command().entry() == MavCmd.MAV_CMD_DO_FENCE_ENABLE) {
                if (payload.param1().roundToInt() == 1) enable() else disable()
                droneWR.get()?.sendMessage(commandAccepted(MavCmd.MAV_CMD_DO_FENCE_ENABLE, msg.originSystemId, msg.originComponentId))
                return true
            }
        }

        return false
    }

    fun enable() {
        isEnabled = true
        breachCount = 0
        log.i("Geofence enabled!")
    }

    fun disable() {
        isEnabled = false
        log.i("Geofence disabled!")
    }

    private fun nearDJILimit(oldValue: Boolean, newValue: Boolean, reason: FenceBreach) {
        if (isEnabled && !oldValue && newValue) {
            handleFenceBreach(reason)
        }
    }

    private fun checkGeofence(location: LocationCoordinate3D) {
        missionManagerWR.get()?.let {
            val geofenceMission = it.currentGeofenceMission
            if (isEnabled && geofenceMission != null && fenceAction != FenceAction.FENCE_ACTION_NONE) {
                var breach = false
                for (fenceItem in geofenceMission.getFenceItems()) {
                    if (!fenceItem.isAllowed(location)) {
                        breach = true
                        break
                    }
                }

                // When there is no more fence breach, reset the currentBreachHandled to handle a possible
                // next breach
                if (breach) handleFenceBreach(FenceBreach.FENCE_BREACH_BOUNDARY) else currentBreachHandled = false
            }
        }
    }

    private fun handleFenceBreach(reason: FenceBreach) {
        if (currentBreachHandled) {
            return
        }
        breachCount += 1
        breachTimeLast = droneWR.get()!!.millisSinceBoot
        breachTypeLast = reason

        doFenceAction()

        currentBreachHandled = true
    }

    private fun sendGeofenceWarning(solution: String) {
        connectionWR.get()?.mavlinkThreadCommunicationSubject?.onNext(
            ShouldShowWarning(ContextUtil.getContext().getString(R.string.geofence_breach_warning_title), solution)
        )
    }

    private fun doFenceAction() {
        missionManagerWR.get()?.let {
            val missionState = it.waypointManager.waypointMissionExecuteState
            val geofenceMission = it.currentGeofenceMission ?: return

            when (fenceAction) {
                FenceAction.FENCE_ACTION_GUIDED, FenceAction.FENCE_ACTION_GUIDED_THR_PASS -> {
                    log.w("Executing geofence breach action GUIDED!")
                    if (missionState == WaypointMissionExecuteState.EXECUTING || missionState == WaypointMissionExecuteState.ENTER_WAYLINE) {
                        if (geofenceMission.returnPoint == null) {
                            log.w("Cannot return to returnPoint because it is not defined! Returning to home instead...")

                            sendGeofenceWarning(ContextUtil.getContext().getString(R.string.geofence_breach_home))

                            // Stop current mission and return to launch
                            compositeDisposable.add(
                                it.waypointManager.stopCurrentMission()
                                    .subscribeOn(Schedulers.newThread())
                                    .andThen(djiPerformAction(FlightControllerKey.KeyStartGoHome))
                                    .subscribe(
                                        { log.i("Start go-home") },
                                        { error -> RXUtils.handleError(error) })
                            )
                            return
                        }

                        sendGeofenceWarning(ContextUtil.getContext().getString(R.string.geofence_breach_return_point))

                        compositeDisposable.add(
                            it.waypointManager.stopCurrentMission()
                                .subscribeOn(Schedulers.io())
                                .andThen(Completable.fromAction { this.disable() })
                                .andThen(
                                    waitForVariable(
                                        { missionManagerWR.get()?.waypointManager?.waypointMissionExecuteState },
                                        WaypointMissionExecuteState.READY,
                                        50L
                                    )
                                )
                                .andThen(createReturnMissionFromReturnPoint(geofenceMission.returnPoint!!))
                                .flatMapCompletable { m: WaypointMission ->
                                    m.convertFromMavlink()
                                        .andThen(Completable.fromAction {
                                            missionManagerWR.get()?.currentWaypointMission = m.copy()
                                        })
                                        .andThen(Completable.defer { missionManagerWR.get()?.currentWaypointMission!!.pushToAircraftUnsafe() })
                                }
                                .andThen(
                                    waitForVariable(
                                        { missionManagerWR.get()?.currentWaypointMission?.isUploadedToAircraft },
                                        true,
                                        50L
                                    )
                                )
                                .andThen(Completable.fromAction {
                                    missionManagerWR.get()?.waypointManager?.uploadedMissionPath =
                                        missionManagerWR.get()?.currentWaypointMission!!.kmz!!.path
                                })
                                .andThen(
                                    waitForVariable(
                                        { missionManagerWR.get()?.waypointManager?.waypointMissionExecuteState },
                                        WaypointMissionExecuteState.READY,
                                        50L
                                    )
                                )
                                .andThen(
                                    djiCompletableFunction(WaypointMissionManager.getInstance()::startMission,
                                        { missionManagerWR.get()?.waypointManager?.uploadedMissionPath!! },
                                        { missionManagerWR.get()?.waypointManager?.waylineNumbersList!! }
                                    )
                                )
                                .subscribe(
                                    { log.w("Started flying to predefined return point") },
                                    { error -> RXUtils.handleError(error) }
                                )
                        )
                    }
                }

                FenceAction.FENCE_ACTION_RTL -> {
                    log.w("Executing geofence breach action RTL!")
                    sendGeofenceWarning(ContextUtil.getContext().getString(R.string.geofence_breach_home))

                    // Stop mission if executing and go home
                    if (missionState == WaypointMissionExecuteState.EXECUTING || missionState == WaypointMissionExecuteState.ENTER_WAYLINE) {
                        compositeDisposable.add(
                            it.waypointManager.stopCurrentMission()
                                .subscribeOn(Schedulers.newThread())
                                .andThen(djiPerformAction(FlightControllerKey.KeyStartGoHome))
                                .subscribe(
                                    { log.w("Started go-home") },
                                    { error -> RXUtils.handleError(error) })
                        )
                    }
                }

                FenceAction.FENCE_ACTION_LAND -> {
                    log.w("Executing geofence breach action LAND!")
                    sendGeofenceWarning(ContextUtil.getContext().getString(R.string.geofence_breach_land))

                    // Stop mission if executing and land
                    if (missionState == WaypointMissionExecuteState.EXECUTING || missionState == WaypointMissionExecuteState.ENTER_WAYLINE) {
                        compositeDisposable.add(
                            it.waypointManager.stopCurrentMission()
                                .subscribeOn(Schedulers.newThread())
                                .andThen(djiPerformAction(FlightControllerKey.KeyStartAutoLanding))
                                .subscribe(
                                    { log.w("Started auto-landing") },
                                    { error -> RXUtils.handleError(error) })
                        )
                    }
                }

                FenceAction.FENCE_ACTION_REPORT -> {
                    log.w("Executing geofence breach action REPORT!")
                    sendGeofenceWarning(ContextUtil.getContext().getString(R.string.geofence_breach_warning))
                }

                FenceAction.FENCE_ACTION_TERMINATE -> {
                    log.w("Executing geofence breach action TERMINATE!")
                    sendGeofenceWarning(ContextUtil.getContext().getString(R.string.geofence_breach_terminate))

                    // Stop mission if executing
                    if (missionState == WaypointMissionExecuteState.EXECUTING || missionState == WaypointMissionExecuteState.ENTER_WAYLINE) {
                        compositeDisposable.add(
                            it.waypointManager.stopCurrentMission()
                                .subscribeOn(Schedulers.newThread())
                                .subscribe(
                                    { log.w("Stopped current mission") },
                                    { error -> RXUtils.handleError(error) })
                        )
                    }
                }

                FenceAction.FENCE_ACTION_HOLD -> {
                    log.w("Executing geofence breach action HOLD!")
                    sendGeofenceWarning(ContextUtil.getContext().getString(R.string.geofence_breach_pause))

                    // Pause mission if executing
                    if (missionState == WaypointMissionExecuteState.EXECUTING || missionState == WaypointMissionExecuteState.ENTER_WAYLINE) {
                        compositeDisposable.add(
                            it.waypointManager.pauseCurrentMission()
                                .subscribeOn(Schedulers.newThread())
                                .subscribe(
                                    { log.w("Paused current mission") },
                                    { error -> RXUtils.handleError(error) })
                        )
                    }
                }

                else -> {}
            }
        }
    }

    fun toMavlinkFenceStatus(): FenceStatus {
        return FenceStatus.builder()
            .breachStatus(0)
            .breachCount(breachCount)
            .breachType(breachTypeLast)
            .breachTime(breachTimeLast)
            .breachMitigation(FenceMitigate.FENCE_MITIGATE_NONE)
            .build()
    }

    //region: parameter changes
    fun onFenceActionChanged(newValue: FenceAction) {
        fenceAction = newValue

        val geofenceMission = missionManagerWR.get()!!.currentGeofenceMission
        if (geofenceMission != null && geofenceMission.returnPoint == null && newValue == FenceAction.FENCE_ACTION_GUIDED) {
            log.w("Set fence action to $newValue but no return point is provided!")
        }

        log.i("Updated fence action to: $newValue")
    }

    fun onAutoEnableChanged(autoEnable: Boolean) {
        this.autoEnable = autoEnable

        log.i("Update auto enable geofence to $autoEnable")
    }

    //endregion
    //region: utils
    private fun createReturnMissionFromReturnPoint(returnPoint: Coordinate): Single<WaypointMission> {
        return Single.fromCallable {
            val kmz = create(connectionWR, "_return_mission")
            val waypointMission = WaypointMission(connectionWR, missionManagerWR, kmz)

            // If the return height is not specified, return at current height
            var returnHeight = returnPoint.z
            if (returnHeight.isNaN()) {
                val droneAltitude = droneWR.get()?.aircraftLocation?.altitude
                returnHeight = droneAltitude ?: 12.0
            }

            val waypointItem = createWaypointItem(
                returnPoint,
                returnHeight.toFloat(),
                0.0f,
                Float.NaN,
                0,
                0,
                systemID,
                droneComponentIdValue,
            )
            waypointMission.addItem(waypointItem, systemID, droneComponentIdValue)
            waypointMission
        }
    } //endregion
}
