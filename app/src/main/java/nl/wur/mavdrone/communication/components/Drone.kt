package nl.wur.mavdrone.communication.components

import java.lang.ref.WeakReference
import java.math.BigInteger
import java.util.concurrent.TimeUnit

import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.value.common.Attitude
import dji.sdk.keyvalue.value.common.LocationCoordinate2D
import dji.sdk.keyvalue.value.common.LocationCoordinate3D
import dji.sdk.keyvalue.value.common.Velocity3D
import dji.sdk.keyvalue.value.flightcontroller.FlightMode
import dji.sdk.keyvalue.value.flightcontroller.GPSSignalLevel
import dji.sdk.keyvalue.value.product.ProductType
import dji.sdk.wpmz.value.mission.WaylineLocationCoordinate2D
import dji.v5.manager.KeyManager
import dji.v5.manager.aircraft.simulator.SimulatorManager
import dji.v5.manager.aircraft.simulator.SimulatorState
import dji.v5.manager.aircraft.simulator.SimulatorStatusListener
import dji.v5.manager.aircraft.waypoint3.model.WaypointMissionExecuteState

import io.reactivex.rxjava3.core.Completable
import io.reactivex.rxjava3.core.Observable
import io.reactivex.rxjava3.core.Single
import io.reactivex.rxjava3.schedulers.Schedulers

import io.dronefleet.mavlink.MavlinkMessage
import io.dronefleet.mavlink.common.Altitude
import io.dronefleet.mavlink.common.AutopilotVersion
import io.dronefleet.mavlink.common.CommandCancel
import io.dronefleet.mavlink.common.CommandLong
import io.dronefleet.mavlink.common.ComponentInformation
import io.dronefleet.mavlink.common.ComponentMetadata
import io.dronefleet.mavlink.common.ExtendedSysState
import io.dronefleet.mavlink.common.FlightInformation
import io.dronefleet.mavlink.common.GlobalPositionInt
import io.dronefleet.mavlink.common.GpsFixType
import io.dronefleet.mavlink.common.GpsRawInt
import io.dronefleet.mavlink.common.HomePosition
import io.dronefleet.mavlink.common.LocalPositionNed
import io.dronefleet.mavlink.common.SetPositionTargetLocalNed
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.MavFrame
import io.dronefleet.mavlink.common.MavGoto
import io.dronefleet.mavlink.common.MavLandedState
import io.dronefleet.mavlink.common.MavProtocolCapability
import io.dronefleet.mavlink.common.MavSeverity
import io.dronefleet.mavlink.common.MavSysStatusSensor
import io.dronefleet.mavlink.common.MavVtolState
import io.dronefleet.mavlink.common.Statustext
import io.dronefleet.mavlink.common.SysStatus
import io.dronefleet.mavlink.common.VfrHud
import io.dronefleet.mavlink.standard.MavAutopilot
import io.dronefleet.mavlink.standard.MavComponent
import io.dronefleet.mavlink.standard.MavModeFlag
import io.dronefleet.mavlink.standard.MavState
import io.dronefleet.mavlink.standard.MavType
import io.dronefleet.mavlink.util.EnumValue

import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.impl.DeviceHealth
import nl.wur.mavdrone.communication.components.impl.DroneParameters
import nl.wur.mavdrone.communication.components.impl.FTPManager
import nl.wur.mavdrone.communication.components.impl.FlightLogManager
import nl.wur.mavdrone.communication.components.impl.HealthManager
import nl.wur.mavdrone.communication.components.impl.MissionManager
import nl.wur.mavdrone.communication.components.impl.OnBoardCamera
import nl.wur.mavdrone.communication.components.impl.RTK
import nl.wur.mavdrone.communication.components.impl.Radar
import nl.wur.mavdrone.communication.components.impl.RemoteController
import nl.wur.mavdrone.communication.components.impl.VirtualStickManager
import nl.wur.mavdrone.communication.components.impl.mission.WaypointMission
import nl.wur.mavdrone.communication.utils.ComponentInfoGenerator
import nl.wur.mavdrone.communication.utils.CustomMode.MAIN_MODE
import nl.wur.mavdrone.communication.utils.CustomMode.SUB_MODE
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createLandItem
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createWaypointItem
import nl.wur.mavdrone.communication.utils.EnumUtils
import nl.wur.mavdrone.communication.utils.EnumUtils.Companion.enumToList
import nl.wur.mavdrone.communication.utils.KMZ
import nl.wur.mavdrone.communication.utils.KMZ.Companion.create
import nl.wur.mavdrone.communication.utils.StandardResponses
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandAccepted
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandCanceled
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandDenied
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandFailed
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandTemporarilyRejected
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandUnsupported
import nl.wur.mavdrone.utils.MavlinkTexts
import nl.wur.mavdrone.utils.RXUtils
import nl.wur.mavdrone.utils.RXUtils.Companion.djiGetValue
import nl.wur.mavdrone.utils.RXUtils.Companion.djiPerformAction
import nl.wur.mavdrone.utils.RXUtils.Companion.djiSetValue
import nl.wur.mavdrone.utils.RxException.DroneNotArmedException
import nl.wur.mavdrone.utils.RxException.NoMissionUploadedException
import nl.wur.mavdrone.utils.toRadians
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.math.sqrt

// Suppress deprecation warnings because we use some unstable Mavlink commands
@Suppress("deprecation")
class Drone(connectionWR: WeakReference<MavConnection>) : MavBaseComponent(
    connectionWR,
    KeyTools.createKey(FlightControllerKey.KeyConnection),
    MavComponent.MAV_COMP_ID_AUTOPILOT1,
    MavType.MAV_TYPE_QUADROTOR,
    MavAutopilot.MAV_AUTOPILOT_GENERIC_MISSION_FULL,
    "MavDrone",
    false
) {
    private val missionManager: MissionManager
    private val ftpManager: FTPManager
    private val flightLogManager: FlightLogManager
    private val droneParameters: DroneParameters
    private val healthManager: HealthManager
    private val radar: Radar
    private val rtk: RTK
    private val remoteController: RemoteController
    private val deviceHealth: DeviceHealth
    private val virtualStickManager: VirtualStickManager
    private var onboardCamera: OnBoardCamera? = null

    private var attitude: Attitude? = null

    var aircraftLocation: LocationCoordinate3D? = null
        private set
    private var aircraftVelocity: Velocity3D? = null

    private var gpsSatelliteCount = 0
    private var gpsSignalLevel: GPSSignalLevel? = null

    private var rtlHomeLocation: LocationCoordinate2D? = null
    private var rtlHomeHeight = 0

    private var motorsOn = false
    var isFlying: Boolean = false
        private set

    var flightMode: FlightMode = FlightMode.UNKNOWN
        private set

    private var mavLandedState = MavLandedState.MAV_LANDED_STATE_UNDEFINED

    private var disArmAfterLandTimeout: Float

    private var armingTime = 0L
    private var takeOffTime = 0L

    var prearmChecksEnabled = true

    private var simulatorHomeSet = false
    private val simulatorStatusListener = SimulatorStatusListener { state -> onSimulatorStateUpdate(state) }

    var productType: ProductType = ProductType.UNKNOWN
        private set

    private var overriddenKMZ: KMZ? = null
    private var overriddenWaypointId = 0


    init {
        addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)

        // Load parameters from system
        disArmAfterLandTimeout = connectionWR.get()?.androidPreferences?.getFloat("COM_DISARM_LAND", 2.0f) ?: 2.0f

        // Implementations
        val droneWR = WeakReference(this)
        missionManager = MissionManager(connectionWR, droneWR)
        flightLogManager = FlightLogManager(connectionWR, droneWR)
        droneParameters = DroneParameters(connectionWR, droneWR, WeakReference(missionManager))
        ftpManager = FTPManager(connectionWR, WeakReference(this))
        radar = Radar(connectionWR, droneWR)
        rtk = RTK(connectionWR, droneWR)
        healthManager = HealthManager(connectionWR, WeakReference(missionManager), WeakReference(rtk))
        remoteController = RemoteController(connectionWR, droneWR)
        deviceHealth = DeviceHealth(connectionWR, droneWR)
        virtualStickManager = VirtualStickManager(connectionWR, droneWR) { attitude?.yaw }
        if (connectionWR.get()?.androidPreferences?.getBoolean("ENABLE_ONBOARD", true) != false) {
            onboardCamera = OnBoardCamera(connectionWR, droneWR)
        }

        connectToProduct()

        // Add flag when we are in simulated mode, check every 2 seconds
        compositeDisposable.add(
            Observable
                .interval(2000, TimeUnit.MILLISECONDS)
                .subscribeOn(Schedulers.newThread())
                .subscribe({
                    if (SimulatorManager.getInstance().isSimulatorEnabled) {
                        addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_HIL_ENABLED)
                    } else {
                        removeMavModeFlag(MavModeFlag.MAV_MODE_FLAG_HIL_ENABLED)
                        simulatorHomeSet = false
                    }
                }, { error -> RXUtils.handleError(error) })
        )

        // The simulator battery drains with simulated flight time and the firmware then
        // interrupts long test flights with its low battery RTH; disable it while the
        // simulator is running and restore the default (enabled) when it stops
        compositeDisposable.add(
            Observable
                .interval(2000, 5000, TimeUnit.MILLISECONDS)
                .subscribeOn(Schedulers.io())
                .observeOn(Schedulers.io())
                .flatMapSingle { Single.fromCallable { SimulatorManager.getInstance().isSimulatorEnabled } }
                .distinctUntilChanged()
                .flatMapCompletable { simEnabled ->
                    djiSetValue(FlightControllerKey.KeyLowBatteryRTHEnabled, !simEnabled)
                }
                .subscribe(
                    { log.i("Changed low battery RTH state because simulation changed") },
                    { error -> RXUtils.handleError(error) }
                )
        )
    }

    override fun create() {
        motorsOn = false
        rtlHomeHeight = 50
        isFlying = false

        compositeDisposable.add(djiGetValue(ProductKey.KeyProductType).subscribe { p: ProductType -> productType = p })

        SimulatorManager.getInstance().addSimulatorStateListener(simulatorStatusListener)

        addDJIListener(KeyTools.createKey(FlightControllerKey.KeyFlightMode)) { value ->
            flightMode = value
            log.i("Changed DJI flight mode: $value")
            when (value) {
                FlightMode.MANUAL -> {
                    removeMavModeFlag(MavModeFlag.MAV_MODE_FLAG_STABILIZE_ENABLED)
                    removeMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                    setCustomMode(MAIN_MODE.PX4_CUSTOM_MAIN_MODE_MANUAL)
                }

                FlightMode.ATTI -> {
                    addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_STABILIZE_ENABLED)
                    removeMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                    setCustomMode(MAIN_MODE.PX4_CUSTOM_MAIN_MODE_ALTCTL)
                }

                FlightMode.GPS_NORMAL, FlightMode.GPS_TRIPOD, FlightMode.GPS_SPORT, FlightMode.TAKE_OFF_READY -> {
                    addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_STABILIZE_ENABLED)
                    setCustomMode(MAIN_MODE.PX4_CUSTOM_MAIN_MODE_POSCTL)
                }
                FlightMode.MOTOR_START -> {
                    addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_SAFETY_ARMED)

                    armingTime = System.currentTimeMillis() * 1000
                }

                FlightMode.POI, FlightMode.VIRTUAL_STICK, FlightMode.PANO, FlightMode.SMART_FLIGHT, FlightMode.SMART_FLY, FlightMode.ATTI_LANDING, FlightMode.CLICK_GO, FlightMode.CINEMATIC, FlightMode.DRAW, FlightMode.GPS_NOVICE, FlightMode.QUICK_MOVIE, FlightMode.TAP_FLY, FlightMode.MASTER_SHOT, FlightMode.APAS, FlightMode.TIME_LAPSE, FlightMode.UNKNOWN -> {}
                FlightMode.AUTO_TAKE_OFF -> {
                    addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                    setCustomMode(MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO)
                    setCustomSubMode(SUB_MODE.PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF)

                    mavLandedState = MavLandedState.MAV_LANDED_STATE_TAKEOFF
                    takeOffTime = System.currentTimeMillis() * 1000
                }

                FlightMode.AUTO_LANDING -> {
                    addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                    setCustomMode(MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO)
                    setCustomSubMode(SUB_MODE.PX4_CUSTOM_SUB_MODE_AUTO_LAND)

                    mavLandedState = MavLandedState.MAV_LANDED_STATE_LANDING
                }

                FlightMode.FORCE_LANDING -> {
                    removeMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                    setCustomMode(MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO)
                    setCustomSubMode(SUB_MODE.PX4_CUSTOM_SUB_MODE_AUTO_LAND)

                    mavLandedState = MavLandedState.MAV_LANDED_STATE_LANDING
                }

                FlightMode.GO_HOME -> {
                    addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                    setCustomMode(MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO)
                    setCustomSubMode(SUB_MODE.PX4_CUSTOM_SUB_MODE_AUTO_RTL)
                    reportGoHomeReason()
                }

                FlightMode.WAYPOINT -> {
                    addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                    setCustomMode(MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO)
                    setCustomSubMode(SUB_MODE.PX4_CUSTOM_SUB_MODE_AUTO_MISSION)
                }

                FlightMode.AUTO_AVOIDANCE -> addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                FlightMode.FOLLOW_ME -> {
                    addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                    setCustomMode(MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO)
                    setCustomSubMode(SUB_MODE.PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET)
                    log.i("Not implemented flight mode: " + flightMode.name)
                }

                else -> log.i("Not implemented flight mode: ${flightMode.name}")
            }
        }

        addDJIListener(KeyTools.createKey(FlightControllerKey.KeyAreMotorsOn)) { value ->
            motorsOn = value
            setState(if (motorsOn) MavState.MAV_STATE_ACTIVE else MavState.MAV_STATE_STANDBY)
        }

        addDJIListener(KeyTools.createKey(FlightControllerKey.KeyIsFlying)) { oldValue, newValue ->
            newValue?.let { nv ->
                isFlying = nv

                // When landing and we have set automatic disarm, add automatic disarm disposable.
                oldValue?.let { ov ->
                    if (ov && !nv  && disArmAfterLandTimeout > 0) {
                        mavLandedState = MavLandedState.MAV_LANDED_STATE_ON_GROUND

                        // Disarm the drone after X seconds after landing. Cancel this when the drone
                        // takes off within the X seconds.
                        compositeDisposable.add(
                            Observable
                                .interval(500, TimeUnit.MILLISECONDS)
                                .take((disArmAfterLandTimeout * 1e3).toLong(), TimeUnit.MILLISECONDS)
                                .takeWhile { !isFlying }
                                .doOnComplete {
                                    if (!isFlying) {
                                        disarmMotors()
                                    }
                                }
                                .subscribe({ }, { error -> RXUtils.handleError(error) })
                        )
                    }

                    if (!ov && nv) {
                        mavLandedState = MavLandedState.MAV_LANDED_STATE_IN_AIR
                    }
                }
            }
        }

        addDJIListener(KeyTools.createKey(FlightControllerKey.KeyHomeLocation)) { value -> rtlHomeLocation = value }
        addDJIListener(KeyTools.createKey(FlightControllerKey.KeyGoHomeHeight)) { value -> rtlHomeHeight = value }
        addDJIListener(KeyTools.createKey(FlightControllerKey.KeyAircraftLocation3D)) { value -> aircraftLocation = value }
        addDJIListener(KeyTools.createKey(FlightControllerKey.KeyAircraftVelocity)) { value -> aircraftVelocity = value }
        addDJIListener(KeyTools.createKey(FlightControllerKey.KeyGPSSatelliteCount)) { value -> gpsSatelliteCount = value }
        addDJIListener(KeyTools.createKey(FlightControllerKey.KeyGPSSignalLevel)) { value -> gpsSignalLevel = value }
        addDJIListener(KeyTools.createKey(FlightControllerKey.KeyAircraftAttitude)) { value -> attitude = value }

        // Add message senders
        addMavlinkMessageSender(SysStatus::class.java, { this.toMavlinkSysStatusMessage() }, 1000, true)
        addMavlinkMessageSender(ExtendedSysState::class.java, { this.toMavlinkExtendedSysStateMessage() }, 1000, true)
        addMavlinkMessageSender(GpsRawInt::class.java, { this.toMavlinkGPSFixMessage() }, 500, true)
        addMavlinkMessageSender(io.dronefleet.mavlink.common.Attitude::class.java, { this.toMavlinkAttitudeMessage() }, 100, true)
        addMavlinkMessageSender(AutopilotVersion::class.java) { this.toMavlinkAutopilotVersionMessage() }
        addMavlinkMessageSender(HomePosition::class.java, { this.toMavlinkHomePositionMessage() }, 5000, true)
        addMavlinkMessageSender(ComponentInformation::class.java) { this.toMavlinkComponentInformationMessage() }
        addMavlinkMessageSender(ComponentMetadata::class.java) { this.toMavlinkComponentMetadataMessage() }
        addMavlinkMessageSender(GlobalPositionInt::class.java, { this.toMavlinkGlobalPositionIntMessage() }, 100, true)
        addMavlinkMessageSender(Altitude::class.java, { this.toMavlinkAltitudeMessage() }, 100, true)
        addMavlinkMessageSender(LocalPositionNed::class.java, { this.toMavlinkLocalPositionNedMessage() }, 100, true)
        addMavlinkMessageSender(VfrHud::class.java, { this.toMavlinkVfrHudMessage() }, 200, true)
        addMavlinkMessageSender(FlightInformation::class.java) { this.toMavlinkFlightInformationMessage() }

        // Add message listeners
        addMavlinkListener(CommandCancel::class.java) { msg -> this.onCommandCancel(msg) }
        addMavlinkListener(SetPositionTargetLocalNed::class.java) { msg -> this.onSetPositionTargetLocalNed(msg) }

        // Forward the firmware battery failsafe states to the GCS; DJI only reports
        // them on its own UI, otherwise an auto RTH appears out of nowhere
        addDJIListener(KeyTools.createKey(FlightControllerKey.KeyIsLowBatteryWarning)) { value ->
            if (value) {
                log.w("Low battery warning from firmware")
                sendMessage(Statustext.builder().severity(MavSeverity.MAV_SEVERITY_WARNING).text("Low battery warning").build())
            }
        }
        addDJIListener(KeyTools.createKey(FlightControllerKey.KeyIsSeriousLowBatteryWarning)) { value ->
            if (value) {
                log.w("Serious low battery warning from firmware")
                sendMessage(Statustext.builder().severity(MavSeverity.MAV_SEVERITY_ERROR).text("Serious low battery: landing").build())
            }
        }
        addDJIListener(KeyTools.createKey(FlightControllerKey.KeyRequireGoHomeByLowBatteryVoltage)) { value ->
            if (value) {
                log.w("Firmware triggered low battery RTH (voltage)")
                sendMessage(Statustext.builder().severity(MavSeverity.MAV_SEVERITY_ERROR).text("Low battery: firmware RTH").build())
            }
        }
        addDJIListener(KeyTools.createKey(FlightControllerKey.KeyRequireGoHomeBySmartBatteryPercent)) { value ->
            if (value) {
                log.w("Firmware triggered low battery RTH (percent)")
                sendMessage(Statustext.builder().severity(MavSeverity.MAV_SEVERITY_ERROR).text("Low battery: firmware RTH").build())
            }
        }

        // Initialize landed state after 1 second to make sure we have both the isFlying key as the flightMode key
        compositeDisposable.add(
            Completable
                .timer(1, TimeUnit.SECONDS, Schedulers.newThread())
                .subscribe {
                    mavLandedState = when {
                        isFlying -> MavLandedState.MAV_LANDED_STATE_IN_AIR
                        else -> MavLandedState.MAV_LANDED_STATE_ON_GROUND
                    }
                }
        )

        missionManager.create()
        flightLogManager.create()
        droneParameters.create()
        ftpManager.create()
        radar.create()
        rtk.create()
        healthManager.create()
        remoteController.create()
        deviceHealth.create()
        onboardCamera?.create()
    }

    override fun destroy() {
        virtualStickManager.stopAll()
        SimulatorManager.getInstance().removeSimulatorStateListener(simulatorStatusListener)
        missionManager.destroy()
        flightLogManager.destroy()
        droneParameters.destroy()
        ftpManager.destroy()
        radar.destroy()
        rtk.destroy()
        healthManager.destroy()
        remoteController.destroy()
        deviceHealth.destroy()
        onboardCamera?.destroy()

        super.destroy()
    }

    override fun onGCSDisconnected() {
        // When running a mission, pause that mission
        if (flightMode == FlightMode.WAYPOINT) {
            compositeDisposable.add(
                missionManager.waypointManager.stopCurrentMission()
                    .subscribeOn(Schedulers.newThread())
                    .subscribe({
                        log.w("Mission stopped because GCS connection is gone!")
                        connectionWR.get()?.showToast("Mission stopped because GCS connection is gone!", 2)
                    }, { error -> RXUtils.handleError(error) })
            )
        }
    }

    //region: commands
    public override fun onCommandLong(msg: MavlinkMessage<*>): Boolean {
        if (super.onCommandLong(msg)
            || (onboardCamera?.onCommandLong(msg) == true)
            || missionManager.onCommandLong(msg)) {
            return true
        }

        val payload = msg.payload as CommandLong

        if (isCommandForMe(payload)) {
            log.i("Received command: " + payload.command().entry())

            when (payload.command().entry()) {
                MavCmd.MAV_CMD_DO_SET_MODE -> {
                    changeFlightMode(
                        payload.param1().roundToInt(),
                        payload.param2().roundToInt(),
                        payload.param3().roundToInt(),
                        msg.originSystemId,
                        msg.originComponentId
                    )
                    return true
                }

                MavCmd.MAV_CMD_LOGGING_START -> {
                    sendMessage(commandAccepted(MavCmd.MAV_CMD_LOGGING_START, msg.originSystemId, msg.originComponentId))
                    return true
                }

                MavCmd.MAV_CMD_LOGGING_STOP -> {
                    sendMessage(commandAccepted(MavCmd.MAV_CMD_LOGGING_STOP, msg.originSystemId, msg.originComponentId))
                    return true
                }

                MavCmd.MAV_CMD_COMPONENT_ARM_DISARM -> {
                    if (payload.param1().roundToInt() == 0) {
                        cmdDisarmMotors(msg.originSystemId, msg.originComponentId)
                    } else {
                        cmdArmMotors(msg.originSystemId, msg.originComponentId)
                    }
                    return true
                }

                MavCmd.MAV_CMD_DO_SET_HOME -> {
                    if (payload.param1() > 0.5) {
                        log.i("Set home to current location")
                        compositeDisposable.add(
                            djiPerformAction(FlightControllerKey.KeyHomeLocationUsingCurrentAircraftLocation)
                                .subscribe(
                                    { sendMessage(commandAccepted(MavCmd.MAV_CMD_DO_SET_HOME, msg.originSystemId, msg.originComponentId)) },
                                    { error ->
                                        onError(
                                            error,
                                            StandardResponses::commandFailed,
                                            MavCmd.MAV_CMD_DO_SET_HOME,
                                            msg.originSystemId,
                                            msg.originComponentId
                                        )
                                    })
                        )
                    } else {
                        val homeLocation = LocationCoordinate2D()
                        homeLocation.latitude = payload.param5().toDouble()
                        homeLocation.longitude = payload.param6().toDouble()
                        val altitudeMeter = payload.param7().roundToInt()

                        log.i("Set home to $homeLocation")
                        compositeDisposable.add(
                            djiSetValue(FlightControllerKey.KeyHomeLocation, homeLocation)
                                .andThen(djiSetValue(FlightControllerKey.KeyGoHomeHeight, altitudeMeter))
                                .subscribe(
                                    { sendMessage(commandAccepted(MavCmd.MAV_CMD_DO_SET_HOME, msg.originSystemId, msg.originComponentId)) },
                                    { error -> onError(error, StandardResponses::commandFailed, MavCmd.MAV_CMD_DO_SET_HOME) })
                        )
                    }
                    return true
                }

                MavCmd.MAV_CMD_GET_HOME_POSITION -> {
                    if (!isHomePositionSet) {
                        sendMessage(commandFailed(MavCmd.MAV_CMD_GET_HOME_POSITION, msg.originSystemId, msg.originComponentId))
                        return true
                    }

                    sendMessage(commandAccepted(MavCmd.MAV_CMD_GET_HOME_POSITION, msg.originSystemId, msg.originComponentId))
                    val homePosition = toMavlinkHomePositionMessage()
                    sendMessage(homePosition)
                    return true
                }

                MavCmd.MAV_CMD_OVERRIDE_GOTO -> {
                    val continueParam = EnumValue.create(MavGoto::class.java, payload.param1().roundToInt()).entry()
                    val positionParam = EnumValue.create(MavGoto::class.java, payload.param2().roundToInt()).entry()

                    if (continueParam == MavGoto.MAV_GOTO_DO_CONTINUE) {
                        overriddenKMZ?.let { kmz ->
                            compositeDisposable.add(
                                WaypointMission.fromKMZ(connectionWR,WeakReference(missionManager), kmz)
                                    .flatMapCompletable { waypointMission: WaypointMission? ->
                                        missionManager.currentWaypointMission = waypointMission
                                        missionManager.waypointManager.setCurrentWaylineIdFromMavlink(overriddenWaypointId)
                                        Completable.complete()
                                    }
                                    .andThen(missionManager.currentWaypointMission?.convertToMavlink(systemID, EnumValue.create(componentID).value()) ?: Completable.complete())
                                    .andThen(missionManager.waypointManager.startCurrentMission())
                                    .subscribe(
                                        { sendMessage(commandAccepted(payload.command().entry(), msg.originSystemId, msg.originComponentId)) },
                                        { error ->
                                            onError(
                                                error,
                                                StandardResponses::commandFailed,
                                                payload.command().entry(),
                                                msg.originSystemId,
                                                msg.originComponentId
                                            )
                                        })
                            )
                        } ?: run {
                            compositeDisposable.add(
                                missionManager.waypointManager.resumeCurrentMission()
                                    .subscribeOn(Schedulers.newThread())
                                    .subscribe(
                                        { sendMessage(commandAccepted(MavCmd.MAV_CMD_OVERRIDE_GOTO, msg.originSystemId, msg.originComponentId)) },
                                        { error -> onError(error, StandardResponses::commandFailed, MavCmd.MAV_CMD_OVERRIDE_GOTO, msg.originSystemId, msg.originComponentId) },
                                    )
                            )
                        }

                        return true
                    }

                    if (continueParam == MavGoto.MAV_GOTO_DO_HOLD && positionParam == MavGoto.MAV_GOTO_HOLD_AT_CURRENT_POSITION) {
                        compositeDisposable.add(
                            missionManager.waypointManager.pauseCurrentMission()
                                .subscribeOn(Schedulers.newThread())
                                .subscribe(
                                    { sendMessage(commandAccepted(MavCmd.MAV_CMD_OVERRIDE_GOTO, msg.originSystemId, msg.originComponentId)) },
                                    { error -> onError(error, StandardResponses::commandFailed, MavCmd.MAV_CMD_OVERRIDE_GOTO, msg.originSystemId, msg.originComponentId) },
                                )
                        )
                        return true
                    }

                    if (continueParam == MavGoto.MAV_GOTO_DO_HOLD && positionParam == MavGoto.MAV_GOTO_HOLD_AT_SPECIFIED_POSITION) {
                        overriddenKMZ = missionManager.currentWaypointMission?.kmz
                        overriddenWaypointId = missionManager.currentWaypointMission?.currentMavlinkId ?: 0

                        val frame = EnumValue.create(MavFrame::class.java, payload.param3().roundToInt()).entry()
                        if (frame != MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT) {
                            log.e("MavFrame for GOTO mission is not valid. Should be MAV_FRAME_GLOBAL_RELATIVE_ALT")
                            return false
                        }

                        val location = WaylineLocationCoordinate2D(payload.param5().toDouble(), payload.param6().toDouble())
                        val waypointItem = createWaypointItem(location, payload.param7(), 0.0f, payload.param4(), 0, 0, payload.targetSystem(), payload.targetComponent())

                        val kmz = create(connectionWR, "_goto_mission")
                        val gotoMission = WaypointMission(connectionWR, WeakReference(missionManager), kmz)
                        gotoMission.addItem(waypointItem, msg.originSystemId, msg.originComponentId)

                        compositeDisposable.add(
                            missionManager.waypointManager.stopCurrentMissionWhenNeeded()  // Mission could already be finished
                                .andThen(gotoMission.convertFromMavlink())
                                .andThen(Completable.fromAction { missionManager.currentWaypointMission = gotoMission })
                                .andThen(missionManager.waypointManager.startCurrentMission())
                                .subscribe(
                                    { sendMessage(commandAccepted(payload.command().entry(), msg.originSystemId, msg.originComponentId)) },
                                    { error ->
                                        onError(
                                            error,
                                            StandardResponses::commandFailed,
                                            payload.command().entry(),
                                            msg.originSystemId,
                                            msg.originComponentId
                                        )
                                    })

                        )

                        return true
                    }

                    log.e("Unsupported combination of continue and position!")
                    sendMessage(commandDenied(MavCmd.MAV_CMD_OVERRIDE_GOTO, msg.originSystemId, msg.originComponentId))
                    return true
                }

                MavCmd.MAV_CMD_NAV_TAKEOFF -> {
                    if (!containsMavModeFlag(MavModeFlag.MAV_MODE_FLAG_STABILIZE_ENABLED)
                        && !containsMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                    ) {
                        log.w("Cannot takeoff because remote controller is not in a stabilized or guided mode!")
                        sendMessage(commandDenied(MavCmd.MAV_CMD_NAV_TAKEOFF, msg.originSystemId, msg.originComponentId))
                    } else if (!containsMavModeFlag(MavModeFlag.MAV_MODE_FLAG_SAFETY_ARMED)) {
                        log.w("Cannot takeoff because the drone is not armed!")
                        sendMessage(commandDenied(MavCmd.MAV_CMD_NAV_TAKEOFF, msg.originSystemId, msg.originComponentId))
                    } else {
                        val requestedHeight = payload.param7()

                        // DJI auto takeoff only hovers at ~1.2 m (the firmware minimum);
                        // targets below that are covered by the plain takeoff, a higher
                        // NAV_TAKEOFF target is climbed with the virtual stick afterwards
                        val climbTarget =
                            if (!requestedHeight.isNaN() && requestedHeight > VirtualStickManager.MIN_TAKEOFF_HEIGHT) requestedHeight
                            else null

                        compositeDisposable.add(
                            performStartTakeoff()
                                .subscribe(
                                    {
                                        sendMessage(commandAccepted(MavCmd.MAV_CMD_NAV_TAKEOFF, msg.originSystemId, msg.originComponentId))
                                        takeOffTime = System.currentTimeMillis() * 1000
                                        climbTarget?.let { virtualStickManager.climbToHeight(it) }
                                    },
                                    { error ->
                                        onError(
                                            error,
                                            StandardResponses::commandFailed,
                                            MavCmd.MAV_CMD_NAV_TAKEOFF,
                                            msg.originSystemId,
                                            msg.originComponentId
                                        )
                                    })
                        )
                    }
                    return true
                }

                MavCmd.MAV_CMD_DO_LAND_START, MavCmd.MAV_CMD_NAV_LAND -> {
                    // If latitude and longitude are provided, create land mission with waypoint at land location and
                    // land as finish action. Otherwise, just start auto-landing.
                    if (!payload.param5().isNaN()
                        && !payload.param6().isNaN()
                        && payload.param5().toDouble() != 0.0
                        && payload.param6().toDouble() != 0.0
                    ) {
                        val landLocation = WaylineLocationCoordinate2D(payload.param5().toDouble(), payload.param6().toDouble())
                        val landItem = createLandItem(landLocation, payload.param4(), 0, 0, payload.targetSystem(), payload.targetComponent())

                        val kmz = create(connectionWR, "_land_mission")
                        val landMission = WaypointMission(connectionWR, WeakReference(missionManager), kmz)
                        landMission.addItem(landItem, msg.originSystemId, msg.originComponentId)

                        compositeDisposable.add(
                            landMission.convertFromMavlink()
                                .andThen(Completable.fromAction { missionManager.currentWaypointMission = landMission })
                                .andThen(missionManager.waypointManager.startCurrentMission())
                                .subscribe(
                                    { sendMessage(commandAccepted(payload.command().entry(), msg.originSystemId, msg.originComponentId)) },
                                    { error ->
                                        onError(
                                            error,
                                            StandardResponses::commandFailed,
                                            payload.command().entry(),
                                            msg.originSystemId,
                                            msg.originComponentId
                                        )
                                    })

                        )
                    } else {
                        compositeDisposable.add(
                            djiPerformAction(FlightControllerKey.KeyStartAutoLanding)
                                .subscribe(
                                    {
                                        sendMessage(commandAccepted(payload.command().entry(), msg.originSystemId, msg.originComponentId))
                                        watchLandingConfirmation()
                                    },
                                    { error ->
                                        onError(
                                            error,
                                            StandardResponses::commandFailed,
                                            payload.command().entry(),
                                            msg.originSystemId,
                                            msg.originComponentId
                                        )
                                    })
                        )
                    }
                    return true
                }

                MavCmd.MAV_CMD_NAV_RETURN_TO_LAUNCH -> {
                    // Attribute a firmware RTH to the GCS command that caused it
                    sendMessage(
                        Statustext.builder()
                            .severity(MavSeverity.MAV_SEVERITY_WARNING)
                            .text(MavlinkTexts.truncateStatustext("RTH commanded by GCS ${msg.originSystemId}/${msg.originComponentId}"))
                            .build()
                    )
                    compositeDisposable.add(
                        djiPerformAction(FlightControllerKey.KeyStartGoHome)
                            .subscribe(
                                { sendMessage(commandAccepted(MavCmd.MAV_CMD_NAV_RETURN_TO_LAUNCH, msg.originSystemId, msg.originComponentId)) },
                                { error ->
                                    onError(
                                        error,
                                        StandardResponses::commandFailed,
                                        MavCmd.MAV_CMD_NAV_LAND,
                                        msg.originSystemId,
                                        msg.originComponentId
                                    )
                                })
                    )
                    return true
                }

                MavCmd.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES -> {
                    sendMessage(commandAccepted(MavCmd.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, msg.originSystemId, msg.originComponentId))
                    val autopilotVersion = toMavlinkAutopilotVersionMessage()
                    sendMessage(autopilotVersion)
                    return true
                }

                MavCmd.MAV_CMD_REQUEST_FLIGHT_INFORMATION -> {
                    sendMessage(commandAccepted(MavCmd.MAV_CMD_REQUEST_FLIGHT_INFORMATION, msg.originSystemId, msg.originComponentId))

                    val flightInformation = toMavlinkFlightInformationMessage()
                    sendMessage(flightInformation)
                    return true
                }

                MavCmd.MAV_CMD_RUN_PREARM_CHECKS -> {
                    val failedChecks = runPrearmChecks()

                    failedChecks.forEach { check ->
                        sendMessage(
                            Statustext.builder()
                                .severity(MavSeverity.MAV_SEVERITY_ERROR)
                                .text(MavlinkTexts.truncateStatustext("PreArm: $check"))
                                .build()
                        )
                    }

                    if (failedChecks.isEmpty()) {
                        log.i("Prearm checks passed")
                        sendMessage(commandAccepted(MavCmd.MAV_CMD_RUN_PREARM_CHECKS, msg.originSystemId, msg.originComponentId))
                    } else {
                        log.w("Prearm checks failed: $failedChecks")
                        sendMessage(commandDenied(MavCmd.MAV_CMD_RUN_PREARM_CHECKS, msg.originSystemId, msg.originComponentId))
                    }
                    return true
                }

                else -> {
                    log.i("Unsupported command: " + payload.command().entry())
                    sendMessage(commandUnsupported(payload.command().entry(), msg.originSystemId, msg.originComponentId))
                    return false
                }
            }
        }

        return false
    }

    // Offboard velocity setpoints from the GCS (MAVSDK offboard sends these at 20 Hz).
    // Only the velocity components are used, DJI has no horizontal position mode; the
    // GCS runs its own closed-loop control on top of these.
    private fun onSetPositionTargetLocalNed(msg: MavlinkMessage<*>) {
        val payload = msg.payload as SetPositionTargetLocalNed

        if (payload.targetSystem() != systemID
            || (payload.targetComponent() != EnumValue.create(componentID).value() && payload.targetComponent() != 0)
        ) {
            return
        }

        // Never engage the virtual stick on the ground
        if (!isFlying && !motorsOn) {
            return
        }

        // type mask bits 3-5 mark the velocity components as ignored
        val typeMask = payload.typeMask().value()
        if ((typeMask and VELOCITY_IGNORED_MASK) == VELOCITY_IGNORED_MASK) {
            return
        }

        val forward: Double
        val right: Double
        when (payload.coordinateFrame().entry()) {
            MavFrame.MAV_FRAME_BODY_NED -> {
                forward = payload.vx().toDouble()
                right = payload.vy().toDouble()
            }

            MavFrame.MAV_FRAME_LOCAL_NED -> {
                // Rotate the NED velocity into the body frame with the current heading
                val yaw = attitude?.yaw?.toRadians() ?: return
                val north = payload.vx().toDouble()
                val east = payload.vy().toDouble()
                forward = north * cos(yaw) + east * sin(yaw)
                right = -north * sin(yaw) + east * cos(yaw)
            }

            else -> return
        }

        virtualStickManager.feedVelocitySetpoint(
            forward, right,
            payload.vz().toDouble(),   // down positive, DJI vertical velocity is up positive
            Math.toDegrees(payload.yawRate().toDouble())
        )
    }

    // An unexpected firmware RTH is hard to attribute afterwards (failsafe, low
    // battery, RC action...); capture the FC's own assessment the moment the GO_HOME
    // mode is reported and broadcast it to the GCS
    private fun reportGoHomeReason() {
        compositeDisposable.add(
            Single.zip(
                djiGetValue(FlightControllerKey.KeyIsFailSafe),
                djiGetValue(FlightControllerKey.KeyFailsafeAction),
                djiGetValue(FlightControllerKey.KeyRequireGoHomeByLowBatteryVoltage),
                djiGetValue(FlightControllerKey.KeyRequireGoHomeBySmartBatteryPercent),
                djiGetValue(FlightControllerKey.KeyBatteryPowerPercent)
            ) { failsafe, failsafeAction, byVoltage, byPercent, batteryPercent ->
                "RTL: fs=$failsafe/$failsafeAction lowV=$byVoltage lowP=$byPercent batt=$batteryPercent%"
            }.subscribe(
                { reason ->
                    log.w("Go home reason: $reason")
                    sendMessage(
                        Statustext.builder()
                            .severity(MavSeverity.MAV_SEVERITY_WARNING)
                            .text(MavlinkTexts.truncateStatustext(reason))
                            .build()
                    )
                },
                { error -> RXUtils.handleError(error) }
            )
        )
    }

    // StartTakeoff races the FC connection coming back online after an app restart or
    // link rebuild: KeyManager returns REQUEST_HANDLER_NOT_FOUND (-1) until the FC
    // component is up again. Retry briefly before giving up so GCS commands sent in
    // that window succeed once the connection settles.
    private fun performStartTakeoff(): Completable =
        Completable.create { emitter ->
            emitter.setDisposable(
                Observable.interval(0, TAKEOFF_RETRY_INTERVAL_MS, TimeUnit.MILLISECONDS)
                    .take(TAKEOFF_RETRY_COUNT.toLong())
                    .subscribe({
                        djiPerformAction(FlightControllerKey.KeyStartTakeoff)
                            .subscribe(
                                { emitter.onComplete() },
                                { error ->
                                    log.w("StartTakeoff attempt failed, retrying: ${error.message}")
                                }
                            )
                    }, { emitter.tryOnError(it) })
            )
        }.timeout(
            TAKEOFF_RETRY_INTERVAL_MS * (TAKEOFF_RETRY_COUNT + 1),
            TimeUnit.MILLISECONDS,
            Schedulers.newThread()
        )

    // The firmware pauses auto landing at the confirmation height and waits for the
    // DJI app "slide to confirm landing"; confirm via the SDK instead so GCS-initiated
    // landings complete without manual interaction
    private fun watchLandingConfirmation() {
        compositeDisposable.add(
            Observable.interval(500, TimeUnit.MILLISECONDS)
                .take(LANDING_CONFIRMATION_POLL_COUNT)
                .flatMapSingle { djiGetValue(FlightControllerKey.KeyIsLandingConfirmationNeeded) }
                .filter { it }
                .firstOrError()
                .flatMapCompletable {
                    log.i("Landing confirmation needed, confirming automatically")
                    djiPerformAction(FlightControllerKey.KeyConfirmLanding)
                }
                .subscribe(
                    { log.i("Landing confirmation done") },
                    { error ->
                        if (error is NoSuchElementException) {
                            // No confirmation request within the watch window
                            log.i("No landing confirmation was requested")
                        } else {
                            RXUtils.handleError(error)
                        }
                    }
                )
        )
    }

    private fun onCommandCancel(msg: MavlinkMessage<*>) {
        val payload = msg.payload as CommandCancel

        if (payload.targetSystem() == systemID && (payload.targetComponent() == EnumValue.create(componentID).value() || payload.targetComponent() == 0)) {
            when (payload.command().entry()) {
                MavCmd.MAV_CMD_NAV_TAKEOFF -> {
                    virtualStickManager.stopClimb()
                    compositeDisposable.add(
                        djiPerformAction(FlightControllerKey.KeyStopTakeoff)
                            .subscribe(
                                { sendMessage(commandCanceled(MavCmd.MAV_CMD_NAV_TAKEOFF, msg.originSystemId, msg.originComponentId)) },
                                { error ->
                                    onError(
                                        error,
                                        StandardResponses::commandFailed,
                                        MavCmd.MAV_CMD_NAV_TAKEOFF,
                                        msg.originSystemId,
                                        msg.originComponentId
                                    )
                                })
                    )
                }

                MavCmd.MAV_CMD_DO_LAND_START, MavCmd.MAV_CMD_NAV_LAND -> compositeDisposable.add(
                    djiPerformAction(FlightControllerKey.KeyStopAutoLanding)
                        .subscribe(
                            { sendMessage(commandCanceled(payload.command().entry(), msg.originSystemId, msg.originComponentId)) },
                            { error ->
                                onError(
                                    error,
                                    StandardResponses::commandFailed,
                                    payload.command().entry(),
                                    msg.originSystemId,
                                    msg.originComponentId
                                )
                            })
                )

                MavCmd.MAV_CMD_NAV_RETURN_TO_LAUNCH -> compositeDisposable.add(
                    djiPerformAction(FlightControllerKey.KeyStopGoHome)
                        .subscribe(
                            { sendMessage(commandCanceled(MavCmd.MAV_CMD_NAV_RETURN_TO_LAUNCH, msg.originSystemId, msg.originComponentId)) },
                            { error ->
                                onError(
                                    error,
                                    StandardResponses::commandFailed,
                                    MavCmd.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                                    msg.originSystemId,
                                    msg.originComponentId
                                )
                            })
                )

                else -> {}
            }
        }
    }

    private fun runPrearmChecks(): List<String> {
        if (!prearmChecksEnabled) {
            return emptyList()
        }

        val failedChecks = mutableListOf<String>()

        // GPS
        when (gpsSignalLevel) {
            null, GPSSignalLevel.LEVEL_0, GPSSignalLevel.LEVEL_1 -> failedChecks.add("GPS has no fix")
            GPSSignalLevel.LEVEL_2 -> failedChecks.add("GPS only has a 2D fix")
            else -> {}
        }

        // RTK
        if (rtk.isRtkEnabled && !rtk.isRtkHealthy) {
            failedChecks.add("RTK is not healthy")
        }

        // Home location
        if (rtlHomeLocation == null) {
            failedChecks.add("Home location not set")
        }

        // Sensor health as determined by the DJI health manager
        healthManager.getUnhealthySensors().forEach {
            failedChecks.add("${prearmSensorName(it)} is not healthy")
        }

        // Battery level
        connectionWR.get()?.mavlinkComponents?.batteries?.forEach { battery ->
            if (battery.remainingPercent in 0 until battery.lowWarningThreshold) {
                failedChecks.add("Battery ${battery.componentIndex + 1} is low (${battery.remainingPercent}%)")
            }
        }

        return failedChecks
    }

    private fun prearmSensorName(sensor: MavSysStatusSensor): String = when (sensor) {
        MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_3D_GYRO, MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_YAW_POSITION -> "Compass"
        MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_GPS -> "GPS"
        MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_RC_RECEIVER -> "Remote controller"
        MavSysStatusSensor.MAV_SYS_STATUS_GEOFENCE -> "Geofence"
        MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_BATTERY -> "Battery"
        MavSysStatusSensor.MAV_SYS_STATUS_OBSTACLE_AVOIDANCE -> "Obstacle avoidance"
        else -> sensor.name
    }

    private fun onSimulatorStateUpdate(state: SimulatorState) {
        if (!SimulatorManager.getInstance().isSimulatorEnabled) {
            simulatorHomeSet = false
            return
        }

        val location = state.location ?: return
        // SimulatorState positionX/Y/Z are in the takeoff-centered NED frame, so the
        // Z axis points down and must be negated to get an up-positive altitude
        aircraftLocation = LocationCoordinate3D(location.latitude, location.longitude, -state.positionZ.toDouble())
        attitude = Attitude(state.pitch.toDouble(), state.roll.toDouble(), state.yaw.toDouble())

        if (!simulatorHomeSet) {
            rtlHomeLocation = LocationCoordinate2D(location.latitude, location.longitude)
            simulatorHomeSet = true
            log.i("Simulator home set to $rtlHomeLocation")
        }
    }

    private fun changeFlightMode(mavMode: Int, customMode: Int, subMode: Int, systemId: Int, componentId: Int) {
        // Set the Mavlink modes
        val mavModeFlagList = enumToList(EnumValue.create(MavModeFlag::class.java, mavMode), MavModeFlag::class.java)
        mavModeFlagList.forEach { addMavModeFlag(it) }
        log.i("Changed mode. Current flags: ${getMavModeFlags()}")

        // MAVSDK may send custom PX4 modes
        if (containsMavModeFlag(MavModeFlag.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) {
            val customFlightMode: MAIN_MODE
            val customSubFlightMode: SUB_MODE
            try {
                customFlightMode = MAIN_MODE.fromValue(customMode)
                customSubFlightMode = SUB_MODE.fromValue(subMode)
            } catch (e: IllegalArgumentException) {
                log.w("Cannot change mode: unknown custom mode $customMode/$subMode")
                sendMessage(commandFailed(MavCmd.MAV_CMD_DO_SET_MODE, systemId, componentId))
                return
            }

            log.i("Set mode to $customFlightMode-$customSubFlightMode")

            if (customFlightMode == MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD
                || (customFlightMode == MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO && customSubFlightMode == SUB_MODE.INVALID)
            ) {
                // PX4 OFFBOARD and ArduPilot-style GUIDED (main mode 4 without a PX4 auto
                // sub mode) both map to the GCS-controlled guided state, since DJI has no
                // dedicated mode for it
                addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                setCustomMode(MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD)
                setCustomSubMode(SUB_MODE.INVALID)
                sendMessage(commandAccepted(MavCmd.MAV_CMD_DO_SET_MODE, systemId, componentId))
            } else if (customFlightMode == MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO && customSubFlightMode == SUB_MODE.PX4_CUSTOM_SUB_MODE_AUTO_MISSION) {
                // If there is a mission present and current status is paused, resume mission
                // instead of restarting it (https://github.com/mavlink/MAVSDK-Python/issues/442)
                if (missionManager.waypointManager.waypointMissionExecuteState == WaypointMissionExecuteState.INTERRUPTED) {
                    log.i("Mission active, resuming mission...")
                    compositeDisposable.add(
                        missionManager.waypointManager.resumeCurrentMission()
                            .subscribeOn(Schedulers.newThread())
                            .subscribe(
                                { sendMessage(commandAccepted(MavCmd.MAV_CMD_DO_SET_MODE, systemId, componentId)) },
                                { error -> onError(error, StandardResponses::commandFailed,MavCmd.MAV_CMD_DO_SET_MODE, systemId, componentId) },
                            )
                    )
                } else {
                    compositeDisposable.add(
                        missionManager.waypointManager.startCurrentMission()
                            .subscribeOn(Schedulers.newThread())
                            .subscribe(
                                {
                                    sendMessage(
                                        commandAccepted(
                                            MavCmd.MAV_CMD_DO_SET_MODE,
                                            systemId,
                                            componentId
                                        )
                                    )
                                },
                                { error ->
                                    when (error) {
                                        is DroneNotArmedException -> {
                                            onError(
                                                error,
                                                StandardResponses::commandFailed,
                                                MavCmd.MAV_CMD_DO_SET_MODE,
                                                systemId,
                                                componentId
                                            )
                                        }

                                        is NoMissionUploadedException -> {
                                            // If no mission loaded, just send a warning and accept this command
                                            log.w("Cannot start mission: no mission is loaded!")
                                            sendMessage(
                                                commandAccepted(
                                                    MavCmd.MAV_CMD_DO_SET_MODE,
                                                    systemId,
                                                    componentId
                                                )
                                            )
                                        }

                                        else -> {
                                            onError(
                                                error,
                                                StandardResponses::commandFailed,
                                                MavCmd.MAV_CMD_DO_SET_MODE,
                                                systemId,
                                                componentId
                                            )
                                        }
                                    }
                                })
                    )
                }
            } else if (customFlightMode == MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO && customSubFlightMode == SUB_MODE.PX4_CUSTOM_SUB_MODE_AUTO_RTL) {
                // Attribute a firmware RTH to the GCS mode switch that caused it
                sendMessage(
                    Statustext.builder()
                        .severity(MavSeverity.MAV_SEVERITY_WARNING)
                        .text(MavlinkTexts.truncateStatustext("RTH (mode) commanded by GCS $systemId/$componentId"))
                        .build()
                )
                compositeDisposable.add(
                    djiPerformAction(FlightControllerKey.KeyStartGoHome)
                        .subscribeOn(Schedulers.newThread())
                        .subscribe(
                            { sendMessage(commandAccepted(MavCmd.MAV_CMD_DO_SET_MODE, systemId, componentId)) },
                            { error ->
                                onError(
                                    error,
                                    StandardResponses::commandFailed,
                                    MavCmd.MAV_CMD_DO_SET_MODE,
                                    systemId,
                                    componentId
                                )
                            }
                        )
                )
            } else {
                sendMessage(commandFailed(MavCmd.MAV_CMD_DO_SET_MODE, systemId, componentId))
            }
        } else {
            sendMessage(commandAccepted(MavCmd.MAV_CMD_DO_SET_MODE, systemId, componentId))
        }
    }

    private fun cmdArmMotors(systemId: Int, componentId: Int) {
        if (gpsSignalLevel == GPSSignalLevel.UNKNOWN || gpsSignalLevel == GPSSignalLevel.LEVEL_NONE || gpsSignalLevel == GPSSignalLevel.LEVEL_0 || gpsSignalLevel == GPSSignalLevel.LEVEL_1 || gpsSignalLevel == GPSSignalLevel.LEVEL_2) {
            log.w("Cannot arm drone! GPS signal is too weak.")
            sendMessage(commandTemporarilyRejected(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM, systemId, componentId))
            return
        }

        if (prearmChecksEnabled) {
            val failedChecks = runPrearmChecks()
            if (failedChecks.isNotEmpty()) {
                failedChecks.forEach { check ->
                    sendMessage(
                        Statustext.builder()
                            .severity(MavSeverity.MAV_SEVERITY_ERROR)
                            .text(MavlinkTexts.truncateStatustext("PreArm: $check"))
                            .build()
                    )
                }
                log.w("Cannot arm drone, prearm checks failed: $failedChecks")
                sendMessage(commandDenied(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM, systemId, componentId))
                return
            }
            log.i("Prearm checks passed")
        }

        if (!containsMavModeFlag(MavModeFlag.MAV_MODE_FLAG_SAFETY_ARMED)) {
            log.i("Motors are armed!")
            addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_SAFETY_ARMED)
            sendMessage(commandAccepted(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM, systemId, componentId))
        } else {
            log.w("Cannot arm drone because it's already armed!")
            sendMessage(commandFailed(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM, systemId, componentId))
        }

        armingTime = System.currentTimeMillis() * 1000
    }

    private fun cmdDisarmMotors(systemId: Int, componentId: Int) {
        if (isFlying) {
            log.w("Cannot disarm drone because it's still flying!")
            sendMessage(commandDenied(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM, systemId, componentId))
            return
        }

        if (!containsMavModeFlag(MavModeFlag.MAV_MODE_FLAG_SAFETY_ARMED)) {
            log.w("Cannot disarm drone because it's not armed!")
            sendMessage(commandFailed(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM, systemId, componentId))
            return
        }

        disarmMotors()
        sendMessage(commandAccepted(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM, systemId, componentId))
    }

    private fun disarmMotors() {
        if (containsMavModeFlag(MavModeFlag.MAV_MODE_FLAG_SAFETY_ARMED)) {
            log.i("Motors are disarmed!")
            removeMavModeFlag(MavModeFlag.MAV_MODE_FLAG_SAFETY_ARMED)
        }
    }

    //endregion
    //region: messages
    private fun toMavlinkFlightInformationMessage(): FlightInformation {
        return FlightInformation.builder()
            .timeBootMs(bootMillis)
            .armingTimeUtc(BigInteger.valueOf(armingTime))
            .takeoffTimeUtc(BigInteger.valueOf(takeOffTime))
            .build()
    }

    private fun toMavlinkAltitudeMessage(): Altitude {
        return Altitude.builder()
            .timeUsec(BigInteger.valueOf((millisSinceBoot * 1e3).toLong()))
            .altitudeRelative(aircraftLocation?.altitude?.toFloat() ?: 0.0f)
            .build()
    }

    private fun toMavlinkGlobalPositionIntMessage(): GlobalPositionInt? {
        val location = aircraftLocation ?: return null
        val currentAttitude = attitude ?: return null
        // The velocity key is not pushed on some aircraft and has no simulator state
        // equivalent; without this fallback the message would never be sent and the
        // GCS would see neither position nor altitude
        val velocity = aircraftVelocity ?: Velocity3D()

        return GlobalPositionInt.builder()
            .timeBootMs(millisSinceBoot)
            .lat((location.latitude * 1e7).toInt())
            .lon((location.longitude * 1e7).toInt())
            .relativeAlt((location.altitude * 1e3).toInt())
            .vx((velocity.x * 1e2).toInt())
            .vy((velocity.y * 1e2).toInt())
            .vz((velocity.z * 1e2).toInt())
            .hdg((((currentAttitude.yaw * 1e2 + 360) % 360) * 1e2).toInt())
            .build()
    }

    private fun toMavlinkAttitudeMessage(): io.dronefleet.mavlink.common.Attitude? {
        val currentAttitude = attitude ?: return null

        return io.dronefleet.mavlink.common.Attitude.builder()
            .timeBootMs(millisSinceBoot)
            .roll(currentAttitude.roll.toRadians().toFloat())
            .pitch((currentAttitude.pitch).toRadians().toFloat())
            .yaw(currentAttitude.yaw.toRadians().toFloat())
            .build()
    }

    private fun toMavlinkSysStatusMessage(): SysStatus {
        var voltage = UINT16_MAX
        var current = -1
        var remaining = -1
        connectionWR.get()?.let { connection ->
            connection.mavlinkComponents?.batteries?.forEach {
                if (it.voltage < voltage) {
                    voltage = it.voltage
                    current = it.current
                    remaining = it.remainingPercent.takeIf { p -> p in 0..100 } ?: -1
                }
            }
        }

        // Evaluate at build time so the prearm health bit tracks the current state
        // (returns empty list, i.e. passed, when ARMING_CHECK is disabled)
        val prearmChecksPassed = runPrearmChecks().isEmpty()

        return SysStatus.builder()
            .onboardControlSensorsPresent(HealthManager.AVAILABLE_SENSORS)
            .onboardControlSensorsEnabled(healthManager.enabledSensors)
            .onboardControlSensorsHealth(healthManager.getHealthFlags(prearmChecksPassed))
            .voltageBattery(voltage)
            .currentBattery((current / 10.0f).roundToInt())
            .batteryRemaining(remaining)
            .build()
    }

    private fun toMavlinkExtendedSysStateMessage(): ExtendedSysState {
        return ExtendedSysState.builder()
            .vtolState(MavVtolState.MAV_VTOL_STATE_UNDEFINED)
            .landedState(mavLandedState)
            .build()
    }

    private fun toMavlinkAutopilotVersionMessage(): AutopilotVersion {
        val protocolFlags = mutableListOf(
            MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_MISSION_INT,
            MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_COMMAND_INT,
            MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE,
            MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_FTP,
            MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE,
            MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_COMPONENT_IMPLEMENTS_GIMBAL_MANAGER,
            MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_MAVLINK2,
        )

        val flightControllerVersion = KeyManager.getInstance().getValue(KeyTools.createKey(FlightControllerKey.KeyFirmwareVersion), "v05.01.01.00")

        return AutopilotVersion.builder()
            .capabilities(EnumUtils.listToEnum(protocolFlags))
            .flightSwVersion(toMavlinkVersion(flightControllerVersion))
            .middlewareSwVersion(0x040107)
            .osSwVersion(0x040107)
            .build()
    }

    private fun toMavlinkComponentInformationMessage(): ComponentInformation {
        val productType = KeyManager.getInstance().getValue(KeyTools.createKey(ProductKey.KeyProductType), ProductType.M300_RTK)
        val firmwareVersion = KeyManager.getInstance().getValue(KeyTools.createKey(ProductKey.KeyFirmwareVersion), "1.0.0")

        val componentInfoGenerator = ComponentInfoGenerator(connectionWR, componentID)
        componentInfoGenerator.writeParameterMetadataFile(ArrayList(mavlinkParameters.values))
        componentInfoGenerator.writeComponentInformation("DJI", productType.name, firmwareVersion, "1.0.0")

        return ComponentInformation.builder()
            .timeBootMs(millisSinceBoot)
            .generalMetadataFileCrc(componentInfoGenerator.componentMetadataCRC)
            .generalMetadataUri(componentInfoGenerator.componentMetadataURI)
            .build()
    }

    private fun toMavlinkComponentMetadataMessage(): ComponentMetadata {
        val productType = KeyManager.getInstance().getValue(KeyTools.createKey(ProductKey.KeyProductType), ProductType.M300_RTK)
        val firmwareVersion = KeyManager.getInstance().getValue(KeyTools.createKey(ProductKey.KeyFirmwareVersion), "1.0.0")

        val componentInfoGenerator = ComponentInfoGenerator(connectionWR, componentID)
        componentInfoGenerator.writeParameterMetadataFile(ArrayList(mavlinkParameters.values))
        componentInfoGenerator.writeComponentInformation("DJI", productType.name, firmwareVersion, "1.0.0")

        return ComponentMetadata.builder()
            .timeBootMs(millisSinceBoot)
            .fileCrc(componentInfoGenerator.parameterCRC)
            .uri(componentInfoGenerator.componentMetadataURI)
            .build()
    }

    private fun toMavlinkVfrHudMessage(): VfrHud {
        // DJI exposes no motor output telemetry, so the throttle is synthesized from
        // the flight state: GCS implementations use it to tell ground (0), armed idle
        // (below the hover threshold) and flying (hover level) apart
        val throttle = when {
            !motorsOn -> 0
            isFlying -> 50
            else -> 5
        }

        val heading = attitude?.let { ((it.yaw % 360.0) + 360.0) % 360.0 } ?: 0.0
        val groundSpeed = velocityHorizontalSpeed()
        val climbRate = aircraftVelocity?.z?.toDouble() ?: 0.0

        return VfrHud.builder()
            .airspeed(groundSpeed.toFloat())
            .groundspeed(groundSpeed.toFloat())
            .heading(heading.roundToInt())
            .throttle(throttle)
            // DJI altitudes are relative to the takeoff location, no AMSL available
            .alt(aircraftLocation?.altitude?.toFloat() ?: 0f)
            .climb((-climbRate).toFloat())
            .build()
    }

    private fun velocityHorizontalSpeed(): Double {
        val velocity = aircraftVelocity ?: return 0.0
        val x = velocity.x.toDouble()
        val y = velocity.y.toDouble()
        return sqrt(x * x + y * y)
    }

    private fun toMavlinkLocalPositionNedMessage(): LocalPositionNed? {
        val location = aircraftLocation ?: return null
        val home = rtlHomeLocation ?: return null
        val velocity = aircraftVelocity

        // DJI provides no local position, derive it from the GPS offset to the home
        // location (local tangent plane, good enough for the short ranges of a drone)
        val north = (location.latitude - home.latitude) * 111320.0
        val east = (location.longitude - home.longitude) * 111320.0 *
                cos(home.latitude.toRadians())

        return LocalPositionNed.builder()
            .timeBootMs(millisSinceBoot)
            .x(north.toFloat())
            .y(east.toFloat())
            .z((-location.altitude).toFloat())
            .vx((velocity?.x?.toDouble() ?: 0.0).toFloat())
            .vy((velocity?.y?.toDouble() ?: 0.0).toFloat())
            .vz((velocity?.z?.toDouble() ?: 0.0).toFloat())
            .build()
    }

    private fun toMavlinkHomePositionMessage(): HomePosition? {
        if (rtlHomeLocation == null) {
            return null
        }

        return HomePosition.builder()
            .timeUsec(BigInteger.valueOf(millisSinceBoot))
            .latitude((rtlHomeLocation!!.latitude * 1e7).toInt())
            .longitude((rtlHomeLocation!!.longitude * 1e7).toInt())
            .altitude(rtlHomeHeight)
            .build()
    }

    private fun toMavlinkGPSFixMessage(): GpsRawInt? {
        val gpsFixType = when (gpsSignalLevel) {
            GPSSignalLevel.LEVEL_0 -> GpsFixType.GPS_FIX_TYPE_NO_FIX
            GPSSignalLevel.LEVEL_1 -> GpsFixType.GPS_FIX_TYPE_NO_FIX
            GPSSignalLevel.LEVEL_2 -> GpsFixType.GPS_FIX_TYPE_2D_FIX
            GPSSignalLevel.LEVEL_3 -> GpsFixType.GPS_FIX_TYPE_3D_FIX
            GPSSignalLevel.LEVEL_4 -> GpsFixType.GPS_FIX_TYPE_3D_FIX
            GPSSignalLevel.LEVEL_5 -> GpsFixType.GPS_FIX_TYPE_3D_FIX
            GPSSignalLevel.LEVEL_10 -> GpsFixType.GPS_FIX_TYPE_RTK_FIXED
            else -> GpsFixType.GPS_FIX_TYPE_NO_GPS
        }

        val location = aircraftLocation ?: return null
        val velocity = aircraftVelocity ?: Velocity3D()

        return GpsRawInt.builder()
            .timeUsec(BigInteger.valueOf(millisSinceBoot / 10))
            .fixType(gpsFixType)
            .lat((location.latitude * 1e7).roundToInt())
            .lon((location.longitude * 1e7).roundToInt())
            .alt(location.altitude.roundToInt())
            .eph(UINT16_MAX)
            .epv(UINT16_MAX)
            .vel((velocity.x * 100).roundToInt())
            .cog(UINT16_MAX)
            .satellitesVisible(gpsSatelliteCount)
            .build()
    }

    fun enableOnboardCamera(enable: Boolean) {
        if (enable && onboardCamera == null) {
            log.i("Start onboard camera!")
            onboardCamera = OnBoardCamera(connectionWR, WeakReference(this))
            onboardCamera?.create()
        } else if (!enable && onboardCamera != null) {
            log.i("Stop onboard camera!")
            onboardCamera?.destroy()
            onboardCamera = null
        }
    }

    // endregion
    //region: functions
    fun setDisArmAfterLandTimeout(timeout: Float) {
        disArmAfterLandTimeout = timeout

        log.i("Update auto disarm after land timeout to ${disArmAfterLandTimeout}s")
    }

    val isHomePositionSet: Boolean
        get() = rtlHomeLocation != null

    val isLocationAvailable: Boolean
        get() = aircraftLocation != null

    //endregion
    //region: utils
    private fun toMavlinkVersion(djiVersion: String?): Long {
        if (djiVersion == null) {
            log.i("Cannot parse DJI version because its null")
            return 0
        }

        val majorMinorPatch = djiVersion.substring(1).split("\\.".toRegex()).dropLastWhile { it.isEmpty() }.toTypedArray()

        if (majorMinorPatch.size != 4) {
            log.i("Cannot parse DJI version")
            return 0
        }

        val versionString = majorMinorPatch[0] + majorMinorPatch[1] + majorMinorPatch[2] + majorMinorPatch[3]

        return versionString.toLong()
    }

    companion object {
        // SET_POSITION_TARGET_LOCAL_NED type mask bits 3-5 (velocity x/y/z ignored)
        private const val VELOCITY_IGNORED_MASK = (1 shl 3) or (1 shl 4) or (1 shl 5)

        // Landing confirmation watch window: 30 s in 500 ms steps
        private const val LANDING_CONFIRMATION_POLL_COUNT = 60L

        // StartTakeoff retry: 3 attempts, 500 ms apart, covers FC reconnection races
        private const val TAKEOFF_RETRY_INTERVAL_MS = 500L
        private const val TAKEOFF_RETRY_COUNT = 3
    }
}
