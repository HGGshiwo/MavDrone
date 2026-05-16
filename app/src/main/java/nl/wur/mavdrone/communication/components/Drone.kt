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
import dji.v5.manager.aircraft.waypoint3.model.WaypointMissionExecuteState

import io.reactivex.rxjava3.core.Completable
import io.reactivex.rxjava3.core.Observable
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
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.MavFrame
import io.dronefleet.mavlink.common.MavGoto
import io.dronefleet.mavlink.common.MavLandedState
import io.dronefleet.mavlink.common.MavProtocolCapability
import io.dronefleet.mavlink.common.MavVtolState
import io.dronefleet.mavlink.common.SysStatus
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
import nl.wur.mavdrone.utils.RXUtils
import nl.wur.mavdrone.utils.RXUtils.Companion.djiGetValue
import nl.wur.mavdrone.utils.RXUtils.Companion.djiPerformAction
import nl.wur.mavdrone.utils.RXUtils.Companion.djiSetValue
import nl.wur.mavdrone.utils.RxException.DroneNotArmedException
import nl.wur.mavdrone.utils.RxException.NoMissionUploadedException
import nl.wur.mavdrone.utils.toRadians
import kotlin.math.roundToInt

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

    var productType: ProductType = ProductType.UNKNOWN
        private set
    private var customSubMode = SUB_MODE.PX4_CUSTOM_SUB_MODE_AUTO_READY

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
                    }
                }, { error -> RXUtils.handleError(error) })
        )
    }

    override fun create() {
        motorsOn = false
        rtlHomeHeight = 50
        isFlying = false

        compositeDisposable.add(djiGetValue(ProductKey.KeyProductType).subscribe { p: ProductType -> productType = p })

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
                }

                FlightMode.GPS_NORMAL, FlightMode.GPS_TRIPOD, FlightMode.GPS_SPORT -> addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_STABILIZE_ENABLED)
                FlightMode.MOTOR_START -> {
                    addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_SAFETY_ARMED)

                    armingTime = System.currentTimeMillis() * 1000
                }

                FlightMode.POI, FlightMode.TAKE_OFF_READY, FlightMode.VIRTUAL_STICK, FlightMode.PANO, FlightMode.SMART_FLIGHT, FlightMode.SMART_FLY, FlightMode.ATTI_LANDING, FlightMode.CLICK_GO, FlightMode.CINEMATIC, FlightMode.DRAW, FlightMode.GPS_NOVICE, FlightMode.QUICK_MOVIE, FlightMode.TAP_FLY, FlightMode.MASTER_SHOT, FlightMode.APAS, FlightMode.TIME_LAPSE, FlightMode.UNKNOWN -> {}
                FlightMode.AUTO_TAKE_OFF -> {
                    addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                    setCustomSubMode(SUB_MODE.PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF)

                    mavLandedState = MavLandedState.MAV_LANDED_STATE_TAKEOFF
                    takeOffTime = System.currentTimeMillis() * 1000
                }

                FlightMode.AUTO_LANDING -> {
                    addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                    setCustomSubMode(SUB_MODE.PX4_CUSTOM_SUB_MODE_AUTO_LAND)

                    mavLandedState = MavLandedState.MAV_LANDED_STATE_LANDING
                }

                FlightMode.FORCE_LANDING -> {
                    removeMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                    setCustomSubMode(SUB_MODE.PX4_CUSTOM_SUB_MODE_AUTO_LAND)

                    mavLandedState = MavLandedState.MAV_LANDED_STATE_LANDING
                }

                FlightMode.GO_HOME -> {
                    addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                    setCustomSubMode(SUB_MODE.PX4_CUSTOM_SUB_MODE_AUTO_RTL)
                }

                FlightMode.WAYPOINT -> {
                    addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                    setCustomMode(MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO)
                    setCustomSubMode(SUB_MODE.PX4_CUSTOM_SUB_MODE_AUTO_MISSION)
                }

                FlightMode.AUTO_AVOIDANCE -> addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                FlightMode.FOLLOW_ME -> {
                    addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
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
        addMavlinkMessageSender(FlightInformation::class.java) { this.toMavlinkFlightInformationMessage() }

        // Add message listeners
        addMavlinkListener(CommandCancel::class.java) { msg -> this.onCommandCancel(msg) }

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
                    if (!containsMavModeFlag(MavModeFlag.MAV_MODE_FLAG_STABILIZE_ENABLED)) {
                        log.w("Cannot takeoff because remote controller is not in a stabilized mode!")
                        sendMessage(commandDenied(MavCmd.MAV_CMD_NAV_TAKEOFF, msg.originSystemId, msg.originComponentId))
                    } else if (!containsMavModeFlag(MavModeFlag.MAV_MODE_FLAG_SAFETY_ARMED)) {
                        log.w("Cannot takeoff because the drone is not armed!")
                        sendMessage(commandDenied(MavCmd.MAV_CMD_NAV_TAKEOFF, msg.originSystemId, msg.originComponentId))
                    } else {
                        compositeDisposable.add(
                            djiPerformAction(FlightControllerKey.KeyStartTakeoff)
                                .subscribe(
                                    {
                                        sendMessage(commandAccepted(MavCmd.MAV_CMD_NAV_TAKEOFF, msg.originSystemId, msg.originComponentId))
                                        takeOffTime = System.currentTimeMillis() * 1000
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
                    }
                    return true
                }

                MavCmd.MAV_CMD_NAV_RETURN_TO_LAUNCH -> {
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

                else -> {
                    log.i("Unsupported command: " + payload.command().entry())
                    sendMessage(commandUnsupported(payload.command().entry(), msg.originSystemId, msg.originComponentId))
                    return false
                }
            }
        }

        return false
    }

    private fun onCommandCancel(msg: MavlinkMessage<*>) {
        val payload = msg.payload as CommandCancel

        if (payload.targetSystem() == systemID && (payload.targetComponent() == EnumValue.create(componentID).value() || payload.targetComponent() == 0)) {
            when (payload.command().entry()) {
                MavCmd.MAV_CMD_NAV_TAKEOFF -> compositeDisposable.add(
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

    private fun changeFlightMode(mavMode: Int, customMode: Int, subMode: Int, systemId: Int, componentId: Int) {
        // Set the Mavlink modes
        val mavModeFlagList = enumToList(EnumValue.create(MavModeFlag::class.java, mavMode), MavModeFlag::class.java)
        mavModeFlagList.forEach { addMavModeFlag(it) }
        log.i("Changed mode. Current flags: ${getMavModeFlags()}")

        // MAVSDK may send custom PX4 modes
        if (containsMavModeFlag(MavModeFlag.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) {
            val customFlightMode = MAIN_MODE.fromValue(customMode)
            val customSubFlightMode = SUB_MODE.fromValue(subMode)

            log.i("Set mode to $customFlightMode-$customSubFlightMode")

            if (customFlightMode == MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD) {
                setCustomMode(customFlightMode)
                setCustomSubMode(customSubFlightMode)
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

    private fun toMavlinkGlobalPositionIntMessage(): GlobalPositionInt {
        return GlobalPositionInt.builder()
            .timeBootMs(millisSinceBoot)
            .lat((aircraftLocation!!.latitude * 1e7).toInt())
            .lon((aircraftLocation!!.longitude * 1e7).toInt())
            .relativeAlt((aircraftLocation!!.altitude * 1e3).toInt())
            .vx((aircraftVelocity!!.x * 1e2).toInt())
            .vy((aircraftVelocity!!.y * 1e2).toInt())
            .vz((aircraftVelocity!!.z * 1e2).toInt())
            .hdg((((attitude!!.yaw * 1e2 + 360) % 360) * 1e2).toInt())
            .build()
    }

    private fun toMavlinkAttitudeMessage(): io.dronefleet.mavlink.common.Attitude {
        return io.dronefleet.mavlink.common.Attitude.builder()
            .timeBootMs(millisSinceBoot)
            .roll(attitude!!.roll.toRadians().toFloat())
            .pitch((attitude!!.pitch).toRadians().toFloat())
            .yaw((attitude!!.yaw).toRadians().toFloat())
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
                    remaining = it.remainingPercent
                }
            }
        }

        return SysStatus.builder()
            .onboardControlSensorsPresent(HealthManager.AVAILABLE_SENSORS)
            .onboardControlSensorsEnabled(healthManager.enabledSensors)
            .onboardControlSensorsHealth(healthManager.healthySensors)
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

    private fun toMavlinkGPSFixMessage(): GpsRawInt {
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

        return GpsRawInt.builder()
            .timeUsec(BigInteger.valueOf(millisSinceBoot / 10))
            .fixType(gpsFixType)
            .lat((aircraftLocation!!.latitude * 1e7).roundToInt())
            .lon((aircraftLocation!!.longitude * 1e7).roundToInt())
            .alt(aircraftLocation!!.altitude.roundToInt())
            .eph(UINT16_MAX)
            .epv(UINT16_MAX)
            .vel((aircraftVelocity!!.x * 100).roundToInt())
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

    fun setCustomSubMode(subMode: SUB_MODE) {
        customSubMode = subMode
    }

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
}
