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
import io.dronefleet.mavlink.common.DistanceSensor
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
import io.dronefleet.mavlink.common.MavDistanceSensor
import io.dronefleet.mavlink.common.MavProtocolCapability
import io.dronefleet.mavlink.common.MavSensorOrientation
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

import nl.wur.mavdrone.MavManager.MavlinkManagerCommunicationModel.GuidedModeUpdated
import nl.wur.mavdrone.MavManager.MavlinkManagerCommunicationModel.SetGuidedMode
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
import nl.wur.mavdrone.communication.utils.LocalPositionFilter
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
    val virtualStickManager: VirtualStickManager
    private var onboardCamera: OnBoardCamera? = null

    private var attitude: Attitude? = null

    var aircraftLocation: LocationCoordinate3D? = null
        private set
    private var aircraftVelocity: Velocity3D? = null

    private var gpsSatelliteCount = 0
    private var gpsSignalLevel: GPSSignalLevel? = null

    // 下视超声/TOF 测距 (厘米) 与有效性。DJI 气压相对高度近地会漂移且被落地
    // 保护冻结, 地面站触地/盲降判决需要真实离地高, 经 DISTANCE_SENSOR 转发
    private var ultrasonicHeightCm: Int? = null
    private var ultrasonicUsed = false

    // 起飞点海拔 (AMSL, DJI 注明不精确)。SDK 无实时 AMSL 接口, 用它加相对
    // 高度合成 GLOBAL_POSITION_INT.alt, 避免下游把 0 米当真
    private var takeoffLocationAltitude: Double? = null

    // 下降冻结检测 (近地保护): 固件在 ~0.6-0.8 m 冻结一切下降指令且不请求
    // 降落确认 (bag 实测: 无人干预则一直悬停, 手动点降落才结束), GCS 从遥测
    // 无法分辨。记录最新 offboard 设定点的 vz 用于判定"持续下令下降但实际不动"
    private var offboardDownVz = 0.0
    private var offboardSetpointMillis = 0L
    private var descentBlockedTicks = 0
    private var descentBlockedAnnounced = false

    private var rtlHomeLocation: LocationCoordinate2D? = null
    private var rtlHomeHeight = 0

    private var motorsOn = false
    var isFlying: Boolean = false
        private set

    var flightMode: FlightMode = FlightMode.UNKNOWN
        private set

    // GUIDED 是本应用"地面站接管"的权威状态: 虚拟摇杆只在 guided 下准入。
    // 进入: GUIDED 按钮 / GCS DO_SET_MODE(offboard|guided) / NAV_TAKEOFF 带爬升目标
    // 退出: 再次点击按钮(回 PosHold) / 飞行模式回到手动系(MANUAL/ATTI/GPS_NORMAL 等,
    // 覆盖起飞结束与 RC 切档) / RC 夺权(切档/急停/一键返航, 带锁存) / 落地
    // 注意: offboard setpoint 超时自动释放不退出 guided —— GCS 可能只是暂停续流,
    // 此时退出会造成恢复流指令被误拦
    @Volatile
    private var guidedActive = false

    // RC 夺权锁存: 夺权后禁止任何非显式路径重新进入 guided, 防止 GO_HOME 等
    // 自动模式分支在事件乱序下把 guided 拉回去; 只有点击 GUIDED 按钮或 GCS
    // 显式 DO_SET_MODE 才复位
    @Volatile
    private var guidedSuppressedByRC = false

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

    // Smooths the GPS-derived local position with the DJI fused velocity (per-axis
    // Kalman); home-reset detection invalidates it because the north/east frame moves
    private val localPositionFilter = LocalPositionFilter()
    private var filterHomeLatitude = Double.NaN
    private var filterHomeLongitude = Double.NaN


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

        watchLandingConfirmation()

        // GUIDED 按钮(App 本地)请求经 MavManager 服务总线转发到这里
        connectionWR.get()?.let { connection ->
            compositeDisposable.add(
                connection.mavlinkThreadCommunicationSubject
                    .ofType(SetGuidedMode::class.java)
                    .observeOn(Schedulers.newThread())
                    .subscribeOn(Schedulers.newThread())
                    .subscribe(
                        { model -> setGuidedFromButton(model.enabled) },
                        { error -> RXUtils.handleError(error) }
                    )
            )
        }

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
                    exitGuided("manual mode")
                }

                FlightMode.ATTI -> {
                    addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_STABILIZE_ENABLED)
                    removeMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                    setCustomMode(MAIN_MODE.PX4_CUSTOM_MAIN_MODE_ALTCTL)
                    exitGuided("attitude mode")
                }

                FlightMode.GPS_NORMAL, FlightMode.GPS_TRIPOD, FlightMode.GPS_SPORT, FlightMode.TAKE_OFF_READY -> {
                    addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_STABILIZE_ENABLED)
                    setCustomMode(MAIN_MODE.PX4_CUSTOM_MAIN_MODE_POSCTL)
                    // 覆盖两个场景: 起飞(含爬升)结束回到位置模式, 以及 RC 切档夺权
                    exitGuided("position mode")
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
                    if (ov && !nv) {
                        // 落地即退出 guided(静默, 仅同步按钮/状态), 避免降落完成后
                        // 按钮仍显示接管中
                        exitGuided("landed")
                    }

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
        addDJIListener(KeyTools.createKey(FlightControllerKey.KeyTakeoffLocationAltitude)) { value -> takeoffLocationAltitude = value }

        // 超声键在部分机型上不主动推送 (README 已知问题), 轮询兜底
        watchUltrasonicHeight()
        watchDescentBlocked()

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
        addMavlinkMessageSender(DistanceSensor::class.java, { this.toMavlinkDistanceSensorMessage() }, 200, true)
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
        exitGuided("component destroyed")
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
                                    climbTarget?.let {
                                        // 爬升由虚拟摇杆完成, 这里先行进入 guided —— 若等
                                        // AUTO_TAKE_OFF 模式推送再进, 起飞指令回调可能先到,
                                        // 爬升请求会被 guided 门控误拦
                                        enterGuided("GCS takeoff")
                                        virtualStickManager.climbToHeight(it)
                                    }
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
                    // Gimbal commands are owned by the gimbal component. When they
                    // arrive broadcast (targetComponent=0), answering with an
                    // UNSUPPORTED ack races against the gimbal's ACCEPTED ack.
                    if (payload.command().entry() in GIMBAL_OWNED_COMMANDS) {
                        return false
                    }

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

        offboardDownVz = payload.vz().toDouble()
        offboardSetpointMillis = System.currentTimeMillis()

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

    // 落地保护: 固件在下视系统检测到地面 (~0.5-1 m, 机型相关) 时会暂停
    // 下降并请求"滑动确认降落"——auto-land 和虚拟摇杆速度下降都会触发,
    // 保护高度以下下降指令会被固件忽略。常驻监视并自动确认, 放行到地面
    private fun watchLandingConfirmation() {
        log.i("Landing confirmation watcher started")
        compositeDisposable.add(
            Observable.interval(500, TimeUnit.MILLISECONDS)
                .flatMapSingle {
                    // djiGetValue 无超时, 键读取挂起会让链路无声零输出, 变成
                    // "永远 false"的假象; 超时转为错误后由 retry 重启轮询
                    djiGetValue(FlightControllerKey.KeyIsLandingConfirmationNeeded)
                        .timeout(1, TimeUnit.SECONDS)
                }
                .distinctUntilChanged()
                .doOnNext { needed ->
                    // 记录双向变化, 会话日志里能直接读出该键的完整轨迹
                    if (!needed) log.i("Landing confirmation cleared")
                }
                .flatMapCompletable { needed ->
                    if (needed) {
                        log.i("Landing confirmation needed, confirming automatically")
                        djiPerformAction(FlightControllerKey.KeyConfirmLanding)
                    } else {
                        Completable.complete()
                    }
                }
                .retry()
                .subscribe(
                    {},
                    { error -> RXUtils.handleError(error) }
                )
        )
    }

    // 下视测距轮询。超声键在部分机型不推送 (README 已知注意事项 2), 用与
    // watchLandingConfirmation 相同的 interval+读键模式兜底; 读键偶发失败
    // 时 retry() 重订整个轮询链, 不让单次错误永久杀掉数据源
    private fun watchUltrasonicHeight() {
        compositeDisposable.add(
            Observable.interval(200, TimeUnit.MILLISECONDS)
                .flatMapSingle { djiGetValue(FlightControllerKey.KeyIsUltrasonicUsed) }
                .retry()
                .subscribe(
                    { ultrasonicUsed = it },
                    { }
                )
        )
        compositeDisposable.add(
            Observable.interval(200, TimeUnit.MILLISECONDS)
                .flatMapSingle { djiGetValue(FlightControllerKey.KeyUltrasonicHeight) }
                .retry()
                .subscribe(
                    { ultrasonicHeightCm = it },
                    { }
                )
        )
    }

    // 下降冻结监视。近地保护冻结期间固件不置位 KeyIsLandingConfirmationNeeded
    // (会话日志零条确认记录), GCS 只能从遥测看到"高度不动"; 这里把判定结果
    // 主动做成 STATUSTEXT 告知, 提示地面站此时应切换固件 LAND 而不是继续
    // 发下降指令。只告警不接管控制——切 LAND 的决策权在地面站
    private fun watchDescentBlocked() {
        compositeDisposable.add(
            Observable.interval(500, TimeUnit.MILLISECONDS)
                .subscribe(
                    {
                        val altitude = aircraftLocation?.altitude ?: return@subscribe
                        val setpointFresh = System.currentTimeMillis() - offboardSetpointMillis < 1000
                        // NED vz 下正; DJI 垂直速度上正且量化到 0.1 m/s
                        val blocked = isFlying && setpointFresh &&
                                offboardDownVz >= 0.15 &&
                                Math.abs(aircraftVelocity?.z?.toDouble() ?: 0.0) <= 0.05 &&
                                altitude <= 2.0
                        if (blocked) {
                            descentBlockedTicks++
                            if (descentBlockedTicks == 6 && !descentBlockedAnnounced) {
                                descentBlockedAnnounced = true
                                log.i("Descent blocked by landing protection at $altitude m")
                                sendMessage(
                                    Statustext.builder()
                                        .severity(MavSeverity.MAV_SEVERITY_WARNING)
                                        .text(
                                            MavlinkTexts.truncateStatustext(
                                                "Landing protection: descent blocked at %.1f m, firmware ignores descent commands; switch to LAND".format(
                                                    altitude
                                                )
                                            )
                                        )
                                        .build()
                                )
                            }
                        } else {
                            descentBlockedTicks = 0
                            descentBlockedAnnounced = false
                        }
                    },
                    { }
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

    fun isGuidedActive(): Boolean = guidedActive

    fun enterGuided(source: String) {
        guidedSuppressedByRC = false
        if (guidedActive) {
            return
        }

        guidedActive = true
        addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
        setCustomMode(MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        setCustomSubMode(SUB_MODE.INVALID)
        log.i("Guided mode entered ($source)")
        emitGuidedState()
    }

    // 释放虚拟摇杆(若有), 清 guided 状态与标志并同步 UI; 自定义模式由调用方
    // 决定: 按钮退出回 PosHold, 飞行模式分支沿用各自的映射
    fun exitGuided(source: String, notifyGCS: Boolean = false) {
        if (!guidedActive) {
            return
        }

        guidedActive = false
        removeMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
        virtualStickManager.stopAll()
        log.i("Guided mode exited ($source)")

        if (notifyGCS) {
            sendMessage(
                Statustext.builder()
                    .severity(MavSeverity.MAV_SEVERITY_WARNING)
                    .text(MavlinkTexts.truncateStatustext("Guided interrupted: RC took control"))
                    .build()
            )
        }
        emitGuidedState()
    }

    // 虚拟摇杆组件回调: RC 通过切档/急停/一键返航/低电量行为等夺走控制权
    // (FlightControlAuthorityChangeReason 除 MSDK_REQUEST/UNKNOWN 之外的全部取值)
    fun onControlAuthorityTakenByRC(reason: String) {
        guidedSuppressedByRC = true
        if (guidedActive) {
            exitGuided("RC takeover ($reason)", notifyGCS = true)
        } else {
            log.i("Control authority taken by RC ($reason), guided latch armed")
        }
    }

    private fun setGuidedFromButton(enabled: Boolean) {
        if (enabled) {
            enterGuided("GUIDED button")
        } else {
            exitGuided("GUIDED button")
            setCustomMode(MAIN_MODE.PX4_CUSTOM_MAIN_MODE_POSCTL)
            setCustomSubMode(SUB_MODE.INVALID)
        }
    }

    private fun emitGuidedState() {
        connectionWR.get()?.mavlinkThreadCommunicationSubject?.onNext(GuidedModeUpdated(guidedActive))
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
                enterGuided("GCS set mode")
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

    // DJI 无空中停桨接口: KeyLockMotors 是未起飞状态下的电机锁 (锁定后掰杆
    // 无法启动电机, 防盗/运输用途), 空中调用只会触发"锁定飞机, 要求更新固件"
    // (2026-10-10 真机实测, 电机不停)。因此 MAVLink 强制停桨魔数
    // (param2=21196) 不做特殊处理, 与普通 disarm 相同: 飞行中拒绝, 地面照常
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
            // KeyAircraftLocation3D 的高度是相对起飞点高度 (DJI 官方文档), alt
            // 字段要求 AMSL: 用起飞点海拔近似合成, 起飞点海拔未知时保持 0
            .alt(amslAltitudeMm(location.altitude) ?: 0)
            .vx((velocity.x * 1e2).toInt())
            .vy((velocity.y * 1e2).toInt())
            .vz((velocity.z * 1e2).toInt())
            .hdg((((currentAttitude.yaw * 1e2 + 360) % 360) * 1e2).toInt())
            .build()
    }

    // DJI SDK 没有实时 AMSL 接口 (KeyAircraftLocation3D 为相对起飞点高度),
    // 用起飞点海拔 (KeyTakeoffLocationAltitude, 官方注明不精确) 加相对高度近似
    private fun amslAltitudeMm(relativeAltMeters: Double): Int? {
        return takeoffLocationAltitude?.let { ((it + relativeAltMeters) * 1e3).toInt() }
    }

    // 下视超声/TOF 真实离地高。气压相对高度近地漂移且会被落地保护冻结读数
    // (bag 实测悬停 50 s 死钉 0.8 m), 地面站的触地/盲降判决依赖本消息
    // (MAVLink id 132, dankong 侧已在请求该消息间隔并接入 pland rangefinder 链路)
    private fun toMavlinkDistanceSensorMessage(): DistanceSensor? {
        val heightCm = ultrasonicHeightCm ?: return null
        return DistanceSensor.builder()
            .timeBootMs(millisSinceBoot)
            .minDistance(10)
            .maxDistance(1000)
            .currentDistance(heightCm.coerceIn(0, 65535))
            .type(MavDistanceSensor.MAV_DISTANCE_SENSOR_LASER)
            .id(0)
            .orientation(MavSensorOrientation.MAV_SENSOR_ROTATION_PITCH_270)
            .covariance(65535)
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
        val rawNorth = (location.latitude - home.latitude) * 111320.0
        val rawEast = (location.longitude - home.longitude) * 111320.0 *
                cos(home.latitude.toRadians())

        // The home point defines the local frame origin; when it moves (e.g. the
        // simulator re-anchors it) the filter state is invalid and must re-lock
        if (home.latitude != filterHomeLatitude || home.longitude != filterHomeLongitude) {
            filterHomeLatitude = home.latitude
            filterHomeLongitude = home.longitude
            localPositionFilter.reset(rawNorth, rawEast)
        }

        val hasVelocity = velocity != null
        val filtered = localPositionFilter.update(
            rawNorth, rawEast,
            velocity?.x?.toDouble() ?: 0.0,
            velocity?.y?.toDouble() ?: 0.0,
            hasVelocity
        )

        return LocalPositionNed.builder()
            .timeBootMs(millisSinceBoot)
            .x(filtered.first.toFloat())
            .y(filtered.second.toFloat())
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
            // GPS_RAW_INT.alt 规格为 AMSL 毫米, 相对高度在此语义错误; 无起飞点
            // 海拔时退回旧行为 (相对高度)
            .alt(amslAltitudeMm(location.altitude) ?: location.altitude.roundToInt())
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

        // StartTakeoff retry: 3 attempts, 500 ms apart, covers FC reconnection races
        private const val TAKEOFF_RETRY_INTERVAL_MS = 500L
        private const val TAKEOFF_RETRY_COUNT = 3

        // Commands handled by the gimbal component (Gimbal.kt); the autopilot
        // must stay silent on them to avoid competing command acks
        private val GIMBAL_OWNED_COMMANDS = setOf(
            MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
            MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            MavCmd.MAV_CMD_DO_MOUNT_CONTROL
        )
    }
}
