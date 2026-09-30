package nl.wur.mavdrone.communication.components

import java.lang.ref.WeakReference
import java.math.BigInteger
import kotlin.math.roundToInt

import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.GimbalKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.Attitude
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotation
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotationMode
import dji.sdk.keyvalue.value.gimbal.GimbalMode
import dji.sdk.keyvalue.value.gimbal.GimbalResetType
import dji.v5.manager.KeyManager

import io.reactivex.rxjava3.core.Completable

import io.dronefleet.mavlink.MavlinkMessage
import io.dronefleet.mavlink.common.CommandLong
import io.dronefleet.mavlink.common.GimbalDeviceAttitudeStatus
import io.dronefleet.mavlink.common.GimbalDeviceCapFlags
import io.dronefleet.mavlink.common.GimbalDeviceFlags
import io.dronefleet.mavlink.common.GimbalDeviceInformation
import io.dronefleet.mavlink.common.GimbalManagerCapFlags
import io.dronefleet.mavlink.common.GimbalManagerFlags
import io.dronefleet.mavlink.common.GimbalManagerInformation
import io.dronefleet.mavlink.common.GimbalManagerSetAttitude
import io.dronefleet.mavlink.common.GimbalManagerSetPitchyaw
import io.dronefleet.mavlink.common.GimbalManagerStatus
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.MavMountMode
import io.dronefleet.mavlink.standard.MavAutopilot
import io.dronefleet.mavlink.standard.MavComponent
import io.dronefleet.mavlink.standard.MavType
import io.dronefleet.mavlink.util.EnumValue

import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.utils.EnumUtils.Companion.enumToList
import nl.wur.mavdrone.communication.utils.EnumUtils.Companion.listToEnum
import nl.wur.mavdrone.communication.utils.StandardResponses
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandAccepted
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandDenied
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandFailed
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandUnsupported
import nl.wur.mavdrone.utils.Quaternion.Companion.fromEulerDegrees
import nl.wur.mavdrone.utils.Quaternion.Companion.fromList
import nl.wur.mavdrone.utils.RXUtils
import nl.wur.mavdrone.utils.RXUtils.Companion.djiSetValue
import nl.wur.mavdrone.utils.RXUtils.Companion.djiPerformAction
import nl.wur.mavdrone.utils.toDegrees
import nl.wur.mavdrone.utils.toRadians

@Suppress("deprecation")
class Gimbal(connectionWR: WeakReference<MavConnection>, componentId: MavComponent, private val componentIndex: ComponentIndexType) : MavBaseComponent(
    connectionWR,
    KeyTools.createKey(GimbalKey.KeyConnection, componentIndex),
    componentId,
    MavType.MAV_TYPE_GIMBAL,
    MavAutopilot.MAV_AUTOPILOT_INVALID,
    "MavGimbal" + componentIndex.ordinal,
    true
) {
    private var primaryControlSysID = -1
    private var primaryControlCompID = -1
    private var secondaryControlSysID = -1
    private var secondaryControlCompID = -1

    private val gimbalManagerAppliedFlags: MutableList<GimbalManagerFlags> = ArrayList()

    private val gimbalDeviceId = COMPONENT_TO_GIMBAL_ID[componentId] ?: -1
    private var gimbalMode: GimbalMode? = null
    private var gimbalAttitudeFallbackTs = 0L
    private var warnedNoGimbalAttitude = false
    private var loggedPolledAttitude = false
    private var gimbalAttitude: Attitude? = null
    var gimbalYawAircraftHeading = 0.0

    var vendorName = ""
        private set
    var modelName = ""
        private set
    var firmwareVersion = 0L
        private set
    var rollMin = Float.NaN
        private set
    var rollMax = Float.NaN
        private set
    var pitchMin = Float.NaN
        private set
    var pitchMax = Float.NaN
        private set
    var yawMin = Float.NaN
        private set
    var yawMax = Float.NaN
        private set

    init {
        connectToProduct()
        gatherProductInfo()
    }

    override fun create() {
        // Create DJI listeners
        addDJIListener(KeyTools.createKey(GimbalKey.KeyGimbalMode, componentIndex)) { value -> gimbalMode = value }
        addDJIListener(KeyTools.createKey(GimbalKey.KeyGimbalAttitude, componentIndex)) { value -> gimbalAttitude = value }
        addDJIListener(KeyTools.createKey(GimbalKey.KeyYawRelativeToAircraftHeading, componentIndex)) { value -> gimbalYawAircraftHeading = value }

        // Add message senders
        addMavlinkMessageSender(GimbalManagerInformation::class.java) { this.toGimbalManagerInformationMessage() }
        addMavlinkMessageSender(GimbalDeviceInformation::class.java){ this.toGimbalDeviceInformationMessage() }
        addMavlinkMessageSender(GimbalManagerStatus::class.java, {this.toGimbalManagerStatusMessage() }, 200, true)
        addMavlinkMessageSender(GimbalDeviceAttitudeStatus::class.java, { this.toGimbalDeviceAttitudeStatus() }, 100, true)

        // Add message listeners
        addMavlinkListener(GimbalManagerSetPitchyaw::class.java, this::onGimbalManagerSetPitchYaw)
        addMavlinkListener(GimbalManagerSetAttitude::class.java, this::onGimbalManagerSetAttitude)
    }

    public override fun onCommandLong(msg: MavlinkMessage<*>): Boolean {
        if (super.onCommandLong(msg)) {
            return true
        }

        val payload = msg.payload as CommandLong

        if (isCommandForMe(payload)) {
            log.i("Command: ${payload.command().entry()}")

            when (payload.command().entry()) {
                MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE -> {
                    if (payload.param7().roundToInt() == gimbalDeviceId || payload.param7().roundToInt() == 0
                    ) {
                        primaryControlSysID = payload.param1().roundToInt()
                        primaryControlCompID = payload.param2().roundToInt()
                        secondaryControlSysID = payload.param3().roundToInt()
                        secondaryControlCompID = payload.param4().roundToInt()

                        log.i("Updated primary control to sysID=$primaryControlSysID, compID=$primaryControlCompID")
                        sendMessage(
                            commandAccepted(
                                MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                                msg.originSystemId,
                                msg.originComponentId
                            )
                        )
                        return true
                    }

                    sendMessage(
                        commandFailed(
                            MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                            msg.originSystemId,
                            msg.originComponentId
                        )
                    )
                    return true
                }

                MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW -> {
                    if (payload.param7().roundToInt() == gimbalDeviceId || payload.param7().roundToInt() == 0) {
                        if (!isAllowedToControl(msg.originSystemId, msg.originComponentId)) {
                            log.w("Refuse gimbal control from sysID=" + msg.originSystemId + ", compID=" + msg.originComponentId)
                            sendMessage(
                                commandDenied(
                                    MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                                    msg.originSystemId,
                                    msg.originComponentId
                                )
                            )
                            return true
                        }

                        val gimbalManagerFlags = enumToList(payload.param5().roundToInt(),GimbalManagerFlags::class.java)

                        if (gimbalManagerFlags.contains(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_NEUTRAL)) {
                            compositeDisposable.add(down()
                                    .subscribe(
                                        {
                                            sendMessage(
                                                commandAccepted(
                                                    MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                                                    msg.originSystemId,
                                                    msg.originComponentId
                                                )
                                            )
                                        },
                                        { error ->
                                            onError(
                                                error,
                                                StandardResponses::commandFailed,
                                                MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                                                msg.originSystemId,
                                                msg.originComponentId
                                            )
                                        }
                                    )
                            )

                            return true
                        }

                        if (gimbalManagerFlags.contains(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_RETRACT)) {
                            compositeDisposable.add(reset()
                                    .subscribe(
                                        {
                                            sendMessage(
                                                commandAccepted(
                                                    MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                                                    msg.originSystemId,
                                                    msg.originComponentId
                                                )
                                            )
                                        },
                                        { error ->
                                            onError(
                                                error,
                                                StandardResponses::commandFailed,
                                                MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                                                msg.originSystemId,
                                                msg.originComponentId
                                            )
                                        }
                                    )
                            )

                            return true
                        }

                        val pitch = payload.param1()
                        var yaw = payload.param2()

                        if (gimbalManagerFlags.contains(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME)) {
                            yaw = (yaw - gimbalYawAircraftHeading).toFloat()
                        }

                        // 世界系绝对角 (param1 = 相对大地的 pitch, DJI 符号
                        // 负值向下)。ABSOLUTE_ANGLE 的 0 度参考是指令时刻
                        // 的机体姿态, 换算成机体相对指令后一次性执行;
                        // DJI 稳定器随后按世界系保持, 无需后续指令。
                        // 换算后必须按机体可控范围 clamp (飞机抬头时世界
                        // 目标会越界), 保证最接近目标的可达角度, 不被拒收
                        val pitchNow = aircraftPitchDji()
                        val pitchBody = (pitch.toDouble() - pitchNow)
                            .coerceIn(BODY_PITCH_MIN, BODY_PITCH_MAX)

                        compositeDisposable.add(setGimbalAngle(
                                Float.NaN.toDouble(),
                                pitchBody,
                                yaw.toDouble()
                            )
                                .subscribe(
                                    {
                                        sendMessage(
                                            commandAccepted(
                                                MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                                                msg.originSystemId,
                                                msg.originComponentId
                                            )
                                        )
                                    },
                                    { error ->
                                        onError(
                                            error,
                                            StandardResponses::commandFailed,
                                            MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                                            msg.originSystemId,
                                            msg.originComponentId
                                        )
                                    }
                                )
                        )
                        return true
                    }

                    sendMessage(
                        commandFailed(
                            MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                            msg.originSystemId,
                            msg.originComponentId
                        )
                    )

                    return true
                }

                MavCmd.MAV_CMD_DO_MOUNT_CONFIGURE -> {
                    if (payload.param7().roundToInt() == gimbalDeviceId || payload.param7().roundToInt() == 0) {
                        val mountMode = EnumValue.create(MavMountMode::class.java, payload.param4().roundToInt()).entry()

                        // 定稿语义: 所有 mount mode 一律映射 DJI YAW_FOLLOW
                        // (yaw 随机体; pitch/roll 按世界系稳定)。pitch 指令为
                        // 一次性世界角, 机体相对跟随不做模拟
                        val gimbalMode = when (mountMode) {
                            MavMountMode.MAV_MOUNT_MODE_MAVLINK_TARGETING,
                            MavMountMode.MAV_MOUNT_MODE_RC_TARGETING,
                            MavMountMode.MAV_MOUNT_MODE_GPS_POINT -> GimbalMode.YAW_FOLLOW
                            else -> null
                        }

                        if (gimbalMode == null) {
                            log.w("Unsupported mount mode: $mountMode")
                            sendMessage(
                                commandFailed(
                                    MavCmd.MAV_CMD_DO_MOUNT_CONFIGURE,
                                    msg.originSystemId,
                                    msg.originComponentId
                                )
                            )
                            return true
                        }

                        compositeDisposable.add(setGimbalMode(gimbalMode)
                            .subscribe(
                                {
                                    log.i("Set gimbal mode to $gimbalMode")
                                    sendMessage(
                                        commandAccepted(
                                            MavCmd.MAV_CMD_DO_MOUNT_CONFIGURE,
                                            msg.originSystemId,
                                            msg.originComponentId
                                        )
                                    )
                                },
                                { error ->
                                    onError(
                                        error,
                                        StandardResponses::commandFailed,
                                        MavCmd.MAV_CMD_DO_MOUNT_CONFIGURE,
                                        msg.originSystemId,
                                        msg.originComponentId
                                    )
                                })
                        )
                        return true
                    }

                    sendMessage(
                        commandFailed(
                            MavCmd.MAV_CMD_DO_MOUNT_CONFIGURE,
                            msg.originSystemId,
                            msg.originComponentId
                        )
                    )
                    return true
                }

                MavCmd.MAV_CMD_DO_MOUNT_CONTROL -> {
                    if (EnumValue.create(MavMountMode::class.java, payload.param7().roundToInt()).entry() == MavMountMode.MAV_MOUNT_MODE_MAVLINK_TARGETING) {
                        compositeDisposable.add(
                            setGimbalAngle(payload.param1().toDouble(), payload.param2().toDouble(), payload.param3().toDouble())
                                .subscribe(
                                    { sendMessage(commandAccepted(MavCmd.MAV_CMD_DO_MOUNT_CONTROL, msg.originSystemId, msg.originComponentId)) },
                                    { error ->
                                        onError(
                                            error,
                                            StandardResponses::commandFailed,
                                            MavCmd.MAV_CMD_DO_MOUNT_CONTROL,
                                            msg.originSystemId,
                                            msg.originComponentId
                                        )
                                    })
                        )
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

    private fun gatherProductInfo() {
        KeyManager.getInstance().getValue(KeyTools.createKey(GimbalKey.KeyFirmwareVersion))?.let { firmwareVersion = parseDJIFirmwareVersion(it) }

        // TODO: retrieve these values from DJI (values are for P1)
        // https://enterprise.dji.com/zenmuse-p1/specs
        vendorName = "DJI"
        modelName = "Zenmuse P1"

        rollMin = -55f
        rollMax = 55f
        pitchMin = -130f
        pitchMax = 40f
        yawMin = -160f
        yawMax = 160f
    }

    private fun isAllowedToControl(systemID: Int, compID: Int): Boolean {
        if (systemID == primaryControlSysID && compID == primaryControlCompID) {
            return true
        }

        return systemID == secondaryControlSysID && compID == secondaryControlCompID
    }

    private fun onGimbalManagerSetAttitude(msg: MavlinkMessage<*>) {
        val payload = msg.payload as GimbalManagerSetAttitude

        if (payload.targetSystem() == systemID && (payload.targetComponent() == EnumValue.create(componentID).value() || payload.targetComponent() == 0)) {
            if (!isAllowedToControl(msg.originSystemId, msg.originComponentId)) {
                log.w("Refuse gimbal control from sysID=" + msg.originSystemId + ", compID=" + msg.originComponentId)
                return
            }

            if (payload.gimbalDeviceId() != 0 && payload.gimbalDeviceId() != gimbalDeviceId) {
                log.w("Requested a gimbalID (" + payload.gimbalDeviceId() + ") that's not on MavComp " + componentID)
                return
            }

            if (payload.q().size != 4) {
                log.e("Ignoring GimbalManagerSetAttitude message because quaternion is not valid!")
                return
            }

            if (payload.flags().flagsEnabled(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_NEUTRAL)) {
                compositeDisposable.add(reset()
                    .subscribe({}, { error -> RXUtils.handleError(error) })
                )
                return
            }

            val quaternion = fromList(payload.q())
            val rpy = quaternion.toEulerDegrees()

            val pitch = rpy.second
            var yaw = rpy.third
            if (payload.flags().flagsEnabled(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME)) {
                yaw -= gimbalYawAircraftHeading
            }

            compositeDisposable.add(setGimbalAngle(rpy.first, pitch, yaw)
                .subscribe({}, { error -> RXUtils.handleError(error) })
            )
        }
    }

    private fun onGimbalManagerSetPitchYaw(msg: MavlinkMessage<*>) {
        val payload = msg.payload as GimbalManagerSetPitchyaw

        if (payload.targetSystem() == systemID && (payload.targetComponent() == EnumValue.create(componentID).value() || payload.targetComponent() == 0)) {
            if (!isAllowedToControl(msg.originSystemId, msg.originComponentId)) {
                log.w("Refuse gimbal control from sysID=" + msg.originSystemId + ", compID=" + msg.originComponentId)
                return
            }

            if (payload.gimbalDeviceId() != 0 && payload.gimbalDeviceId() != gimbalDeviceId) {
                log.w("Requested a deviceID (" + payload.gimbalDeviceId() + ") that's not on MavComp " + componentID)
                return
            }

            // Set to neutral/default position, taking precedence over all other flags except
            // RETRACT. Neutral is commonly forward-facing and horizontal (roll=pitch=yaw=0) but may be any orientation.
            if (payload.flags().flagsEnabled(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_NEUTRAL)) {
                compositeDisposable.add(reset()
                    .subscribe({}, { error -> RXUtils.handleError(error) })
                )
                return
            }

            var yaw = Double.NaN
            if (!payload.yaw().isNaN()) {
                yaw = payload.yaw().toDouble().toDegrees()

                if (payload.flags().flagsEnabled(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME)) {
                    yaw = gimbalYawAircraftHeading + payload.yaw().toDouble().toDegrees()
                }
            }

            var pitch = Double.NaN
            if (!payload.pitch().isNaN()) {
                pitch = payload.pitch().toDouble().toDegrees()
            }

            compositeDisposable.add(setGimbalAngle(Double.NaN, pitch, yaw)
                .subscribe({}, { error -> RXUtils.handleError(error) })
            )
        }
    }

    private fun toGimbalManagerInformationMessage(): GimbalManagerInformation {
        return GimbalManagerInformation.builder()
            .timeBootMs(millisSinceBoot)
            .capFlags(ALL_GIMBAL_MANAGER_CAPABILITIES_FLAGS)
            .gimbalDeviceId(gimbalDeviceId)
            .rollMin(rollMin.toRadians())
            .rollMax(rollMax.toRadians())
            .pitchMin(pitchMin.toRadians())
            .pitchMax(pitchMax.toRadians())
            .yawMin(yawMin.toRadians())
            .yawMax(yawMax.toRadians())
            .build()
    }

    private fun toGimbalManagerStatusMessage(): GimbalManagerStatus {
        if (!gimbalManagerAppliedFlags.contains(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME)) {
            gimbalManagerAppliedFlags.add(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME)
        }

        return GimbalManagerStatus.builder()
            .timeBootMs(millisSinceBoot)
            .flags(listToEnum(gimbalManagerAppliedFlags))
            .gimbalDeviceId(gimbalDeviceId)
            .primaryControlSysid(primaryControlSysID)
            .primaryControlCompid(primaryControlCompID)
            .secondaryControlSysid(secondaryControlSysID)
            .secondaryControlCompid(secondaryControlCompID)
            .build()
    }

    private fun toGimbalDeviceInformationMessage(): GimbalDeviceInformation {
        return GimbalDeviceInformation.builder()
            .timeBootMs(millisSinceBoot)
            .vendorName(vendorName)
            .modelName(modelName)
            .customName("")
            .firmwareVersion(firmwareVersion)
            .uid(BigInteger.valueOf(0))
            .capFlags(ALL_GIMBAL_CAPABILITIES_FLAGS)
            .rollMin(rollMin)
            .rollMax(rollMax)
            .pitchMin(pitchMin)
            .pitchMax(pitchMax)
            .yawMin(yawMin)
            .yawMax(yawMax)
            .gimbalDeviceId(gimbalDeviceId)
            .build()
    }

    private fun toGimbalDeviceAttitudeStatus(): GimbalDeviceAttitudeStatus? {
        var attitude = gimbalAttitude
        if (attitude == null) {
            // Some gimbals never push the attitude listener; poll the key instead
            val now = System.currentTimeMillis()
            if (now - gimbalAttitudeFallbackTs > 500) {
                gimbalAttitudeFallbackTs = now
                attitude = KeyManager.getInstance()
                    .getValue(KeyTools.createKey(GimbalKey.KeyGimbalAttitude, componentIndex))
                if (attitude != null && !loggedPolledAttitude) {
                    loggedPolledAttitude = true
                    log.i("Gimbal attitude recovered via key polling: pitch=${attitude.pitch}")
                }
            }
        }

        if (attitude == null) {
            if (!warnedNoGimbalAttitude) {
                warnedNoGimbalAttitude = true
                log.w("Gimbal attitude unavailable on this gimbal; GIMBAL_DEVICE_ATTITUDE_STATUS will not be sent")
            }
            return null
        }

        val quaternion = fromEulerDegrees(
            attitude.roll,
            attitude.pitch,
            gimbalYawAircraftHeading
        )

        return GimbalDeviceAttitudeStatus.builder()
            .timeBootMs(millisSinceBoot)
            .flags(GimbalDeviceFlags.GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME)
            .q(quaternion.toList())
            .angularVelocityX(Float.NaN)
            .angularVelocityY(Float.NaN)
            .angularVelocityZ(Float.NaN)
            .failureFlags()
            .deltaYaw(Float.NaN)
            .deltaYawVelocity(Float.NaN)
            .gimbalDeviceId(gimbalDeviceId)
            .build()
    }

    fun setGimbalMode(mode: GimbalMode): Completable {
        return djiSetValue(KeyTools.createKey(GimbalKey.KeyGimbalMode, componentIndex), mode)
    }

    // 飞机 pitch, 换算到 DJI 云台符号约定 (负值向下)
    private fun aircraftPitchDji(): Double {
        val attitude = KeyManager.getInstance()
            .getValue(KeyTools.createKey(FlightControllerKey.KeyAircraftAttitude))
        return -(attitude?.pitch ?: 0.0)
    }

    fun setGimbalAngle(roll: Double, pitch: Double, yaw: Double): Completable {
        val angleRotation = GimbalAngleRotation()
        if (!roll.isNaN()) {
            angleRotation.roll = roll
        }

        if (!pitch.isNaN()) {
            angleRotation.pitch = pitch
        }

        if (!yaw.isNaN()) {
            angleRotation.yaw = yaw
        }

        angleRotation.mode = GimbalAngleRotationMode.ABSOLUTE_ANGLE

        return djiPerformAction(KeyTools.createKey(GimbalKey.KeyRotateByAngle, componentIndex), angleRotation)
    }

    private fun parseDJIFirmwareVersion(version: String): Long {
        val parts = version.substring(1).split("\\.".toRegex()).dropLastWhile { it.isEmpty() }.toTypedArray()
        if (parts.size != 4) {
            log.w("Could not parse camera firmware version!")
            return 0L
        }

        val dev = parts[3].toInt()
        val patch = parts[2].toInt()
        val minor = parts[1].toInt()
        val major = parts[0].toInt()

        return ((dev and 0xff) shl 24 or
                (patch and 0xff shl 16) or
                (minor and 0xff shl 8) or
                (major and 0xff)).toLong()
    }

    fun reset() : Completable {
        return djiPerformAction(KeyTools.createKey(GimbalKey.KeyGimbalReset, componentIndex), GimbalResetType.PITCH_YAW)
    }

    fun down() : Completable {
        // NEUTRAL 语义。注意: 产品能力文件 (GimbalReset.json) 显示仅
        // SELFIE(1)/PITCH_YAW(2) 受支持, PITCH_UP_OR_DOWN_WITH_YAW_CENTER
        // 在部分机型上会被拒收, 故用 PITCH_YAW 回中近似
        return djiPerformAction(KeyTools.createKey(GimbalKey.KeyGimbalReset, componentIndex), GimbalResetType.PITCH_YAW)
    }

    companion object {
        val COMPONENT_TO_GIMBAL_ID: Map<MavComponent, Int> = mapOf(
            MavComponent.MAV_COMP_ID_GIMBAL to 1,
            MavComponent.MAV_COMP_ID_GIMBAL2 to 2,
            MavComponent.MAV_COMP_ID_GIMBAL3 to 3,
            MavComponent.MAV_COMP_ID_GIMBAL4 to 4,
            MavComponent.MAV_COMP_ID_GIMBAL5 to 5,
            MavComponent.MAV_COMP_ID_GIMBAL6 to 6
        )

        val ALL_GIMBAL_MANAGER_CAPABILITIES_FLAGS = listToEnum(
            listOf(
                GimbalManagerCapFlags.GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT,
                GimbalManagerCapFlags.GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL,
                GimbalManagerCapFlags.GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS,
                GimbalManagerCapFlags.GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS,
                GimbalManagerCapFlags.GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS,
                GimbalManagerCapFlags.GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK,
                GimbalManagerCapFlags.GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK,
                GimbalManagerCapFlags.GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK
            )
        )

        val ALL_GIMBAL_CAPABILITIES_FLAGS = listToEnum(
            listOf(
                GimbalDeviceCapFlags.GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT,
                GimbalDeviceCapFlags.GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL,
                GimbalDeviceCapFlags.GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS,
                GimbalDeviceCapFlags.GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS,
                GimbalDeviceCapFlags.GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS,
                GimbalDeviceCapFlags.GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK,
                GimbalDeviceCapFlags.GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK,
                GimbalDeviceCapFlags.GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK
            )
        )

        // 机体相对 pitch 可控范围的公共安全边界 (当前机队最浅的机型,
        // 如 Mini 4 Pro 为 -90~+35)。世界系目标经姿态换算后 clamp 到
        // 此范围, 保证指令永不越界。TODO: 从 SDK KeyParamRange 读取
        // 各机型真实限位
        const val BODY_PITCH_MIN = -90.0
        const val BODY_PITCH_MAX = 35.0
    }
}









