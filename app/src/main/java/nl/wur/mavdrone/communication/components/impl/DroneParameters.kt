package nl.wur.mavdrone.communication.components.impl

import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.IntValueConfig
import dji.sdk.keyvalue.value.flightcontroller.FailsafeAction
import dji.v5.manager.KeyManager
import io.dronefleet.mavlink.common.FenceAction
import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.BaseComponent
import nl.wur.mavdrone.communication.components.Drone
import nl.wur.mavdrone.communication.utils.MavParameter
import nl.wur.mavdrone.communication.utils.ParameterConverter
import java.lang.ref.WeakReference

class DroneParameters(
    connectionWR: WeakReference<MavConnection>,
    private val droneWR: WeakReference<Drone>,
    private val missionManagerWR: WeakReference<MissionManager>
) : BaseComponent(connectionWR, "MavDroneParameters") {
    override fun create() {
        //region: param SYS_MAV_ID
        val mavSysId = MavParameter.fromAndroidParameter(
            connectionWR,
            droneWR,
            "SYSID_THISMAV",
            1,
            1,
            255,
            ParameterConverter.intConverter,
            false,
            null
        )
        mavSysId.setMetadata(
            "System",
            "Mavlink",
            "MAVLink system ID of this vehicle",
            "Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network",
            ""
        )
        droneWR.get()?.addParameter(mavSysId)

        //endregion
        //region: param SYS_MAV_PORT
        val sysPortParam = MavParameter.fromAndroidParameter(
            connectionWR,
            droneWR,
            "SYS_MAV_PORT",
            5760,
            1,
            9999,
            ParameterConverter.intConverter,
            false,
            null
        )
        sysPortParam.setMetadata(
            "System",
            "Mavlink",
            "Mavlink system TCP port.",
            "System TCP port",
            ""
        )
        droneWR.get()?.addParameter(sysPortParam)

        //endregion
        //region: param SYS_GCS_HB_INT
        val sysGcsHbInt = MavParameter.fromAndroidParameter(
            connectionWR,
            droneWR,
            "SYS_GCS_HB_INT",
            1000,
            1,
            10000,
            ParameterConverter.intConverter,
            false,
            null
        )
        sysGcsHbInt.setMetadata(
            "System",
            "Mavlink",
            "Expected GCS heartbeat",
            "Expected interval for the GCS heartbeat.",
            "ms"
        )
        droneWR.get()?.addParameter(sysGcsHbInt)

        //endregion
        //region: param SYS_GCS_HB_MISS
        val sysGcsHbMiss = MavParameter.fromAndroidParameter(
            connectionWR,
            droneWR,
            "SYS_GCS_HB_MISS",
            4,
            1,
            100,
            ParameterConverter.intConverter,
            false,
            null
        )
        sysGcsHbMiss.setMetadata(
            "System",
            "Mavlink",
            "Max missed heartbeats",
            "Maximum number of missed heartbeats before the connection with the GCS is lost.",
            ""
        )
        droneWR.get()?.addParameter(sysGcsHbMiss)

        //endregion

        //region: param COM_DISARM_LAND
        val comDisarmLand = MavParameter.fromAndroidParameter(
            connectionWR,
            droneWR,
            "COM_DISARM_LAND",
            2.0f,
            200.0f,
            0.0f,
            ParameterConverter.floatConverter,
            false
        ) { timeout: Float ->
            droneWR.get()?.setDisArmAfterLandTimeout(timeout)
        }
        comDisarmLand.setMetadata(
            "Common",
            "Arming",
            "Time-out for auto disarm after landing ",
            "A non-zero, positive value specifies the time-out period in seconds after which the vehicle will be automatically disarmed in case a landing situation has been detected during this period. A zero value means that automatic disarming triggered by landing detection is disabled.",
            "ms"
        )
        droneWR.get()?.addParameter(comDisarmLand)

        //endregion
        //region: param FENCE_ACTION
        val fenceAction = MavParameter.fromAndroidParameter(
            connectionWR,
            droneWR,
            "FENCE_ACTION",
            FenceAction.FENCE_ACTION_HOLD,
            FenceAction::class.java,
            false
        ) { missionManagerWR.get()?.geofenceManager?.onFenceActionChanged(it) }
        fenceAction.setMetadata(
            "Navigation",
            "Fence",
            "Geofence violation action",
            "Note: Setting this value to 6 enables flight termination, which will kill the vehicle on violation of the fence.",
            ""
        )
        droneWR.get()?.addParameter(fenceAction)

        val fenceAutoEnable = MavParameter.fromAndroidParameter(
            connectionWR,
            droneWR,
            "FENCE_AUTO_ENBL",
            true,
            false
        ) { autoEnable: Boolean ->
            missionManagerWR.get()?.geofenceManager?.onAutoEnableChanged(autoEnable)
        }
        fenceAutoEnable.setMetadata(
            "Navigation",
            "Fence",
            "Auto enable geofence",
            "Automatic enable geofence when a geofence is set and a mission is started.",
            ""
        )
        droneWR.get()?.addParameter(fenceAutoEnable)

        //endregion
        //region: param FENCE_MAX_ALT
        val heightLimitRange = KeyManager.getInstance()
            .getValue(KeyTools.createKey(FlightControllerKey.KeyHeightLimitRange))
            ?: IntValueConfig().apply {
                min = 12
                max = 120
                defaultValue = 40
            }

        val fenceMaxAlt = MavParameter.fromDJIParameter(
            connectionWR,
            droneWR,
            "FENCE_MAX_ALT",
            KeyTools.createKey(FlightControllerKey.KeyHeightLimit),
            heightLimitRange.defaultValue,
            heightLimitRange.max,
            heightLimitRange.min,
            ParameterConverter.intConverter,
            false,
            null
        )
        fenceMaxAlt.setMetadata(
            "Navigation",
            "Fence",
            "Maximum flight altitude",
            "Maximum flight altitude of the drone.",
            "meter"
        )
        droneWR.get()?.addParameter(fenceMaxAlt)

        //endregion
        //region: param FENCE_MAX_DIST
        val distanceLimitRange = KeyManager.getInstance()
            .getValue(KeyTools.createKey(FlightControllerKey.KeyDistanceLimitRange))
            ?: IntValueConfig().apply {
                min = 0
                max = 6000
                defaultValue = 6000
            }

        val fenceMaxDist = MavParameter.fromDJIParameter(
            connectionWR,
            droneWR,
            "FENCE_MAX_DIST",
            KeyTools.createKey(FlightControllerKey.KeyDistanceLimit),
            distanceLimitRange.defaultValue,
            distanceLimitRange.max,
            distanceLimitRange.min,
            ParameterConverter.intConverter,
            false,
            null
        )
        fenceMaxDist.setMetadata(
            "Navigation",
            "Fence",
            "Maximum flight distance",
            "Maximum flight distance of the drone measured between the drone and the home point.",
            "meter"
        )
        droneWR.get()?.addParameter(fenceMaxDist)

        //endregion
        //region: param FENCE_ENAB_DIST
        val fenceEnableDist = MavParameter.fromDJIParameter(
            connectionWR,
            droneWR,
            "FENCE_ENAB_DIST",
            KeyTools.createKey(FlightControllerKey.KeyDistanceLimitEnabled),
            true,
            false,
            null
        )
        fenceEnableDist.setMetadata(
            "Navigation",
            "Fence",
            "Enable maximum flight distance",
            "true means the distance limitation switch is opened. After this switch is opened, KeyDistanceLimit can be used to set the maximum flight distance of the aircraft.",
            ""
        )
        droneWR.get()?.addParameter(fenceEnableDist)

        //endregion
        //region: param BAT_LVL_SERIOUS
        val batteryLevelSerious = MavParameter.fromDJIParameter(
            connectionWR,
            droneWR,
            "BAT_LVL_SERIOUS",
            KeyTools.createKey(FlightControllerKey.KeySeriousLowBatteryWarningThreshold),
            10,
            30,
            5,
            ParameterConverter.intConverter,
            false,
            null
        )
        batteryLevelSerious.setMetadata(
            "Navigation",
            "Battery",
            "Serious low battery threshold ",
            "To get the threshold of severely low battery warning. The default threshold is 10%, and it can not be set for Matrice 30 Series. If the battery power is lower than the threshold, the aircraft will go back to home. ",
            "%"
        )
        droneWR.get()?.addParameter(batteryLevelSerious)

        //endregion
        //region: param BAT_LVL_WARN
        val batteryLevelWarning = MavParameter.fromDJIParameter(
            connectionWR,
            droneWR,
            "BAT_LVL_WARN",
            KeyTools.createKey(FlightControllerKey.KeyLowBatteryWarningThreshold),
            25,
            50,
            15,
            ParameterConverter.intConverter,
            false,
            null
        )
        batteryLevelWarning.setMetadata(
            "Navigation",
            "Battery",
            "Low battery threshold ",
            "To set and get the threshold for battery low power warning. The threshold is a percentage and the range is [15,50]. When the battery power is lower than the threshold, the aircraft will make a low battery alarm.",
            "%"
        )
        droneWR.get()?.addParameter(batteryLevelWarning)

        //endregion
        //region: param BAT_LOW_RTH
        val enableBatteryRTH = MavParameter.fromDJIParameter(
            connectionWR,
            droneWR,
            "BAT_LOW_RTH",
            KeyTools.createKey(FlightControllerKey.KeyLowBatteryRTHEnabled),
            true,
            false,
            null
        )
        enableBatteryRTH.setMetadata(
            "Navigation",
            "Battery",
            "Enable Return-To-Home at low battery",
            "true means intelligent low battery return-to-home function is opened. To avoid unnecessary dangers dangerous due to the low battery, aircraft will intelligently judge whether the current battery is sufficient according to the flight position information. If current battery power is only enough to complete the return-to-home process, MSDK will prompt whether operate the return-to-home process. If the user does not make a selection within 10s, MSDK will automatically return-to-home after 10s. During the returning process, you can short press the smart return-to-home button on the remote control to cancel the return-to-home process. Smart low battery return-to-home only occurs once during one flight. If the user cancels the low-battery return-to-home reminder and continues to fly, the aircraft may be forced to land due to insufficient power when returning, resulting in the aircraft being lost or crashed. For security reasons, it is not recommended to turn this function off.",
            ""
        )
        droneWR.get()?.addParameter(enableBatteryRTH)

        //endregion
        //region: param RC_LOST_ACTION
        val rcLostAction = MavParameter.fromDJIParameter(
            connectionWR,
            droneWR,
            "RC_LOST_ACTION",
            KeyTools.createKey(FlightControllerKey.KeyFailsafeAction),
            FailsafeAction.GOHOME,
            FailsafeAction::class.java,
            false,
            null
        )
        rcLostAction.setMetadata(
            "Navigation",
            "Failsafe",
            "Failsafe action",
            "When the remote controller loses connection with the aircraft, the aircraft will perform according to the set fail safe behavior.",
            ""
        )
        droneWR.get()?.addParameter(rcLostAction)

        //endregion

        //region: param RC_STICK_TRIGG
        val rcStickTrigg = MavParameter.fromAndroidParameter(
            connectionWR,
            droneWR,
            "RC_STICK_TRIGG",
            20,
            660,
            1,
            ParameterConverter.intConverter,
            false,
            null
        )
        rcStickTrigg.setMetadata(
            "Remote",
            "Safety",
            "Min remote trigger value",
            "Minimum trigger value to enable safety during flight. If one of the sticks is moved more than this value, safety is enabled and the mission is paused.",
            ""
        )
        droneWR.get()?.addParameter(rcStickTrigg)

        //endregion

        //region: param STREAM_PORT
        val rtspPort = MavParameter.fromAndroidParameter(
            connectionWR,
            droneWR,
            "STREAM_PORT",
            8554,
            9999,
            0,
            ParameterConverter.intConverter,
            false,
            null
        )
        rtspPort.setMetadata(
            "Stream",
            "RTSP",
            "Port",
            "RTSP stream port",
            ""
        )
        droneWR.get()?.addParameter(rtspPort)

        //endregion

        //region: param STREAM_START
        val streamAutoStart = MavParameter.fromAndroidParameter(
            connectionWR,
            droneWR,
            "STREAM_START",
            true,
            false,
            null
        )
        streamAutoStart.setMetadata(
            "Stream",
            "RTSP",
            "Auto start",
            "Automatically start streaming even without MAV_CMD_VIDEO_START_STREAMING command",
            ""
        )
        droneWR.get()?.addParameter(streamAutoStart)

        //endregion

        //region: param ENABLE_ONBOARD_CAMERA
        val enableOnBoardCamera = MavParameter.fromAndroidParameter(
            connectionWR,
            droneWR,
            "ENABLE_ONBOARD",
            false,
            false
        ) {
            enable: Boolean ->
            droneWR.get()?.enableOnboardCamera(enable)
        }
        enableOnBoardCamera.setMetadata(
            "Camera",
            "Onboard",
            "Onboard",
            "Enable onboard camera",
            ""
        )
        droneWR.get()?.addParameter(enableOnBoardCamera)
        //endregion

        //region: param STREAM_USER
        // TODO: this does not work since it is not a mavlink enum
//        MavlinkParameter<StreamQuality> streamQuality = MavlinkParameter.fromAndroidParameter(
//                connectionWR,
//                droneWR,
//                "STREAM_QUALITY",
//                StreamQuality.SD,
//                StreamQuality.class,
//                false,
//                null
//        );
//        streamQuality.setMetadata(
//                "Stream",
//                "RTSP",
//                "Quality",
//                "Stream quality",
//                "");
//        droneWR.get().addParameter(streamQuality);
        //endregion

        // Add parameters to keep MAVSDK happy
        addDummyParameter("CAL_GYRO0_ID", 0, 0, 1)
        addDummyParameter("CAL_ACC0_ID", 0, 0, 1)
        addDummyParameter("CAL_MAG0_ID", 0, 0, 1)
        addDummyParameter("SYS_HITL", 0, -1, 2) // TODO: can be used maybe
        addDummyParameter("MIS_TAKEOFF_ALT", 1.4f, 0.0f, 80.0f)
    }

    private fun addDummyParameter(name: String, defaultValue: Int, min: Int, max: Int) {
        val dummyParameter = MavParameter.fromAndroidParameter(
            connectionWR,
            droneWR,
            name,
            defaultValue,
            min,
            max,
            ParameterConverter.intConverter,
            false,
            null
        )
        droneWR.get()?.addParameter(dummyParameter)
    }

    private fun addDummyParameter(name: String, defaultValue: Float, min: Float, max: Float) {
        val dummyParameter = MavParameter.fromAndroidParameter(
            connectionWR,
            droneWR,
            name,
            defaultValue,
            min,
            max,
            ParameterConverter.floatConverter,
            false,
            null
        )
        droneWR.get()?.addParameter(dummyParameter)
    }
}
