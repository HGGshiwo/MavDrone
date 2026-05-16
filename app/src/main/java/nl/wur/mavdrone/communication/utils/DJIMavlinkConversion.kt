package nl.wur.mavdrone.communication.utils

import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.wpmz.value.mission.ActionAircraftHoverParam
import dji.sdk.wpmz.value.mission.ActionAircraftRotateYawParam
import dji.sdk.wpmz.value.mission.ActionFocusParam
import dji.sdk.wpmz.value.mission.ActionGimbalRotateParam
import dji.sdk.wpmz.value.mission.ActionSetFocusTypeParam
import dji.sdk.wpmz.value.mission.ActionStartRecordParam
import dji.sdk.wpmz.value.mission.ActionStopRecordParam
import dji.sdk.wpmz.value.mission.ActionTakePhotoParam
import dji.sdk.wpmz.value.mission.ActionZoomParam
import dji.sdk.wpmz.value.mission.GimbalHeadingYawBase
import dji.sdk.wpmz.value.mission.WaylineActionCameraFocusType
import dji.sdk.wpmz.value.mission.WaylineActionInfo
import dji.sdk.wpmz.value.mission.WaylineActionType
import dji.sdk.wpmz.value.mission.WaylineExecuteWaypoint
import dji.sdk.wpmz.value.mission.WaylineGimbalActuatorRotateMode
import dji.sdk.wpmz.value.mission.WaylineLocationCoordinate2D
import dji.sdk.wpmz.value.mission.WaylineLocationCoordinate3D
import dji.sdk.wpmz.value.mission.WaylineWaypointTurnParam
import dji.sdk.wpmz.value.mission.WaylineWaypointYawMode
import dji.sdk.wpmz.value.mission.WaylineWaypointYawParam
import dji.sdk.wpmz.value.mission.WaylineWaypointYawPathMode
import io.dronefleet.mavlink.common.CameraZoomType
import io.dronefleet.mavlink.common.GimbalManagerFlags
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.MavFrame
import io.dronefleet.mavlink.common.MavMissionType
import io.dronefleet.mavlink.common.MissionItemInt
import io.dronefleet.mavlink.common.PrecisionLandMode
import io.dronefleet.mavlink.common.SetFocusType
import io.dronefleet.mavlink.util.EnumValue
import nl.wur.mavdrone.communication.components.impl.mission.WaypointMission
import nl.wur.mavdrone.communication.utils.EnumUtils.Companion.listToEnum
import nl.wur.mavdrone.utils.RxException
import org.locationtech.jts.geom.Coordinate

class DJIMavlinkConversion {
    companion object {
        fun missionItemToCoordinate(missionItem: MissionItemInt): Coordinate {
            return Coordinate(missionItem.x() * 1e-7, missionItem.y() * 1e-7)
        }

        fun missionItemToCoordinate3D(missionItem: MissionItemInt): Coordinate {
            return Coordinate(
                missionItem.x() * 1e-7,
                missionItem.y() * 1e-7,
                missionItem.z().toDouble()
            )
        }

        fun missionItemToWaypoint(
            missionItem: MissionItemInt,
            index: Int,
            speed: Double,
            defaultTurnParam: WaylineWaypointTurnParam,
            defaultYawParam: WaylineWaypointYawParam?
        ): WaylineExecuteWaypoint {
            val location = WaylineLocationCoordinate2D()
            location.latitude = missionItem.x() * 1e-7
            location.longitude = missionItem.y() * 1e-7

            val waypoint = WaylineExecuteWaypoint()
            waypoint.waypointIndex = index
            waypoint.location = location
            waypoint.executeHeight = missionItem.z().toDouble()
            waypoint.turnParam = defaultTurnParam
            waypoint.useStraightLine = true
            waypoint.speed = speed

            // Overwrite global heading
            if (!missionItem.param4().isNaN()) {
                val yawParam = WaylineWaypointYawParam()
                yawParam.yawMode = WaylineWaypointYawMode.SMOOTH_TRANSITION
                yawParam.yawAngle = ((missionItem.param4().toDouble() + 180) % 360) - 180  // Normalize between [-180, 180]
                yawParam.enableYawAngle = true
                waypoint.yawParam = yawParam
            } else {
                waypoint.yawParam = defaultYawParam
            }

            return waypoint
        }

        fun createStartRecordAction(
            cameraComponentIndex: ComponentIndexType,
            actionId: Int
        ): WaylineActionInfo {
            val startRecordParam = ActionStartRecordParam()
            startRecordParam.payloadPositionIndex = WaypointMission.PAYLOAD_POSITION_INFORMATION[cameraComponentIndex] ?: 0

            val startRecordAction = WaylineActionInfo()
            startRecordAction.actionId = actionId
            startRecordAction.actionType = WaylineActionType.START_RECORD
            startRecordAction.startRecordParam = startRecordParam

            return startRecordAction
        }

        fun createStopRecordAction(
            cameraComponentIndex: ComponentIndexType,
            actionId: Int
        ): WaylineActionInfo {
            val stopRecordParam = ActionStopRecordParam()
            stopRecordParam.payloadPositionIndex = WaypointMission.PAYLOAD_POSITION_INFORMATION[cameraComponentIndex] ?: 0

            val stopRecordAction = WaylineActionInfo()
            stopRecordAction.actionId = actionId
            stopRecordAction.actionType = WaylineActionType.STOP_RECORD
            stopRecordAction.stopRecordParam = stopRecordParam

            return stopRecordAction
        }

        fun createSinglePhotoAction(
            cameraComponentIndex: ComponentIndexType,
            actionId: Int
        ): WaylineActionInfo {
            val takePhotoParam = ActionTakePhotoParam()
            takePhotoParam.payloadPositionIndex = WaypointMission.PAYLOAD_POSITION_INFORMATION[cameraComponentIndex] ?: 0

            val singlePhotoAction = WaylineActionInfo()
            singlePhotoAction.actionId = actionId
            singlePhotoAction.actionType = WaylineActionType.TAKE_PHOTO
            singlePhotoAction.takePhotoParam = takePhotoParam

            return singlePhotoAction
        }

        fun createGimbalRotateAction(
            pitch: Double,
            yaw: Double,
            gimbalFlags: EnumValue<GimbalManagerFlags>,
            gimbalComponentIndex: ComponentIndexType,
            actionId: Int
        ): WaylineActionInfo {
            val actionGimbalRotateParam = ActionGimbalRotateParam()
            actionGimbalRotateParam.pitch = pitch
            actionGimbalRotateParam.yaw = yaw
            actionGimbalRotateParam.roll = 0.0
            actionGimbalRotateParam.payloadPositionIndex = WaypointMission.PAYLOAD_POSITION_INFORMATION[gimbalComponentIndex] ?: 0

            if (gimbalFlags.flagsEnabled(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME)) {
                actionGimbalRotateParam.base = GimbalHeadingYawBase.NORTH
                actionGimbalRotateParam.rotateMode = WaylineGimbalActuatorRotateMode.ABSOLUTE_ANGLE
            } else if (gimbalFlags.flagsEnabled(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME)) {
                actionGimbalRotateParam.base = GimbalHeadingYawBase.AIRCRAFT
                actionGimbalRotateParam.rotateMode =
                    WaylineGimbalActuatorRotateMode.ABSOLUTE_ANGLE // Relative wrt front of drone
            }

            actionGimbalRotateParam.enableRoll =
                !gimbalFlags.flagsEnabled(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_ROLL_LOCK)
            actionGimbalRotateParam.enablePitch =
                !gimbalFlags.flagsEnabled(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_PITCH_LOCK)
            actionGimbalRotateParam.enableYaw =
                !gimbalFlags.flagsEnabled(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_YAW_LOCK)

            actionGimbalRotateParam.enableRotateTime = false
            actionGimbalRotateParam.rotateTime = 10.0

            val gimbalRotateAction = WaylineActionInfo()
            gimbalRotateAction.actionId = actionId
            gimbalRotateAction.actionType = WaylineActionType.GIMBAL_ROTATE
            gimbalRotateAction.gimbalRotateParam = actionGimbalRotateParam

            return gimbalRotateAction
        }

        fun createZoomAction(
            focalLength: Double,
            cameraComponentIndex: ComponentIndexType,
            actionId: Int
        ): WaylineActionInfo {
            val actionZoomParam = ActionZoomParam()
            actionZoomParam.focalLength = focalLength
            actionZoomParam.payloadPositionIndex = WaypointMission.PAYLOAD_POSITION_INFORMATION[cameraComponentIndex] ?: 0

            val zoomAction = WaylineActionInfo()
            zoomAction.actionId = actionId
            zoomAction.actionType = WaylineActionType.ZOOM
            zoomAction.zoomParam = actionZoomParam

            return zoomAction
        }

        fun createSetFocusTypeAction(
            cameraFocusType: SetFocusType,
            cameraComponentIndex: ComponentIndexType,
            actionId: Int
        ): WaylineActionInfo {
            val waylineActionCameraFocusType: WaylineActionCameraFocusType = when (cameraFocusType) {
                SetFocusType.FOCUS_TYPE_AUTO_CONTINUOUS -> WaylineActionCameraFocusType.CONTINUOUS
                SetFocusType.FOCUS_TYPE_AUTO -> WaylineActionCameraFocusType.AUTO
                SetFocusType.FOCUS_TYPE_AUTO_SINGLE -> WaylineActionCameraFocusType.AUTO
                SetFocusType.FOCUS_TYPE_METERS -> WaylineActionCameraFocusType.MANUAL
                SetFocusType.FOCUS_TYPE_STEP ->  WaylineActionCameraFocusType.UNKNOWN
                SetFocusType.FOCUS_TYPE_CONTINUOUS -> WaylineActionCameraFocusType.UNKNOWN
                SetFocusType.FOCUS_TYPE_RANGE -> WaylineActionCameraFocusType.UNKNOWN
            }
            val setFocusTypeParam = ActionSetFocusTypeParam()
            setFocusTypeParam.focusType = waylineActionCameraFocusType
            setFocusTypeParam.payloadPositionIndex = WaypointMission.PAYLOAD_POSITION_INFORMATION[cameraComponentIndex] ?: 0

            val setFocusTypeAction = WaylineActionInfo()
            setFocusTypeAction.actionId = actionId
            setFocusTypeAction.actionType = WaylineActionType.SET_FOCUS_TYPE
            setFocusTypeAction.setFocusTypeParam = setFocusTypeParam

            return setFocusTypeAction
        }

        fun createFocusAction(
            focusX: Double,
            focusY: Double,
            focusRegionWidth: Double,
            focusRegionHeight: Double,
            isPointFocus: Boolean,
            isInfiniteFocus: Boolean,
            cameraComponentIndex: ComponentIndexType,
            actionId: Int
        ): WaylineActionInfo {
            val focusParam = ActionFocusParam()
            focusParam.focus_x = focusX
            focusParam.focus_y = focusY
            focusParam.width = focusRegionWidth
            focusParam.height = focusRegionHeight
            focusParam.isPointFocus = isPointFocus
            focusParam.isInfiniteFocus = isInfiniteFocus
            focusParam.payloadPositionIndex = WaypointMission.PAYLOAD_POSITION_INFORMATION[cameraComponentIndex] ?: 0

            val setFocusTypeAction = WaylineActionInfo()
            setFocusTypeAction.actionId = actionId
            setFocusTypeAction.actionType = WaylineActionType.FOCUS
            setFocusTypeAction.focusParam = focusParam

            return setFocusTypeAction
        }

        fun createHoverAction(hoverTime: Double, actionId: Int): WaylineActionInfo {
            val hoverParam = ActionAircraftHoverParam()
            hoverParam.hoverTime = hoverTime

            val hoverAction = WaylineActionInfo()
            hoverAction.actionId = actionId
            hoverAction.actionType = WaylineActionType.HOVER
            hoverAction.aircraftHoverParam = hoverParam

            return hoverAction
        }

        fun createRotateYawAction(
            heading: Double,
            pathMode: WaylineWaypointYawPathMode,
            actionId: Int
        ): WaylineActionInfo {
            val actionAircraftRotateYawParam = ActionAircraftRotateYawParam()
            actionAircraftRotateYawParam.heading = heading
            actionAircraftRotateYawParam.pathMode = pathMode

            val rotateYawAction = WaylineActionInfo()
            rotateYawAction.actionId = actionId
            rotateYawAction.actionType = WaylineActionType.ROTATE_YAW
            rotateYawAction.aircraftRotateYawParam = actionAircraftRotateYawParam

            return rotateYawAction
        }

        fun createDelayItem(
            hoverParam: ActionAircraftHoverParam,
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt {
            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(seq)
                .frame(MavFrame.MAV_FRAME_MISSION)
                .command(MavCmd.MAV_CMD_NAV_DELAY)
                .current(if (seq == currentMavlinkId) 1 else 0)
                .autocontinue(1)
                .param1(hoverParam.hoverTime.toFloat())
                .param2(-1f)
                .param3(-1f)
                .param4(-1f)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                .build()
        }

        @Suppress("SpellCheckingInspection")
        fun createDoSetCamTriggDistItem(
            distance: Float,
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt {
            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(seq)
                .frame(MavFrame.MAV_FRAME_MISSION)
                .command(MavCmd.MAV_CMD_DO_SET_CAM_TRIGG_DIST)
                .current(if (seq == currentMavlinkId) 1 else 0)
                .autocontinue(1)
                .param1(distance)
                .param2(-1f)
                .param3(1f)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                .build()
        }

        fun createStartImageCaptureItem(
            interval: Float,
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt {
            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(seq)
                .frame(MavFrame.MAV_FRAME_MISSION)
                .command(MavCmd.MAV_CMD_IMAGE_START_CAPTURE)
                .current(if (seq == currentMavlinkId) 1 else 0)
                .autocontinue(1)
                .param2(interval)
                .param3(0f)
                .param4(0f)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                .build()
        }

        fun createStartImageCaptureItem(
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt {
            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(seq)
                .frame(MavFrame.MAV_FRAME_MISSION)
                .command(MavCmd.MAV_CMD_IMAGE_START_CAPTURE)
                .current(if (seq == currentMavlinkId) 1 else 0)
                .autocontinue(1)
                .param3(1f)
                .param4(0f)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                .build()
        }

        fun createStopImageCaptureItem(
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt {
            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(seq)
                .frame(MavFrame.MAV_FRAME_MISSION)
                .command(MavCmd.MAV_CMD_IMAGE_STOP_CAPTURE)
                .current(if (seq == currentMavlinkId) 1 else 0)
                .autocontinue(1)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                .build()
        }

        private fun createChangeYawItem(
            yawParam: ActionAircraftRotateYawParam,
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt {
            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(seq)
                .frame(MavFrame.MAV_FRAME_MISSION)
                .command(MavCmd.MAV_CMD_CONDITION_YAW)
                .current(if (seq == currentMavlinkId) 1 else 0)
                .autocontinue(1)
                .param1(yawParam.heading.toFloat())
                .param3((if (yawParam.pathMode == WaylineWaypointYawPathMode.CLOCKWISE) 1 else -1).toFloat())
                .param4(0f)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                .build()
        }

        private fun createChangeGimbalPitchYawItem(
            gimbalRotateParam: ActionGimbalRotateParam,
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt {
            val gimbalManagerFlagsList: MutableList<GimbalManagerFlags> = ArrayList()
            if (gimbalRotateParam.base == GimbalHeadingYawBase.NORTH) {
                gimbalManagerFlagsList.add(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME)
            } else {
                gimbalManagerFlagsList.add(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME)
            }

            if (!gimbalRotateParam.enableRoll) {
                gimbalManagerFlagsList.add(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_ROLL_LOCK)
            }

            if (!gimbalRotateParam.enablePitch) {
                gimbalManagerFlagsList.add(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_PITCH_LOCK)
            }

            if (!gimbalRotateParam.enableYaw) {
                gimbalManagerFlagsList.add(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_YAW_LOCK)
            }

            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(seq)
                .frame(MavFrame.MAV_FRAME_MISSION)
                .command(MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)
                .current(if (seq == currentMavlinkId) 1 else 0)
                .autocontinue(1)
                .param1(gimbalRotateParam.pitch.toFloat())
                .param2(gimbalRotateParam.yaw.toFloat())
                .x(listToEnum(gimbalManagerFlagsList).value())
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                .build()
        }

        private fun createSetCameraZoom(
            zoomParam: ActionZoomParam,
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt {
            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(seq)
                .frame(MavFrame.MAV_FRAME_MISSION)
                .command(MavCmd.MAV_CMD_SET_CAMERA_ZOOM)
                .current(if (seq == currentMavlinkId) 1 else 0)
                .autocontinue(1)
                .param1(EnumValue.of(CameraZoomType.ZOOM_TYPE_FOCAL_LENGTH).value().toFloat())
                .param2(zoomParam.focalLength.toFloat())
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                .build()
        }

        private fun createVideoStartCaptureItem(
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt {
            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(seq)
                .frame(MavFrame.MAV_FRAME_MISSION)
                .command(MavCmd.MAV_CMD_VIDEO_START_CAPTURE)
                .current(if (seq == currentMavlinkId) 1 else 0)
                .autocontinue(1)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                .build()
        }

        private fun createVideoStopCaptureItem(
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt {
            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(seq)
                .frame(MavFrame.MAV_FRAME_MISSION)
                .command(MavCmd.MAV_CMD_VIDEO_STOP_CAPTURE)
                .current(if (seq == currentMavlinkId) 1 else 0)
                .autocontinue(1)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                .build()
        }

        private fun createSetCameraFocusItem(
            setFocusTypeParam: ActionSetFocusTypeParam,
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt {
            val focusType: SetFocusType = when (setFocusTypeParam.focusType) {
                WaylineActionCameraFocusType.CONTINUOUS -> SetFocusType.FOCUS_TYPE_AUTO_CONTINUOUS

                WaylineActionCameraFocusType.MANUAL -> SetFocusType.FOCUS_TYPE_METERS
                WaylineActionCameraFocusType.UNKNOWN -> SetFocusType.FOCUS_TYPE_AUTO
                WaylineActionCameraFocusType.AUTO -> SetFocusType.FOCUS_TYPE_AUTO_SINGLE
                WaylineActionCameraFocusType.PRECISE -> SetFocusType.FOCUS_TYPE_METERS
                null -> SetFocusType.FOCUS_TYPE_AUTO
            }
            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(seq)
                .frame(MavFrame.MAV_FRAME_MISSION)
                .command(MavCmd.MAV_CMD_SET_CAMERA_FOCUS)
                .current(if (seq == currentMavlinkId) 1 else 0)
                .autocontinue(1)
                .param1(EnumValue.of(focusType).value().toFloat())
                .param2(-1f)
                .param3(-1f)
                .param4(-1f)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                .build()
        }

        fun createWaypointItem(
            location: WaylineLocationCoordinate2D,
            altitude: Float,
            delayTime: Float,
            yawAngle: Float,
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt {
            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(seq)
                .frame(MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
                .command(MavCmd.MAV_CMD_NAV_WAYPOINT)
                .current(if (seq == currentMavlinkId) 1 else 0)
                .autocontinue(1)
                .param1(delayTime)
                .param4(yawAngle)
                .x((location.latitude * 1e7).toInt())
                .y((location.longitude * 1e7).toInt())
                .z(altitude)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                .build()
        }

        fun createWaypointItem(
            location: WaylineLocationCoordinate3D,
            delayTime: Float,
            yawAngle: Float,
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt {
            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(seq)
                .frame(MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
                .command(MavCmd.MAV_CMD_NAV_WAYPOINT)
                .current(if (seq == currentMavlinkId) 1 else 0)
                .autocontinue(1)
                .param1(delayTime)
                .param4(yawAngle)
                .x((location.latitude * 1e7).toInt())
                .y((location.longitude * 1e7).toInt())
                .z((location.altitude * 1e7).toInt().toFloat())
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                .build()
        }

        fun createWaypointItem(
            location: Coordinate,
            altitude: Float,
            delayTime: Float,
            yawAngle: Float,
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt {
            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(seq)
                .frame(MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
                .command(MavCmd.MAV_CMD_NAV_WAYPOINT)
                .current(if (seq == currentMavlinkId) 1 else 0)
                .autocontinue(1)
                .param1(delayTime)
                .param4(yawAngle)
                .x((location.x * 1e7).toInt())
                .y((location.y * 1e7).toInt())
                .z(altitude)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                .build()
        }

        fun createReturnToLaunchItem(
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt {
            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(seq)
                .frame(MavFrame.MAV_FRAME_MISSION)
                .command(MavCmd.MAV_CMD_NAV_RETURN_TO_LAUNCH)
                .current(if (seq == currentMavlinkId) 1 else 0)
                .autocontinue(1)
                .param4(Float.NaN)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                .build()
        }

        fun createLandItem(
            landCoordinate: WaylineLocationCoordinate2D,
            yaw: Float,
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt {
            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(seq)
                .frame(MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
                .command(MavCmd.MAV_CMD_NAV_LAND)
                .current(if (seq == currentMavlinkId) 1 else 0)
                .autocontinue(1)
                .param1(0f)
                .param2(
                    EnumValue.of(PrecisionLandMode.PRECISION_LAND_MODE_DISABLED).value().toFloat()
                )
                .param4(yaw)
                .x((landCoordinate.latitude * 1e7).toInt())
                .y((landCoordinate.longitude * 1e7).toInt())
                .z(0f)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                .build()
        }

        fun createTakeOffItem(
            takeOffCoordinate: WaylineLocationCoordinate3D,
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt {
            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(seq)
                .frame(MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
                .command(MavCmd.MAV_CMD_NAV_TAKEOFF)
                .current(if (seq == currentMavlinkId) 1 else 0)
                .autocontinue(1)
                .param4(Float.NaN)
                .x((takeOffCoordinate.latitude * 1e7).toInt())
                .y((takeOffCoordinate.longitude * 1e7).toInt())
                .z(takeOffCoordinate.altitude.toFloat())
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                .build()
        }

        fun createChangeSpeedItem(
            speed: Float,
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt {
            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(seq)
                .frame(MavFrame.MAV_FRAME_MISSION)
                .command(MavCmd.MAV_CMD_DO_CHANGE_SPEED)
                .current(if (seq == currentMavlinkId) 1 else 0)
                .autocontinue(1)
                .param1(0f)
                .param2(speed)
                .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
                .build()
        }

        @Throws(RxException::class)
        fun actionToMavlinkItem(
            djiAction: WaylineActionInfo,
            seq: Int,
            currentMavlinkId: Int,
            targetSystem: Int,
            targetComponent: Int
        ): MissionItemInt? {
            val missionItem = when (djiAction.actionType) {
                WaylineActionType.HOVER -> createDelayItem(
                    djiAction.aircraftHoverParam,
                    seq,
                    currentMavlinkId,
                    targetSystem,
                    targetComponent
                )

                WaylineActionType.TAKE_PHOTO -> createStartImageCaptureItem(
                    seq,
                    currentMavlinkId,
                    targetSystem,
                    targetComponent
                )

                WaylineActionType.START_RECORD -> createVideoStartCaptureItem(
                    seq,
                    currentMavlinkId,
                    targetSystem,
                    targetComponent
                )

                WaylineActionType.STOP_RECORD -> createVideoStopCaptureItem(
                    seq,
                    currentMavlinkId,
                    targetSystem,
                    targetComponent
                )

                WaylineActionType.SET_FOCUS_TYPE -> createSetCameraFocusItem(
                    djiAction.setFocusTypeParam,
                    seq,
                    currentMavlinkId,
                    targetSystem,
                    targetComponent
                )

                WaylineActionType.ZOOM -> createSetCameraZoom(
                    djiAction.zoomParam,
                    seq,
                    currentMavlinkId,
                    targetSystem,
                    targetComponent
                )

                WaylineActionType.GIMBAL_ROTATE -> createChangeGimbalPitchYawItem(
                    djiAction.gimbalRotateParam,
                    seq,
                    currentMavlinkId,
                    targetSystem,
                    targetComponent
                )

                WaylineActionType.ROTATE_YAW -> createChangeYawItem(
                    djiAction.aircraftRotateYawParam,
                    seq,
                    currentMavlinkId,
                    targetSystem,
                    targetComponent
                )

                else -> return null
            }
            return missionItem
        }
    }
}
