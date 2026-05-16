package nl.wur.mavdrone.communication.components.impl.mission

import dji.sdk.keyvalue.value.camera.CameraType
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.product.ProductType
import dji.sdk.wpmz.jni.JNIWPMZManager
import dji.sdk.wpmz.value.mission.CameraLensType
import dji.sdk.wpmz.value.mission.WPMLParseError
import dji.sdk.wpmz.value.mission.Wayline
import dji.sdk.wpmz.value.mission.WaylineActionGroup
import dji.sdk.wpmz.value.mission.WaylineActionInfo
import dji.sdk.wpmz.value.mission.WaylineActionTrigger
import dji.sdk.wpmz.value.mission.WaylineActionTriggerType
import dji.sdk.wpmz.value.mission.WaylineActionType
import dji.sdk.wpmz.value.mission.WaylineExecuteAltitudeMode
import dji.sdk.wpmz.value.mission.WaylineExecuteWaypoint
import dji.sdk.wpmz.value.mission.WaylineFinishedAction
import dji.sdk.wpmz.value.mission.WaylineLocationCoordinate2D
import dji.sdk.wpmz.value.mission.WaylineLocationCoordinate3D
import dji.sdk.wpmz.value.mission.WaylineWaypointTurnMode
import dji.sdk.wpmz.value.mission.WaylineWaypointTurnParam
import dji.sdk.wpmz.value.mission.WaylineWaypointYawMode
import dji.sdk.wpmz.value.mission.WaylineWaypointYawParam
import dji.sdk.wpmz.value.mission.WaylineWaypointYawPathMode
import dji.v5.common.callback.CommonCallbacks.CompletionCallbackWithProgress
import dji.v5.manager.aircraft.waypoint3.WaypointMissionManager
import dji.v5.utils.common.ContextUtil
import io.dronefleet.mavlink.MavlinkMessage
import io.dronefleet.mavlink.common.CameraZoomType
import io.dronefleet.mavlink.common.GimbalManagerFlags
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.MavFrame
import io.dronefleet.mavlink.common.MissionItemInt
import io.dronefleet.mavlink.common.SetFocusType
import io.dronefleet.mavlink.util.EnumValue
import io.reactivex.rxjava3.core.Completable
import io.reactivex.rxjava3.core.Single
import nl.wur.mavdrone.R
import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.impl.MissionManager
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.actionToMavlinkItem
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createChangeSpeedItem
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createDoSetCamTriggDistItem
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createFocusAction
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createGimbalRotateAction
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createHoverAction
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createLandItem
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createReturnToLaunchItem
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createRotateYawAction
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createSetFocusTypeAction
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createSinglePhotoAction
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createStartImageCaptureItem
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createStartRecordAction
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createStopImageCaptureItem
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createStopRecordAction
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createTakeOffItem
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createWaypointItem
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.createZoomAction
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.missionItemToWaypoint
import nl.wur.mavdrone.communication.utils.FileUtils.Companion.writeFileUsingTemplate
import nl.wur.mavdrone.communication.utils.KMZ
import nl.wur.mavdrone.communication.utils.KMZ.Companion.create
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.missionInvalid
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.missionInvalidParam1
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.missionInvalidParam3
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.missionInvalidParam4
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.missionInvalidParam5X
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.missionUnsupported
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.missionUnsupportedFrame
import nl.wur.mavdrone.utils.CoordinateUtils.Companion.getDistanceInMeterFromTwoGPSLocations
import nl.wur.mavdrone.utils.RXUtils.Companion.djiCompletableFunctionWithProgress
import nl.wur.mavdrone.utils.RxException
import nl.wur.mavdrone.utils.RxException.MissionConverterException
import nl.wur.mavdrone.utils.RxException.NotEnoughWaypointsException
import nl.wur.mavdrone.utils.RxException.WriteException
import java.io.File
import java.lang.ref.WeakReference
import java.util.EnumMap
import java.util.Locale
import kotlin.math.roundToInt

class WaypointMission (
    connectionWR: WeakReference<MavConnection>,
    missionManagerWR: WeakReference<MissionManager>,
    kmz: KMZ? = null
) : BaseMission(connectionWR, missionManagerWR, "MavWaypointMission") {
    private val shouldAddWaypointHalfway = false  // Adds additional waypoint halfway instead of at current drone position

    private var imageCaptureStarted = false
    private var videoCaptureStarted = false
    var isUploadedToAircraft: Boolean = false
        private set

    var wayline: Wayline? = null
        private set

    private var takeoffRefPoint: WaylineLocationCoordinate3D? = null
    private var finishAction: WaylineFinishedAction = WaylineFinishedAction.NO_ACTION
    private var djiToMavlinkIdMap: MutableMap<Int, Int?> = HashMap()

    var kmz: KMZ?

    // Default turn and yaw parameters, parameters may be changed by other classes
    private var turnParam: WaylineWaypointTurnParam = WaylineWaypointTurnParam()
    private var yawParam: WaylineWaypointYawParam = WaylineWaypointYawParam()

    var currentMavlinkId: Int = 0
    var currentDJIId: Int = 0

    init {
        turnParam.turnMode =
            WaylineWaypointTurnMode.TO_POINT_AND_STOP_WITH_DISCONTINUITY_CURVATURE
        turnParam.turnDampingDistance = 0.0
        yawParam.yawMode = WaylineWaypointYawMode.FOLLOW_WAYLINE

        this.kmz = kmz
    }

    fun setUploadedToAircraft(uploaded: Boolean) {
        isUploadedToAircraft = uploaded
    }

    override fun toString(): String {
        return "Waypoint mission with ${size()} mission items"
    }

    override fun copy(): WaypointMission {
        val copy = WaypointMission(connectionWR, missionManagerWR)

        copy.addItems(missionItems)
        copy.wayline = wayline
        copy.takeoffRefPoint = takeoffRefPoint
        copy.finishAction = finishAction
        copy.djiToMavlinkIdMap = HashMap(djiToMavlinkIdMap)

        copy.isUploadedToAircraft = isUploadedToAircraft
        copy.turnParam = turnParam
        copy.yawParam = yawParam

        copy.kmz = kmz?.let { KMZ(connectionWR, it.path) }
        copy.currentMavlinkId = currentMavlinkId
        return copy
    }

    override fun clear() {
        super.clear()

        wayline = null
        takeoffRefPoint = null
        finishAction = WaylineFinishedAction.NO_ACTION
        djiToMavlinkIdMap.clear()

        imageCaptureStarted = false
        videoCaptureStarted = false
        isUploadedToAircraft = false

        currentMavlinkId = 0

    }

    override fun addItem(msg: MavlinkMessage<*>): Boolean {
        val payload = msg.payload as MissionItemInt

        if (!allowedCommands.contains(payload.command().entry())) {
            log.e("WaypointMission does not support " + payload.command().entry())
            missionManagerWR.get()?.sendMessage(
                missionUnsupported(
                    payload.missionType().entry(),
                    msg.originSystemId,
                    msg.originComponentId
                )
            )
            return false
        }

        when (payload.command().entry()) {
            MavCmd.MAV_CMD_NAV_TAKEOFF -> if (!java.lang.Float.isNaN(payload.param4())) {
                log.e("Cannot specify yaw parameter in " + payload.command().entry())
                missionManagerWR.get()?.sendMessage(
                    missionInvalidParam4(
                        payload.missionType().entry(),
                        msg.originSystemId,
                        msg.originComponentId
                    )
                )
                return false
            }

            MavCmd.MAV_CMD_NAV_WAYPOINT -> if (payload.frame()
                    .entry() != MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT && payload.frame()
                    .entry() != MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT
            ) {
                log.e("Only supports frame 'MAV_FRAME_GLOBAL_RELATIVE_ALT_INT'!")
                missionManagerWR.get()?.sendMessage(
                    missionUnsupportedFrame(
                        payload.missionType().entry(),
                        msg.originSystemId,
                        msg.originComponentId
                    )
                )
                return false
            }

            MavCmd.MAV_CMD_NAV_DELAY -> if (payload.param1() == -1f) {
                log.e("Only supports delays in seconds, not till specified time!")
                missionManagerWR.get()?.sendMessage(
                    missionInvalidParam1(
                        payload.missionType().entry(),
                        msg.originSystemId,
                        msg.originComponentId
                    )
                )
                return false
            }

            MavCmd.MAV_CMD_IMAGE_START_CAPTURE -> {
                if (payload.param3().roundToInt() > 1) {
                    log.e("Only supports single image capture or capture till MAV_CMD_IMAGE_STOP_CAPTURE capture!")
                    missionManagerWR.get()?.sendMessage(
                        missionInvalidParam3(
                            payload.missionType().entry(),
                            msg.originSystemId,
                            msg.originComponentId
                        )
                    )
                    return false
                }

                if (imageCaptureStarted || videoCaptureStarted) {
                    log.e("Image or video capture already started!")
                    missionManagerWR.get()?.sendMessage(
                        missionInvalid(
                            payload.missionType().entry(),
                            msg.originSystemId,
                            msg.originComponentId
                        )
                    )
                    return false
                }

                if (payload.param3().roundToInt() == 0) {
                    imageCaptureStarted = true
                }
            }

            MavCmd.MAV_CMD_IMAGE_STOP_CAPTURE -> {
                if (!imageCaptureStarted) {
                    log.e("MAV_CMD_IMAGE_START_CAPTURE should be before MAV_CMD_IMAGE_STOP_CAPTURE!")
                    missionManagerWR.get()?.sendMessage(
                        missionInvalid(
                            payload.missionType().entry(),
                            msg.originSystemId,
                            msg.originComponentId
                        )
                    )
                    return false
                }

                imageCaptureStarted = false
            }

            MavCmd.MAV_CMD_VIDEO_START_CAPTURE -> {
                if (imageCaptureStarted || videoCaptureStarted) {
                    log.e("Image or video capture already started!")
                    missionManagerWR.get()?.sendMessage(
                        missionInvalid(
                            payload.missionType().entry(),
                            msg.originSystemId,
                            msg.originComponentId
                        )
                    )
                    return false
                }

                videoCaptureStarted = true
            }

            MavCmd.MAV_CMD_VIDEO_STOP_CAPTURE -> {
                if (!imageCaptureStarted) {
                    log.e("MAV_CMD_VIDEO_START_CAPTURE should be before MAV_CMD_VIDEO_STOP_CAPTURE!")
                    missionManagerWR.get()?.sendMessage(
                        missionInvalid(
                            payload.missionType().entry(),
                            msg.originSystemId,
                            msg.originComponentId
                        )
                    )
                    return false
                }

                videoCaptureStarted = false
            }

            MavCmd.MAV_CMD_SET_CAMERA_ZOOM -> if (EnumValue.create(
                    CameraZoomType::class.java,
                    payload.param1().roundToInt()
                ).entry() == CameraZoomType.ZOOM_TYPE_FOCAL_LENGTH
            ) {
                log.e("Cannot download mission! Only ZOOM_TYPE_FOCAL_LENGTH is valid for MAV_CMD_SET_CAMERA_ZOOM.")
                missionManagerWR.get()?.sendMessage(
                    missionInvalidParam1(
                        payload.missionType().entry(),
                        msg.originSystemId,
                        msg.originComponentId
                    )
                )
                return false
            }

            MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW -> {
                // One and only one of GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME and GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME should be set.
                val gimbalFlags = EnumValue.create(
                    GimbalManagerFlags::class.java, payload.x()
                )
                if (gimbalFlags.flagsEnabled(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME)
                    && gimbalFlags.flagsEnabled(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME)
                    || (!gimbalFlags.flagsEnabled(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME) && !gimbalFlags.flagsEnabled(
                        GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME
                    ))
                ) {
                    log.e("Cannot download mission! GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME or GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME should be set for MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW.")
                    missionManagerWR.get()?.sendMessage(
                        missionInvalidParam5X(
                            payload.missionType().entry(),
                            msg.originSystemId,
                            msg.originComponentId
                        )
                    )
                    return false
                }
            }

            else -> {}
        }
        missionItems.add(payload)
        return true
    }

    @Suppress("DEPRECATION")
    override fun convertFromMavlink(): Completable {
        wayline = null
        djiToMavlinkIdMap.clear()
        takeoffRefPoint = null

        val missionItemsCopy: List<MissionItemInt> = ArrayList(missionItems)

        return Completable.fromCallable {
            val currentActionGroups: MutableList<WaylineActionGroup> = ArrayList()
            val currentWaypoints: MutableList<WaylineExecuteWaypoint> = ArrayList()
            val currentStartActions: MutableList<WaylineActionInfo> = ArrayList()
            val runningActions: MutableMap<ComponentIndexType, MultipleWaypointsAction> = EnumMap(ComponentIndexType::class.java)

            // TODO: set global speed from parameters
            var currentGlobalSpeed = 5.0
            var currentSpeed = -2.0
            var currentActionId = 0

            var waypoint: WaylineExecuteWaypoint
            val location: WaylineLocationCoordinate2D
            var action: WaylineActionInfo
            var componentIndex: ComponentIndexType
            var multipleWaypointsAction: MultipleWaypointsAction?
            for (missionItem in missionItemsCopy) {
                when (missionItem.command().entry()) {
                    MavCmd.MAV_CMD_NAV_TAKEOFF -> {
                        this.takeoffRefPoint = WaylineLocationCoordinate3D().apply {
                            latitude = missionItem.x() * 1e-7
                            longitude = missionItem.y() * 1e-7
                            altitude = missionItem.z().toDouble()
                        }

                        if (missionItem.param4().isNaN()) {
                            log.w("Ignoring yaw parameter in MAV_CMD_NAV_TAKEOFF!")
                        }
                    }

                    MavCmd.MAV_CMD_NAV_RETURN_TO_LAUNCH -> {
                        if (missionItem.seq() != missionItemsCopy.size - 1) {
                            throw MissionConverterException("Only accepts MAV_CMD_NAV_RETURN_TO_LAUNCH as last command!").setLogger(
                                log
                            )
                        }

                        this.finishAction = WaylineFinishedAction.GO_HOME
                    }

                    MavCmd.MAV_CMD_NAV_LAND -> {
                        if (missionItem.seq() != missionItemsCopy.size - 1) {
                            throw MissionConverterException("Only accepts MAV_CMD_NAV_LAND as last command!").setLogger(
                                log
                            )
                        }

                        waypoint = missionItemToWaypoint(
                            missionItem,
                            currentWaypoints.size,
                            if (currentSpeed != -2.0) currentSpeed else currentGlobalSpeed,
                            turnParam,
                            yawParam
                        )
                        currentWaypoints.add(waypoint)
                        djiToMavlinkIdMap[waypoint.waypointIndex] = missionItem.seq()

                        // Update the finished action to land
                        this.finishAction = WaylineFinishedAction.AUTO_LAND
                    }

                    MavCmd.MAV_CMD_NAV_WAYPOINT -> {
                        waypoint = missionItemToWaypoint(
                            missionItem,
                            currentWaypoints.size,
                            if (currentSpeed != -2.0) currentSpeed else currentGlobalSpeed,
                            turnParam,
                            yawParam,
                        )
                        currentWaypoints.add(waypoint)
                        djiToMavlinkIdMap[waypoint.waypointIndex] = missionItem.seq()

                        // Add hold action
                        if (missionItem.param1() > 0) {
                            action =
                                createHoverAction(missionItem.param1().toDouble(), currentActionId)
                            addToActionGroup(
                                currentActionGroups,
                                action,
                                currentWaypoints.size - 1,
                                currentWaypoints.size - 1
                            )
                            currentActionId += 1
                        }
                    }

                    MavCmd.MAV_CMD_DO_CHANGE_SPEED -> if (currentWaypoints.isEmpty()) {
                        // If there are no waypoints yet, assume this is the global speed
                        currentGlobalSpeed = missionItem.param2().toDouble()
                    } else if (missionItem.param2().toDouble() == -2.0) {
                        // Return to global speed
                        currentSpeed = -2.0
                    } else if ((missionItem.param2() > 0)) {
                        // Set current speed
                        currentSpeed = missionItem.param2().toDouble()
                    }

                    MavCmd.MAV_CMD_SET_CAMERA_FOCUS -> {
                        componentIndex = ComponentIndexType.LEFT_OR_MAIN
                        val focusType = EnumValue.create(
                            SetFocusType::class.java,
                            missionItem.param1().roundToInt()
                        ).entry()

                        action =
                            createSetFocusTypeAction(focusType, componentIndex, currentActionId)

                        if (currentWaypoints.isEmpty()) {
                            currentStartActions.add(action)
                        } else {
                            addToActionGroup(
                                currentActionGroups,
                                action,
                                currentWaypoints.size - 1,
                                currentWaypoints.size - 1
                            )
                        }
                        currentActionId += 1

                        // No perfect translation between Mavlink and DJI possible. Just set to area focus when type equals auto for now...
                        if (focusType == SetFocusType.FOCUS_TYPE_AUTO) {
                            action = createHoverAction(0.5, currentActionId)

                            if (currentWaypoints.isEmpty()) {
                                currentStartActions.add(action)
                            } else {
                                addToActionGroup(
                                    currentActionGroups,
                                    action,
                                    currentWaypoints.size - 1,
                                    currentWaypoints.size - 1
                                )
                            }
                            currentActionId += 1

                            action = createFocusAction(
                                0.25,
                                0.25,
                                0.5,
                                0.5,
                                false,
                                false,
                                componentIndex,
                                currentActionId
                            )

                            if (currentWaypoints.isEmpty()) {
                                currentStartActions.add(action)
                            } else {
                                addToActionGroup(
                                    currentActionGroups,
                                    action,
                                    currentWaypoints.size - 1,
                                    currentWaypoints.size - 1
                                )
                            }
                            currentActionId += 1
                        }
                    }

                    MavCmd.MAV_CMD_NAV_DELAY -> {
                        val delay = missionItem.param1()
                        if (delay == -1f) {
                            log.e("Cannot delay till specified time, only seconds are allowed!")
                            break
                        }

                        action = createHoverAction(delay.toDouble(), currentActionId)
                        if (currentWaypoints.isEmpty()) {
                            currentStartActions.add(action)
                        } else {
                            addToActionGroup(
                                currentActionGroups,
                                action,
                                currentWaypoints.size - 1,
                                currentWaypoints.size - 1
                            )
                        }
                        currentActionId += 1
                    }

                    MavCmd.MAV_CMD_CONDITION_YAW -> {
                        action = createRotateYawAction(
                            missionItem.param1().toDouble(),
                            if ((missionItem.param3() == 1.0f)) WaylineWaypointYawPathMode.CLOCKWISE else WaylineWaypointYawPathMode.COUNTER_CLOCKWISE,
                            currentActionId
                        )

                        if (currentWaypoints.isEmpty()) {
                            currentStartActions.add(action)
                        } else {
                            addToActionGroup(
                                currentActionGroups,
                                action,
                                currentWaypoints.size - 1,
                                currentWaypoints.size - 1
                            )
                        }
                        currentActionId += 1
                    }

                    MavCmd.MAV_CMD_CONDITION_DELAY -> {
                        action = createHoverAction(missionItem.param1().toDouble(), currentActionId)
                        if (currentWaypoints.isEmpty()) {
                            currentStartActions.add(action)
                        } else {
                            addToActionGroup(
                                currentActionGroups,
                                action,
                                currentWaypoints.size - 1,
                                currentWaypoints.size - 1
                            )
                        }
                        currentActionId += 1
                    }

                    MavCmd.MAV_CMD_DO_SET_CAM_TRIGG_DIST -> {
                        componentIndex = ComponentIndexType.LEFT_OR_MAIN
                        action = createSinglePhotoAction(componentIndex, currentActionId)

                        // Check if we have already started this action. If we already started this action, stop this
                        // action when: (1) param1=0 or (2) param1 has a different value than the action started. Ignore
                        // this action when param1 has the same value as the started action.
                        multipleWaypointsAction = runningActions[componentIndex]

                        // Check if the action exists and the interval matches the missionItem
                        if (multipleWaypointsAction?.interval == missionItem.param1().toDouble()) {
                            break
                        } else {
                            // Action exists but the interval doesn't match, handle it
                            multipleWaypointsAction?.let {
                                if (it.interval != missionItem.param1().toDouble()) {
                                    addToActionGroup(
                                        currentActionGroups,
                                        it.action!!,
                                        it.startId,
                                        currentWaypoints.size - 1,
                                        it.triggerType,
                                        it.interval
                                    )
                                    currentActionId += 1
                                    runningActions.remove(componentIndex)
                                }
                            }

                            // Case where multipleWaypointsAction is null or interval doesn't match the missionItem
                            if ((multipleWaypointsAction == null || multipleWaypointsAction.interval != missionItem.param1().toDouble()) && missionItem.param1() > 0) {
                                multipleWaypointsAction = MultipleWaypointsAction(
                                    action,
                                    MavCmd.MAV_CMD_IMAGE_START_CAPTURE,
                                    currentWaypoints.size - 1,
                                    WaylineActionTriggerType.MULTIPLE_DISTANCE,
                                    missionItem.param1().toDouble(),
                                    componentIndex
                                )
                                runningActions[componentIndex] = multipleWaypointsAction
                            }
                        }
                    }

                    MavCmd.MAV_CMD_IMAGE_START_CAPTURE -> {
                        componentIndex = ComponentIndexType.LEFT_OR_MAIN

                        action = createSinglePhotoAction(componentIndex, currentActionId)
                        if (missionItem.param3().roundToInt() == 1) {
                            if (currentWaypoints.isEmpty()) {
                                currentStartActions.add(action)
                            } else {
                                addToActionGroup(
                                    currentActionGroups,
                                    action,
                                    currentWaypoints.size - 1,
                                    currentWaypoints.size - 1
                                )
                            }
                            currentActionId += 1
                        } else {
                            if (missionItem.param3().roundToInt() > 1) {
                                log.w("Ignoring number of images in MAV_CMD_IMAGE_START_CAPTURE.")
                            }

                            if (runningActions.containsKey(componentIndex)) {
                                log.e("Cannot add MAV_CMD_IMAGE_START_CAPTURE because componentIndex $componentIndex is already in use!")
                                break
                            }

                            // Store the parameters for the action till we get a STOP_CAPTURE command,
                            // afterwards, we can add this as a MULTIPLE_TIMING action.
                            multipleWaypointsAction = MultipleWaypointsAction(
                                action,
                                MavCmd.MAV_CMD_IMAGE_START_CAPTURE,
                                currentWaypoints.size - 1,
                                WaylineActionTriggerType.MULTIPLE_TIMING,
                                missionItem.param2().toDouble(),
                                componentIndex
                            )
                            runningActions[componentIndex] = multipleWaypointsAction
                        }
                    }

                    MavCmd.MAV_CMD_IMAGE_STOP_CAPTURE -> {
                        componentIndex = ComponentIndexType.LEFT_OR_MAIN
                        multipleWaypointsAction = runningActions[componentIndex]

                        if (multipleWaypointsAction == null || multipleWaypointsAction.command != MavCmd.MAV_CMD_IMAGE_START_CAPTURE) {
                            log.w("Ignoring MAV_CMD_IMAGE_STOP_CAPTURE because capture is not started yet!")
                            break
                        }

                        // Add action in group that start at start waypoint and end at current waypoint,
                        // doesn't make sense as start action, so only add to action group
                        addToActionGroup(
                            currentActionGroups,
                            multipleWaypointsAction.action!!,
                            multipleWaypointsAction.startId,
                            currentWaypoints.size - 1,
                            multipleWaypointsAction.triggerType,
                            multipleWaypointsAction.interval
                        )
                        currentActionId += 1
                        runningActions.remove(componentIndex)
                    }

                    MavCmd.MAV_CMD_VIDEO_START_CAPTURE -> {
                        componentIndex = ComponentIndexType.LEFT_OR_MAIN
                        if (runningActions.containsKey(componentIndex)) {
                            log.e("Cannot add MAV_CMD_VIDEO_START_CAPTURE because componentIndex $componentIndex is already in use!")
                            break
                        }

                        action = createStartRecordAction(componentIndex, currentActionId)
                        if (currentWaypoints.isEmpty()) {
                            currentStartActions.add(action)
                        } else {
                            addToActionGroup(
                                currentActionGroups,
                                action,
                                currentWaypoints.size - 1,
                                currentWaypoints.size - 1
                            )
                        }
                        currentActionId += 1

                        multipleWaypointsAction = MultipleWaypointsAction(
                            MavCmd.MAV_CMD_VIDEO_START_CAPTURE,
                            componentIndex
                        )
                        runningActions[componentIndex] = multipleWaypointsAction
                    }

                    MavCmd.MAV_CMD_VIDEO_STOP_CAPTURE -> {
                        componentIndex = ComponentIndexType.LEFT_OR_MAIN
                        multipleWaypointsAction = runningActions[componentIndex]

                        if (multipleWaypointsAction == null || multipleWaypointsAction.command != MavCmd.MAV_CMD_VIDEO_START_CAPTURE) {
                            log.w("Ignoring MAV_CMD_VIDEO_STOP_CAPTURE because capture is not started yet!")
                            break
                        }

                        action = createStopRecordAction(componentIndex, currentActionId)

                        // Doesn't make sense to add as start action, so only add to action group
                        addToActionGroup(
                            currentActionGroups,
                            action,
                            currentWaypoints.size - 1,
                            currentWaypoints.size - 1
                        )
                        currentActionId += 1
                        runningActions.remove(componentIndex)
                    }

                    MavCmd.MAV_CMD_SET_CAMERA_ZOOM -> {
                        componentIndex = ComponentIndexType.LEFT_OR_MAIN
                        action = createZoomAction(
                            missionItem.param2().toDouble(),
                            componentIndex,
                            currentActionId
                        )
                        if (currentWaypoints.isEmpty()) {
                            currentStartActions.add(action)
                        } else {
                            addToActionGroup(
                                currentActionGroups,
                                action,
                                currentWaypoints.size - 1,
                                currentWaypoints.size - 1
                            )
                        }
                        currentActionId += 1
                    }

                    MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW -> {
                        componentIndex = ComponentIndexType.LEFT_OR_MAIN
                        action = createGimbalRotateAction(
                            missionItem.param1().toDouble(),
                            missionItem.param2().toDouble(),
                            EnumValue.create(GimbalManagerFlags::class.java, missionItem.x()),
                            componentIndex,
                            currentActionId
                        )
                        if (currentWaypoints.isEmpty()) {
                            currentStartActions.add(action)
                        } else {
                            addToActionGroup(
                                currentActionGroups,
                                action,
                                currentWaypoints.size - 1,
                                currentWaypoints.size - 1
                            )
                        }
                        currentActionId += 1
                    }

                    MavCmd.MAV_CMD_DO_MOUNT_CONTROL -> {
                        componentIndex = ComponentIndexType.LEFT_OR_MAIN
                        action = createGimbalRotateAction(
                            missionItem.param1().toDouble(),
                            missionItem.param3().toDouble(),
                            EnumValue.create(GimbalManagerFlags.GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME),
                            componentIndex,
                            currentActionId
                        )
                        if (currentWaypoints.isEmpty()) {
                            currentStartActions.add(action)
                        } else {
                            addToActionGroup(
                                currentActionGroups,
                                action,
                                currentWaypoints.size - 1,
                                currentWaypoints.size - 1
                            )
                        }
                        currentActionId += 1
                    }

                    else -> log.e("Unknown Mavlink command: ${missionItem.command().entry().name}")
                }
            }

            // Stop all running actions if not stopped yet
            for (runningAction in runningActions.values) {
                when (runningAction.command) {
                    MavCmd.MAV_CMD_IMAGE_START_CAPTURE, MavCmd.MAV_CMD_DO_SET_CAM_TRIGG_DIST -> {
                        log.w("Did not receive " + runningAction.command.name + ", assuming to stop at last waypoint.")

                        addToActionGroup(
                            currentActionGroups,
                            runningAction.action!!,
                            runningAction.startId,
                            currentWaypoints.size - 1,
                            runningAction.triggerType,
                            runningAction.interval
                        )
                        currentActionId += 1
                    }

                    MavCmd.MAV_CMD_VIDEO_START_CAPTURE -> {
                        log.w("Did not receive MAV_CMD_VIDEO_STOP_CAPTURE, assuming to stop at last waypoint.")

                        action = createStopRecordAction(
                            runningAction.componentIndex,
                            currentActionId
                        )
                        addToActionGroup(
                            currentActionGroups,
                            action,
                            currentWaypoints.size - 1,
                            currentWaypoints.size - 1
                        )
                        currentActionId += 1
                    }

                    else -> throw MissionConverterException("Cannot stop running action of type " + runningAction.command.name).setLogger(
                        log
                    )
                }
            }

            // There should be minimal one waypoint specified
            if (currentWaypoints.isEmpty()) {
                throw NotEnoughWaypointsException("At least two waypoints are needed to create a mission (one if GPS is fixed)!").setLogger(
                    log
                )
            }

            // When only one waypoint is defined and the drone has GPS connection, use the current location as the
            // first waypoint. This is needed because DJI don't accept missions with only one waypoint.
            connectionWR.get()?.mavlinkComponents?.drone?.aircraftLocation?.let { droneLocation ->
                if (currentWaypoints.size == 1) {
                    // Increase waypoint number and fix start and end indices for action groups
                    currentWaypoints[0].waypointIndex = 1
                    currentActionGroups.forEach {
                        it.startIndex += 1
                        it.endIndex += 1
                    }

                    location = WaylineLocationCoordinate2D()
                    if (shouldAddWaypointHalfway) {
                        location.latitude = (droneLocation.latitude + currentWaypoints[0].location.latitude) / 2
                        location.longitude = (droneLocation.longitude + currentWaypoints[0].location.longitude) / 2

                        val distance = getDistanceInMeterFromTwoGPSLocations(
                            currentWaypoints[0].location.latitude,
                            currentWaypoints[0].location.longitude,
                            location.latitude,
                            location.longitude
                        )
                        log.w(
                            "Only received one waypoint. Adding waypoint halfway current " +
                                    "GPS position and goal position " +
                                    "(distance=${distance.roundToInt()}m)!"
                        )
                    } else {
                        location.latitude = droneLocation.latitude // halfWayLatitude
                        location.longitude = droneLocation.longitude // halfWayLongitude
                    }

                    waypoint = WaylineExecuteWaypoint(
                        0,
                        location,
                        currentWaypoints[0].executeHeight,
                        currentWaypoints[0].yawParam,
                        currentWaypoints[0].gimbalHeadingParam,
                        currentWaypoints[0].turnParam,
                        currentWaypoints[0].speed,
                        true,
                        false,
                        currentWaypoints[0].waypointWorkType
                    )

                    // Add add beginning of list
                    currentWaypoints.add(0, waypoint)

                    // Add to map using the same mavlink id as the copied waypoint and changes the first item to -1
                    djiToMavlinkIdMap[1] = djiToMavlinkIdMap[0]
                }
            }

            // DJI requires at least two waypoints
            if (currentWaypoints.size == 1) {
                throw NotEnoughWaypointsException("At least two waypoint is needed to create a mission!").setLogger(
                    log
                )
            }

            // When no MAV_CMD_NAV_TAKEOFF message received, assume takeoff location is the first waypoint
            // location.
            if (this.takeoffRefPoint == null) {
                this.takeoffRefPoint = WaylineLocationCoordinate3D().apply {
                    latitude = currentWaypoints[0].location.latitude
                    longitude = currentWaypoints[0].location.longitude
                    altitude = currentWaypoints[0].executeHeight
                }
            }

            this.wayline = Wayline().apply {
                waylineId = 0
                mode = WaylineExecuteAltitudeMode.RELATIVE_TO_START_POINT
                templateId = 0
                waypoints = currentWaypoints
                waylineStartActions = currentStartActions
                actionGroups = currentActionGroups
                autoFlightSpeed = currentGlobalSpeed
            }

            null
        }
    }

    override fun convertToMavlink(targetSystem: Int, targetComponent: Int): Completable {
        missionItems.clear()
        djiToMavlinkIdMap.clear()

        return Completable.fromCallable {
            if (wayline == null) {
                throw MissionConverterException("There is no wayline available!").setLogger(log)
            }
            val globalSpeed = wayline!!.autoFlightSpeed
            var currentSpeed = -2.0

            val waypoints = wayline!!.waypoints
            val actionGroups = wayline!!.actionGroups

            // Get action groups that are used each waypoint
            val actionGroupsByWaypointId: MutableMap<Int, MutableList<Int>> = HashMap()
            for (actionGroup in actionGroups) {
                for (i in actionGroup.startIndex..actionGroup.endIndex) {
                    actionGroupsByWaypointId.computeIfAbsent(i) { ArrayList() }
                        .add(actionGroup.groupId)
                }
            }

            // Set global speed
            val changeSpeedItem = createChangeSpeedItem(
                globalSpeed.toFloat(),
                missionItems.size,
                currentMavlinkId,
                targetSystem,
                targetComponent
            )
            missionItems.add(changeSpeedItem)

            // Set takeoff reference point is valid
            if (takeoffRefPoint != null && takeoffRefPoint!!.latitude != 0.0 && takeoffRefPoint!!.longitude != 0.0 && takeoffRefPoint!!.altitude != 0.0) {
                val takeOffItem = createTakeOffItem(
                    takeoffRefPoint!!,
                    missionItems.size,
                    currentMavlinkId,
                    targetSystem,
                    targetComponent
                )
                missionItems.add(takeOffItem)
            }

            // Set start actions
            for (action in wayline!!.waylineStartActions) {
                val startAction = actionToMavlinkItem(
                    action,
                    missionItems.size,
                    currentMavlinkId,
                    targetSystem,
                    targetComponent
                )

                if (startAction == null) {
                    throw MissionConverterException("Cannot convert action of type ${action.actionType} to Mavlink!").setLogger(
                        log
                    )
                }

                missionItems.add(startAction)
            }

            for (waypoint in waypoints) {
                val currentWaypointActionGroupIds: List<Int>? =
                    actionGroupsByWaypointId[waypoint.waypointIndex]

                val hoverTime = getHoverTime(actionGroups, currentWaypointActionGroupIds)

                var yawAngle = Float.NaN
                if (waypoint.yawParam.enableYawAngle) {
                    yawAngle = waypoint.yawParam.yawAngle.toFloat()
                }

                val waypointItem = createWaypointItem(
                    waypoint.location,
                    waypoint.executeHeight.toFloat(),
                    hoverTime,
                    yawAngle,
                    missionItems.size,
                    currentMavlinkId,
                    targetSystem,
                    targetComponent
                )
                missionItems.add(waypointItem)

                djiToMavlinkIdMap[waypoint.waypointIndex] = missionItems.size

                // If current speed is set, but new speed value is the global speed value, return to the global speed
                if (currentSpeed > 0 && (waypoint.speed == globalSpeed || waypoint.speed == 0.0)) {
                    val changeCurrentSpeedItem = createChangeSpeedItem(
                        globalSpeed.toFloat(),  // e.g. sending -2 should also work, but this is clearer
                        missionItems.size,
                        currentMavlinkId,
                        targetSystem,
                        targetComponent
                    )
                    missionItems.add(changeCurrentSpeedItem)
                    currentSpeed = -2.0
                } else if ((waypoint.speed != globalSpeed && waypoint.speed != 0.0) && (currentSpeed == -2.0 || waypoint.speed != currentSpeed)) {
                    // If there is no current speed set but the speed differs from the global speed or there is a current
                    // speed set, but this is different from the requested speed
                    val changeCurrentSpeedItem = createChangeSpeedItem(
                        waypoint.speed.toFloat(),
                        missionItems.size,
                        currentMavlinkId,
                        targetSystem,
                        targetComponent
                    )
                    missionItems.add(changeCurrentSpeedItem)
                    currentSpeed = waypoint.speed
                }

                for (currentWaypointActionGroup in actionGroups) {
                    if (currentWaypointActionGroupIds == null || !currentWaypointActionGroupIds.contains(
                            currentWaypointActionGroup.groupId
                        )
                    ) {
                        continue
                    }

                    for (waypointAction in currentWaypointActionGroup.actions) {
                        if (currentWaypointActionGroup.trigger.triggerType == WaylineActionTriggerType.MULTIPLE_TIMING) {
                            if (waypointAction.actionType == WaylineActionType.TAKE_PHOTO) {
                                if (currentWaypointActionGroup.startIndex == waypoint.waypointIndex) {
                                    // Start at interval image
                                    val startCaptureImagesItem = createStartImageCaptureItem(
                                        currentWaypointActionGroup.trigger.timeInterval.toFloat(),
                                        missionItems.size,
                                        currentMavlinkId,
                                        targetSystem,
                                        targetComponent
                                    )
                                    missionItems.add(startCaptureImagesItem)
                                }

                                if (currentWaypointActionGroup.endIndex == waypoint.waypointIndex) {
                                    // Stop image capturing
                                    val stopCaptureImagesItem = createStopImageCaptureItem(
                                        missionItems.size,
                                        currentMavlinkId,
                                        targetSystem,
                                        targetComponent
                                    )
                                    missionItems.add(stopCaptureImagesItem)
                                }
                            } else {
                                throw MissionConverterException("Could not convert action of type " + waypointAction.actionType + " of trigger type " + currentWaypointActionGroup.trigger.triggerType.name + " to Mavlink!").setLogger(
                                    log
                                )
                            }
                        } else if (currentWaypointActionGroup.trigger.triggerType == WaylineActionTriggerType.MULTIPLE_DISTANCE) {
                            if (waypointAction.actionType == WaylineActionType.TAKE_PHOTO) {
                                if (currentWaypointActionGroup.startIndex == waypoint.waypointIndex) {
                                    // Start distance trigger
                                    val camTriggerDistItem = createDoSetCamTriggDistItem(
                                        currentWaypointActionGroup.trigger.distanceInterval.toFloat(),
                                        missionItems.size,
                                        currentMavlinkId,
                                        targetSystem,
                                        targetComponent
                                    )
                                    missionItems.add(camTriggerDistItem)
                                }

                                if (currentWaypointActionGroup.endIndex == waypoint.waypointIndex) {
                                    // Stop distance trigger
                                    val camTriggerDistItem = createDoSetCamTriggDistItem(
                                        0f,
                                        missionItems.size,
                                        currentMavlinkId,
                                        targetSystem,
                                        targetComponent
                                    )
                                    missionItems.add(camTriggerDistItem)
                                }
                            }
                            throw MissionConverterException("Could not convert action of type " + waypointAction.actionType + " of trigger type " + currentWaypointActionGroup.trigger.triggerType.name + " to Mavlink!").setLogger(
                                log
                            )
                        } else if (currentWaypointActionGroup.trigger.triggerType == WaylineActionTriggerType.REACH_POINT) {
                            if (waypointAction.actionType != WaylineActionType.HOVER) { // Hover action should be in waypoint
                                val actionItem = actionToMavlinkItem(
                                    waypointAction,
                                    missionItems.size,
                                    currentMavlinkId,
                                    targetSystem,
                                    targetComponent
                                )

                                if (actionItem == null) {
                                    throw MissionConverterException("Cannot convert action of type ${waypointAction.actionType} to Mavlink!").setLogger(
                                        log
                                    )
                                }

                                missionItems.add(actionItem)
                            }
                        }
                    }
                }
            }

            // End with finish action
            if (finishAction == WaylineFinishedAction.GO_HOME) {
                val returnToLaunchItem = createReturnToLaunchItem(
                    missionItems.size,
                    currentMavlinkId,
                    targetSystem,
                    targetComponent
                )
                missionItems.add(returnToLaunchItem)
            } else if (finishAction == WaylineFinishedAction.AUTO_LAND) {
                val seq: Int
                // If the last mission item is a waypoint, re-use this waypoint and change it into a landing waypoint
                // by removing this from the list of waypoints and re-adding it as land item. The djiIdToMavlinkIdMap
                // should then still be correct.
                if (missionItems[missionItems.size - 1].command()
                        .entry() == MavCmd.MAV_CMD_NAV_WAYPOINT
                ) {
                    seq = missionItems[missionItems.size - 1].seq()
                    missionItems.removeAt(missionItems.size - 1)
                } else {
                    seq = missionItems.size
                }

                val lastWaypoint = waypoints[waypoints.size - 1]
                var yawAngle = Float.NaN
                if (lastWaypoint.yawParam.enableYawAngle) {
                    yawAngle = lastWaypoint.yawParam.yawAngle.toFloat()
                }

                val landItem = createLandItem(
                    lastWaypoint.location,
                    yawAngle,
                    seq,
                    currentMavlinkId,
                    targetSystem,
                    targetComponent
                )
                missionItems.add(landItem)
            }

            // Set takeoff point to first waypoint when not defined
            if (takeoffRefPoint == null) {
                takeoffRefPoint = WaylineLocationCoordinate3D()
                takeoffRefPoint!!.latitude = waypoints[0].location.latitude
                takeoffRefPoint!!.longitude = waypoints[0].location.longitude
                takeoffRefPoint!!.altitude = waypoints[0].executeHeight
            }
            null
        }
    }

    override fun writeToFile(kmz: KMZ): Completable {
        this.kmz = kmz

        return Completable.fromCallable {
            writeWaylineFile(kmz.waylineFilePath)
            writeTemplateFile(kmz.templateFilePath)
            null
        }
    }

    @get:Suppress("deprecation")
    override val allowedCommands: List<MavCmd>
        get() = listOf(
            MavCmd.MAV_CMD_NAV_TAKEOFF,
            MavCmd.MAV_CMD_NAV_LAND,
            MavCmd.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            MavCmd.MAV_CMD_NAV_WAYPOINT,
            MavCmd.MAV_CMD_NAV_DELAY,
            MavCmd.MAV_CMD_DO_CHANGE_SPEED,
            MavCmd.MAV_CMD_CONDITION_YAW,
            MavCmd.MAV_CMD_CONDITION_DELAY,
            MavCmd.MAV_CMD_IMAGE_START_CAPTURE,
            MavCmd.MAV_CMD_IMAGE_STOP_CAPTURE,
            MavCmd.MAV_CMD_VIDEO_START_CAPTURE,
            MavCmd.MAV_CMD_VIDEO_STOP_CAPTURE,
            MavCmd.MAV_CMD_DO_SET_CAM_TRIGG_DIST,
            MavCmd.MAV_CMD_SET_CAMERA_FOCUS,
            MavCmd.MAV_CMD_SET_CAMERA_ZOOM,
            MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            MavCmd.MAV_CMD_DO_MOUNT_CONTROL,
            MavCmd.MAV_CMD_NAV_TAKEOFF,
            MavCmd.MAV_CMD_NAV_WAYPOINT,
            MavCmd.MAV_CMD_NAV_DELAY,
            MavCmd.MAV_CMD_IMAGE_START_CAPTURE,
            MavCmd.MAV_CMD_IMAGE_STOP_CAPTURE,
            MavCmd.MAV_CMD_VIDEO_START_CAPTURE,
            MavCmd.MAV_CMD_VIDEO_STOP_CAPTURE,
            MavCmd.MAV_CMD_SET_CAMERA_ZOOM,
            MavCmd.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
        )

    @Throws(RxException::class)
    private fun readFromFile(kmzPath: String) {
        val parsedMission =
            JNIWPMZManager.getWaylineMissionConfig(KMZ.WAYPOINT_FILE_VERSION, kmzPath)
        val parsedTemplates = JNIWPMZManager.getWaylineTemplates(KMZ.WAYPOINT_FILE_VERSION, kmzPath)
        val parsedWaylines = JNIWPMZManager.getWaylines(KMZ.WAYPOINT_FILE_VERSION, kmzPath)

        if (parsedMission.error.value != WPMLParseError.NO_ERROR) {
            throw MissionConverterException("Cannot parse mission config: " + parsedMission.error.value.name).setLogTag(
                log.tag()
            )
        }

        if (parsedTemplates.error.value != WPMLParseError.NO_ERROR) {
            throw MissionConverterException("Cannot parse templates: " + parsedTemplates.error.value.name).setLogTag(
                log.tag()
            )
        }


        if (parsedWaylines.error.value != WPMLParseError.NO_ERROR) {
            throw MissionConverterException("Cannot parse waylines: " + parsedWaylines.error.value.name).setLogTag(
                log.tag()
            )
        }

        if (parsedWaylines.waylines == null) {
            throw MissionConverterException("Cannot set current mission: there are no waylines!").setLogTag(
                log.tag()
            )
        }

        if (parsedWaylines.waylines.size > 1) {
            throw MissionConverterException("Cannot set current mission: only missions with one wayline are supported!").setLogTag(
                log.tag()
            )
        }

        wayline = parsedWaylines.waylines[0]
        takeoffRefPoint = parsedMission.config.takeOffPositionRef
        finishAction = parsedMission.config.finishAction
    }

    @Throws(RxException::class)
    private fun writeWaylineFile(filename: String) {
        if (wayline == null) {
            throw WriteException("Cannot save mission because there is no wayline!").setLogger(log)
        }

        val content: MutableMap<String, Any> = HashMap(
            missionConfigObjectMap
        )
        content["autoFlightSpeed"] = wayline!!.autoFlightSpeed

        content["hasStartActions"] = false
        if (!wayline!!.waylineStartActions.isEmpty()) {
            content["hasStartActions"] = true
            content["startActions"] = actionsToObjectMap(wayline!!.waylineStartActions)
        }

        val waypoints: MutableList<Map<String, Any>> = ArrayList()
        for (i in wayline!!.waypoints.indices) {
            val waypoint = wayline!!.waypoints[i]

            val waypointObject: MutableMap<String, Any> = HashMap()

            // Coordinates are in format longitude,latitude (note that this differs from the TakeOffRefPoint!)
            waypointObject["waypointCoordinates"] = String.format(
                Locale.US,
                "%.12f,%.12f",
                waypoint.location.longitude,
                waypoint.location.latitude
            )
            waypointObject["waypointIndex"] = waypoint.waypointIndex
            waypointObject["waypointAltitude"] = waypoint.executeHeight

            // Custom speed
            if (waypoint.speed == 0.0) {
                waypointObject["waypointUsesGlobalSpeed"] = true
            } else {
                waypointObject["waypointUsesGlobalSpeed"] = false
                waypointObject["waypointSpeed"] = waypoint.speed
            }

            // Custom waypoint heading
            if (waypoint.yawParam.enableYawAngle) {
                waypointObject["waypointHeadingCustom"] = true
                waypointObject["waypointHeadingMode"] =
                    convertToCamelCase(waypoint.yawParam.yawMode.name)
                waypointObject["waypointHeadingAngle"] = waypoint.yawParam.yawAngle
                waypointObject["waypointHeadingAngleEnable"] = 1
            } else {
                waypointObject["waypointHeadingCustom"] = false
            }

            // Custom waypoint turn mode
            if (waypoint.turnParam.turnMode != WaylineWaypointTurnMode.TO_POINT_AND_STOP_WITH_DISCONTINUITY_CURVATURE || waypoint.turnParam.turnDampingDistance != 0.0) {
                waypointObject["waypointTurnCustom"] = true
                waypointObject["waypointTurnMode"] =
                    convertToCamelCase(waypoint.turnParam.turnMode.name)
                waypointObject["waypointTurnDampingDist"] = waypoint.turnParam.turnDampingDistance
            } else {
                waypointObject["waypointTurnCustom"] = false
            }

            waypointObject["waypointUseStraightLine"] = if (waypoint.useStraightLine) 1 else 0

            val waypointActionGroups: MutableList<Map<String, Any>> = ArrayList()
            for (actionGroup in wayline!!.actionGroups) {
                if (actionGroup.startIndex != waypoint.waypointIndex) {
                    continue
                }

                val waypointActionGroupObject: MutableMap<String, Any> = HashMap()
                waypointActionGroupObject["actionGroupId"] = actionGroup.groupId
                waypointActionGroupObject["actionGroupStartIndex"] = actionGroup.startIndex
                waypointActionGroupObject["actionGroupEndIndex"] = actionGroup.endIndex
                waypointActionGroupObject["actionGroupTriggerType"] =
                    convertToCamelCase(actionGroup.trigger.triggerType.name)
                waypointActionGroupObject["actionGroupTriggerHasParam"] = false
                if (actionGroup.trigger.triggerType == WaylineActionTriggerType.MULTIPLE_DISTANCE) {
                    waypointActionGroupObject["actionGroupTriggerHasParam"] = true
                    waypointActionGroupObject["actionGroupTriggerParam"] =
                        actionGroup.trigger.distanceInterval
                }
                if (actionGroup.trigger.triggerType == WaylineActionTriggerType.MULTIPLE_TIMING) {
                    waypointActionGroupObject["actionGroupTriggerHasParam"] = true
                    waypointActionGroupObject["actionGroupTriggerParam"] =
                        actionGroup.trigger.timeInterval
                }
                waypointActionGroupObject["actionGroupActions"] =
                    actionsToObjectMap(actionGroup.actions)
                waypointActionGroups.add(waypointActionGroupObject)
            }
            waypointObject["waypointActionGroups"] = waypointActionGroups

            waypoints.add(waypointObject)
        }
        content["waypoints"] = waypoints

        writeFileUsingTemplate("templates/wayline.template.wpml", filename, content, "wayline", log)
    }

    @Throws(RxException::class)
    private fun writeTemplateFile(filename: String) {
        if (takeoffRefPoint == null) {
            throw WriteException("Cannot generate template because takeoff reference point is not defined!").setLogger(
                log
            )
        }

        val content: MutableMap<String, Any> = HashMap(
            missionConfigObjectMap
        )

        content["author"] = ContextUtil.getContext().getText(R.string.mavdrone_app_name)
        content["updateTime"] = System.currentTimeMillis()

        // Reference TakeOff Point is only used for route planning and is defined as x,y,z where x = latitude,
        // y = longitude and z = altitude (note that this is different than for the coordinates!)
        content["takeOffRefPoint"] = with(takeoffRefPoint!!) {
            String.format(Locale.US, "%.12f,%.12f,%.2f", latitude, longitude, altitude)
        }

        content["takeOffRefPointAGLHeight"] = takeoffRefPoint!!.altitude
        content["globalTransitionalSpeed"] = wayline!!.waypoints[0].speed  // Speed towards first waypoint

        writeFileUsingTemplate(
            "templates/template.template.kml",
            filename,
            content,
            "template",
            log
        )
    }

    fun getDjiToMavlinkIdMap(): Map<Int, Int?> {
        return djiToMavlinkIdMap
    }

    fun pushToAircraftUnsafe(): Completable {
        return createKmzWhenNeeded()
            .flatMapCompletable { kmz: KMZ ->
                writeToFile(kmz)
                    .andThen(saveGeofenceMissionWhenAvailable(kmz))
                    .andThen(kmz.createKMZ())
                    .andThen(
                        djiCompletableFunctionWithProgress(
                            { missionFilePath: String?, callback: CompletionCallbackWithProgress<Double>? ->
                                WaypointMissionManager.getInstance()
                                    .pushKMZFileToAircraft(missionFilePath, callback)
                            },
                            { progress -> this.uploadProgress(progress) },
                            kmz::path
                        )
                    )
            }
    }

    fun pushToAircraft(): Completable {
        return createKmzWhenNeeded()
            .flatMapCompletable { kmz: KMZ ->
                writeToFile(kmz)
                    .andThen(saveGeofenceMissionWhenAvailable(kmz))
                    .andThen(kmz.createKMZ())
                    .andThen(kmz.checkIfValid())
                    .andThen(
                        djiCompletableFunctionWithProgress(
                            { missionFilePath: String?, callback: CompletionCallbackWithProgress<Double>? ->
                                WaypointMissionManager.getInstance()
                                    .pushKMZFileToAircraft(missionFilePath, callback)
                            },
                            { progress -> this.uploadProgress(progress) },
                            kmz::path
                        )
                    )
            }
    }

    private fun createKmzWhenNeeded(): Single<KMZ> {
        return Single.fromCallable {
            kmz ?: create(connectionWR).also { kmz = it }
        }
    }

    private fun saveGeofenceMissionWhenAvailable(kmzFile: KMZ): Completable {
        val geofenceMission = missionManagerWR.get()?.currentGeofenceMission
        if (geofenceMission != null) {
            return geofenceMission.writeToFile(kmzFile)
        }

        // Remove file when it exists from a previous flight
        return Completable.fromCallable {
            val geofenceFile = File(kmzFile.geofenceFilePath)
            if (geofenceFile.exists()) {
                geofenceFile.delete()
            }
            null
        }
    }

    fun uploadProgress(progress: Double) {
        kmz?.let {
            log.i("Uploading mission '${it.name}': $progress%")
        }
    }

    private val missionConfigObjectMap: Map<String, Any>
        get() {
            val missionConfigObject: MutableMap<String, Any> = HashMap()
            missionConfigObject["finishAction"] = convertToCamelCase(finishAction.name)

            val droneTypeInformation = connectionWR.get()?.let { connection ->
                DRONE_TYPE_INFORMATION.entries.find { (key, _) ->
                    val productType = key.first
                    val cameraType = key.second

                    // Default camera when no camera is connected
                    if (productType == connection.mavlinkComponents!!.drone.productType && cameraType == null) {
                        return@find true
                    }

                    // If there is a camera connected
                    if (connection.mavlinkComponents!!.cameras.isNotEmpty() && connection.mavlinkComponents!!.cameras[0].cameraType == cameraType) {
                        return@find true
                    }

                    return@find false
                }?.value ?: run {
                    log.e("Could not retrieve current drone information, assuming DJI M350!")
                    TypeAndSubTypeInformation(89)
                }
            } ?: run {
                log.e("Could not get connection reference, assuming DJI M350!")
                TypeAndSubTypeInformation(89)
            }

            missionConfigObject["droneEnumValue"] = droneTypeInformation.typeValue
            missionConfigObject["droneEnumSubValue"] = droneTypeInformation.subTypeValue

            val payloads: MutableList<Map<String, Any>> = ArrayList()
            connectionWR.get()?.let {
                for (camera in it.mavlinkComponents!!.cameras) {
                    var cameraTypeInformation: TypeAndSubTypeInformation? = null
                    for ((key, value) in CAMERA_TYPE_INFORMATION) {
                        val cameraType = key.first
                        val focalLength = key.second

                        if (cameraType == camera.cameraType) {
                            if (focalLength == null) {
                                cameraTypeInformation = value
                                break
                            }

                            // TODO: get focal length somehow from the camera
                            if (focalLength == 35) {
                                cameraTypeInformation = value
                                break
                            }
                        }
                    }

                    if (cameraTypeInformation == null) {
                        log.e("Could not retrieve current drone information, assuming DJI Zenmuse P1 35mm camera!")
                        cameraTypeInformation = TypeAndSubTypeInformation(50, 1)
                    }

                    var payloadPositionIndex = PAYLOAD_POSITION_INFORMATION[camera.componentIndex]
                    if (payloadPositionIndex == null) {
                        log.e("Could not retrieve payload position index for camera ${camera.componentIndex}, assuming main camera")
                        payloadPositionIndex = 0
                    }

                    val cameraObject: MutableMap<String, Any> = HashMap()
                    cameraObject["payloadEnumValue"] = cameraTypeInformation.typeValue
                    cameraObject["payloadEnumSubValue"] = cameraTypeInformation.subTypeValue
                    cameraObject["payloadPositionIndex"] = payloadPositionIndex
                    payloads.add(cameraObject)

                }
            }

            missionConfigObject["payloads"] = payloads
            return missionConfigObject
        }

    private fun actionsToObjectMap(actions: List<WaylineActionInfo>): List<Map<String, Any>> {
        val actionObjects: MutableList<Map<String, Any>> = ArrayList()
        for (actionInfo in actions) {
            val actionObject: MutableMap<String, Any> = HashMap()
            actionObject["actionId"] = actionInfo.actionId
            actionObject["actionName"] = convertToCamelCase(actionInfo.actionType.name)

            actionObject["isStartRecordAction"] = false
            actionObject["isStopRecordAction"] = false
            actionObject["isSetFocusAction"] = false
            actionObject["isFocusAction"] = false
            actionObject["isZoomAction"] = false
            actionObject["isCustomDirAction"] = false
            actionObject["isGimbalRotateAction"] = false
            actionObject["isGimbalEvenlyRotateAction"] = false
            actionObject["isRotateYawAction"] = false
            actionObject["isHoverAction"] = false
            when (actionInfo.actionType) {
                WaylineActionType.TAKE_PHOTO -> {
                    actionObject["isStartRecordAction"] = true
                    actionObject["actionPayloadPositionIndex"] =
                        actionInfo.takePhotoParam.payloadPositionIndex
                    actionObject["actionFileSuffix"] = actionInfo.takePhotoParam.fileSuffix
                    actionObject["actionPayloadLensIndex"] =
                        payloadLensIndexToString(actionInfo.takePhotoParam.payloadLensIndex)
                }

                WaylineActionType.START_RECORD -> {
                    actionObject["isStartRecordAction"] = true
                    actionObject["actionPayloadPositionIndex"] =
                        actionInfo.startRecordParam.payloadPositionIndex
                    actionObject["actionFileSuffix"] = actionInfo.startRecordParam.fileSuffix
                    actionObject["actionPayloadLensIndex"] =
                        payloadLensIndexToString(actionInfo.takePhotoParam.payloadLensIndex)
                }

                WaylineActionType.STOP_RECORD -> {
                    actionObject["isStopRecordAction"] = true
                    actionObject["actionPayloadPositionIndex"] =
                        actionInfo.stopRecordParam.payloadPositionIndex
                    actionObject["actionPayloadLensIndex"] =
                        payloadLensIndexToString(actionInfo.takePhotoParam.payloadLensIndex)
                }

                WaylineActionType.SET_FOCUS_TYPE -> {
                    actionObject["isSetFocusAction"] = true
                    actionObject["actionPayloadPositionIndex"] =
                        actionInfo.setFocusTypeParam.payloadPositionIndex
                    actionObject["actionCameraFocusType"] =
                        convertToCamelCase(actionInfo.setFocusTypeParam.focusType.name)
                }

                WaylineActionType.FOCUS -> {
                    actionObject["isFocusAction"] = true
                    actionObject["actionPayloadPositionIndex"] =
                        actionInfo.focusParam.payloadPositionIndex
                    actionObject["actionIsPointFocus"] =
                        if (actionInfo.focusParam.isPointFocus) 1 else 0
                    actionObject["actionFocusX"] = actionInfo.focusParam.focus_x
                    actionObject["actionFocusY"] = actionInfo.focusParam.focus_y
                    actionObject["actionFocusRegionWidth"] = actionInfo.focusParam.width
                    actionObject["actionFocusRegionHeight"] = actionInfo.focusParam.height
                    actionObject["actionIsInfiniteFocus"] =
                        if (actionInfo.focusParam.isInfiniteFocus) 1 else 0
                }

                WaylineActionType.ZOOM -> {
                    actionObject["isZoomAction"] = true
                    actionObject["actionPayloadPositionIndex"] =
                        actionInfo.zoomParam.payloadPositionIndex
                    actionObject["actionFocalLength"] = actionInfo.zoomParam.focalLength
                }

                WaylineActionType.CUSTOM_DIR_NAME -> {
                    actionObject["isCustomDirAction"] = true
                    actionObject["actionPayloadPositionIndex"] =
                        actionInfo.customDirNameParam.payloadPositionIndex
                    actionObject["actionDirectoryName"] =
                        actionInfo.customDirNameParam.directoryName
                }

                WaylineActionType.GIMBAL_ROTATE -> {
                    actionObject["isGimbalRotateAction"] = true
                    actionObject["actionPayloadPositionIndex"] =
                        actionInfo.gimbalRotateParam.payloadPositionIndex
                    actionObject["actionGimbalHeadingYawBase"] =
                        convertToCamelCase(actionInfo.gimbalRotateParam.base.name)
                    actionObject["actionGimbalRotateMode"] =
                        convertToCamelCase(actionInfo.gimbalRotateParam.rotateMode.name)
                    actionObject["actionGimbalPitchRotateEnable"] =
                        if (actionInfo.gimbalRotateParam.enablePitch) 1 else 0
                    actionObject["actionGimbalPitchRotateAngle"] =
                        actionInfo.gimbalRotateParam.pitch
                    actionObject["actionGimbalRollRotateEnable"] =
                        if (actionInfo.gimbalRotateParam.enableRoll) 1 else 0
                    actionObject["actionGimbalRollRotateAngle"] = actionInfo.gimbalRotateParam.roll
                    actionObject["actionGimbalYawRotateEnable"] =
                        if (actionInfo.gimbalRotateParam.enableYaw) 1 else 0
                    actionObject["actionGimbalYawRotateAngle"] = actionInfo.gimbalRotateParam.yaw
                    actionObject["actionGimbalRotateTimeEnable"] =
                        if (actionInfo.gimbalRotateParam.enableRotateTime) 1 else 0
                    actionObject["actionGimbalRotateTime"] = actionInfo.gimbalRotateParam.rotateTime
                }

                WaylineActionType.GIMBAL_EVENLY_ROTATE -> {
                    actionObject["isGimbalRotateAction"] = true
                    actionObject["actionPayloadPositionIndex"] =
                        actionInfo.gimbalEvenlyRotateParam.payloadPositionIndex
                    actionObject["actionGimbalPitchRotateAngle"] =
                        actionInfo.gimbalEvenlyRotateParam.pitch
                }

                WaylineActionType.ROTATE_YAW -> {
                    actionObject["isRotateYawAction"] = true
                    actionObject["actionAircraftHeading"] =
                        actionInfo.aircraftRotateYawParam.heading
                    actionObject["actionAircraftPathMode"] =
                        convertToCamelCase(actionInfo.aircraftRotateYawParam.pathMode.name)
                }

                WaylineActionType.HOVER -> {
                    actionObject["isHoverAction"] = true
                    actionObject["actionHoverTime"] = actionInfo.aircraftHoverParam.hoverTime
                }

                else -> {}
            }
            actionObjects.add(actionObject)
        }

        return actionObjects
    }

    private fun addToActionGroup(
        actionGroups: MutableList<WaylineActionGroup>,
        actionInfo: WaylineActionInfo,
        startIndex: Int,
        endIndex: Int,
        triggerType: WaylineActionTriggerType = WaylineActionTriggerType.REACH_POINT,
        interval: Double = -1.0
    ) {
        var actionGroup: WaylineActionGroup? = null

        // Check actionGroups if there is an action group that contains the combination of
        // triggerType and interval with the same start and end index
        for (ag in actionGroups) {
            if (ag.startIndex == startIndex && ag.endIndex == endIndex) {
                if (triggerType == WaylineActionTriggerType.REACH_POINT
                    && ag.trigger.triggerType == triggerType
                ) {
                    actionGroup = ag
                    actionGroups.remove(actionGroup)
                    break
                } else if (triggerType == WaylineActionTriggerType.MULTIPLE_DISTANCE && ag.trigger.triggerType == triggerType && ag.trigger.distanceInterval == interval) {
                    actionGroup = ag
                    actionGroups.remove(actionGroup)
                    break
                } else if (triggerType == WaylineActionTriggerType.MULTIPLE_TIMING && ag.trigger.triggerType == triggerType && ag.trigger.timeInterval == interval) {
                    actionGroup = ag
                    actionGroups.remove(actionGroup)
                    break
                }
            }
        }

        if (actionGroup == null) {
            actionGroup =
                createActionGroup(actionGroups.size, triggerType, interval, startIndex, endIndex)
        }

        val currentActions = actionGroup.actions
        currentActions.add(actionInfo)
        actionGroup.actions = currentActions

        actionGroups.add(actionGroup)
    }

    private fun getHoverTime(
        actionGroups: List<WaylineActionGroup>,
        waypointActionGroupIds: List<Int>?
    ): Float {
        var hoverTime = 0f
        for (actionGroup in actionGroups) {
            if (waypointActionGroupIds == null || !waypointActionGroupIds.contains(actionGroup.groupId)) {
                continue
            }

            for (waypointAction in actionGroup.actions) {
                if (waypointAction.actionType == WaylineActionType.HOVER) {
                    hoverTime += waypointAction.aircraftHoverParam.hoverTime.toFloat()
                }
            }
        }

        return hoverTime
    }


    private class MultipleWaypointsAction(
        val action: WaylineActionInfo?,
        val command: MavCmd,
        val startId: Int,
        val triggerType: WaylineActionTriggerType,
        val interval: Double,
        val componentIndex: ComponentIndexType
    ) {
        constructor(command: MavCmd, componentIndex: ComponentIndexType) : this(
            action = null,
            command = command,
            startId = -1,
            triggerType = WaylineActionTriggerType.UNKNOWN,
            interval = -1.0,
            componentIndex = componentIndex
        )
    }

    companion object {
        fun fromKMZ(
            connectionWR: WeakReference<MavConnection>,
            missionManagerWR: WeakReference<MissionManager>,
            kmz: KMZ
        ): Single<WaypointMission> {
            return Single.fromCallable {
                val mission = WaypointMission(connectionWR, missionManagerWR)
                mission.readFromFile(kmz.path)
                mission.kmz = kmz
                mission
            }
        }

        private fun payloadLensIndexToString(list: List<CameraLensType>): String {
            val sb = StringBuilder()
            for (i in list.indices) {
                sb.append(list[i].name.lowercase(Locale.getDefault()))
                if (i < list.size - 1) {
                    sb.append(",")
                }
            }

            return sb.toString()
        }

        private fun convertToCamelCase(input: String): String {
            val result = StringBuilder()
            for (word in input.split("_".toRegex()).dropLastWhile { it.isEmpty() }.toTypedArray()) {
                result.append(word.substring(0, 1).uppercase(Locale.getDefault())).append(
                    word.substring(1).lowercase(
                        Locale.getDefault()
                    )
                )
            }

            // First character should be lowercase
            result.replace(0, 1, result.substring(0, 1).lowercase(Locale.getDefault()))
            return result.toString()
        }

        private fun createActionGroup(
            id: Int,
            triggerType: WaylineActionTriggerType,
            interval: Double,
            startIndex: Int,
            endIndex: Int
        ): WaylineActionGroup {
            val trigger = WaylineActionTrigger()
            trigger.triggerType = triggerType

            if (triggerType == WaylineActionTriggerType.MULTIPLE_DISTANCE) {
                trigger.distanceInterval = interval
            } else if (triggerType == WaylineActionTriggerType.MULTIPLE_TIMING) {
                trigger.timeInterval = interval
            }

            val actionGroup = WaylineActionGroup()
            actionGroup.groupId = id
            actionGroup.trigger = trigger
            actionGroup.startIndex = startIndex
            actionGroup.endIndex = endIndex
            return actionGroup
        }

        class TypeAndSubTypeInformation(val typeValue: Int, val subTypeValue: Int) {
            constructor(typeValue: Int) : this (
                typeValue = typeValue,
                subTypeValue = -1
            )
        }

        // TODO: Add more drone and camera types here
        // https://developer.dji.com/doc/cloud-api-tutorial/en/api-reference/dji-wpml/common-element.html
        // Pair<ProductType, CameraType>
        val DRONE_TYPE_INFORMATION: Map<Pair<ProductType, CameraType?>, TypeAndSubTypeInformation> = mapOf(
            Pair(ProductType.M350_RTK, null) to TypeAndSubTypeInformation(89),
            Pair(ProductType.M300_RTK, null) to TypeAndSubTypeInformation(60),
            Pair(ProductType.M30_SERIES, CameraType.M30) to TypeAndSubTypeInformation(67, 0),
            Pair(ProductType.M30_SERIES, CameraType.M30T) to TypeAndSubTypeInformation(67, 1),
            Pair(ProductType.DJI_MAVIC_3_ENTERPRISE_SERIES, CameraType.M3E) to TypeAndSubTypeInformation(77, 0),
            Pair(ProductType.DJI_MAVIC_3_ENTERPRISE_SERIES, CameraType.M3T) to TypeAndSubTypeInformation(77, 1),
            Pair(ProductType.DJI_MAVIC_3_ENTERPRISE_SERIES, CameraType.M3M) to TypeAndSubTypeInformation(77, 2),
        )

        // Pair<CameraType, FocalLength>
        val CAMERA_TYPE_INFORMATION: Map<Pair<CameraType, Int?>, TypeAndSubTypeInformation> = mapOf(
            Pair(CameraType.ZENMUSE_H20, null) to TypeAndSubTypeInformation(42),
            Pair(CameraType.ZENMUSE_H20T, null) to TypeAndSubTypeInformation(43),
            Pair(CameraType.ZENMUSE_P1, 24) to TypeAndSubTypeInformation(50, 0),
            Pair(CameraType.ZENMUSE_P1, 35) to TypeAndSubTypeInformation(50, 1),
            Pair(CameraType.ZENMUSE_P1, 50) to TypeAndSubTypeInformation(50, 2),
            Pair(CameraType.M30, null) to TypeAndSubTypeInformation(52),
            Pair(CameraType.M30T, null) to TypeAndSubTypeInformation(53),
            Pair(CameraType.ZENMUSE_H20N, null) to TypeAndSubTypeInformation(61),
            Pair(CameraType.ZENMUSE_L1, null) to TypeAndSubTypeInformation(90742),
            Pair(CameraType.MAVIC_3, null) to TypeAndSubTypeInformation(66),
        )

        val PAYLOAD_POSITION_INFORMATION: Map<ComponentIndexType, Int> = mapOf(
            ComponentIndexType.LEFT_OR_MAIN to 0,
            ComponentIndexType.RIGHT to 1,
            ComponentIndexType.UP to 2,
        )
    }
}
