package nl.wur.mavdrone.communication.components.impl

import java.lang.ref.WeakReference

import dji.sdk.keyvalue.value.flightcontroller.FlightMode
import dji.v5.common.error.IDJIError
import dji.v5.manager.aircraft.waypoint3.WaylineExecutingInfoListener
import dji.v5.manager.aircraft.waypoint3.WaypointActionListener
import dji.v5.manager.aircraft.waypoint3.WaypointMissionExecuteStateListener
import dji.v5.manager.aircraft.waypoint3.WaypointMissionManager
import dji.v5.manager.aircraft.waypoint3.model.WaylineExecutingInfo
import dji.v5.manager.aircraft.waypoint3.model.WaypointMissionExecuteState

import io.reactivex.rxjava3.core.Completable
import io.reactivex.rxjava3.schedulers.Schedulers

import io.dronefleet.mavlink.MavlinkMessage
import io.dronefleet.mavlink.common.CommandLong
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.MissionCurrent
import io.dronefleet.mavlink.common.MissionItemReached
import io.dronefleet.mavlink.common.MissionSetCurrent
import io.dronefleet.mavlink.common.MissionState
import io.dronefleet.mavlink.standard.MavModeFlag
import io.dronefleet.mavlink.util.EnumValue

import nl.wur.mavdrone.MavManager.MavlinkManagerCommunicationModel.MissionStateUpdated
import nl.wur.mavdrone.MavManager.MavlinkManagerCommunicationModel.PauseMission
import nl.wur.mavdrone.MavManager.MavlinkManagerCommunicationModel.ResumeMission
import nl.wur.mavdrone.MavManager.MavlinkManagerCommunicationModel.StartMission
import nl.wur.mavdrone.MavManager.MavlinkManagerCommunicationModel.StopMission
import nl.wur.mavdrone.MavManager.MavlinkManagerCommunicationModel.WaypointMissionLoaded
import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.BaseComponent
import nl.wur.mavdrone.communication.components.Drone
import nl.wur.mavdrone.communication.utils.CustomMode
import nl.wur.mavdrone.communication.utils.StandardResponses
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandAccepted
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.missionAccepted
import nl.wur.mavdrone.utils.CurrentMissionState
import nl.wur.mavdrone.utils.RXUtils
import nl.wur.mavdrone.utils.RXUtils.Companion.djiCompletableFunction
import nl.wur.mavdrone.utils.RXUtils.Companion.waitForVariable
import nl.wur.mavdrone.utils.RxException
import nl.wur.mavdrone.utils.RxException.DroneNotArmedException
import nl.wur.mavdrone.utils.RxException.DroneNotExecutingMissionException
import nl.wur.mavdrone.utils.RxException.MissionUploadException
import nl.wur.mavdrone.utils.RxException.NoMissionUploadedException
import java.util.concurrent.TimeUnit
import kotlin.math.roundToInt

class WaypointManager(
    connectionWR: WeakReference<MavConnection>,
    private val droneWR: WeakReference<Drone>,
    private val missionManagerWR: WeakReference<MissionManager>
) : BaseComponent(connectionWR, "MavWaypointManager"), WaypointMissionExecuteStateListener,
    WaylineExecutingInfoListener, WaypointActionListener {
    var waylineNumbersList: List<Int>? = null
        private set

    // DJI Mission is from ENTER_WAYLINE to FINISHED (RTL and landing is done after FINISHED!!)
    private var isFlyingMission = false
    var uploadedMissionPath: String? = null

    var waypointMissionExecuteState: WaypointMissionExecuteState = WaypointMissionExecuteState.UNKNOWN
        private set

    private var droneComponentIdValue: Int = 1

    override fun create() {
        WaypointMissionManager.getInstance().addWaypointMissionExecuteStateListener(this)
        WaypointMissionManager.getInstance().addWaylineExecutingInfoListener(this)
        WaypointMissionManager.getInstance().addWaypointActionListener(this)

        connectionWR.get()?.let {
            compositeDisposable.add(
                it.mavlinkThreadCommunicationSubject
                    .ofType(StartMission::class.java)
                    .observeOn(Schedulers.io())
                    .subscribeOn(Schedulers.io())
                    .filter(StartMission::shouldStart)
                    .flatMapCompletable { startCurrentMission().onErrorResumeNext { error -> RXUtils.handleErrorResume(error) } }
                    .subscribe(
                        { log.i("Started mission from GUI") },
                        {  error -> RXUtils.handleError(error) })
            )
            compositeDisposable.add(
                it.mavlinkThreadCommunicationSubject
                    .ofType(PauseMission::class.java)
                    .observeOn(Schedulers.io())
                    .subscribeOn(Schedulers.io())
                    .filter(PauseMission::shouldPause)
                    .flatMapCompletable { pauseCurrentMission().onErrorResumeNext { error -> RXUtils.handleErrorResume(error) } }
                    .subscribe(
                        { log.i("Paused mission from GUI") },
                        { error -> RXUtils.handleError(error) })
            )
            compositeDisposable.add(
                it.mavlinkThreadCommunicationSubject
                    .ofType(ResumeMission::class.java)
                    .observeOn(Schedulers.io())
                    .subscribeOn(Schedulers.io())
                    .filter(ResumeMission::shouldResume)
                    .flatMapCompletable { resumeCurrentMission().onErrorResumeNext { error -> RXUtils.handleErrorResume(error) } }
                    .subscribe(
                        { log.i("Resumed mission from GUI") },
                        { error -> RXUtils.handleError(error) })
            )
            compositeDisposable.add(
                it.mavlinkThreadCommunicationSubject
                    .ofType(StopMission::class.java)
                    .observeOn(Schedulers.io())
                    .subscribeOn(Schedulers.io())
                    .filter(StopMission::shouldStop)
                    .flatMapCompletable { stopCurrentMission().onErrorResumeNext { error -> RXUtils.handleErrorResume(error) } }
                    .subscribe(
                        { log.i("Stopped mission from GUI") },
                        { error -> RXUtils.handleError(error) })
            )
        }

        droneComponentIdValue = EnumValue.create(droneWR.get()?.componentID).value()
    }

    override fun destroy() {
        WaypointMissionManager.getInstance().removeWaypointMissionExecuteStateListener(this)
        WaypointMissionManager.getInstance().removeWaylineExecutingInfoListener(this)
        WaypointMissionManager.getInstance().removeWaypointActionListener(this)

        super.destroy()
    }

    override fun onMissionStateUpdate(missionState: WaypointMissionExecuteState) {
        connectionWR.get()?.let {
            waypointMissionExecuteState = missionState

            log.i("Change ExecuteState to $waypointMissionExecuteState")

            when (missionState) {
                WaypointMissionExecuteState.ENTER_WAYLINE, WaypointMissionExecuteState.EXECUTING -> {
                    updateMavlinkMission()

                    it.mavlinkThreadCommunicationSubject.onNext(
                        MissionStateUpdated(CurrentMissionState.ACTIVE)
                    )
                }

                WaypointMissionExecuteState.FINISHED -> if (isFlyingMission) {
                    endMavlinkMission()

                    it.mavlinkThreadCommunicationSubject.onNext(
                        MissionStateUpdated(CurrentMissionState.NOT_STARTED)
                    )
                }

                WaypointMissionExecuteState.READY -> {
                    // Finished is not always emitted, so assume mission is finished when entering READY state
                    if (isFlyingMission) endMavlinkMission()

                    if (missionManagerWR.get()?.currentWaypointMission == null) {
                        it.mavlinkThreadCommunicationSubject.onNext(
                            MissionStateUpdated(CurrentMissionState.NO_MISSION)
                        )
                    } else {
                        it.mavlinkThreadCommunicationSubject.onNext(
                            MissionStateUpdated(CurrentMissionState.NOT_STARTED)
                        )
                    }
                }

                WaypointMissionExecuteState.INTERRUPTED ->
                    it.mavlinkThreadCommunicationSubject.onNext(
                        MissionStateUpdated(CurrentMissionState.PAUSED)
                    )

                WaypointMissionExecuteState.RECOVERING ->
                    it.mavlinkThreadCommunicationSubject.onNext(
                        MissionStateUpdated(CurrentMissionState.ACTIVE)
                    )

                WaypointMissionExecuteState.IDLE, WaypointMissionExecuteState.NOT_SUPPORTED ->
                    it.mavlinkThreadCommunicationSubject.onNext(
                        MissionStateUpdated(CurrentMissionState.NO_MISSION)
                    )

                else -> {}
            }
        }
    }

    override fun onWaylineExecutingInfoUpdate(waylineInfo: WaylineExecutingInfo) {
        currentDJIId = waylineInfo.currentWaypointIndex
    }

    override fun onWaylineExecutingInterruptReasonUpdate(error: IDJIError) {
        val currentDJIId = currentDJIId

        log.i("Waypoint $currentDJIId interrupted: ${error.description() ?: "UNKNOWN"}")
    }

    @Deprecated("Deprecated in Java")
    override fun onExecutionStart(actionId: Int) {
    }

    @Deprecated("Deprecated in Java")
    override fun onExecutionFinish(actionId: Int, error: IDJIError?) {
    }

    override fun onExecutionStart(actionGroup: Int, actionId: Int) {
        log.i("Start action $actionId for waypoint $currentDJIId")
    }

    override fun onExecutionFinish(actionGroup: Int, actionId: Int, error: IDJIError?) {
        if (error == null || error.errorCode() == "NO_ERROR") {
            log.i("Finish action $actionId for waypoint $currentDJIId")
        } else {
            log.e("Could not finish action $actionId (group $actionGroup) for waypoint $currentDJIId: ${error.errorCode()}")
        }
    }

    fun onCommandLong(msg: MavlinkMessage<*>): Boolean {
        val payload = msg.payload as CommandLong

        if (payload.targetSystem() == systemID && (payload.targetComponent() == droneComponentIdValue || payload.targetComponent() == 0)
        ) {
            log.i("Received command: ${payload.command().entry()}")

            when (payload.command().entry()) {
                MavCmd.MAV_CMD_DO_PAUSE_CONTINUE -> {
                    if (payload.param1().roundToInt() == 0) {
                        compositeDisposable.add(
                            pauseCurrentMission()
                                .subscribe(
                                    { droneWR.get()?.sendMessage(commandAccepted(MavCmd.MAV_CMD_DO_PAUSE_CONTINUE, msg.originSystemId, msg.originComponentId)) },
                                    { error -> droneWR.get()?.onError(error, StandardResponses::commandFailed, MavCmd.MAV_CMD_DO_PAUSE_CONTINUE, msg.originSystemId, msg.originComponentId) }
                                )
                        )
                    } else {
                        compositeDisposable.add(
                            resumeCurrentMission()
                                .subscribe(
                                    { droneWR.get()?.sendMessage(commandAccepted(MavCmd.MAV_CMD_DO_PAUSE_CONTINUE, msg.originSystemId, msg.originComponentId)) },
                                    { error -> droneWR.get()?.onError(error, StandardResponses::commandFailed, MavCmd.MAV_CMD_DO_PAUSE_CONTINUE, msg.originSystemId, msg.originComponentId) }
                                )
                        )
                    }
                    return true
                }

                MavCmd.MAV_CMD_DO_SET_MISSION_CURRENT -> {
                    val number = payload.param1().roundToInt()

                    if (number >= 0) {
                        setCurrentWaylineIdFromMavlink(number)

                        val missionCurrent = toMavlinkMissionCurrent()
                        droneWR.get()!!.sendMessage(missionCurrent)
                    } else if (payload.param2().roundToInt() == 1) {
                        compositeDisposable.add(
                            stopCurrentMission()
                                .subscribe(
                                    { droneWR.get()?.sendMessage(missionAccepted(msg.originSystemId,msg.originComponentId)) },
                                    { error -> droneWR.get()?.onError(error, StandardResponses::missionError, msg.originSystemId, msg.originComponentId) }
                              )
                        )
                    }
                    return true
                }

                MavCmd.MAV_CMD_MISSION_START -> {
                    // TODO: param1() and param2()
                    compositeDisposable.add(
                        startCurrentMission()
                            .subscribeOn(Schedulers.newThread())
                            .subscribe(
                                { droneWR.get()?.sendMessage(missionAccepted(msg.originSystemId,msg.originComponentId)) },
                                { error -> droneWR.get()?.onError(error, StandardResponses::missionError, msg.originSystemId, msg.originComponentId) }
                            )
                    )
                    return true
                }

                else -> {}
            }
        }

        return false
    }

    private fun updateMavlinkMission() {
        val waypointMission = missionManagerWR.get()!!.currentWaypointMission
        if (waypointMission == null) {
            log.w("MavlinkMission out of sync with DJI mission! No mission present!")
            return
        }

        val currentDJIId = currentDJIId

        // ENTER_WAYLINE is not always emitted, so check whether the mission is already started or
        // not. If not started, send all items before the first waypoint as completed.
        if (!isFlyingMission) {
            log.i("MavlinkMission started!")

            // Set all mavlink items before first waypoint as reached
            if (waypointMission.getDjiToMavlinkIdMap().containsKey(0)) {
                val currentMavlinkIdTmp = waypointMission.getDjiToMavlinkIdMap()[0]
                if (currentMavlinkIdTmp == null) {
                    log.w("MavlinkMission out of sync with DJI mission! DJI-ID 0 not found!")
                    return
                }
                waypointMission.currentMavlinkId = currentMavlinkIdTmp

                for (i in 0 until waypointMission.currentMavlinkId) {
                    handleMavlinkItemReached(i)
                }

                log.i("Flying towards mavlink waypoint ${waypointMission.currentMavlinkId}")
            }

            isFlyingMission = true
        }

        // When there is a next waypoint, set all items between this and next waypoint as completed
        if (waypointMissionExecuteState == WaypointMissionExecuteState.EXECUTING
            && waypointMission.getDjiToMavlinkIdMap().containsKey(currentDJIId + 1)
        ) {
            val mavlinkId = waypointMission.getDjiToMavlinkIdMap()[currentDJIId + 1]

            if (mavlinkId == null) {
                log.w("MavlinkMission out of sync with DJI mission! DJI-ID $currentDJIId not found!")
                return
            }

            if (waypointMission.currentMavlinkId != mavlinkId) {
                // Report all previous waypoints as done
                if (waypointMission.getDjiToMavlinkIdMap().containsKey(currentDJIId)) {
                    (waypointMission.currentMavlinkId until mavlinkId).forEach {
                        handleMavlinkItemReached(it)
                    }
                }

                waypointMission.currentMavlinkId = mavlinkId
                log.i("Flying towards mavlink waypoint ${waypointMission.currentMavlinkId}")
            }
        }
    }

    private fun handleMavlinkItemReached(id: Int) {
        val waypointMission = missionManagerWR.get()!!.currentWaypointMission ?: return

        log.i("Reached Mavlink item $id: ${waypointMission.get(id).command().entry()}")
        val missionItemReached = MissionItemReached.builder()
            .seq(id)
            .build()
        droneWR.get()?.sendMessage(missionItemReached)

        // If auto-continue is set to false, pause the mission after the item completed
        if (waypointMission.get(waypointMission.currentMavlinkId).autocontinue() == 0) {
            compositeDisposable.add(
                pauseCurrentMission()
                    .observeOn(Schedulers.io())
                    .subscribe(
                        { log.i("Paused mission after item $id because auto-continue is 0") },
                        { error -> RXUtils.handleError(error) }
                    )
            )
        }
    }

    private fun endMavlinkMission() {
        // When mission is finished, report all waypoints as done including the current one (e.g. actions can come
        // after the last waypoint).
        val waypointMission = missionManagerWR.get()!!.currentWaypointMission
        if (waypointMission == null) {
            log.w("MavlinkMission out of sync with DJI mission! No mission present!")
            return
        }
        val currentDJIId = currentDJIId

        val lastMavlinkWaypointId = waypointMission.getDjiToMavlinkIdMap()[currentDJIId]

        if (lastMavlinkWaypointId == null) {
            log.w("MavlinkMission out of sync with DJI mission! DJI-ID $currentDJIId not found!")
        } else {
            for (i in lastMavlinkWaypointId..waypointMission.currentMavlinkId) {
                handleMavlinkItemReached(i)
            }

            waypointMission.currentMavlinkId = lastMavlinkWaypointId + 1
        }

        log.i("DJI mission is finished.")

        // If the last item is a return to launch or land item, set the new current mavlink id
        val lastMissionItemCmd = waypointMission.get(waypointMission.size() - 1).command().entry()
        if (lastMissionItemCmd == MavCmd.MAV_CMD_NAV_RETURN_TO_LAUNCH || lastMissionItemCmd == MavCmd.MAV_CMD_NAV_LAND) {
            // Send all items completed except last item
            (waypointMission.currentMavlinkId until waypointMission.size() - 2).forEach {
                handleMavlinkItemReached(it)
            }

            waypointMission.currentMavlinkId = waypointMission.size() - 1

            // Add broadcaster to send itemReached for RTL and Land finished action
            droneWR.get()?.addBroadcaster("RTL_LAND_MISSION_ITEM", 500) {
                droneWR.get()?.let {
                    if (it.flightMode != FlightMode.GO_HOME && it.flightMode != FlightMode.AUTO_LANDING) {
                        handleMavlinkItemReached(waypointMission.currentMavlinkId)

                        log.i("Reached last RTL or landing waypoint")

                        // Reset flight mode
                        it.setCustomMode(CustomMode.MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD)
                        it.setCustomSubMode(CustomMode.SUB_MODE.INVALID)

                        it.stopBroadcaster("RTL_LAND_MISSION_ITEM")
                    }
                }
            }

        } else {
            // Send all last actions to reached
            (waypointMission.currentMavlinkId until waypointMission.size()).forEach {
                handleMavlinkItemReached(it)
            }

            // Reset flight mode
            droneWR.get()?.let {
                it.setCustomMode(CustomMode.MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD)
                it.setCustomSubMode(CustomMode.SUB_MODE.INVALID)
            }
        }

        // After finishing a mission (finished or stopped), remove auto mode flag
        droneWR.get()?.let {
            it.removeMavModeFlag(MavModeFlag.MAV_MODE_FLAG_AUTO_ENABLED)
            it.setCustomMode(CustomMode.MAIN_MODE.PX4_CUSTOM_MAIN_MODE_POSCTL)
        }
        isFlyingMission = false
    }

    @Suppress("deprecation")
    fun onMissionSetCurrent(msg: MavlinkMessage<*>) {
        val payload = msg.payload as MissionSetCurrent

        // TODO: untested, probably have to make new mission starting from the requested item
        if (payload.targetSystem() == systemID && (payload.targetComponent() == droneComponentIdValue || payload.targetComponent() == 0)
        ) {
            if (setCurrentWaylineIdFromMavlink(payload.seq())) {
                // Send current mission item
                val missionCurrent = toMavlinkMissionCurrent()
                droneWR.get()?.sendMessage(missionCurrent)
            }
        }
    }

    fun setCurrentWaylineIdFromMavlink(id: Int): Boolean {
        val waypointMission = missionManagerWR.get()?.currentWaypointMission

        if (waypointMission == null) {
            log.e("Could not set current mission item: could not find corresponding waypoint!")
            return false
        }

        // TODO: untested
        var waylineID = -1
        for ((key, mavlinkID) in waypointMission.getDjiToMavlinkIdMap()) {
            if (id >= (mavlinkID ?: 0)) {
                waylineID = key
                break
            }
        }

        if (waylineID == -1) {
            log.e("Could not set current mission item: could not find corresponding waypoint!")
            return false
        }

        waypointMission.currentMavlinkId = id
        currentDJIId = waylineID
        return true
    }

    //region:messages
    fun toMavlinkMissionCurrent(): MissionCurrent {
        val waypointMission = missionManagerWR.get()!!.currentWaypointMission

        val missionMode: Int
        var missionState: MissionState
        when (waypointMissionExecuteState) {
            WaypointMissionExecuteState.PREPARING, WaypointMissionExecuteState.UPLOADING, WaypointMissionExecuteState.READY -> {
                missionMode = 1
                missionState = MissionState.MISSION_STATE_NOT_STARTED
            }

            WaypointMissionExecuteState.INTERRUPTED -> {
                missionMode = 1
                missionState = MissionState.MISSION_STATE_PAUSED
            }

            WaypointMissionExecuteState.FINISHED -> {
                missionMode = 1
                missionState = MissionState.MISSION_STATE_COMPLETE
            }

            WaypointMissionExecuteState.RECOVERING, WaypointMissionExecuteState.EXECUTING, WaypointMissionExecuteState.ENTER_WAYLINE, WaypointMissionExecuteState.RETURN_TO_START_POINT -> {
                missionMode = 1
                missionState = MissionState.MISSION_STATE_ACTIVE
            }

            else -> {
                missionMode = 2
                missionState = MissionState.MISSION_STATE_UNKNOWN
            }
        }
        if (waypointMission == null) {
            missionState = MissionState.MISSION_STATE_NO_MISSION
        }

        return MissionCurrent.builder()
            .seq(waypointMission?.currentMavlinkId ?: 0)
            .total(waypointMission?.size() ?: UINT16_MAX)
            .missionState(missionState)
            .missionMode(missionMode)
            .build()
    }

    //endregion
    //region:mission transfer to drone
    fun startCurrentMission(): Completable {
        return Completable.fromAction {
            droneWR.get()?.let { drone ->
                val waypointMission = missionManagerWR.get()!!.currentWaypointMission
                if (waypointMission?.wayline == null) {
                    throw NoMissionUploadedException("Cannot start mission because no mission is uploaded to the drone!").setLogger(log)
                }

                if (!drone.containsMavModeFlag(MavModeFlag.MAV_MODE_FLAG_SAFETY_ARMED) && drone.containsMavModeFlag(MavModeFlag.MAV_MODE_FLAG_AUTO_ENABLED)) {
                    throw DroneNotArmedException("Cannot start mission because the drone is not in AUTO_ARMED or AUTO_ENABLED mode!").setLogger(log)
                }

                if (!drone.isHomePositionSet || !drone.isLocationAvailable) {
                    throw MissionUploadException("Cannot start mission before the drone has a home point and it's current GPS location!").setLogger(log)
                }

                waylineNumbersList = listOf(waypointMission.wayline!!.waylineId)

                missionManagerWR.get()?.enableGeofenceWhenNeeded()
                log.i("Start uploading mission to aircraft")
            } ?: throw RxException.ReferenceNotFoundException("Cannot find reference to Drone!").setLogger(log)
        }
            .andThen(stopCurrentMissionWhenNeeded())
            // The finish signal of pushToAircraft() is not always emitted. When state changes to
            // ready and finish signal is not emitted within 500ms, just continue. Probably a bug
            // in the M350 firmware.
            .andThen(
                Completable.ambArray(
                    missionManagerWR.get()!!.currentWaypointMission!!.pushToAircraft(),
                    waitForVariable(
                        { this.waypointMissionExecuteState },
                        WaypointMissionExecuteState.READY,
                        50L,
                    )
                        .delay(500, TimeUnit.MILLISECONDS)
                        .doOnComplete {
                            log.w("pushToAircraft() appears stuck, but mission state became READY — continuing.")
                        }
                )
            )
            .andThen(Completable.fromAction {
                missionManagerWR.get()?.currentWaypointMission?.let {
                    uploadedMissionPath = it.kmz?.path


                    log.i("Uploaded mission '${it.kmz?.name}' to aircraft")
                    it.setUploadedToAircraft(true)

                    connectionWR.get()!!.mavlinkThreadCommunicationSubject.onNext(
                        WaypointMissionLoaded(true, it.kmz?.name)
                    )
                }
            })
            .andThen(
                waitForVariable(
                    { this.waypointMissionExecuteState }, WaypointMissionExecuteState.READY, 50L
                )
            )
            .andThen(
                djiCompletableFunction(
                    WaypointMissionManager.getInstance()::startMission,
                    { uploadedMissionPath!! },
                    { this.waylineNumbersList!! }
                )
            )
    }

    fun stopCurrentMissionWhenNeeded(): Completable {
        if (uploadedMissionPath != null &&
            (waypointMissionExecuteState == WaypointMissionExecuteState.ENTER_WAYLINE || waypointMissionExecuteState == WaypointMissionExecuteState.EXECUTING || waypointMissionExecuteState == WaypointMissionExecuteState.INTERRUPTED)
        ) {
            log.i("Stop current active mission")

            return stopCurrentMission()
        }

        return Completable.complete()
    }

    fun stopCurrentMission(): Completable {
        return Completable.fromAction {
            if (uploadedMissionPath == null) {
                throw NoMissionUploadedException("Cannot stop current mission because no mission is uploaded to the drone!").setLogger(log)
            }
            if (waypointMissionExecuteState != WaypointMissionExecuteState.ENTER_WAYLINE && waypointMissionExecuteState != WaypointMissionExecuteState.EXECUTING && waypointMissionExecuteState != WaypointMissionExecuteState.INTERRUPTED) {
                throw DroneNotExecutingMissionException("Cannot stop current mission because drone is not executing a mission!").setLogger(log)
            }
        }
            .andThen(djiCompletableFunction(WaypointMissionManager.getInstance()::stopMission) { uploadedMissionPath!! })
            .andThen(Completable.fromAction { uploadedMissionPath = null })
    }

    fun pauseCurrentMission(): Completable {
        return djiCompletableFunction(WaypointMissionManager.getInstance()::pauseMission)
    }

    fun resumeCurrentMission(): Completable {
        return djiCompletableFunction(WaypointMissionManager.getInstance()::resumeMission)
    }

    private var currentDJIId: Int
        get() {
            val waypointMission = missionManagerWR.get()?.currentWaypointMission ?: return -1
            return waypointMission.currentDJIId
        }
        private set(id) {
            val waypointMission = missionManagerWR.get()?.currentWaypointMission
            if (waypointMission == null) {
                log.w("Could not set current DJI id!")
                return
            }
            waypointMission.currentDJIId = id
        }
}


