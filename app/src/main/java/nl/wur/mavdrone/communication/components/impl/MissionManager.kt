package nl.wur.mavdrone.communication.components.impl

import java.lang.ref.WeakReference
import java.util.EnumMap
import java.util.concurrent.TimeUnit

import dji.v5.manager.aircraft.waypoint3.model.WaypointMissionExecuteState

import io.reactivex.rxjava3.core.Completable
import io.reactivex.rxjava3.core.Observable
import io.reactivex.rxjava3.disposables.Disposable
import io.reactivex.rxjava3.schedulers.Schedulers

import io.dronefleet.mavlink.MavlinkMessage
import io.dronefleet.mavlink.common.FenceStatus
import io.dronefleet.mavlink.common.MavMissionResult
import io.dronefleet.mavlink.common.MavMissionType
import io.dronefleet.mavlink.common.MissionAck
import io.dronefleet.mavlink.common.MissionClearAll
import io.dronefleet.mavlink.common.MissionCount
import io.dronefleet.mavlink.common.MissionCurrent
import io.dronefleet.mavlink.common.MissionItemInt
import io.dronefleet.mavlink.common.MissionRequestInt
import io.dronefleet.mavlink.common.MissionRequestList
import io.dronefleet.mavlink.common.MissionSetCurrent
import io.dronefleet.mavlink.standard.MavComponent
import io.dronefleet.mavlink.standard.MavModeFlag
import io.dronefleet.mavlink.util.EnumValue

import nl.wur.mavdrone.MavManager.MavlinkManagerCommunicationModel.GeoFenceMissionLoaded
import nl.wur.mavdrone.MavManager.MavlinkManagerCommunicationModel.MissionStateUpdated
import nl.wur.mavdrone.MavManager.MavlinkManagerCommunicationModel.SetMissionFile
import nl.wur.mavdrone.MavManager.MavlinkManagerCommunicationModel.ShouldShowProgressBar
import nl.wur.mavdrone.MavManager.MavlinkManagerCommunicationModel.WaypointMissionLoaded
import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.BaseComponent
import nl.wur.mavdrone.communication.components.Drone
import nl.wur.mavdrone.communication.components.impl.mission.BaseMission
import nl.wur.mavdrone.communication.components.impl.mission.GeofenceMission
import nl.wur.mavdrone.communication.components.impl.mission.WaypointMission
import nl.wur.mavdrone.communication.utils.CustomMode
import nl.wur.mavdrone.communication.utils.KMZ
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.missionAccepted
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.missionError
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.missionInvalidSequence
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.missionOperationCancelled
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.missionUnsupported
import nl.wur.mavdrone.utils.CurrentMissionState
import nl.wur.mavdrone.utils.RXUtils.Companion.handleError

class MissionManager(
    connectionWR: WeakReference<MavConnection>, private val droneWR: WeakReference<Drone>
) : BaseComponent(connectionWR, "MavMissionManager") {
        val waypointManager: WaypointManager = WaypointManager(connectionWR, droneWR, WeakReference(this))

        val geofenceManager: GeofenceManager = GeofenceManager(connectionWR, droneWR, WeakReference(this))

        var currentWaypointMission: WaypointMission? = null

        var currentGeofenceMission: GeofenceMission? = null

    private val missionTransferStateMap: MutableMap<MavMissionType, MissionTransferState> = EnumMap(MavMissionType::class.java)

    private var droneComponentIdValue: Int

    init {
        // Support both mission and fence type
        missionTransferStateMap[MavMissionType.MAV_MISSION_TYPE_MISSION] = MissionTransferState(WaypointMission(connectionWR, WeakReference(this)))
        missionTransferStateMap[MavMissionType.MAV_MISSION_TYPE_FENCE] = MissionTransferState(GeofenceMission(connectionWR, WeakReference(this)))

        droneComponentIdValue = EnumValue.of(droneWR.get()?.componentID ?: MavComponent.MAV_COMP_ID_AUTOPILOT1).value()
    }

    @Suppress("deprecation")
    override fun create() {
        droneWR.get()?.apply {
            addMavlinkMessageSender(MissionCurrent::class.java, waypointManager::toMavlinkMissionCurrent, 1000, true)
            addMavlinkMessageSender(FenceStatus::class.java, geofenceManager::toMavlinkFenceStatus)

            addMavlinkListener(MissionClearAll::class.java, this@MissionManager::onCmdMissionClearAll)
            addMavlinkListener(MissionRequestList::class.java, this@MissionManager::onMissionRequestList)
            addMavlinkListener(MissionRequestInt::class.java, this@MissionManager::onMissionRequestInt)
            addMavlinkListener(MissionCount::class.java, this@MissionManager::onMissionCount)
            addMavlinkListener(MissionItemInt::class.java, this@MissionManager::onMissionItemInt)
            addMavlinkListener(MissionAck::class.java, this@MissionManager::onMissionAck)
            addMavlinkListener(MissionSetCurrent::class.java, waypointManager::onMissionSetCurrent)
        }

        // Add reaction GUI open mission
        connectionWR.get()?.let { it ->
            compositeDisposable.add(it.mavlinkThreadCommunicationSubject.ofType(SetMissionFile::class.java).observeOn(Schedulers.io())
                .flatMap { msg: SetMissionFile -> Observable.fromCallable { KMZ(connectionWR, msg.path) } }
                .flatMap { kmz: KMZ ->
                    WaypointMission.fromKMZ(connectionWR, WeakReference(this), kmz).toObservable()
                        .flatMap { waypointMission: WaypointMission? ->
                            currentWaypointMission = waypointMission
                            currentWaypointMission?.convertToMavlink(
                                systemID, droneComponentIdValue
                            )?.andThen(Observable.just<KMZ>(kmz)) ?: Observable.just(kmz)
                        }
                }.flatMap { kmz: KMZ ->
                    GeofenceMission.fromKMZ(connectionWR, WeakReference(this), kmz).toObservable()
                        .flatMap { geofenceMission: GeofenceMission? ->
                            currentGeofenceMission = geofenceMission
                            currentGeofenceMission?.convertToMavlink(
                                systemID, droneComponentIdValue
                            )?.andThen(Observable.just(kmz)) ?: Observable.just(kmz)
                        }
                }.subscribe({ kmz: KMZ ->
                    // Send message to connected GCS that we received a new mission
                    val waypointCount = currentWaypointMission?.size() ?: 0
                    val geofenceCount = currentGeofenceMission?.size() ?: 0

                    it.connectedGCS.forEach { connection ->
                        val missionCount = MissionCount.builder().targetSystem(connection.systemId).targetComponent(connection.componentId)
                            .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION).count(waypointCount).build()
                        sendMessage(missionCount)
                    }

                    it.connectedGCS.forEach { connection ->
                        val missionCount = MissionCount.builder().targetSystem(connection.systemId).targetComponent(connection.componentId)
                            .missionType(MavMissionType.MAV_MISSION_TYPE_FENCE).count(geofenceCount).build()

                        sendMessage(missionCount)
                    }

                    it.mavlinkThreadCommunicationSubject.onNext(WaypointMissionLoaded(true, kmz.path))
                    it.mavlinkThreadCommunicationSubject.onNext(MissionStateUpdated(CurrentMissionState.NOT_STARTED))
                    log.i("Successfully loaded mission " + kmz.path)
                }, { handleError(it) })
            )
        }

        waypointManager.create()
        geofenceManager.create()
    }

    override fun destroy() {
        missionTransferStateMap.values.forEach { transferState ->
            transferState.receivingTimeoutNewRequestDisposable?.dispose()
        }

        waypointManager.destroy()
        geofenceManager.destroy()

        super.destroy()
    }

    fun sendMessage(message: Any?) {
        droneWR.get()?.sendMessage(message)
    }

    //region: commands
    fun onCommandLong(msg: MavlinkMessage<*>): Boolean {
        return geofenceManager.onCommandLong(msg) || waypointManager.onCommandLong(msg)
    }

    private fun onCmdMissionClearAll(msg: MavlinkMessage<*>) {
        val payload = msg.payload as MissionClearAll

        if (payload.targetSystem() == systemID && (payload.targetComponent() == droneComponentIdValue || payload.targetComponent() == 0)) {
            if (payload.missionType().entry() == MavMissionType.MAV_MISSION_TYPE_MISSION || payload.missionType()
                    .entry() == MavMissionType.MAV_MISSION_TYPE_ALL
            ) {
                val waypointMissionExecuteState = waypointManager.waypointMissionExecuteState

                if (waypointMissionExecuteState == WaypointMissionExecuteState.ENTER_WAYLINE || waypointMissionExecuteState == WaypointMissionExecuteState.EXECUTING || waypointMissionExecuteState == WaypointMissionExecuteState.INTERRUPTED || waypointMissionExecuteState == WaypointMissionExecuteState.RECOVERING) {
                    // Do not accept clear mission now since a mission is running
                    log.w("Cannot clear mission while running!")
                    sendMessage(
                        missionError(
                            payload.missionType().entry(), msg.originSystemId, msg.originComponentId
                        )
                    )
                } else {
                    log.i("Clear Mission")
                    currentWaypointMission = null
                    sendMessage(missionAccepted(payload.missionType().entry(), msg.originSystemId, msg.originComponentId))

                    connectionWR.get()!!.mavlinkThreadCommunicationSubject.onNext(WaypointMissionLoaded(false, null))
                    connectionWR.get()!!.mavlinkThreadCommunicationSubject.onNext(MissionStateUpdated(CurrentMissionState.NO_MISSION))

                    // Remove guided mode
                    droneWR.get()!!.removeMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                }
            }

            if (payload.missionType().entry() == MavMissionType.MAV_MISSION_TYPE_FENCE || payload.missionType()
                    .entry() == MavMissionType.MAV_MISSION_TYPE_ALL
            ) {
                log.i("Clear GeoFence")
                currentGeofenceMission = null
                geofenceManager.disable()
                sendMessage(missionAccepted(payload.missionType().entry(), msg.originSystemId, msg.originComponentId))

                connectionWR.get()!!.mavlinkThreadCommunicationSubject.onNext(GeoFenceMissionLoaded(false))
            }
        }
    }

    //region:download mission
    private fun onMissionCount(msg: MavlinkMessage<*>) {
        val payload = msg.payload as MissionCount

        if (payload.targetSystem() == systemID && (payload.targetComponent() == droneComponentIdValue || payload.targetComponent() == 0)) {
            log.i("GCS requested to upload a " + payload.missionType().entry().name + " mission with " + payload.count() + " commands to drone")

            val missionState = missionTransferStateMap[payload.missionType().entry()]
            if (missionState == null) {
                log.e("Unsupported mission type " + payload.missionType())
                sendMessage(missionUnsupported(msg.originSystemId, msg.originComponentId))
                return
            }

            if (missionState.isReceivingMission) {
                log.w("Canceling downloading mission of type " + payload.missionType() + " because we received new MissionCount!")

                missionState.receivingCount = payload.count()
                missionState.mission.clear()

                if (missionState.receivingTimeoutNewRequestDisposable != null) {
                    missionState.receivingTimeoutNewRequestDisposable!!.dispose()
                    missionState.receivingTimeoutNewRequestDisposable = null
                }

                // Remove guided mode
                droneWR.get()!!.removeMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
            }

            if (payload.count() > 0) {
                missionState.isReceivingMission = true
                missionState.receivingCount = payload.count()
                missionState.mission.clear()

                requestMissionItem(0, payload.missionType().entry(), msg.originSystemId, msg.originComponentId)
            } else {
                // Clear current mission
                if (payload.missionType().entry() == MavMissionType.MAV_MISSION_TYPE_MISSION) {
                    log.i("Clear Waypoints")
                    currentWaypointMission = null

                    connectionWR.get()!!.mavlinkThreadCommunicationSubject.onNext(WaypointMissionLoaded(false, null))
                    connectionWR.get()!!.mavlinkThreadCommunicationSubject.onNext(MissionStateUpdated(CurrentMissionState.NO_MISSION))
                } else {
                    log.i("Clear Geofence")
                    currentGeofenceMission = null
                    geofenceManager.disable()

                    connectionWR.get()!!.mavlinkThreadCommunicationSubject.onNext(GeoFenceMissionLoaded(false))
                }

                sendMessage(missionAccepted(payload.missionType().entry(), msg.originSystemId, msg.originComponentId))
            }
        }
    }

    private fun onMissionItemInt(msg: MavlinkMessage<*>) {
        val payload = msg.payload as MissionItemInt

        if (payload.targetSystem() == systemID && (payload.targetComponent() == droneComponentIdValue || payload.targetComponent() == 0)) {
            val missionState = missionTransferStateMap[payload.missionType().entry()]
            if (missionState == null) {
                log.e("Unsupported mission type ${payload.missionType()}")
                sendMessage(missionUnsupported(payload.missionType().entry(), msg.originSystemId, msg.originComponentId))
                return
            }

            if (!missionState.isReceivingMission) {
                log.e("Not expecting a mission item of type ${payload.missionType().entry()} now")
                sendMessage(missionError(payload.missionType().entry(), msg.originSystemId, msg.originComponentId))
                return
            }

            missionState.receivingTimeoutNewRequestDisposable?.dispose().also {
                missionState.receivingTimeoutNewRequestDisposable = null
            }

            log.i(
                "Received mission item ${payload.seq()} of type ${
                    payload.command().entry().name
                }"
            )

            if (missionState.receivingCount > 20) {
                connectionWR.get()!!.mavlinkThreadCommunicationSubject.onNext(
                    ShouldShowProgressBar(
                        "Transfer Mavlink items", "Receiving", payload.seq() + 1, missionState.receivingCount
                    )
                )
            }

            if (payload.seq() != missionState.receivingExpectedItemId) {
                sendMessage(missionInvalidSequence(payload.missionType().entry(), msg.originSystemId, msg.originComponentId))
                requestMissionItem(missionState.receivingExpectedItemId, payload.missionType().entry(), msg.originSystemId, msg.originComponentId)
                return
            }

            if (!missionState.mission.addItem(msg)) {
                cancelMissionTransfer(payload.missionType().entry(), msg.originSystemId, msg.originComponentId)
                return
            }

            if (payload.seq() + 1 < missionState.receivingCount) {
                requestMissionItem(payload.seq() + 1, payload.missionType().entry(), msg.originSystemId, msg.originComponentId)
            } else {
                log.i("Finished receiving ${payload.missionType().entry()}")

                connectionWR.get()!!.showToast("Received all mission items!", 2)

                onFinishReceiving(payload.missionType().entry(), msg.originSystemId, msg.originComponentId)
            }
        }
    }

    //endregion
    //region:upload mission
    private fun onMissionRequestList(msg: MavlinkMessage<*>) {
        val payload = msg.payload as MissionRequestList

        if (payload.targetSystem() == systemID && (payload.targetComponent() == droneComponentIdValue || payload.targetComponent() == 0)) {
            val missionState = missionTransferStateMap[payload.missionType().entry()]
            if (missionState == null) {
                log.e("Unsupported mission type ${payload.missionType()}")
                sendMessage(missionUnsupported(payload.missionType().entry(), msg.originSystemId, msg.originComponentId))
                return
            }

            if (missionState.isReceivingMission) {
                log.w("GCS requested to receive current ${payload.missionType().entry().name} mission while sending mission!")
                sendMessage(missionError(payload.missionType().entry(), msg.originSystemId, msg.originComponentId))
                return
            }
            missionState.mission.clear()

            if (payload.missionType().entry() == MavMissionType.MAV_MISSION_TYPE_MISSION) {
                currentWaypointMission?.let {
                    missionState.mission.addItems(it.missionItems)
                }
            } else {
                currentGeofenceMission?.let {
                    missionState.mission.addItems(it.missionItems)
                }
            }

            val count = missionState.mission.size()
            if (count > 0) {
                missionState.isSendingMission = true
            }

            val missionCount = MissionCount.builder().targetSystem(msg.originSystemId).targetComponent(msg.originComponentId)
                .missionType(payload.missionType()).count(count).build()
            sendMessage(missionCount)
        }
    }

    private fun onMissionRequestInt(msg: MavlinkMessage<*>) {
        val payload = msg.payload as MissionRequestInt

        if (payload.targetSystem() == systemID && (payload.targetComponent() == droneComponentIdValue || payload.targetComponent() == 0)) {
            val missionState = missionTransferStateMap[payload.missionType().entry()]
            if (missionState == null) {
                log.e("Unsupported mission type ${payload.missionType()}")
                sendMessage(
                    missionUnsupported(
                        payload.missionType().entry(), msg.originSystemId, msg.originComponentId
                    )
                )
                return
            }

            val requestedItem = payload.seq()

            log.i("GCS request mission item $requestedItem/${(missionState.mission.size() - 1)}")

            if (requestedItem >= missionState.mission.size()) {
                sendMessage(missionInvalidSequence(payload.missionType().entry(), msg.originSystemId, msg.originComponentId))
                return
            }

            if (!missionState.isSendingMission) {
                log.e("Cannot download mission while uploading!")
                sendMessage(missionError(payload.missionType().entry(), msg.originSystemId, msg.originComponentId))
                return
            }

            if (missionState.mission.size() > 20) {
                connectionWR.get()!!.mavlinkThreadCommunicationSubject.onNext(
                    ShouldShowProgressBar("Transfer Mavlink items", "Sending", requestedItem + 1, missionState.mission.size())
                )
            }

            // Send mission item. If the system and component id's do not match, change them to the
            // id's from the system and component where the request was coming from
            val requestedMissionItem = missionState.mission.get(requestedItem)
            if (requestedMissionItem.targetSystem() == msg.originSystemId && requestedMissionItem.targetComponent() == msg.originComponentId) {
                sendMessage(requestedMissionItem)
            } else {
                sendMessage(changeSystemAndComponentIdForMissionItemInt(requestedMissionItem, msg.originSystemId, msg.originComponentId))
            }
            log.i("Drone sends mission item $requestedItem of type ${missionState.mission.get(requestedItem).command().entry().name}")
        }
    }

    private fun onMissionAck(msg: MavlinkMessage<*>) {
        val payload = msg.payload as MissionAck

        if (payload.targetSystem() == systemID && (payload.targetComponent() == droneComponentIdValue || payload.targetComponent() == 0)) {
            val missionState = missionTransferStateMap[payload.missionType().entry()]
            if (missionState == null) {
                log.e("Unsupported mission type " + payload.missionType())
                sendMessage(missionUnsupported(payload.missionType().entry(), msg.originSystemId, msg.originComponentId))
                return
            }

            if (payload.type().entry() == MavMissionResult.MAV_MISSION_ACCEPTED && missionState.isSendingMission) {
                missionState.clear()
            }
        }
    }

    //endregion
    private fun cancelMissionTransfer(
        missionType: MavMissionType, targetSystemID: Int, targetComponentID: Int
    ) {
        sendMessage(missionOperationCancelled(missionType, targetSystemID, targetComponentID))
        val missionState = checkNotNull(missionTransferStateMap[missionType])
        missionState.clear()
    }

    private fun requestMissionItem(
        id: Int, missionType: MavMissionType, targetSystemId: Int, targetComponentId: Int
    ) {
        val missionState = checkNotNull(missionTransferStateMap[missionType])
        missionState.receivingExpectedItemId = id

        // Create a timer that resends requests for mission items when they timeout
        missionState.receivingTimeoutNewRequestDisposable = Observable
            .interval(MAVLINK_WAYPOINT_TIMEOUT_MISSION_ITEMS_MS.toLong(), MAVLINK_WAYPOINT_TIMEOUT_MISSION_ITEMS_MS.toLong(), TimeUnit.MILLISECONDS)
            .take(MAVLINK_WAYPOINT_MAX_RETRIES.toLong())
            .subscribeOn(Schedulers.newThread())
            .doOnComplete {
                log.e("Timeout for receiving mission item $id")
                cancelMissionTransfer(missionType, targetSystemId, targetComponentId)
            }.subscribe({ currentNumberOfTries: Long ->
                log.i("Resend request GCS for mission item $id ($currentNumberOfTries/$MAVLINK_WAYPOINT_MAX_RETRIES)")
                val missionRequestInt = MissionRequestInt.builder().seq(id).missionType(missionType).build()
                sendMessage(missionRequestInt)
            }, { handleError(it) })

        val missionRequestInt = MissionRequestInt.builder()
            .seq(id)
            .targetSystem(targetSystemId)
            .targetComponent(targetComponentId)
            .missionType(missionType)
            .build()
        sendMessage(missionRequestInt)

        log.i("Request GCS for mission item $id/${(missionState.receivingCount - 1)}")
    }

    //endregion
    private fun onFinishReceiving(missionType: MavMissionType, systemId: Int, componentId: Int) {
        val missionState = checkNotNull(missionTransferStateMap[missionType])
        try {
            if (missionType == MavMissionType.MAV_MISSION_TYPE_MISSION) {
                // Convert to DJI mission, write mission
                compositeDisposable.add(missionState.mission.convertFromMavlink()
                    .subscribeOn(Schedulers.io())
                    .andThen(Completable.fromAction {
                        currentWaypointMission = missionState.mission.copy() as WaypointMission
                    })
                    .subscribe({
                        sendMessage(missionAccepted(missionType, systemId, componentId))
                        log.i("Accepted waypoint mission.")

                        // Send message to other connected GCS that we received a new mission
                        val count = currentGeofenceMission?.size() ?: 0

                        connectionWR.get()?.let {
                            it.connectedGCS.forEach { connection ->
                                if (connection.systemId == systemId && connection.componentId == componentId) {
                                    // We can skip this one, since this is the one that have send us the mission
                                    return@forEach
                                }

                                val missionCount = MissionCount.builder()
                                    .targetSystem(connection.systemId)
                                    .targetComponent(connection.componentId)
                                    .missionType(missionType)
                                    .count(count)
                                    .build()
                                sendMessage(missionCount)

                                it.mavlinkThreadCommunicationSubject.onNext(
                                    MissionStateUpdated(CurrentMissionState.NOT_STARTED)
                                )
                            }
                        }

                        // Enable guided mode since we have a mission
                        droneWR.get()?.let {
                            it.addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
                            it.setCustomMode(CustomMode.MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO)
                        }

                        missionState.clear()
                    }, {
                        handleError(it)
                        sendMessage(missionError(missionType, systemId, componentId))
                        cancelMissionTransfer(missionType, systemId, componentId)
                        missionState.clear()
                    })
                )
            } else if (missionType == MavMissionType.MAV_MISSION_TYPE_FENCE) {
                compositeDisposable.add(missionState.mission.convertFromMavlink()
                    .subscribeOn(Schedulers.newThread())
                    .andThen(Completable.fromAction {
                        currentGeofenceMission = missionState.mission.copy() as GeofenceMission
                    }).subscribe({
                        log.i("Accepted geofence mission")
                        sendMessage(missionAccepted(missionType, systemId, componentId))

                        // Send message to other connected GCS that we received a new mission
                        val count = currentGeofenceMission?.size() ?: 0

                        connectionWR.get()?.let {
                            it.connectedGCS.forEach { connection ->
                                if (connection.systemId == systemId && connection.componentId == componentId) {
                                    // We can skip this one, since this is the one that have send us the mission
                                    return@forEach
                                }

                                val missionCount =
                                    MissionCount.builder().targetSystem(connection.systemId).targetComponent(connection.componentId)
                                        .missionType(missionType).count(count).build()
                                sendMessage(missionCount)
                            }

                            it.mavlinkThreadCommunicationSubject.onNext(
                                GeoFenceMissionLoaded(true)
                            )
                        }

                        // Enable geofence if not done already
                        enableGeofenceWhenNeeded()
                        missionState.clear()
                    }, { throwable: Throwable? ->
                        handleError(throwable!!)
                        sendMessage(missionError(missionType, systemId, componentId))
                        cancelMissionTransfer(missionType, systemId, componentId)
                        missionState.clear()
                    })
                )
            }
        } catch (e: Exception) {
            log.e("Cannot finish " + missionType.name + " mission: " + e.message)
            sendMessage(missionError(missionType, systemId, componentId))
            cancelMissionTransfer(missionType, systemId, componentId)
        }
    }

    fun enableGeofenceWhenNeeded() {
        if (geofenceManager.autoEnable && currentGeofenceMission != null && !geofenceManager.isEnabled) {
            geofenceManager.enable()
        }
    }

    private class MissionTransferState(val mission: BaseMission) {
        var isReceivingMission: Boolean = false
        var isSendingMission: Boolean = false

        var receivingTimeoutNewRequestDisposable: Disposable? = null
        var receivingCount: Int = 0
        var receivingExpectedItemId: Int = 0

        fun clear() {
            mission.clear()
            isReceivingMission = false
            isSendingMission = false
        }
    }

    companion object {
        private const val MAVLINK_WAYPOINT_TIMEOUT_MISSION_ITEMS_MS = 4000 // Default 250 however MavSDK sometimes takes longer
        private const val MAVLINK_WAYPOINT_MAX_RETRIES = 3

        private fun changeSystemAndComponentIdForMissionItemInt(missionItem: MissionItemInt, targetSystem: Int, targetComponent: Int): MissionItemInt {
            return MissionItemInt.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .seq(missionItem.seq())
                .frame(missionItem.frame())
                .command(missionItem.command())
                .current(missionItem.current())
                .autocontinue(missionItem.autocontinue())
                .param1(missionItem.param1())
                .param2(missionItem.param2())
                .param3(missionItem.param3())
                .param4(missionItem.param4())
                .x(missionItem.x())
                .y(missionItem.y())
                .z(missionItem.z())
                .missionType(missionItem.missionType())
                .build()
        }
    }
}
