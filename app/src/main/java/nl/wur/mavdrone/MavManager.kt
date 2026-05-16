package nl.wur.mavdrone

import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.ServiceConnection
import android.os.Build
import android.os.IBinder
import android.os.Parcelable
import io.reactivex.rxjava3.android.schedulers.AndroidSchedulers
import io.reactivex.rxjava3.core.Completable
import io.reactivex.rxjava3.disposables.CompositeDisposable
import io.reactivex.rxjava3.schedulers.Schedulers
import kotlinx.parcelize.Parcelize
import nl.wur.mavdrone.utils.CurrentMissionState
import nl.wur.mavdrone.utils.Logger
import nl.wur.mavdrone.utils.RxException
import java.util.function.Consumer


class MavManager private constructor() {
    var connectionInformation: ConnectionInformation = ConnectionInformation()
    var streamInformation: StreamInformation = StreamInformation()

    private var mService: MavService? = null
    private var mBound: Boolean = false
    private val connection = object : ServiceConnection {

        override fun onServiceConnected(className: ComponentName, service: IBinder) {
            val binder = service as MavService.MavlinkServiceBinder
            mService = binder.getService()
            mBound = true

            log.i("Mavlink service is connected, starting data processors")
            setupDataProcessors()
        }

        override fun onServiceDisconnected(arg0: ComponentName) {
            mBound = false
        }
    }

    private val mavlinkStatusListeners = mutableListOf<MavlinkStatusListener>()
    private val mavlinkGUIListeners = mutableListOf<MavlinkGUIListener>()

    private val mavlinkStatus: MavlinkStatus = MavlinkStatus()

    private var compositeDisposable: CompositeDisposable? = null

    fun addMavlinkStatusListener(listener: MavlinkStatusListener) {
        if (!mavlinkStatusListeners.contains(listener)) {
            mavlinkStatusListeners.add(listener)
        }

        mavlinkStatus.let {
            listener.onStatusChanged(it)
        }
    }

    fun removeMavlinkStatusListener(listener: MavlinkStatusListener) {
        mavlinkStatusListeners.remove(listener)
    }

    fun addMavlinkGUIListener(listener: MavlinkGUIListener) {
        if (!mavlinkGUIListeners.contains(listener)) {
            mavlinkGUIListeners.add(listener)
        }
    }

    fun removeMavlinkGUIListener(listener: MavlinkGUIListener) {
        mavlinkGUIListeners.remove(listener)
    }

    fun startServer(context: Context): Completable {
        log.i("Start Mavlink service")
        compositeDisposable = CompositeDisposable()

        return Completable.fromCallable {
            Intent(context, MavService::class.java).also { intent ->
                intent.putExtra("connection_information", connectionInformation)
                intent.putExtra("stream_information", streamInformation)
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                    context.startForegroundService(intent)
                } else {
                    context.startService(intent)
                }
                context.bindService(intent, connection, Context.BIND_AUTO_CREATE)
            }
        }
    }

    fun stopServer(context: Context): Completable {
        log.i("Stop Mavlink service")
        compositeDisposable?.dispose()

        return Completable.fromCallable {
            context.unbindService(connection)
            context.stopService(Intent(context, MavService::class.java))

            mavlinkStatus.reset()
            mavlinkStatusListeners.forEach { it.onStatusChanged(mavlinkStatus) }
        }
    }

    fun setAutoStartEnabled(enabled: Boolean) {
        mService?.mavlinkCommunicationSubject?.onNext(MavlinkManagerCommunicationModel.SetAutoStartEnabled(enabled))
    }

    fun setMissionFile(missionFile: String) {
        mService?.mavlinkCommunicationSubject?.onNext(MavlinkManagerCommunicationModel.SetMissionFile(missionFile))
    }

    fun startMission() {
        mService?.mavlinkCommunicationSubject?.onNext(MavlinkManagerCommunicationModel.StartMission(true))
    }

    fun pauseMission() {
        mService?.mavlinkCommunicationSubject?.onNext(MavlinkManagerCommunicationModel.PauseMission(true))
    }

    fun resumeMission() {
        mService?.mavlinkCommunicationSubject?.onNext(MavlinkManagerCommunicationModel.ResumeMission(true))
    }

    fun stopMission() {
        mService?.mavlinkCommunicationSubject?.onNext(MavlinkManagerCommunicationModel.StopMission(true))
    }

    private fun setupDataProcessors() {
        if (mService == null) {
            log.e("Cannot setup data processors, since service is not available!")
            return
        }

        addDataProcessor(MavlinkManagerCommunicationModel.MavlinkServerRunning::class.java) { model ->
            mavlinkStatus.isServerRunning = model.isRunning
            mavlinkStatus.serverIPAddress = model.ipAddress
            mavlinkStatusListeners.forEach { it.onStatusChanged(mavlinkStatus) }
        }
        addDataProcessor(MavlinkManagerCommunicationModel.GCSConnected::class.java) { model ->
            mavlinkStatus.gcsConnected = model.isConnected
            mavlinkStatus.gcsIPAddress = model.ipAddress
            mavlinkStatus.connectedComponents = model.connectedComponentsInformation
            mavlinkStatusListeners.forEach { it.onStatusChanged(mavlinkStatus) }
        }
        addDataProcessor(MavlinkManagerCommunicationModel.MissionStateUpdated::class.java) { model ->
            mavlinkStatus.missionState = model.missionState
            mavlinkStatusListeners.forEach { it.onStatusChanged(mavlinkStatus) }
        }
        addDataProcessor(MavlinkManagerCommunicationModel.WaypointMissionLoaded::class.java) { model ->
            mavlinkStatus.waypointMissionLoaded = model.loaded
            mavlinkStatus.loadedWaypointMission = model.waypointMission
            mavlinkStatusListeners.forEach { it.onStatusChanged(mavlinkStatus) }
        }
        addDataProcessor(MavlinkManagerCommunicationModel.GeoFenceMissionLoaded::class.java) { model ->
            mavlinkStatus.geoFenceMissionLoaded = model.loaded
            mavlinkStatusListeners.forEach { it.onStatusChanged(mavlinkStatus) }
        }
        addDataProcessor(MavlinkManagerCommunicationModel.ShouldShowWarning::class.java) { model ->
            mavlinkGUIListeners.forEach { it.onShowWarning(model.title, model.msg) }
        }
        addDataProcessor(MavlinkManagerCommunicationModel.ShouldShowProgressBar::class.java) { model ->
            mavlinkGUIListeners.forEach { it.onShowProgressBar(model.title, model.msg, model.count, model.total) }
        }
    }

    private fun <T> addDataProcessor(
        type: Class<T>,
        consumer: Consumer<T>
    ) {
        compositeDisposable!!.add(
            mService!!.mavlinkCommunicationSubject
                .ofType(type)
                .subscribeOn(Schedulers.io())
                .observeOn(AndroidSchedulers.mainThread())
                .subscribe(
                    { model -> consumer.accept(model) },
                    { throwable -> mavlinkStatusListeners.forEach { it.onError(RxException(throwable.toString())) }}
                )
        )
    }

    @Parcelize
    data class ConnectionInformation(
        val port: Int = 5760,
        val systemId: Int = 1,
        val maxMissedHeartBeats: Int = 4,
        val expectedHeartBeatInterval: Int = 1000
    ) : Parcelable

    @Parcelize
    data class StreamInformation(
        val port: Int = 8554,
        val user: String = "StreamUser",
        val password: String = "StreamPassword",
        val autoStart: Boolean = false
    ) : Parcelable

    data class ConnectedComponentInformation(val systemId: Int, val componentID: Int)

    sealed class MavlinkManagerCommunicationModel {
        data class MavlinkServerRunning(val isRunning: Boolean, val ipAddress: String?) : MavlinkManagerCommunicationModel()
        data class GCSConnected(val isConnected: Boolean, val ipAddress: String?, val connectedComponentsInformation: List<ConnectedComponentInformation>) : MavlinkManagerCommunicationModel()
        data class MissionStateUpdated(val missionState: CurrentMissionState) : MavlinkManagerCommunicationModel()
        data class WaypointMissionLoaded(val loaded: Boolean, val waypointMission: String?) : MavlinkManagerCommunicationModel()
        data class GeoFenceMissionLoaded(val loaded: Boolean) : MavlinkManagerCommunicationModel()

        data class StartMission(val shouldStart: Boolean) : MavlinkManagerCommunicationModel()
        data class PauseMission(val shouldPause: Boolean) : MavlinkManagerCommunicationModel()
        data class ResumeMission(val shouldResume: Boolean) : MavlinkManagerCommunicationModel()
        data class StopMission(val shouldStop: Boolean) : MavlinkManagerCommunicationModel()

        data class SetAutoStartEnabled(val enabled: Boolean) : MavlinkManagerCommunicationModel()
        data class SetMissionFile(val path: String) : MavlinkManagerCommunicationModel()

        data class ShowServiceNotification(val notification: String) : MavlinkManagerCommunicationModel()
        data class ShouldShowWarning(val title: String, val msg: String) : MavlinkManagerCommunicationModel()
        data class ShouldShowProgressBar(val title: String, val msg: String, val count: Int, val total: Int) : MavlinkManagerCommunicationModel()
    }

    interface MavlinkStatusListener {
        fun onStatusChanged(status: MavlinkStatus)
        fun onError(error: RxException)
    }

    interface MavlinkGUIListener {
        fun onShowWarning(title: String, msg: String)
        fun onShowProgressBar(title: String, msg: String, count: Int, total: Int)
        fun onError(error: RxException)
    }

    data class MavlinkStatus(
        var isServerRunning: Boolean = false,
        var serverIPAddress: String? = null,
        var gcsConnected: Boolean = false,
        var gcsIPAddress: String? = null,
        var connectedComponents: List<ConnectedComponentInformation> = emptyList(),
        var missionState: CurrentMissionState = CurrentMissionState.NO_MISSION,
        var waypointMissionLoaded: Boolean = false,
        var loadedWaypointMission: String? = null,
        var geoFenceMissionLoaded: Boolean = false) {
        fun reset() {
            this.isServerRunning = false
            this.serverIPAddress = null
            this.gcsConnected = false
            this.gcsIPAddress = null
            this.connectedComponents = emptyList()
            this.missionState = CurrentMissionState.NO_MISSION
            this.waypointMissionLoaded = false
            this.loadedWaypointMission = null
            this.geoFenceMissionLoaded = false
        }
    }

    companion object {
        private val log = Logger("MavlinkManager")

        @Volatile
        private var instance: MavManager? = null

                fun getInstance(): MavManager {
            return instance ?: synchronized(this) {
                instance ?: MavManager().also { instance = it }
            }
        }
    }
}
