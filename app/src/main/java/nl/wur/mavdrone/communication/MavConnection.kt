package nl.wur.mavdrone.communication

import java.io.EOFException
import java.io.IOException
import java.net.ServerSocket
import java.util.concurrent.TimeUnit
import android.content.Context
import android.content.SharedPreferences
import android.os.Handler
import android.os.Looper
import timber.log.Timber

import dji.v5.ux.core.util.ViewUtil

import io.reactivex.rxjava3.core.Observable
import io.reactivex.rxjava3.disposables.CompositeDisposable
import io.reactivex.rxjava3.schedulers.Schedulers
import io.reactivex.rxjava3.subjects.BehaviorSubject
import io.reactivex.rxjava3.subjects.PublishSubject

import io.dronefleet.mavlink.MavlinkConnection
import io.dronefleet.mavlink.MavlinkMessage
import io.dronefleet.mavlink.standard.Heartbeat
import io.dronefleet.mavlink.standard.MavType

import nl.wur.mavdrone.MavManager.ConnectedComponentInformation
import nl.wur.mavdrone.MavManager.ConnectionInformation
import nl.wur.mavdrone.MavManager.MavlinkManagerCommunicationModel
import nl.wur.mavdrone.MavManager.MavlinkManagerCommunicationModel.MavlinkServerRunning
import nl.wur.mavdrone.MavManager.MavlinkManagerCommunicationModel.ShowServiceNotification
import nl.wur.mavdrone.MavManager.StreamInformation
import nl.wur.mavdrone.R
import nl.wur.mavdrone.communication.utils.GCSConnection
import nl.wur.mavdrone.communication.utils.MavMessageWithComponentID
import nl.wur.mavdrone.utils.Logger
import nl.wur.mavdrone.utils.NetworkUtils.Companion.getLocalWifiIpAddress

class MavConnection(
    private val context: Context, val connectionInformation: ConnectionInformation, val streamInformation: StreamInformation, //endregion
    val mavlinkThreadCommunicationSubject: BehaviorSubject<MavlinkManagerCommunicationModel>
) : Thread() {
    private var server: ServerSocket? = null
    var mavlinkComponents: MavComponents? = null
        private set

    var mavlinkIncomingMessagesObservable: PublishSubject<MavlinkMessage<*>>? = null
    private var mavlinkOutgoingMessagesObservable: PublishSubject<MavMessageWithComponentID>? = null

    private val gcsConnections: MutableMap<Int, GCSConnection> = HashMap()
    private var connectionAddress: String? = null

    val androidPreferences: SharedPreferences = context.getSharedPreferences("default_preferences", Context.MODE_PRIVATE)

    override fun run() {
        try {
            val hostAddress = getLocalWifiIpAddress(context) + ":" + connectionInformation.port
            server = ServerSocket()
            server!!.reuseAddress = true
            server!!.bind(java.net.InetSocketAddress(connectionInformation.port))

            while (!currentThread().isInterrupted) {
                LOG.i("Waiting for connection")

                mavlinkThreadCommunicationSubject.onNext(MavlinkServerRunning(true, hostAddress))
                val socket = server!!.accept()

                // 检测半开连接: 客户端异常死亡(休眠/断网)不会有 FIN/RST,
                // 没有 SO_TIMEOUT 时下面的阻塞读会永远卡住, 新客户端
                // 将再也无法被 accept。存活的 GCS 心跳为 1Hz, 不会触发超时
                socket.tcpNoDelay = true
                socket.soTimeout = 10_000

                connectionAddress = socket.inetAddress.toString()
                LOG.i("Trying to make a connection to $connectionAddress")

                // Initialise connection
                val connection = MavlinkConnection.create(socket.getInputStream(), socket.getOutputStream())
                mavlinkIncomingMessagesObservable = PublishSubject.create()
                mavlinkOutgoingMessagesObservable = PublishSubject.create()

                // Create the components, should be started before the sender
                mavlinkComponents = MavComponents(this)
                mavlinkComponents?.createAllComponents()

                val disposable = CompositeDisposable()
                disposable.add(
                    mavlinkOutgoingMessagesObservable!!
                        .observeOn(Schedulers.newThread())
                        .subscribe({ messageWithCompId ->
                            messageWithCompId.message?.let {
                                connection.send2(
                                    connectionInformation.systemId,
                                    messageWithCompId.getComponentId(),
                                    messageWithCompId.message
                                )
                            }
                        }, { error -> LOG.e("Could not send Mavlink message: ${error.message}") })
                )
                disposable.add(
                    mavlinkIncomingMessagesObservable!!
                        .subscribeOn(Schedulers.newThread())
                        .filter { mavlinkMessage: MavlinkMessage<*> -> mavlinkMessage.payload is Heartbeat }
                        .subscribe({ msg: MavlinkMessage<*> ->
                            val payload = msg.payload as Heartbeat
                            if (payload.type().entry() == MavType.MAV_TYPE_GCS) {
                                val gcsConnection = gcsConnections.getOrPut(msg.originSystemId * msg.originComponentId) {
                                    GCSConnection(msg.originSystemId, msg.originComponentId).apply {
                                        updateGCSConnections()
                                    }
                                }

                                gcsConnection.updateLastHeartbeatTime()
                            }
                        }, { error -> LOG.e("Could not listen to GCS heartbeat: ${error.message}") })
                )
                disposable.add(
                    Observable
                        .interval(0, connectionInformation.expectedHeartBeatInterval.toLong(), TimeUnit.MILLISECONDS)
                        .observeOn(Schedulers.newThread())
                        .subscribe(
                            {
                                val now = System.currentTimeMillis()
                                for ((_, entry) in gcsConnections) {
                                    if (now - entry.lastHeartbeat <= connectionInformation.expectedHeartBeatInterval + 250) {
                                        if (entry.missedHeartbeats > 0) {
                                            LOG.i("Connection to $entry restored")
                                            entry.resetMissedHeartbeats()
                                        }

                                        if (!entry.isConnected) {
                                            entry.isConnected = true

                                            if (numConnectedGCS == 1) {
                                                updateGCSConnections()
                                                mavlinkComponents?.onGCSConnected()
                                            }

                                            LOG.i("Connected to $entry")
                                        }
                                    } else if (now - entry.lastHeartbeat > connectionInformation.expectedHeartBeatInterval + 250) {
                                        if (entry.isConnected) {
                                            if (entry.missedHeartbeats < connectionInformation.maxMissedHeartBeats) {
                                                entry.addMissedHeartbeats()
                                                LOG.w("Missed ${entry}'s heartbeat (${entry.missedHeartbeats}/${connectionInformation.maxMissedHeartBeats})")
                                            } else {
                                                entry.isConnected = false

                                                if (numConnectedGCS == 0) {
                                                    onAllGCSDisconnected()
                                                    mavlinkComponents?.onGCSDisconnected()
                                                }

                                                LOG.e("Connection to $entry lost!")
                                            }
                                        }
                                    }
                                }
                            },
                            { error -> LOG.e("Could not check GCS connection: ${error.message}") }
                        )
                )

                // Create Mavlink Log handler
                val mavlinkLogHandler = MavLogTree(this)
                Timber.plant(mavlinkLogHandler)

                try {
                    generateSequence { connection.next() }
                        .takeWhile { !currentThread().isInterrupted }
                        .forEach { mavlinkIncomingMessagesObservable?.onNext(it) }

                    mavlinkIncomingMessagesObservable?.onComplete()
                } catch (e: EOFException) {
                    LOG.w("Connection stream has ended!")
                } catch (e: java.net.SocketTimeoutException) {
                    // 读超时: 客户端 10s 内没有任何数据(心跳已停), 判定
                    // 半开连接, 丢弃该会话回到 accept() 等待新客户端
                    LOG.w("Connection read timed out, dropping stale client")
                } catch (e: IOException) {
                    LOG.w("Cannot parse Mavlink message (IOException): ${e.message}")
                }

                onAllGCSDisconnected()
                mavlinkComponents?.onGCSDisconnected()
                gcsConnections.clear()

                mavlinkIncomingMessagesObservable?.onComplete()
                mavlinkOutgoingMessagesObservable?.onComplete()

                mavlinkComponents?.destroyAll()

                disposable.dispose()

                // Remove log handler
                Timber.uproot(mavlinkLogHandler)
            }

            mavlinkThreadCommunicationSubject.onNext(MavlinkServerRunning(false, null))
            connectionAddress = null
        } catch (e: IOException) {
            LOG.e("Mavlink server error (IOException): %s", e.message)
            connectionAddress = null
        }
    }

    fun close() {
        server?.takeIf { it.isBound && !it.isClosed }?.let {
            try {
                it.close()
                LOG.i("Closed SocketServer and released port")
            } catch (e: IOException) {
                LOG.e("Cannot close SocketServer: %s", e.message)
            }
        }
    }

    private fun updateGCSConnections() {
        showToast(R.string.mavlink_service_notification_connected, 2)

        LOG.i("Starting GCS telemetry link on port %d", connectionInformation.port)

        val gcsInformation: MutableList<ConnectedComponentInformation> = gcsConnections.values.map {
            ConnectedComponentInformation(it.systemId, it.componentId)
        }.toMutableList()

        mavlinkThreadCommunicationSubject.onNext(
            ShowServiceNotification(context.getString(R.string.mavlink_service_notification_connected))
        )
        mavlinkThreadCommunicationSubject.onNext(
            MavlinkManagerCommunicationModel.GCSConnected(true, connectionAddress, gcsInformation)
        )
    }

    private fun onAllGCSDisconnected() {
        showToast(R.string.mavlink_service_notification_disconnected, 2)

        LOG.i("Disconnected from GCS")

        mavlinkThreadCommunicationSubject.onNext(
            ShowServiceNotification(context.getString(R.string.mavlink_service_notification_disconnected))
        )
        mavlinkThreadCommunicationSubject.onNext(
            MavlinkManagerCommunicationModel.GCSConnected(false, null, ArrayList())
        )
    }

    //region utils
    fun showToast(msg: String?, duration: Int) {
        val handler = Handler(Looper.getMainLooper())
        handler.post { ViewUtil.showToast(context, msg, duration) }
    }

    fun showToast(resourceId: Int, duration: Int) {
        val handler = Handler(Looper.getMainLooper())
        handler.post { ViewUtil.showToast(context, resourceId, duration) }
    }

    private val numConnectedGCS: Int
        get() = gcsConnections.values.count { it.isConnected }

    val isNotConnectedToGCS: Boolean
        get() = gcsConnections.isEmpty()

    val connectedGCS: Collection<GCSConnection>
        get() = gcsConnections.values

    fun sendMavlinkMessage(message: MavMessageWithComponentID) {
        mavlinkOutgoingMessagesObservable?.onNext(message)
    }

    fun getMavlinkIncomingMessagesObservable(): Observable<MavlinkMessage<*>>? {
        return mavlinkIncomingMessagesObservable
    }

    companion object {
        private val LOG = Logger("MavBackgroundThread")
    }
}
