package nl.wur.mavdrone

import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Intent
import android.os.Binder
import android.os.Build
import android.os.IBinder
import androidx.core.app.NotificationCompat
import io.reactivex.rxjava3.android.schedulers.AndroidSchedulers
import io.reactivex.rxjava3.disposables.CompositeDisposable
import io.reactivex.rxjava3.subjects.BehaviorSubject
import nl.wur.mavdrone.MavManager.ConnectionInformation
import nl.wur.mavdrone.MavManager.StreamInformation
import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.utils.FileLogTree
import nl.wur.mavdrone.utils.Logger
import nl.wur.mavdrone.utils.RxException
import nl.wur.mavdrone.video.DownCamStreamer

import timber.log.Timber

class MavService : Service() {
    private var mNotificationBuilder: NotificationCompat.Builder? = null
    private var mNotificationManager: NotificationManager? = null

    private var connectionInformation: ConnectionInformation? = null
    private var streamInformation: StreamInformation? = null

    inner class MavlinkServiceBinder : Binder() {
        fun getService(): MavService = this@MavService
    }
    private val binder = MavlinkServiceBinder()

    val mavlinkCommunicationSubject: BehaviorSubject<MavManager.MavlinkManagerCommunicationModel> = BehaviorSubject.create()
    private var compositeDisposable: CompositeDisposable? = null
    private var backgroundThread: MavConnection? = null
    private var downCamStreamer: DownCamStreamer? = null

    private val fileLogger by lazy { FileLogTree(this) }

    override fun onStartCommand(intent: Intent, flags: Int, startId: Int): Int {
        if (BuildConfig.DEBUG) {
            Timber.plant(fileLogger)
        }

        if (Build.VERSION.SDK_INT >= 33 /* Build.VERSION_CODES.TIRAMISU */) {
            connectionInformation = intent.getParcelableExtra("connection_information", ConnectionInformation::class.java)
            streamInformation = intent.getParcelableExtra("stream_information", StreamInformation::class.java)
        } else {
            @Suppress("DEPRECATION")
            connectionInformation = intent.getParcelableExtra("connection_information")
            @Suppress("DEPRECATION")
            streamInformation = intent.getParcelableExtra("stream_information")
        }

        if (connectionInformation == null || streamInformation == null) {
            LOG.e("Could not retrieve connection and stream information from activity!")
        }

        createNotificationChannel()
        val notificationIntent = Intent(this, MainActivity::class.java)

        val pendingIntent: PendingIntent =
            PendingIntent.getActivity(this, 0, notificationIntent, PendingIntent.FLAG_IMMUTABLE)
        mNotificationBuilder = NotificationCompat.Builder(this, CHANNEL_ID)
            .setContentTitle(getText(R.string.mavlink_service_notification_title))
            .setContentText(getText(R.string.mavlink_service_notification_disconnected))
            .setSmallIcon(applicationContext.applicationInfo.icon)
            .setContentIntent(pendingIntent)
            .setSilent(true)
        startForeground(NOTIFICATION_ID, mNotificationBuilder!!.build())

        IS_RUNNING = true

        createMAVLinkConnectionInBackground()

        // Downward camera MJPEG stream for the ROS ground station
        if (getSharedPreferences("default_preferences", MODE_PRIVATE).getBoolean("down_cam_enabled", true)) {
            downCamStreamer = DownCamStreamer(port = 8090).also { it.start() }
        }

        return START_NOT_STICKY
    }

    override fun onDestroy() {
        LOG.i("Stopping Mavlink service")

        downCamStreamer?.stop()
        downCamStreamer = null

        compositeDisposable?.dispose()

        backgroundThread?.takeIf { it.isAlive }?.apply {
            close()
            interrupt()
        }

        IS_RUNNING = false

        if (BuildConfig.DEBUG) {
            Timber.uproot(fileLogger)
            fileLogger.close()
        }

        super.onDestroy()
    }

    override fun onBind(intent: Intent): IBinder {
        return binder
    }

    private fun createNotificationChannel() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val serviceChannel = NotificationChannel(
                CHANNEL_ID,
                "Foreground Service Channel",
                NotificationManager.IMPORTANCE_DEFAULT
            )
            mNotificationManager = getSystemService(NotificationManager::class.java)
            mNotificationManager?.createNotificationChannel(serviceChannel)
        }
    }

    private fun updateNotification(message: CharSequence) {
        mNotificationBuilder?.let {
            if (mNotificationManager == null) {
                mNotificationManager = getSystemService(NOTIFICATION_SERVICE) as NotificationManager
            }

            it.setContentText(message)
            mNotificationManager?.notify(NOTIFICATION_ID, it.build())
        }
    }

    // region MAVLink connection
    private fun createMAVLinkConnectionInBackground() {
        backgroundThread = MavConnection(applicationContext, connectionInformation!!, streamInformation!!, mavlinkCommunicationSubject)
        backgroundThread?.start()

        compositeDisposable = CompositeDisposable()
        compositeDisposable?.add(mavlinkCommunicationSubject
            .ofType(MavManager.MavlinkManagerCommunicationModel.ShowServiceNotification::class.java)
            .observeOn(AndroidSchedulers.mainThread())
            .subscribe(
                { s -> updateNotification(s.notification) },
                { error -> RxException(error.toString()).setLogTag(LOG.tag()) }
            )
        )
    }

    companion object {
        private val LOG = Logger("MavlinkService")
        private const val CHANNEL_ID: String = "MAVLinkService"
        private const val NOTIFICATION_ID: Int = 1
        var IS_RUNNING: Boolean = false
    }
}