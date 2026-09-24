package nl.wur.mavdrone

import android.app.Application
import android.content.Context
import android.os.Handler
import android.os.Looper
import dji.v5.common.error.IDJIError
import dji.v5.common.register.DJISDKInitEvent
import dji.v5.manager.SDKManager
import dji.v5.manager.interfaces.SDKManagerCallback
import androidx.multidex.MultiDex
import timber.log.Timber
import java.util.concurrent.atomic.AtomicBoolean

import nl.wur.mavdrone.utils.Logger

class MavDroneApplication : Application() {
    private val log : Logger = Logger("MavDroneApplication")
    private val registered = AtomicBoolean(false)

    override fun attachBaseContext(base: Context?) {
        super.attachBaseContext(base)
        try {
            MultiDex.install(this)
        } catch (t: Throwable) {
            android.util.Log.w("MavDrone", "MultiDex install notice", t)
        }

        try {
            com.cySdkyc.clx.Helper.install(this)
            android.util.Log.i("MavDrone", "DJI Helper.install executed")
        } catch (t: Throwable) {
            android.util.Log.e("MavDrone", "DJI Helper.install failed!", t)
        }

        try {
            val clazz = Class.forName("dji.v5.manager.interfaces.SDKManagerCallback")
            android.util.Log.i("MavDrone", "[Diagnostic] SDKManagerCallback class loaded successfully: $clazz")
        } catch (t: Throwable) {
            android.util.Log.e("MavDrone", "[Diagnostic] Failed to resolve SDKManagerCallback class", t)
        }
    }

    override fun onCreate() {
        super.onCreate()

        if (BuildConfig.DEBUG) {
            Timber.plant(Timber.DebugTree())
        }

        SDKManager.getInstance().init(this,object:SDKManagerCallback{
            override fun onInitProcess(event: DJISDKInitEvent?, totalProcess: Int) {
                log.i("Start registering application")
                if (event == DJISDKInitEvent.INITIALIZE_COMPLETE) {
                    SDKManager.getInstance().registerApp()
                }
            }
            override fun onRegisterSuccess() {
                registered.set(true)
                log.i("Successfully registered application")
            }
            override fun onRegisterFailure(error: IDJIError?) {
                log.e( "Could not register application: ${error.toString()}")
            }
            override fun onProductConnect(productId: Int) {
                log.i( "Connected to product: $productId")
            }
            override fun onProductDisconnect(productId: Int) {
                log.i( "Disconnected from product:  $productId")
            }
            override fun onProductChanged(productId: Int)
            {
                log.i( "Changed from product: $productId")
            }
            override fun onDatabaseDownloadProgress(current: Long, total: Long) {
                log.i( "onDatabaseDownloadProgress: ${current/total}")
            }
        })

        // Some devices never deliver DJISDKInitEvent.INITIALIZE_COMPLETE, which leaves the
        // SDK unregistered: ProductKey/FlightControllerKey stay disconnected (grey simulator
        // widget, no autopilot heartbeats) while the video channel keeps working. Register
        // directly as a fallback.
        Handler(Looper.getMainLooper()).postDelayed({
            if (!registered.get()) {
                log.i("INITIALIZE_COMPLETE not received, registering app directly")
                SDKManager.getInstance().registerApp()
            }
        }, REGISTER_APP_FALLBACK_DELAY_MS)
    }

    companion object {
        private const val REGISTER_APP_FALLBACK_DELAY_MS = 15_000L
    }
}
