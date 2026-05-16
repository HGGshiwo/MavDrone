package nl.wur.mavdrone

import android.app.Application
import android.content.Context
import dji.v5.common.error.IDJIError
import dji.v5.common.register.DJISDKInitEvent
import dji.v5.manager.SDKManager
import dji.v5.manager.interfaces.SDKManagerCallback
import timber.log.Timber

import nl.wur.mavdrone.utils.Logger

class MavDroneApplication : Application() {
    private val log : Logger = Logger("MavDroneApplication")

    override fun attachBaseContext(base: Context?) {
        super.attachBaseContext(base)
        com.cySdkyc.clx.Helper.install(this)
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
    }
}
