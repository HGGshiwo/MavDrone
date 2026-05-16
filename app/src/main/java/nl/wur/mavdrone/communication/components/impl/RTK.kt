package nl.wur.mavdrone.communication.components.impl

import dji.sdk.keyvalue.value.common.LocationCoordinate3D
import dji.v5.common.callback.CommonCallbacks.CompletionCallback
import dji.v5.manager.aircraft.rtk.RTKCenter
import dji.v5.manager.aircraft.rtk.RTKLocationInfo
import dji.v5.manager.aircraft.rtk.RTKSystemState
import dji.v5.manager.aircraft.simulator.SimulatorManager
import io.reactivex.rxjava3.core.Observable
import io.reactivex.rxjava3.core.Single
import io.reactivex.rxjava3.schedulers.Schedulers
import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.BaseComponent
import nl.wur.mavdrone.communication.components.MavBaseComponent
import nl.wur.mavdrone.utils.RXUtils
import nl.wur.mavdrone.utils.RXUtils.Companion.djiCompletableFunction
import java.lang.ref.WeakReference
import java.util.concurrent.TimeUnit
import java.util.function.BiConsumer
import java.util.function.Supplier

class RTK(
    connectionWR: WeakReference<MavConnection>,
    private val componentWR: WeakReference<out MavBaseComponent>
) : BaseComponent(connectionWR, "mavRTK") {
    private var rtkWasEnabled: Boolean? = null

        var isRtkEnabled: Boolean = false

        var isRtkHealthy: Boolean = false

        var rtkLocation: LocationCoordinate3D? = null

    override fun create() {
        RTKCenter.getInstance().addRTKLocationInfoListener { rtkLocationInfo: RTKLocationInfo ->
            this.onRTKLocationInfo(rtkLocationInfo)
        }
        RTKCenter.getInstance().addRTKSystemStateListener { rtkSystemState: RTKSystemState ->
            this.onRTKSystemStateListener(rtkSystemState)
        }

        // Disable RTK in simulation mode, since that's not working
        compositeDisposable.add(Observable
            .interval(2000, 5000, TimeUnit.MILLISECONDS)
            .subscribeOn(Schedulers.io())
            .observeOn(Schedulers.io())
            .flatMapSingle<Boolean> { Single.fromCallable<Boolean> { SimulatorManager.getInstance().isSimulatorEnabled } }
            .distinctUntilChanged()
            .flatMapSingle<Boolean> { getEnableRTKStateBySimulator(it) }
            .filter { newRTKState: Boolean ->
                if (newRTKState != isRtkEnabled) {
                    log.i("Changing RTK state to " + (if (newRTKState) "enabled" else "disabled"))
                    return@filter true
                }
                return@filter false
            }
            .flatMapCompletable { newRTKState: Boolean ->
                djiCompletableFunction(
                    { isEnabled: Boolean, callback: CompletionCallback? ->
                        RTKCenter.getInstance().setAircraftRTKModuleEnabled(
                            isEnabled, callback
                        )
                    }, { newRTKState })
            }
            .subscribe(
                { log.w("Changed RTK state because simulation changed!") },  // TODO: this does not emit somehow
                { RXUtils.handleError(it) }
            )
        )
    }

    override fun destroy() {
        RTKCenter.getInstance().removeRTKLocationInfoListener(this::onRTKLocationInfo)
        RTKCenter.getInstance().removeRTKSystemStateListener(this::onRTKSystemStateListener)

        super.destroy()
    }

    private fun onRTKLocationInfo(rtkLocationInfo: RTKLocationInfo) {
        rtkLocation = rtkLocationInfo.real3DLocation
    }

    private fun onRTKSystemStateListener(rtkSystemState: RTKSystemState) {
        if (rtkWasEnabled == null) {
            rtkWasEnabled = rtkSystemState.isRTKEnabled
        }

        isRtkEnabled = rtkSystemState.isRTKEnabled
        isRtkHealthy = rtkSystemState.rtkHealthy
    }

    private fun getEnableRTKStateBySimulator(simulatorEnabled: Boolean): Single<Boolean> {
        return Single.fromCallable {
            if (simulatorEnabled) {
                rtkWasEnabled = isRtkEnabled
                return@fromCallable false
            } else {
                return@fromCallable rtkWasEnabled ?: false
            }
        }
    }
}
