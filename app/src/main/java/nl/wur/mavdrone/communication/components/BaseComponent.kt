package nl.wur.mavdrone.communication.components

import java.lang.ref.WeakReference

import io.reactivex.rxjava3.disposables.CompositeDisposable

import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.utils.Logger


abstract class BaseComponent(
protected val connectionWR: WeakReference<MavConnection>, logName: String) {
        val log: Logger = Logger(logName)

        val systemID: Int = connectionWR.get()!!.connectionInformation.systemId

        protected var compositeDisposable: CompositeDisposable = CompositeDisposable()

    open fun create() {}

    open fun destroy() {
        compositeDisposable.dispose()
    }

    fun onGCSConnected() {
    }

    open fun onGCSDisconnected() {
    }

    companion object {
        const val UINT16_MAX: Int = 65535
        const val INT32_MAX: Int = 2147483647
    }
}
