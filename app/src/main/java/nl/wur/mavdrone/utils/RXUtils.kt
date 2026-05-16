package nl.wur.mavdrone.utils

import android.os.Handler
import android.os.Looper

import java.util.concurrent.TimeUnit
import java.util.function.BiConsumer
import java.util.function.Consumer
import java.util.function.Supplier

import dji.sdk.keyvalue.key.DJIActionKeyInfo
import dji.sdk.keyvalue.key.DJIKey
import dji.sdk.keyvalue.key.DJIKey.ActionKey
import dji.sdk.keyvalue.key.DJIKeyInfo
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.v5.common.callback.CommonCallbacks.CompletionCallback
import dji.v5.common.callback.CommonCallbacks.CompletionCallbackWithParam
import dji.v5.common.callback.CommonCallbacks.CompletionCallbackWithProgress
import dji.v5.common.error.IDJIError
import dji.v5.manager.KeyManager
import dji.v5.utils.common.ContextUtil
import dji.v5.ux.core.util.ViewUtil

import io.reactivex.rxjava3.core.Completable
import io.reactivex.rxjava3.core.CompletableEmitter
import io.reactivex.rxjava3.core.Observable
import io.reactivex.rxjava3.core.Single
import io.reactivex.rxjava3.core.SingleEmitter
import timber.log.Timber

class RXUtils {
    companion object {
        fun <VarType> waitForVariable(
            valueFunction: Supplier<VarType>,
            requiredValue: VarType,
            interval: Long = 50L
        ): Completable {
            return Observable
                .interval(0, interval, TimeUnit.MILLISECONDS)
                .filter { valueFunction.get() == requiredValue }
                .take(1)
                .ignoreElements()
        }

        fun handleErrorResume(throwable: Throwable): Completable {
            return Completable.fromCallable { handleError(throwable) }
        }

        fun handleError(throwable: Throwable) {
            if (throwable is RxException) {
                if (throwable.logTag.isEmpty()) {
                    Timber.tag("MavRxUtils").e(throwable)
                } else {
                    Timber.tag(throwable.logTag).e("%s: %s", throwable.javaClass.simpleName, throwable.message)
                }

                if (throwable.message != null && throwable.showToast) {
                    val handler = Handler(Looper.getMainLooper())
                    handler.post {
                        ViewUtil.showToast(
                            ContextUtil.getContext(),
                            throwable.message,
                            5
                        )
                    }
                }
            } else {
                Timber.tag("MavRxUtils").e("%s: %s", throwable.javaClass.name, throwable.message)
            }
        }

        fun <ParamType : Any> djiGetValue(djiKey: DJIKey<ParamType>): Single<ParamType> {
            return Single.create { subscriber: SingleEmitter<ParamType> ->
                KeyManager.getInstance()
                    .getValue(djiKey, object : CompletionCallbackWithParam<ParamType> {
                        override fun onSuccess(t: ParamType) {
                            subscriber.onSuccess(t)
                        }

                        override fun onFailure(error: IDJIError) {
                            subscriber.onError(RxException.DJIException.fromDJI(error))
                        }
                    })
            }
        }

        fun <ParamType : Any> djiGetValue(djiKey: DJIKeyInfo<ParamType>): Single<ParamType> {
            return djiGetValue(KeyTools.createKey(djiKey))
        }

        fun <ParamType : Any> djiGetValue(djiFunction: Consumer<CompletionCallbackWithParam<ParamType>>): Single<ParamType> {
            return Single.create { subscriber: SingleEmitter<ParamType> ->
                djiFunction.accept(object : CompletionCallbackWithParam<ParamType> {
                    override fun onSuccess(t: ParamType) {
                        subscriber.onSuccess(t)
                    }

                    override fun onFailure(error: IDJIError) {
                        subscriber.onError(RxException.DJIException.fromDJI(error))
                    }
                })
            }
        }

        fun <ParamType : Any> djiSetValue(
            djiKey: DJIKeyInfo<ParamType>,
            value: ParamType
        ): Completable {
            return djiSetValue(KeyTools.createKey(djiKey), value)
        }

        fun <ParamType : Any> djiSetValue(
            djiKey: DJIKey<ParamType>,
            value: ParamType
        ): Completable {
            return Completable.create { emitter: CompletableEmitter ->
                KeyManager.getInstance().setValue(djiKey, value, object : CompletionCallback {
                    override fun onSuccess() {
                        emitter.onComplete()
                    }

                    override fun onFailure(error: IDJIError) {
                        emitter.onError(RxException.DJIException.fromDJI(error))
                    }
                })
            }
        }

        fun <ParamType : Any, ResultType : Any> djiPerformAction(
            djiKey: ActionKey<ParamType, ResultType>,
            param: ParamType
        ): Completable {
            return Completable.create { emitter: CompletableEmitter ->
                KeyManager.getInstance()
                    .performAction(djiKey, param, object : CompletionCallbackWithParam<ResultType> {
                        override fun onSuccess(emptyMsg: ResultType) {
                            emitter.onComplete()
                        }

                        override fun onFailure(error: IDJIError) {
                            emitter.onError(RxException.DJIException.fromDJI(error))
                        }
                    })
            }
        }

        fun <ParamType : Any, ResultType : Any> djiPerformAction(
            djiKey: DJIActionKeyInfo<ParamType, ResultType>,
            param: ParamType
        ): Completable {
            return djiPerformAction(KeyTools.createKey(djiKey), param)
        }

        fun <ResultType : Any> djiPerformAction(djiKey: DJIActionKeyInfo<EmptyMsg, ResultType>): Completable {
            return djiPerformAction(KeyTools.createKey(djiKey))
        }

        fun <ResultType : Any> djiPerformAction(djiKey: ActionKey<*, ResultType>): Completable {
            return Completable.create { emitter: CompletableEmitter ->
                KeyManager.getInstance()
                    .performAction(djiKey, object : CompletionCallbackWithParam<ResultType> {
                        override fun onSuccess(emptyMsg: ResultType) {
                            emitter.onComplete()
                        }

                        override fun onFailure(error: IDJIError) {
                            emitter.onError(RxException.DJIException.fromDJI(error))
                        }
                    })
            }
        }

        fun <ParamType : Any> djiCompletableFunction(
            djiFunction: BiConsumer<ParamType, CompletionCallback>,
            param: Supplier<ParamType>
        ): Completable {
            return Completable.create { emitter: CompletableEmitter ->
                djiFunction.accept(param.get(), object : CompletionCallback {
                    override fun onSuccess() {
                        emitter.onComplete()
                    }

                    override fun onFailure(error: IDJIError) {
                        emitter.onError(RxException.DJIException.fromDJI(error))
                    }
                })
            }
        }

        fun <ParamType1 : Any, ParamType2 : Any> djiCompletableFunction(
            djiFunction: TriConsumer<ParamType1, ParamType2, CompletionCallback>,
            param1: Supplier<ParamType1>,
            param2: Supplier<ParamType2>
        ): Completable {
            return Completable.create { emitter: CompletableEmitter ->
                djiFunction.accept(param1.get(), param2.get(), object : CompletionCallback {
                    override fun onSuccess() {
                        emitter.onComplete()
                    }

                    override fun onFailure(error: IDJIError) {
                        emitter.onError(RxException.DJIException.fromDJI(error))
                    }
                })
            }
        }

        fun djiCompletableFunction(djiFunction: Consumer<CompletionCallback>): Completable {
            return Completable.create { emitter: CompletableEmitter ->
                djiFunction.accept(object : CompletionCallback {
                    override fun onSuccess() {
                        emitter.onComplete()
                    }

                    override fun onFailure(error: IDJIError) {
                        emitter.onError(RxException.DJIException.fromDJI(error))
                    }
                })
            }
        }

        fun <ParamType : Any, ProgressType : Any> djiCompletableFunctionWithProgress(
            djiFunction: BiConsumer<ParamType, CompletionCallbackWithProgress<ProgressType>>,
            progressCallback: Consumer<ProgressType>,
            value: Supplier<ParamType>
        ): Completable {
            return Completable.create { emitter: CompletableEmitter ->
                djiFunction.accept(
                    value.get(),
                    object : CompletionCallbackWithProgress<ProgressType> {
                        override fun onProgressUpdate(progress: ProgressType) {
                            progressCallback.accept(progress)
                        }

                        override fun onSuccess() {
                            emitter.onComplete()
                        }

                        override fun onFailure(error: IDJIError) {
                            emitter.onError(
                                RxException.DJIException.fromDJI(error).setShowToast(false)
                            )
                        }
                    })
            }
        }
    }
}
