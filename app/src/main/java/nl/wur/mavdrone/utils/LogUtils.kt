package nl.wur.mavdrone.utils

import org.jetbrains.annotations.NonNls

import timber.log.Timber


class Logger(private var logName: String) {
    fun d(t: Throwable?) {
        Timber.tag(logName).d(t)
    }

    fun d(@NonNls message: String?, vararg args: Any?) {
        Timber.tag(logName).d(message, *args)
    }

    fun d(t: Throwable?, @NonNls message: String?, vararg args: Any?) {
        Timber.tag(logName).d(t, message, *args)
    }

    fun i(t: Throwable?) {
        Timber.tag(logName).i(t)
    }

    fun i(@NonNls message: String?, vararg args: Any?) {
        Timber.tag(logName).i(message, *args)
    }

    fun i(t: Throwable?, @NonNls message: String?, vararg args: Any?) {
        Timber.tag(logName).i(t, message, *args)
    }

    fun w(t: Throwable?) {
        Timber.tag(logName).w(t)
    }

    fun w(@NonNls message: String?, vararg args: Any?) {
        Timber.tag(logName).w(message, *args)
    }

    fun w(t: Throwable?, @NonNls message: String?, vararg args: Any?) {
        Timber.tag(logName).w(t, message, *args)
    }

    fun e(t: Throwable?) {
        Timber.tag(logName).e(t)
    }

    fun e(@NonNls message: String?, vararg args: Any?) {
        Timber.tag(logName).e(message, *args)
    }

    fun e(t: Throwable?, @NonNls message: String?, vararg args: Any?) {
        Timber.tag(logName).e(t, message, *args)
    }

    fun tag(): String {
        return logName
    }
}
