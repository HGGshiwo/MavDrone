package nl.wur.mavdrone.utils

import android.annotation.SuppressLint
import android.content.Context
import android.util.Log
import io.reactivex.rxjava3.disposables.CompositeDisposable
import io.reactivex.rxjava3.schedulers.Schedulers
import io.reactivex.rxjava3.subjects.PublishSubject
import nl.wur.mavdrone.communication.utils.FileUtils.Companion.getExternalCacheDirPath
import timber.log.Timber
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.logging.Logger

@SuppressLint("LogNotTimber")
class FileLogTree(var context: Context?) : Timber.Tree() {
    private val compositeDisposable = CompositeDisposable()
    private val writeQueue = PublishSubject.create<String>()

    private val logfile: File? by lazy { context?.let { generateFile(it) } }

    init {
        compositeDisposable.add(writeQueue
            .observeOn(Schedulers.io())
            .doOnNext { logMessage ->
                logfile?.appendText(logMessage)
            }
            .doOnError { error -> Log.e(TAG, "Could not write to log file: $error") }
            .onErrorComplete()
            .subscribe()
        )
    }

    override fun log(priority: Int, tag: String?, message: String, t: Throwable?) {
        if (priority >= Log.INFO && tag != null && tag.startsWith("Mav")) {
            val logTimeStamp: String =
                SimpleDateFormat("dd-MM-yyyy hh:mm:ss:SSS", Locale.getDefault())
                    .format(Date())

            writeQueue.onNext("$logTimeStamp : $tag - $message\n")
        }
    }

    fun close() {
        compositeDisposable.dispose()
    }

    companion object {
        const val TAG = "MavFileLogging"

        private fun generateFile(context: Context): File {
            val logFolder = getExternalCacheDirPath(context, "MavlinkLogs")
            val logFolderFile = File(logFolder)

            if (!logFolderFile.exists() && !logFolderFile.mkdir()) {
                Logger.getGlobal().warning("Cannot create save folder for log files!")
            }

            val fileNameTimeStamp: String =
                SimpleDateFormat("dd-MM-yyyy-hh:mm:ss", Locale.getDefault()).format(Date())
            val fileName = "$fileNameTimeStamp.txt"

            return File(logFolder, fileName)
        }
    }
}