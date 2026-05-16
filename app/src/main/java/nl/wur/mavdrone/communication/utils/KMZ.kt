package nl.wur.mavdrone.communication.utils

import android.annotation.SuppressLint
import dji.sdk.wpmz.jni.JNIWPMZManager
import dji.sdk.wpmz.value.mission.WaylineCheckError
import dji.v5.manager.aircraft.waypoint3.WPMZParserManager
import dji.v5.utils.common.ContextUtil
import io.reactivex.rxjava3.core.Completable
import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.BaseComponent
import nl.wur.mavdrone.communication.utils.FileUtils.Companion.createFolderIfNeeded
import nl.wur.mavdrone.utils.RxException.WriteException
import nl.wur.mavdrone.communication.utils.FileUtils.Companion.getExternalCacheDirPath
import java.io.File
import java.lang.ref.WeakReference
import java.text.SimpleDateFormat
import java.util.Date

class KMZ(connectionWR: WeakReference<MavConnection>, private val kmzFile: File) :
    BaseComponent(connectionWR, "MavKMZGenerator") {
    private val tmpFolder: String = getExternalCacheDirPath(TMP_FILE_DIR)
    private val generatedWPMZFolder: String

    init {
        createFolderIfNeeded(tmpFolder)
        createFolderIfNeeded(tmpFolder + WPMZ_DIR)

        generatedWPMZFolder = getExternalCacheDirPath(WAYPOINT_FILE_DIR)
        createFolderIfNeeded(generatedWPMZFolder)
    }

    constructor(connectionWR: WeakReference<MavConnection>, path: String) : this(
        connectionWR,
        File(path)
    )

    fun extractKMZ(): Completable {
        return Completable.fromAction {
            WPMZParserManager.unZipFolder(
                ContextUtil.getContext(),
                kmzFile.path,
                tmpFolder + WPMZ_DIR,
                true
            )
        }
    }

    fun createKMZ(): Completable {
        return Completable.fromAction {
            try {
                WPMZParserManager.zipFiles(
                    ContextUtil.getContext(),
                    listOf(tmpFolder + WPMZ_DIR),
                    kmzFile.path
                )
            } catch (e: Exception) {
                throw WriteException("Cannot write KMZ: " + e.message).setLogger(log)
            }
        }
    }

    val waylineFilePath: String
        get() = tmpFolder + WPMZ_DIR + WPMZParserManager.WAYLINE_FILE

    val templateFilePath: String
        get() = tmpFolder + WPMZ_DIR + WPMZParserManager.TEMPLATE_FILE

    val geofenceFilePath: String
        get() = tmpFolder + WPMZ_DIR + "geofence.xml"

    val path: String
        get() = kmzFile.path

    val name: String
        get() = kmzFile.name

    fun checkIfValid(): Completable {
        return Completable.fromAction {
            val errors = StringBuilder()
            for (error in JNIWPMZManager.checkWPMZValid(
                WAYPOINT_FILE_VERSION,
                kmzFile.path
            ).value) {
                if (error == WaylineCheckError.NoError) {
                    continue
                }

                errors.append(error.name).append(", ")
            }
            if (errors.isNotEmpty()) {
                errors.setLength(errors.length - 2)
                if (!kmzFile.delete()) {
                    log.w("Could not delete " + kmzFile.path)
                }
                throw WriteException("Generated KMZ is not valid: $errors!")
            }
        }
    }

    companion object {
        const val WAYPOINT_FILE_VERSION: String = "1.0.0"
        const val WPMZ_DIR: String = "wpmz/"
        const val WAYPOINT_FILE_DIR: String = "GeneratedWMPZ/"
        const val TMP_FILE_DIR: String = "TmpFiles/"
        const val KMZ_EXT: String = ".kmz"

        
        @SuppressLint("SimpleDateFormat")
        fun create(connectionWR: WeakReference<MavConnection>, suffix: String): KMZ {
            val generatedWPMZFolder = getExternalCacheDirPath(WAYPOINT_FILE_DIR)
            createFolderIfNeeded(generatedWPMZFolder)

            val dateFormat = SimpleDateFormat("yyyy-MM-dd").format(Date())
            val WPMZStorageFolder = "$generatedWPMZFolder$dateFormat/"
            createFolderIfNeeded(WPMZStorageFolder)

            val kmzFileName = SimpleDateFormat("HH-mm-ss").format(Date())
            val kmzFilePath = WPMZStorageFolder + kmzFileName + suffix + KMZ_EXT

            return KMZ(connectionWR, kmzFilePath)
        }

        
        fun create(connectionWR: WeakReference<MavConnection>): KMZ {
            return create(connectionWR, "")
        }
    }
}
