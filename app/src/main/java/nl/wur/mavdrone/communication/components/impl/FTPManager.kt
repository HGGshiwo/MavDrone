package nl.wur.mavdrone.communication.components.impl

import java.io.File
import java.io.IOException
import java.io.RandomAccessFile
import java.lang.ref.WeakReference
import java.nio.charset.StandardCharsets
import java.util.concurrent.TimeUnit
import java.util.regex.Pattern
import java.util.zip.CRC32
import kotlin.math.min

import dji.v5.common.callback.CommonCallbacks.CompletionCallback
import dji.v5.manager.datacenter.media.MediaFile
import dji.v5.manager.datacenter.media.MediaFileFilter
import dji.v5.manager.datacenter.media.MediaFileListState
import dji.v5.manager.datacenter.media.MediaManager
import dji.v5.manager.datacenter.media.PullMediaFileListParam
import dji.v5.utils.common.ContextUtil
import dji.v5.utils.common.DiskUtil

import io.reactivex.rxjava3.core.Completable
import io.reactivex.rxjava3.core.CompletableEmitter
import io.reactivex.rxjava3.core.Observable
import io.reactivex.rxjava3.core.Single
import io.reactivex.rxjava3.schedulers.Schedulers

import io.dronefleet.mavlink.MavlinkMessage
import io.dronefleet.mavlink.common.FileTransferProtocol
import io.dronefleet.mavlink.util.EnumValue

import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.BaseComponent
import nl.wur.mavdrone.communication.components.MavBaseComponent
import nl.wur.mavdrone.communication.utils.FTPProtocol
import nl.wur.mavdrone.utils.ByteUtils.Companion.intToBytes
import nl.wur.mavdrone.utils.RXUtils.Companion.djiCompletableFunction

class FTPManager(
    connectionWR: WeakReference<MavConnection>,
    private val componentWR: WeakReference<out MavBaseComponent>
) : BaseComponent(connectionWR, "MavFTPManager") {
    private val activeSessions: MutableMap<Int, FTPSession> = HashMap()

    private var mediaManagerEnabled = false

    private var componentIdValue: Int = 1

    override fun create() {
        componentWR.get()?.apply {
            addMavlinkListener(FileTransferProtocol::class.java, this@FTPManager::onFileTransferProtocol)
            componentIdValue = EnumValue.create(componentID).value()
        }
    }

    private fun onFileTransferProtocol(msg: MavlinkMessage<*>) {
        val payload = msg.payload as FileTransferProtocol

        if (payload.targetSystem() == systemID && (payload.targetComponent() == componentIdValue)
        ) {
            val sessionID = payload.payload()[FTPProtocol.FTP_SESSION_ID_BYTE].toInt()
            val opCode = payload.payload()[FTPProtocol.FTP_OPCODE_BYTE].toInt()

            val ftpSession = activeSessions[sessionID] ?: FTPSession()

            when (opCode) {
                FTPProtocol.NONE -> {
                    log.d("NONE")

                    val cmdNone = createCommandAck(null, getHeaderFromPayload(payload.payload()))
                    sendCommand(cmdNone, msg.originSystemId, msg.originComponentId)
                }

                FTPProtocol.TERMINATE_SESSION -> {
                    log.d("TERMINATE_SESSION")

                    compositeDisposable.add(
                        ftpSession.terminate(payload.payload())
                            .subscribe(
                                {
                                    sendCommand(it, msg.originSystemId, msg.originComponentId)
                                    activeSessions.remove(sessionID)
                                },
                                { error ->
                                    val cmdNak = createCommandNak(FTPProtocol.FAIL,0,getHeaderFromPayload(payload.payload()))
                                    sendCommand(cmdNak, msg.originSystemId, msg.originComponentId)
                                    log.e("TERMINATE_SESSION fail: ${error.message}")
                                }
                            )
                    )
                }

                FTPProtocol.RESET_SESSIONS -> {
                    log.d("RESET_SESSIONS")

                    activeSessions.values.forEach { session ->
                        compositeDisposable.add(
                            session.terminate(payload.payload())
                                .subscribeOn(Schedulers.io())
                                .subscribe(
                                    { sendCommand(it, payload.targetSystem(), payload.targetComponent())},
                                    { error ->
                                        val cmdNak = createCommandNak(FTPProtocol.FAIL, 0, getHeaderFromPayload(payload.payload()))
                                        sendCommand(cmdNak, msg.originSystemId, msg.originComponentId)
                                        log.e("RESET_SESSIONS fail: ${error.message}")
                                    }
                                )
                        )
                    }
                    activeSessions.clear()
                }

                FTPProtocol.LIST_DIRECTORY -> {
                    log.d("LIST_DIRECTORY")

                    compositeDisposable.add(
                        ftpSession.listDirectory(payload.payload())
                            .observeOn(Schedulers.io())
                            .subscribe(
                                { sendCommand(it, msg.originSystemId, msg.originComponentId) },
                                { error ->
                                    val cmdNak = createCommandNak(FTPProtocol.FAIL,0, getHeaderFromPayload(payload.payload()))
                                    sendCommand(cmdNak, msg.originSystemId, msg.originComponentId)
                                    log.e("LIST_DIRECTORY fail: ${error.message}")
                                }
                            )
                    )
                }

                FTPProtocol.OPEN_FILE_RO -> {
                    log.d("OPEN_FILE_RO")

                    compositeDisposable.add(
                        ftpSession.openFileRO(payload.payload())
                            .observeOn(Schedulers.io())
                            .subscribe(
                                { sendCommand(it, msg.originSystemId, msg.originComponentId) },
                                { error ->
                                    val cmdNak = createCommandNak(FTPProtocol.FAIL, 0, getHeaderFromPayload(payload.payload()))
                                    sendCommand(cmdNak, msg.originSystemId, msg.originComponentId)
                                    log.e("OPEN_FILE_RO fail: ${error.message}")
                                }
                            )
                    )
                }

                FTPProtocol.READ_FILE -> {
                    log.d("READ_FILE")

                    compositeDisposable.add(
                        ftpSession.readFile(payload.payload())
                            .observeOn(Schedulers.io())
                            .subscribe(
                                { sendCommand(it, msg.originSystemId, msg.originComponentId)},
                                { error ->
                                    val cmdNak = createCommandNak(FTPProtocol.FAIL, 0, getHeaderFromPayload(payload.payload()))
                                    sendCommand(cmdNak, msg.originSystemId, msg.originComponentId)
                                    log.e("READ_FILE fail: ${error.message}")
                                }
                            )
                    )
                }

                FTPProtocol.CREATE_FILE -> {
                    log.d("CREATE_FILE")

                    compositeDisposable.add(
                        ftpSession.createFile(payload.payload())
                            .observeOn(Schedulers.io())
                            .subscribe(
                                { sendCommand(it, msg.originSystemId, msg.originComponentId) },
                                { error ->
                                    val cmdNak = createCommandNak(FTPProtocol.FAIL, 0, getHeaderFromPayload(payload.payload()))
                                    sendCommand(cmdNak, msg.originSystemId, msg.originComponentId)
                                    log.e("CREATE_FILE fail: ${error.message}")
                                }
                            )
                    )
                }

                FTPProtocol.WRITE_FILE -> {
                    log.d("WRITE_FILE")

                    compositeDisposable.add(
                        ftpSession.writeFile(payload.payload())
                            .observeOn(Schedulers.io())
                            .subscribe(
                                { sendCommand(it, msg.originSystemId, msg.originComponentId) },
                                { error ->
                                    val cmdNak = createCommandNak(FTPProtocol.FAIL, 0, getHeaderFromPayload(payload.payload()))
                                    sendCommand(cmdNak, msg.originSystemId, msg.originComponentId)
                                    log.e("WRITE_FILE fail: ${error.message}")
                                }
                            )
                    )
                }

                FTPProtocol.REMOVE_FILE -> {
                    log.d("REMOVE_FILE")

                    compositeDisposable.add(
                        ftpSession.removeFile(payload.payload())
                            .observeOn(Schedulers.io())
                            .subscribe(
                                { sendCommand(it, msg.originSystemId, msg.originComponentId) },
                                { error ->
                                    val cmdNak = createCommandNak(FTPProtocol.FAIL, 0, getHeaderFromPayload(payload.payload()))
                                    sendCommand(cmdNak, msg.originSystemId, msg.originComponentId)
                                    log.e("REMOVE_FILE fail: ${error.message}")
                                }
                            )
                    )
                }

                FTPProtocol.CREATE_DIRECTORY -> {
                    log.d("CREATE_DIRECTORY")

                    compositeDisposable.add(
                        ftpSession.createDirectory(payload.payload())
                            .observeOn(Schedulers.io())
                            .subscribe(
                                { sendCommand(it, msg.originSystemId, msg.originComponentId) },
                                { error ->
                                    val cmdNak = createCommandNak(FTPProtocol.FAIL, 0, getHeaderFromPayload(payload.payload()))
                                    sendCommand(cmdNak, msg.originSystemId, msg.originComponentId)
                                    log.e("CREATE_DIRECTORY fail: ${error.message}")
                                }
                            )
                    )
                }

                FTPProtocol.REMOVE_DIRECTORY -> {
                    log.d("REMOVE_DIRECTORY")

                    compositeDisposable.add(
                        ftpSession.removeDirectory(payload.payload())
                            .observeOn(Schedulers.io())
                            .subscribe(
                                { sendCommand(it, msg.originSystemId, msg.originComponentId) },
                                { error ->
                                    val cmdNak = createCommandNak(FTPProtocol.FAIL, 0, getHeaderFromPayload(payload.payload()))
                                    sendCommand(cmdNak, msg.originSystemId, msg.originComponentId)
                                    log.e("REMOVE_DIRECTORY fail: ${error.message}")
                            })
                    )
                }

                FTPProtocol.OPEN_FILE_WO -> {
                    log.d("OPEN_FILE_WO")

                    compositeDisposable.add(
                        ftpSession.openFileWO(payload.payload())
                            .observeOn(Schedulers.io())
                            .subscribe(
                                { sendCommand(it, msg.originSystemId, msg.originComponentId) },
                                { error ->
                                    val cmdNak = createCommandNak(FTPProtocol.FAIL, 0, getHeaderFromPayload(payload.payload()))
                                    sendCommand(cmdNak, msg.originSystemId, msg.originComponentId)
                                    log.e("OPEN_FILE_WO fail: ${error.message}")
                            })
                    )
                }

                FTPProtocol.TRUNCATE_FILE -> {
                    log.d("TRUNCATE_FILE")

                    compositeDisposable.add(
                        ftpSession.truncateFile(payload.payload())
                            .observeOn(Schedulers.io())
                            .subscribe(
                                { sendCommand(it, msg.originSystemId, msg.originComponentId) },
                                { error ->
                                    val cmdNak = createCommandNak(FTPProtocol.FAIL, 0, getHeaderFromPayload(payload.payload()))
                                    sendCommand(cmdNak, msg.originSystemId, msg.originComponentId)
                                    log.e("TRUNCATE_FILE fail: ${error.message}")
                            })
                    )
                }

                FTPProtocol.RENAME -> {
                    log.d("RENAME")

                    compositeDisposable.add(
                        ftpSession.rename(payload.payload())
                            .observeOn(Schedulers.io())
                            .subscribe(
                                { sendCommand(it, msg.originSystemId, msg.originComponentId) },
                                { error ->
                                    val cmdNak = createCommandNak(FTPProtocol.FAIL, 0, getHeaderFromPayload(payload.payload()))
                                    sendCommand(cmdNak, msg.originSystemId, msg.originComponentId)
                                    log.e("RENAME fail: ${error.message}")
                            })
                    )
                }

                FTPProtocol.CALC_FILE_CRC32 -> {
                    log.d("CALC_FILE_CRC32")

                    compositeDisposable.add(
                        ftpSession.calculateFileCRC32(payload.payload())
                            .observeOn(Schedulers.io())
                            .subscribe(
                                { sendCommand(it, msg.originSystemId, msg.originComponentId) },
                                { error ->
                                    val cmdNak = createCommandNak(FTPProtocol.FAIL, 0, getHeaderFromPayload(payload.payload()))
                                    sendCommand(cmdNak, msg.originSystemId, msg.originComponentId)
                                    log.e("CALC_FILE_CRC32 fail: ${error.message}")
                            })
                    )
                }

                FTPProtocol.BURST_READ_FILE -> {
                    log.d("BURST_READ_FILE")

                    compositeDisposable.add(
                        ftpSession.burstReadFile(
                            payload.payload(),
                            msg.originSystemId,
                            msg.originComponentId
                        )
                            .observeOn(Schedulers.io())
                            .subscribe(
                                { },
                                { error ->
                                    val cmdNak = createCommandNak(FTPProtocol.FAIL, 0, getHeaderFromPayload(payload.payload()))
                                    sendCommand(cmdNak, msg.originSystemId, msg.originComponentId)
                                    log.e("BURST_READ_FILE FAIL: (${error.javaClass.name}) ${error.message}")
                            })
                    )
                }

                else -> {
                    log.e("Unknown FTP command: $opCode")

                    val cmdDef = createCommandNak(FTPProtocol.UNKNOWN_COMMAND, 0, getHeaderFromPayload(payload.payload()))
                    sendCommand(cmdDef, msg.originSystemId, msg.originComponentId)
                }
            }
        }
    }

    private fun sendCommand(payload: ByteArray?, targetSystem: Int, targetComponent: Int) {
        payload?.let {
            val fileTransferProtocolMessage = FileTransferProtocol.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .payload(payload)
                .build()
            componentWR.get()?.sendMessage(fileTransferProtocolMessage)
        }
    }

    private inner class FTPSession {
        private val ftpFileList: MutableList<FTPFile> = ArrayList()

        private var openFile: File? = null
        private var openAccessFile: RandomAccessFile? = null

        private var sessionID = 0

        init {
            // Create unique session id
            while (activeSessions.containsKey(sessionID)) {
                sessionID += 1
            }

            // Add session to list of sessions
            activeSessions[sessionID] = this
        }

        fun terminate(payload: ByteArray): Single<ByteArray> {
            return disableMediaManagerWhenNeeded()
                .andThen(Single.fromCallable {
                    openAccessFile?.close()
                    createCommandAck(null, getHeaderFromPayload(payload))
                })
        }

        fun listDirectory(payload: ByteArray): Single<ByteArray> {
            val offset = getOffsetFromPayload(payload)

            return enableMediaManagerWhenNeeded()
                .andThen(pullMediaFilesWhenNeeded())
                .andThen(createFileListWhenNeeded())
                .andThen(Single.fromCallable {
                    val header = getHeaderFromPayload(payload)
                    header[FTPProtocol.FTP_SESSION_ID_BYTE] = sessionID.toByte()

                    if (offset >= ftpFileList.size) {
                        return@fromCallable createCommandNak(FTPProtocol.EOF, 0, header)
                    }

                    val ftpFileListString = StringBuilder()

                    for (i in offset until ftpFileList.size) {
                        val type = if ((ftpFileList[i].isDir)) "D" else "F"
                        val currentFileString = "$type${ftpFileList[i].filePath}\\t${ftpFileList[i].fileSize}\\0"

                        if (ftpFileListString.toString().toByteArray().size + currentFileString.toByteArray().size < (FTPProtocol.FTP_DATA_END_BYTE - FTPProtocol.FTP_DATA_START_BYTE)) {
                            ftpFileListString.append(currentFileString)
                        }
                    }
                    createCommandAck(ftpFileListString.toString().toByteArray(), header)
                })
        }

        fun openFileRO(payload: ByteArray): Single<ByteArray> {
            return Single.fromCallable {
                val header = getHeaderFromPayload(payload)

                // Close current file when we open a new file
                if (openAccessFile != null || openFile != null) {
                    log.e("This session already has an opened file! Create a new session!")
                    return@fromCallable createCommandNak(FTPProtocol.FAIL, 0, header)
                }
                var filename = String(
                    getDataFromPayload(payload),
                    StandardCharsets.ISO_8859_1
                ).trim { it <= ' ' }
                val matcher = COMPONENT_FILE_PATH_PATTERN.matcher(filename)

                // QGC asks without componentID, MAVSDK uses the componentID in the filepath
                var filenameWithoutCompId = ""
                if (!matcher.find()) {
                    filenameWithoutCompId = filename
                    if (filename.startsWith("/")) {
                        filename = filename.substring(1)
                    }

                    filename = "[;comp=$componentIdValue]$filename"
                } else {
                    val targetComponentIndex = matcher.group(1)?.toInt()
                    if (targetComponentIndex != componentIdValue) {
                        log.w("Could not open file, requested file from component $targetComponentIndex to ${componentWR.get()?.componentID}")
                    }

                    filenameWithoutCompId = matcher.group(2)?.toString() ?: filename
                }
                log.i("Session $sessionID: Request to open (R) $filename")

                // Check if images are requested or regular files
                val tmpOpenFile: File
                if (filenameWithoutCompId.startsWith("images/")) {
                    tmpOpenFile = File("")
                    // TODO
                } else {
                    val djiFilePath = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), "ComponentMetadata/$filename")
                        ?: run {
                            log.e("Cannot get external cache dir!")
                            return@fromCallable createCommandNak(FTPProtocol.FAIL, 0, header)
                        }

                    tmpOpenFile = File(djiFilePath)
                }

                if (!tmpOpenFile.exists()) {
                    log.w("Cannot find file '$filename' in filesystem.")
                    return@fromCallable createCommandNak(FTPProtocol.FILE_NOT_FOUND, 0, header)
                }

                openFile = tmpOpenFile
                openAccessFile = RandomAccessFile(openFile, "r")

                if (openFile!!.length() == 0L) {
                    log.e("Cannot open file $filename because it has no content!")
                    return@fromCallable createCommandNak(FTPProtocol.FAIL, 0, header)
                }

                // Insert session ID in header
                header[FTPProtocol.FTP_SESSION_ID_BYTE] = sessionID.toByte()

                val data = intToBytes(openFile!!.length().toInt())
                createCommandAck(data, header)
            }
        }

        fun readFile(payload: ByteArray): Single<ByteArray> {
            return Single.fromCallable {
                val header = getHeaderFromPayload(payload)
                // Implemented after https://github.com/ArduPilot/ardupilot/blob/master/libraries/GCS_MAVLink/GCS_FTP.cpp
                if (openFile == null || openAccessFile == null || !openFile!!.canRead()) {
                    log.e("Cannot read " + openFile!!.path + " because file is not open or in read mode!")
                    return@fromCallable createCommandNak(FTPProtocol.FAIL, 0, header)
                }

                val offset = getOffsetFromPayload(payload)
                val size = payload[FTPProtocol.FTP_SIZE_BYTE].toInt() and 0xFF
                openAccessFile!!.seek(offset.toLong())

                val data = ByteArray(
                    min(size.toDouble(), MAX_DATA_SIZE.toDouble()).toInt()
                )
                val totalNumberOfBytes = openAccessFile!!.read(data)

                if (totalNumberOfBytes <= 0) {
                    return@fromCallable createCommandNak(FTPProtocol.EOF, 0, header)
                }
                createCommandAck(data, header)
            }
        }

        fun createFile(payload: ByteArray): Single<ByteArray> {
            return Single.fromCallable {
                val header = getHeaderFromPayload(payload)
                header[FTPProtocol.FTP_SESSION_ID_BYTE] = sessionID.toByte()

                val filename = String(getDataFromPayload(payload), StandardCharsets.ISO_8859_1).trim { it <= ' ' }

                // We cannot write to the camera SD card, so send FAIL when requested
                if (filename.startsWith("images/")) {
                    log.e("Cannot write to camera SD card!")
                    return@fromCallable createCommandNak(FTPProtocol.FAIL, 0, header)
                }

                val djiFilePath = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), filename)
                val file = File(djiFilePath)

                if (file.parentFile == null || !file.parentFile.exists()) {
                    log.e("Error creating new file: parent directory does not exist!")
                    return@fromCallable createCommandNak(FTPProtocol.FILE_NOT_FOUND, 0, header)
                }

                if (file.exists()) {
                    return@fromCallable createCommandNak(FTPProtocol.FILE_EXISTS, 0, header)
                }

                if (!file.createNewFile()) {
                    log.e("Error creating new file $filename")
                    return@fromCallable createCommandNak(FTPProtocol.FAIL, 0, header)
                }

                openFile = file
                openAccessFile = RandomAccessFile(file, "rw")
                createCommandAck(null, header)
            }
        }

        fun writeFile(payload: ByteArray): Single<ByteArray> {
            return Single.fromCallable {
                val header = getHeaderFromPayload(payload)
                if (!(openFile != null && openFile!!.exists())) {
                    log.e("Cannot write to file. No file exist!")
                    return@fromCallable createCommandNak(FTPProtocol.FILE_NOT_FOUND, 0, header)
                }

                if (!openFile!!.canWrite()) {
                    log.e("Cannot write to file. File is not in write mode!")
                    return@fromCallable createCommandNak(FTPProtocol.FILE_PROTECTED, 0, header)
                }

                val offset = getOffsetFromPayload(payload)
                val data = getDataFromPayload(payload)

                openAccessFile!!.seek(offset.toLong())
                openAccessFile!!.write(data)
                createCommandAck(null, header)
            }
        }

        fun removeFile(payload: ByteArray): Single<ByteArray> {
            val header = getHeaderFromPayload(payload)
            header[FTPProtocol.FTP_SESSION_ID_BYTE] = sessionID.toByte()

            val filename =
                String(getDataFromPayload(payload), StandardCharsets.ISO_8859_1).trim { it <= ' ' }

            if (filename.startsWith("images/")) {
                ftpFileList.forEach {
                    if (it.mediaFile != null && it.mediaFile!!.fileName == filename.substring(7)) {
                        return djiCompletableFunction(
                            {
                                files: List<MediaFile?>?, callback: CompletionCallback? ->
                                MediaManager.getInstance().deleteMediaFiles(files, callback)
                            },
                            { listOf(it.mediaFile) })
                            .andThen(Single.just(createCommandAck(null, header)))
                    }

                    return Single.just(createCommandNak(FTPProtocol.FILE_NOT_FOUND, 0, header))
                }
            }

            return Single.fromCallable {
                val djiFilePath = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), filename)
                val file = File(djiFilePath)

                if (!file.exists()) {
                    return@fromCallable createCommandNak(FTPProtocol.FILE_NOT_FOUND, 0, header)
                }

                if (!file.delete()) {
                    log.e("Could not delete file!")
                    return@fromCallable createCommandNak(FTPProtocol.FAIL, 0, header)
                }
                createCommandAck(null, header)
            }
        }

        fun createDirectory(payload: ByteArray): Single<ByteArray> {
            return Single.fromCallable {
                val header = getHeaderFromPayload(payload)
                header[FTPProtocol.FTP_SESSION_ID_BYTE] = sessionID.toByte()

                val dirname = String(getDataFromPayload(payload), StandardCharsets.ISO_8859_1).trim { it <= ' ' }

                // We cannot write to the camera SD card, so fail here
                if (dirname == "images") {
                    return@fromCallable createCommandNak(FTPProtocol.FILE_EXISTS, 0, header)
                }

                val djiDirPath = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), dirname)
                val directory = File(djiDirPath)

                if (directory.exists()) {
                    return@fromCallable createCommandNak(FTPProtocol.FILE_EXISTS, 0, header)
                }

                if (!directory.mkdir()) {
                    log.e("Could not create directory!")
                    return@fromCallable createCommandNak(FTPProtocol.FAIL, 0, header)
                }
                createCommandAck(null, header)
            }
        }

        fun removeDirectory(payload: ByteArray): Single<ByteArray> {
            return Single.fromCallable {
                val header = getHeaderFromPayload(payload)
                header[FTPProtocol.FTP_SESSION_ID_BYTE] = sessionID.toByte()

                val dirname = String(getDataFromPayload(payload), StandardCharsets.ISO_8859_1).trim { it <= ' ' }

                if (dirname.startsWith("images")) {
                    log.e("Cannot remove the images folder")
                    return@fromCallable createCommandNak(FTPProtocol.FILE_PROTECTED, 0, header)
                }

                val djiDirPath = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), dirname)
                val directory = File(djiDirPath)
                if (!directory.isDirectory) {
                    return@fromCallable createCommandNak(FTPProtocol.FILE_NOT_FOUND, 0, header)
                }

                val files = directory.list()
                if (files != null && files.isNotEmpty()) {
                    log.e("Could not delete directory because it is not empty!")
                    return@fromCallable createCommandNak(FTPProtocol.FAIL, 0, header)
                }

                if (!directory.delete()) {
                    log.e("Could not delete directory!")
                    return@fromCallable createCommandNak(FTPProtocol.FAIL, 0, header)
                }
                createCommandAck(null, header)
            }
        }


        fun openFileWO(payload: ByteArray): Single<ByteArray> {
            return Single.fromCallable {
                // Insert session ID in header
                val header = getHeaderFromPayload(payload)
                header[FTPProtocol.FTP_SESSION_ID_BYTE] = sessionID.toByte()

                // Close current file when we open a new file
                openAccessFile?.close()

                val filename = String(getDataFromPayload(payload), StandardCharsets.ISO_8859_1).trim { it <= ' ' }
                log.i("Session $sessionID: Request to open (WR) $filename")

                // We cannot write to the camera SD card, so send FAIL when requested
                if (filename.startsWith("images/")) {
                    log.e("Cannot write to camera SD card!")
                    return@fromCallable createCommandNak(FTPProtocol.FILE_PROTECTED, 0, header)
                }

                val djiFilePath = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), filename)
                val tmpOpenFile = File(djiFilePath)


                if (!tmpOpenFile.exists()) {
                    if (!tmpOpenFile.createNewFile()) {
                        log.e("Could not create empty file!")
                        return@fromCallable createCommandNak(FTPProtocol.FAIL, 0, header)
                    }
                }

                openFile = tmpOpenFile
                openAccessFile = RandomAccessFile(openFile, "wr")
                createCommandAck(null, header)
            }
        }

        fun truncateFile(payload: ByteArray): Single<ByteArray> {
            return Single.fromCallable {
                // Insert session ID in header
                val header = getHeaderFromPayload(payload)
                header[FTPProtocol.FTP_SESSION_ID_BYTE] = sessionID.toByte()

                val filename = String(getDataFromPayload(payload), StandardCharsets.ISO_8859_1).trim { it <= ' ' }
                log.i("Session $sessionID: Request to open $filename")

                // We cannot write to the camera SD card, so send FAIL when requested
                if (filename.startsWith("images/")) {
                    log.e("Cannot write to camera SD card!")
                    return@fromCallable createCommandNak(FTPProtocol.FAIL, 0, header)
                }

                val djiFilePath = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), filename)
                val tmpOpenFile = File(djiFilePath)

                if (!tmpOpenFile.exists()) {
                    return@fromCallable createCommandNak(FTPProtocol.FILE_NOT_FOUND, 0, header)
                }

                openFile = tmpOpenFile
                openAccessFile = RandomAccessFile(openFile, "rw")

                val offset = getOffsetFromPayload(payload)
                openAccessFile!!.setLength(offset.toLong())
                createCommandAck(null, header)
            }
        }

        fun rename(payload: ByteArray): Single<ByteArray> {
            return Single.fromCallable {
                val header = getHeaderFromPayload(payload)
                val cmd = String(getDataFromPayload(payload), StandardCharsets.ISO_8859_1).trim { it <= ' ' }
                val result = cmd.split("\u0000".toRegex()).dropLastWhile { it.isEmpty() }.toTypedArray()

                if (result.size != 2) {
                    log.e("Cannot rename file because command has not the expected length!")
                    return@fromCallable createCommandNak(FTPProtocol.FAIL, 0, header)
                }

                // We cannot rename files on the camera SD card, so send FAIL when requested
                if (result[0].startsWith("images/")) {
                    log.e("Cannot rename files on the camera SD card!")
                    return@fromCallable createCommandNak(FTPProtocol.FAIL, 0, header)
                }

                val srcDjiFilePath = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), result[0])
                val dstDjiFilePath = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), result[1])

                val tmpSrcFile = File(srcDjiFilePath)
                val tmpDstFile = File(dstDjiFilePath)

                if (!tmpSrcFile.exists()) {
                    return@fromCallable createCommandNak(FTPProtocol.FILE_NOT_FOUND, 0, header)
                }

                if (!tmpSrcFile.renameTo(tmpDstFile)) {
                    log.e("Error renaming file " + result[0] + " to " + result[1])
                    return@fromCallable createCommandNak(FTPProtocol.FAIL, 0, header)
                }
                createCommandAck(null, header)
            }
        }

        fun calculateFileCRC32(payload: ByteArray): Single<ByteArray> {
            return Single.fromCallable {
                // Insert session ID in header
                val header = getHeaderFromPayload(payload)
                header[FTPProtocol.FTP_SESSION_ID_BYTE] = sessionID.toByte()

                val filename = String(getDataFromPayload(payload), StandardCharsets.ISO_8859_1).trim { it <= ' ' }
                log.i("Session $sessionID: Request to open $filename")

                // Check if images are requested or regular files
                val tmpOpenFile: File
                if (filename.startsWith("images/")) {
                    tmpOpenFile = File("")
                    // TODO: read images from DJI camera
                } else {
                    val djiFilePath = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), filename)
                    tmpOpenFile = File(djiFilePath)
                }

                if (!tmpOpenFile.exists()) {
                    return@fromCallable createCommandNak(FTPProtocol.FILE_NOT_FOUND, 0, header)
                }

                openFile = tmpOpenFile
                openAccessFile = RandomAccessFile(openFile, "r")

                val crc32 = CRC32()
                val buffer = ByteArray(1024)
                var bytesRead: Int
                while ((openAccessFile!!.read(buffer).also { bytesRead = it }) != -1) {
                    crc32.update(buffer, 0, bytesRead)
                }

                val checksum = crc32.value
                val data = ByteArray(Long.SIZE_BYTES)
                for (i in 0 until Long.SIZE_BYTES) {
                    data[i] = (checksum ushr (i * 8)).toByte()
                }
                createCommandAck(data, header)
            }
        }

        fun burstReadFile(
            payload: ByteArray,
            targetSystem: Int,
            targetComponent: Int
        ): Completable {
            // Source: https://github.com/ArduPilot/ardupilot/blob/master/libraries/GCS_MAVLink/GCS_FTP.cpp
            return Completable.fromCallable {
                val header = getHeaderFromPayload(payload)
                var cmd: ByteArray

                if (openFile == null || !openFile!!.canRead()) {
                    cmd = createCommandNak(FTPProtocol.FAIL, 0, header)
                    sendCommand(cmd, targetSystem, targetComponent)

                    throw IOException("Cannot read opened file: file is not in read mode.")
                }

                val size = payload[FTPProtocol.FTP_SIZE_BYTE].toInt() and 0xFF
                val seq = getSeqFromPayload(payload)
                var offset = getOffsetFromPayload(payload)

                log.i("Session " + sessionID + ": Start burst read " + openFile!!.name + " from offset " + offset)

                openAccessFile!!.seek(offset.toLong())

                val transferSize = 500
                for (i in 0 until transferSize) {
                    var data = ByteArray(min(size, MAX_DATA_SIZE))
                    val totalNumberOfBytes = openAccessFile!!.read(data)

                    if (totalNumberOfBytes <= 0) {
                        log.d("Send EOF")

                        cmd = createCommandNak(FTPProtocol.EOF, 0, header, seq + i + 1)
                        sendCommand(cmd, targetSystem, targetComponent)
                        break
                    }

                    if (totalNumberOfBytes != MAX_DATA_SIZE) {
                        data = data.copyOfRange(0, totalNumberOfBytes)
                    }

                    val offsetBytes = intToBytes(offset)
                    offsetBytes.copyInto(header, FTPProtocol.FTP_OFFSET_START_BYTE)

                    header[FTPProtocol.FTP_BURST_COMPLETE_BYTE] = (if ((i == (transferSize - 1) || totalNumberOfBytes != data.size)) 1 else 0).toByte()

                    cmd = createCommandAck(data, header, seq + i + 1)
                    sendCommand(cmd, targetSystem, targetComponent)

                    offset += totalNumberOfBytes

                    log.d("Send burst message $i, current offset $offset")
                }
                Completable.complete()
            }
        }

        private fun createFileListWhenNeeded(): Completable {
            if (ftpFileList.isEmpty()) {
                return Completable.create { emitter: CompletableEmitter ->
                    MediaManager.getInstance().mediaFileListData.data.forEach {
                        ftpFileList.add(FTPFile(it))
                    }

                    val djiCacheFolder = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), "")
                    if (djiCacheFolder == null || !File(djiCacheFolder).exists()) {
                        emitter.onError(Throwable("Cannot get all files in DJI cache folder because folder does not exist!"))
                    }

                    getFilesRecursively(File(djiCacheFolder)).forEach {
                        ftpFileList.add(FTPFile(it, djiCacheFolder))
                    }
                }
            }

            return Completable.complete()
        }

        private fun pullMediaFilesWhenNeeded(): Completable {
            // When we are in this function, we know for sure that the media manager is enabled
            mediaManagerEnabled = true

            if (MediaManager.getInstance().mediaFileListState == MediaFileListState.IDLE) {
                val pullMediaFileListParam = PullMediaFileListParam.Builder().filter(MediaFileFilter.ALL).build()
                return djiCompletableFunction(
                        {
                            param: PullMediaFileListParam, callback: CompletionCallback ->
                            MediaManager.getInstance().pullMediaFileListFromCamera(param, callback)
                        }, { pullMediaFileListParam }
                    )
                    .andThen(Observable.interval(100, TimeUnit.MILLISECONDS))
                    .map { MediaManager.getInstance().mediaFileListState }
                    .takeUntil { state: MediaFileListState -> state == MediaFileListState.UP_TO_DATE }
                    .ignoreElements()
            }

            return Completable.complete()
        }

        private fun enableMediaManagerWhenNeeded(): Completable {
            if (!mediaManagerEnabled) {
                return djiCompletableFunction { callback: CompletionCallback ->
                    MediaManager.getInstance().enable(callback)
                }
            }

            return Completable.complete()
        }

        private fun disableMediaManagerWhenNeeded(): Completable {
            if (activeSessions.size == 1 && mediaManagerEnabled) {
                return djiCompletableFunction { callback: CompletionCallback ->
                    MediaManager.getInstance().disable(callback)
                }
            }

            return Completable.complete()
        }


        private fun getFilesRecursively(directory: File): List<File> {
            val fileList: MutableList<File> = ArrayList()

            // Get all the files in the directory
            val files = directory.listFiles()

            // Loop through the files and add them to the list
            files?.let {
                files.forEach {
                    if (it.isFile) {
                        fileList.add(it)
                    } else if (it.isDirectory) {
                        fileList.addAll(getFilesRecursively(it))
                        fileList.add(it)
                    }
                }
            }

            return fileList
        }

        private inner class FTPFile {
            var fileSize: Long
            var filePath: String
            var isDir: Boolean = false
            var mediaFile: MediaFile? = null

            constructor(file: File, parentFolder: String?) {
                this.fileSize = file.length()
                this.isDir = file.isDirectory

                val absPath = file.absolutePath
                this.filePath = absPath.replace("$parentFolder/", "")
            }

            constructor(file: MediaFile) {
                this.fileSize = file.fileSize
                this.filePath = "images/${file.fileName}"
                this.mediaFile = file
            }
        }
    }

    companion object {
        private const val MAX_DATA_SIZE = FTPProtocol.FTP_DATA_END_BYTE - FTPProtocol.FTP_DATA_START_BYTE + 1
        private val COMPONENT_FILE_PATH_PATTERN: Pattern = Pattern.compile("\\[;comp=(\\d+)](.+)")

        //region: utils
        private fun getHeaderFromPayload(payload: ByteArray): ByteArray {
            val length = FTPProtocol.FTP_OFFSET_END_BYTE + 1
            val byteArray = ByteArray(length)
            payload.copyInto(
                destination = byteArray,
                destinationOffset = 0,
                startIndex = 0,
                endIndex = length.coerceAtMost(payload.size)
            )
            return byteArray
        }

        private fun getDataFromPayload(payload: ByteArray): ByteArray {
            val start = FTPProtocol.FTP_DATA_START_BYTE.coerceIn(0, payload.size)
            val end = (FTPProtocol.FTP_DATA_END_BYTE + 1).coerceAtMost(payload.size)
            val byteArray = ByteArray(end - start)
            payload.copyInto(
                byteArray,
                destinationOffset = 0,
                startIndex = start,
                endIndex = end // endIndex is exclusive
            )
            return byteArray
        }

        private fun getOffsetFromPayload(payload: ByteArray): Int {
            val byteArray = ByteArray(FTPProtocol.FTP_OFFSET_END_BYTE - FTPProtocol.FTP_OFFSET_START_BYTE + 1)
            payload.copyInto(
                destination = byteArray,
                destinationOffset = 0,
                startIndex = FTPProtocol.FTP_OFFSET_START_BYTE,
                endIndex = FTPProtocol.FTP_OFFSET_END_BYTE + 1 // endIndex is exclusive
            )
            return ((byteArray[3].toInt() and 0xFF) shl 24) or
                    ((byteArray[2].toInt() and 0xFF) shl 16) or
                    ((byteArray[1].toInt() and 0xFF) shl 8) or
                    ((byteArray[0].toInt() and 0xFF))
        }

        private fun getSeqFromPayload(payload: ByteArray): Int {
            val byteArray = ByteArray(FTPProtocol.FTP_SEQ_END_BYTE - FTPProtocol.FTP_SEQ_START_BYTE + 1)
            payload.copyInto(
                destination = byteArray,
                destinationOffset = 0,
                startIndex = FTPProtocol.FTP_SEQ_START_BYTE,
                endIndex = FTPProtocol.FTP_SEQ_END_BYTE + 1 // endIndex is exclusive
            )
            return ((byteArray[1].toInt() and 0xFF) shl 8) or
                    ((byteArray[0].toInt() and 0xFF))
        }

        //endregion
        private fun createCommandAck(data: ByteArray?, header: ByteArray): ByteArray {
            // Increase sequence number by 1
            val seq = getSeqFromPayload(header) + 1
            return createCommandAck(data, header, seq)
        }

        private fun createCommandAck(data: ByteArray?, header: ByteArray, seq: Int): ByteArray {
            val payload = ByteArray(FTPProtocol.FTP_DATA_END_BYTE + 1)

            header.copyInto(
                payload,
                destinationOffset = FTPProtocol.FTP_SEQ_START_BYTE,
                startIndex = FTPProtocol.FTP_SEQ_START_BYTE,
                endIndex = FTPProtocol.FTP_OFFSET_END_BYTE + 1
            )

            payload[FTPProtocol.FTP_SIZE_BYTE] = 0
            data?.let {
                it.copyInto(payload, destinationOffset = FTPProtocol.FTP_DATA_START_BYTE, startIndex = 0, endIndex = it.size)
                payload[FTPProtocol.FTP_SIZE_BYTE] = it.size.toByte()
            }

            val seqBytes = intToBytes(seq)
            payload[FTPProtocol.FTP_SEQ_START_BYTE] = seqBytes[0]
            payload[FTPProtocol.FTP_SEQ_END_BYTE] = seqBytes[1]

            payload[FTPProtocol.FTP_REQ_OPCODE_BYTE] = payload[FTPProtocol.FTP_OPCODE_BYTE]
            payload[FTPProtocol.FTP_OPCODE_BYTE] = FTPProtocol.ACK.toByte()
            return payload
        }

        private fun createCommandNak(errorCode: Int, failCode: Int, header: ByteArray): ByteArray {
            // Increase sequence number by 1
            val seq = getSeqFromPayload(header) + 1
            return createCommandNak(errorCode, failCode, header, seq)
        }

        private fun createCommandNak(
            errorCode: Int,
            failCode: Int,
            header: ByteArray,
            seq: Int
        ): ByteArray {
            val payload = ByteArray(FTPProtocol.FTP_DATA_END_BYTE + 1)

            header.copyInto(
                payload,
                destinationOffset = FTPProtocol.FTP_SEQ_START_BYTE,
                startIndex = FTPProtocol.FTP_SEQ_START_BYTE,
                endIndex = FTPProtocol.FTP_SEQ_START_BYTE + FTPProtocol.FTP_OFFSET_END_BYTE
            )

            val seqBytes = intToBytes(seq)
            payload[FTPProtocol.FTP_SEQ_START_BYTE] = seqBytes[0]
            payload[FTPProtocol.FTP_SEQ_END_BYTE] = seqBytes[1]

            payload[FTPProtocol.FTP_DATA_START_BYTE] = errorCode.toByte()

            if (failCode != 0) {
                payload[FTPProtocol.FTP_SIZE_BYTE] = 2.toByte()
                payload[FTPProtocol.FTP_DATA_START_BYTE + 1] = failCode.toByte()
            } else {
                payload[FTPProtocol.FTP_SIZE_BYTE] = 1.toByte()
            }

            payload[FTPProtocol.FTP_REQ_OPCODE_BYTE] = payload[FTPProtocol.FTP_OPCODE_BYTE]
            payload[FTPProtocol.FTP_OPCODE_BYTE] = FTPProtocol.NAK.toByte()

            return payload
        }
    }
}
