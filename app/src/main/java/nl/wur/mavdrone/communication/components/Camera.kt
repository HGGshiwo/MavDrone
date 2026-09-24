package nl.wur.mavdrone.communication.components

import java.lang.ref.WeakReference
import java.math.BigInteger
import java.util.Calendar
import java.util.Locale
import java.util.TimeZone
import kotlin.math.atan
import kotlin.math.roundToInt

import dji.sdk.keyvalue.key.CameraKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.camera.CameraMode
import dji.sdk.keyvalue.value.camera.CameraType
import dji.sdk.keyvalue.value.camera.GeneratedMediaFileInfo
import dji.sdk.keyvalue.value.camera.MediaFileType
import dji.sdk.keyvalue.value.camera.PhotoIntervalShootSettings
import dji.sdk.keyvalue.value.camera.SDCardLoadState
import dji.sdk.keyvalue.value.common.CameraLensType
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.common.callback.CommonCallbacks.CompletionCallback
import dji.v5.common.error.IDJIError
import dji.v5.manager.KeyManager
import dji.v5.manager.datacenter.livestream.LiveStreamManager
import dji.v5.manager.datacenter.livestream.LiveStreamSettings
import dji.v5.manager.datacenter.livestream.LiveStreamStatus
import dji.v5.manager.datacenter.livestream.LiveStreamStatusListener
import dji.v5.manager.datacenter.livestream.LiveStreamType
import dji.v5.manager.datacenter.livestream.StreamQuality
import dji.v5.manager.datacenter.livestream.settings.RtspSettings
import dji.v5.utils.common.ContextUtil

import io.reactivex.rxjava3.android.schedulers.AndroidSchedulers
import io.reactivex.rxjava3.core.Completable
import io.reactivex.rxjava3.schedulers.Schedulers

import io.dronefleet.mavlink.MavlinkMessage
import io.dronefleet.mavlink.common.CameraCapFlags
import io.dronefleet.mavlink.common.CameraCaptureStatus
import io.dronefleet.mavlink.common.CameraImageCaptured
import io.dronefleet.mavlink.common.CameraInformation
import io.dronefleet.mavlink.common.CameraSettings
import io.dronefleet.mavlink.common.CommandLong
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.StorageInformation
import io.dronefleet.mavlink.common.StorageStatus
import io.dronefleet.mavlink.common.StorageType
import io.dronefleet.mavlink.common.StorageUsageFlag
import io.dronefleet.mavlink.common.VideoStreamInformation
import io.dronefleet.mavlink.common.VideoStreamStatus
import io.dronefleet.mavlink.common.VideoStreamStatusFlags
import io.dronefleet.mavlink.common.VideoStreamType
import io.dronefleet.mavlink.standard.MavAutopilot
import io.dronefleet.mavlink.standard.MavComponent
import io.dronefleet.mavlink.standard.MavType
import io.dronefleet.mavlink.util.EnumValue

import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.impl.CameraParameters
import nl.wur.mavdrone.communication.components.impl.FTPManager
import nl.wur.mavdrone.communication.utils.CameraDefinitionGenerator
import nl.wur.mavdrone.communication.utils.StandardResponses
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandAccepted
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandDenied
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandFailed
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandUnsupported
import nl.wur.mavdrone.utils.NetworkUtils.Companion.getLocalWifiIpAddress
import nl.wur.mavdrone.utils.RXUtils.Companion.djiCompletableFunction
import nl.wur.mavdrone.utils.RXUtils.Companion.djiGetValue
import nl.wur.mavdrone.utils.RXUtils.Companion.djiPerformAction
import nl.wur.mavdrone.utils.RXUtils.Companion.djiSetValue
import nl.wur.mavdrone.utils.toRadians

class Camera(
    connectionWR: WeakReference<MavConnection>, componentId: MavComponent,
    val componentIndex: ComponentIndexType, private val cameraLensType: CameraLensType,
) : MavBaseComponent(
    connectionWR,
    KeyTools.createCameraKey(CameraKey.KeyConnection, componentIndex, cameraLensType),
    componentId,
    MavType.MAV_TYPE_CAMERA,
    MavAutopilot.MAV_AUTOPILOT_INVALID,
    "MavCamera${componentIndex.ordinal}",
    true
), LiveStreamStatusListener {
    private val cameraDefinitionGenerator: CameraDefinitionGenerator
    private val cameraParameters: CameraParameters
    private val ftpManager: FTPManager

    private val streamUsername = connectionWR.get()!!.streamInformation.user
    private val streamPassword = connectionWR.get()!!.streamInformation.password
    private val streamPort = connectionWR.get()!!.streamInformation.port
    private val streamAddress: String

    private var focalLength = 0
    private var horizontalFOV = 0

    private var djiCameraMode = CameraMode.UNKNOWN

    private var isCapturingImages = false
    private var imageCapturingInterval = 0.0f
    private var capturedNumberOfImages = 0

    private var videoIsRecording = false
    private var videoRecordingTime = 0
    private var lastLiveStreamStatus: LiveStreamStatus? = null

    var cameraType: CameraType = CameraType.NOT_SUPPORTED
        private set

    init {
        val ip = getLocalWifiIpAddress(ContextUtil.getContext())
        streamAddress = "rtsp://$streamUsername:$streamPassword@$ip:$streamPort/streaming/live/1"

        cameraDefinitionGenerator = CameraDefinitionGenerator(connectionWR, componentIndex, cameraLensType, componentId)
        cameraParameters = CameraParameters(connectionWR, WeakReference(this), componentIndex, cameraLensType)
        ftpManager = FTPManager(connectionWR, WeakReference(this))

        connectToProduct()
    }

    override fun create() {
        cameraDefinitionGenerator.create()
        cameraParameters.create()
        ftpManager.create()

        focalLength = KeyManager.getInstance().getValue(KeyTools.createKey(CameraKey.KeyCameraZoomFocalLength, componentIndex), 0)
        val sensorWidth = cameraDefinitionGenerator.cameraDetails?.sensorWidth ?: 0f
        horizontalFOV = if (focalLength > 0 && sensorWidth > 0f) {
            (2 * atan(sensorWidth / (2 * focalLength))).toDouble().toRadians().roundToInt()
        } else {
            0
        }

        compositeDisposable.add(
            djiGetValue(KeyTools.createCameraKey(CameraKey.KeyCameraType, componentIndex, cameraLensType))
                .subscribe { t -> cameraType = t }
        )

        //region: add listeners
        addDJIListener(KeyTools.createCameraKey(CameraKey.KeyCameraMode, componentIndex, cameraLensType)) { value -> djiCameraMode = value }
        addDJIListener(KeyTools.createCameraKey(CameraKey.KeyNewlyGeneratedMediaFile, componentIndex, cameraLensType)) { value ->
            if (value.type == MediaFileType.JPEG || value.type == MediaFileType.DNG) {
                val cameraImageCaptured = toMavlinkCameraImageCapturedMessage(value)
                sendMessage(cameraImageCaptured)

                capturedNumberOfImages++
            }
        }
        addDJIListener(KeyTools.createCameraKey(CameraKey.KeyIsRecording, componentIndex, cameraLensType)) { value -> videoIsRecording = value }
        addDJIListener(KeyTools.createCameraKey(CameraKey.KeyRecordingTime, componentIndex, cameraLensType)) { value -> videoRecordingTime = value }

        //endregion

        // Start message senders
        addMavlinkMessageSender(CameraInformation::class.java) { this.toMavlinkCameraInformationMessage() }
        addMavlinkMessageSender(VideoStreamInformation::class.java) { this.toMavlinkVideoStreamInformationMessage() }
        addMavlinkMessageSender(CameraSettings::class.java) { this.toMavlinkCameraSettingsMessage() }
        addMavlinkMessageSender(StorageInformation::class.java) { this.toMavlinkStorageInformationMessage() }
        addMavlinkMessageSender(CameraCaptureStatus::class.java) { this.toMavlinkCameraCaptureStatusMessage() }
        addMavlinkMessageSender(VideoStreamStatus::class.java) { this.toMavlinkVideoStreamStatusMessage() }

        LiveStreamManager.getInstance().addLiveStreamStatusListener(this)

        // Start streaming when autostart is enabled
        if (connectionWR.get()!!.streamInformation.autoStart) {
            val rtspSettings = RtspSettings.Builder()
                .setPort(streamPort)
                .setUserName(streamUsername)
                .setPassWord(streamPassword)
                .build()
            val liveStreamSettings = LiveStreamSettings.Builder()
                .setLiveStreamType(LiveStreamType.RTSP)
                .setRtspSettings(rtspSettings)
                .build()
            LiveStreamManager.getInstance().liveStreamSettings = liveStreamSettings

            if (!LiveStreamManager.getInstance().isStreaming) {
                compositeDisposable.add(djiCompletableFunction { callback: CompletionCallback? -> LiveStreamManager.getInstance().startStream(callback) }
                    .subscribeOn(AndroidSchedulers.mainThread())
                    .observeOn(Schedulers.io())
                    .subscribe({
                        LiveStreamManager.getInstance().liveStreamQuality = StreamQuality.FULL_HD
                        if (LiveStreamManager.getInstance().cameraIndex != componentIndex) {
                            LiveStreamManager.getInstance().cameraIndex = componentIndex
                        }
                        log.i("Started camera stream at $streamAddress")
                    }, { error -> log.e("Could not start stream: " + error.message) })
                )
            } else {
                LiveStreamManager.getInstance().liveStreamQuality = StreamQuality.FULL_HD
                if (LiveStreamManager.getInstance().cameraIndex != componentIndex) {
                    LiveStreamManager.getInstance().cameraIndex = componentIndex
                    log.i("Changed camera stream to $componentIndex")
                }
            }
        }
    }

    override fun destroyListeners() {
        LiveStreamManager.getInstance().removeLiveStreamStatusListener(this)

        super.destroyListeners()
    }

    override fun destroy() {
        cameraDefinitionGenerator.destroy()
        ftpManager.destroy()

        super.destroy()
    }

    @Suppress("deprecation")
    public override fun onCommandLong(msg: MavlinkMessage<*>): Boolean {
        if (super.onCommandLong(msg)) { return true }

        val payload = msg.payload as CommandLong

        if (isCommandForMe(payload)) {
            log.i("Command: " + payload.command().entry())

            when (payload.command().entry()) {
                MavCmd.MAV_CMD_VIDEO_START_STREAMING -> {
                    if (payload.param1().roundToInt() != 0 || payload.param1().roundToInt() == 1) {
                        log.e("Cannot start stream ${payload.param1().roundToInt()}, only stream with id 0 exists!")
                        sendMessage(commandFailed(MavCmd.MAV_CMD_VIDEO_START_STREAMING))
                        return true
                    }

                    if (!LiveStreamManager.getInstance().isStreaming) {
                        val rtspSettings = RtspSettings.Builder()
                            .setPort(streamPort)
                            .setUserName(streamUsername)
                            .setPassWord(streamPassword)
                            .build()
                        val liveStreamSettings = LiveStreamSettings.Builder()
                            .setLiveStreamType(LiveStreamType.RTSP)
                            .setRtspSettings(rtspSettings)
                            .build()
                        LiveStreamManager.getInstance().liveStreamSettings = liveStreamSettings

                        compositeDisposable.add(djiCompletableFunction { callback: CompletionCallback? ->
                            LiveStreamManager.getInstance().startStream(callback)
                        }
                            .subscribeOn(AndroidSchedulers.mainThread())
                            .observeOn(Schedulers.io())
                            .subscribe({
                                LiveStreamManager.getInstance().liveStreamQuality = StreamQuality.FULL_HD
                                if (LiveStreamManager.getInstance().cameraIndex != componentIndex) {
                                    LiveStreamManager.getInstance().setCameraIndex(componentIndex)
                                }
                                log.i("Started camera stream at $streamAddress")
                                sendMessage(commandAccepted(MavCmd.MAV_CMD_VIDEO_START_STREAMING, msg.originSystemId, msg.originComponentId))
                                sendMessage(toMavlinkVideoStreamInformationMessage())
                            }, { error ->
                                log.e("Could not start stream: ${error.message}")
                                sendMessage(commandFailed(MavCmd.MAV_CMD_VIDEO_START_STREAMING, msg.originSystemId, msg.originComponentId))
                            })
                        )
                    } else {
                        if (LiveStreamManager.getInstance().cameraIndex != componentIndex) {
                            LiveStreamManager.getInstance().cameraIndex = componentIndex
                        }
                        log.i("Changed streaming camera index to $cameraType")
                        sendMessage(commandAccepted(MavCmd.MAV_CMD_VIDEO_START_STREAMING, msg.originSystemId, msg.originComponentId))
                        sendMessage(toMavlinkVideoStreamInformationMessage())
                    }
                    return true
                }

                MavCmd.MAV_CMD_VIDEO_STOP_STREAMING -> {
                    if (payload.param1().roundToInt() != 0 || payload.param1().roundToInt() == 1) {
                        log.e("Cannot stop stream ${payload.param1().roundToInt()}, only stream with id 1 exists!")
                        sendMessage(commandFailed(MavCmd.MAV_CMD_VIDEO_START_STREAMING, msg.originSystemId, msg.originComponentId))
                        return true
                    }

                    if (LiveStreamManager.getInstance().isStreaming
                        && LiveStreamManager.getInstance().cameraIndex == ComponentIndexType.FPV
                    ) {
                        compositeDisposable.add(djiCompletableFunction { callback: CompletionCallback? -> LiveStreamManager.getInstance().stopStream(callback) }
                            .subscribeOn(AndroidSchedulers.mainThread())
                            .observeOn(Schedulers.io())
                            .subscribe(
                                { sendMessage(commandAccepted(MavCmd.MAV_CMD_VIDEO_STOP_STREAMING, msg.originSystemId, msg.originComponentId)) },
                                { error ->
                                    log.e("Could not stop stream: ${error.message}")
                                    sendMessage(commandFailed(MavCmd.MAV_CMD_VIDEO_STOP_STREAMING, msg.originSystemId, msg.originComponentId))
                                })
                        )
                    } else {
                        sendMessage(commandAccepted(MavCmd.MAV_CMD_VIDEO_STOP_STREAMING, msg.originSystemId, msg.originComponentId))
                    }
                    return true
                }

                MavCmd.MAV_CMD_RESET_CAMERA_SETTINGS -> {
                    if (payload.param1().toDouble() == 1.0) {
                        compositeDisposable.add(
                            djiPerformAction(KeyTools.createCameraKey(CameraKey.KeyResetCameraSetting, componentIndex, cameraLensType))
                                .subscribe(
                                    { sendMessage(commandAccepted(MavCmd.MAV_CMD_RESET_CAMERA_SETTINGS, msg.originSystemId, msg.originComponentId)) },
                                    { error ->
                                        onError(
                                            error,
                                            StandardResponses::commandFailed,
                                            MavCmd.MAV_CMD_RESET_CAMERA_SETTINGS,
                                            msg.originSystemId,
                                            msg.originComponentId
                                        )
                                    }
                                )
                        )
                    } else {
                        sendMessage(commandAccepted(MavCmd.MAV_CMD_RESET_CAMERA_SETTINGS, msg.originSystemId, msg.originComponentId))
                    }
                    return true
                }

                MavCmd.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS -> {
                    sendMessage(commandAccepted(MavCmd.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS, msg.originSystemId, msg.originComponentId))
                    val cameraCaptureStatus = toMavlinkCameraCaptureStatusMessage()
                    sendMessage(cameraCaptureStatus)
                    return true
                }

                MavCmd.MAV_CMD_REQUEST_CAMERA_SETTINGS -> {
                    sendMessage(commandAccepted(MavCmd.MAV_CMD_REQUEST_CAMERA_SETTINGS, msg.originSystemId, msg.originComponentId))
                    val cameraSettingsMessage = toMavlinkCameraSettingsMessage()
                    sendMessage(cameraSettingsMessage)
                    return true
                }

                MavCmd.MAV_CMD_REQUEST_CAMERA_INFORMATION -> {
                    sendMessage(commandAccepted(MavCmd.MAV_CMD_REQUEST_CAMERA_INFORMATION, msg.originSystemId, msg.originComponentId))
                    val cameraInformation = toMavlinkCameraInformationMessage()
                    sendMessage(cameraInformation)
                }

                MavCmd.MAV_CMD_REQUEST_STORAGE_INFORMATION -> {
                    sendMessage(commandAccepted(MavCmd.MAV_CMD_REQUEST_STORAGE_INFORMATION, msg.originSystemId, msg.originComponentId))
                    val storageInformation = toMavlinkStorageInformationMessage()
                    sendMessage(storageInformation)
                    return true
                }

                MavCmd.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION -> {
                    if (payload.param1().roundToInt() == 0 || payload.param1().roundToInt() == 1) {
                        sendMessage(commandAccepted(MavCmd.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION, msg.originSystemId, msg.originComponentId))
                        val videoStreamInformation = toMavlinkVideoStreamInformationMessage()
                        sendMessage(videoStreamInformation)
                    } else {
                        sendMessage(commandDenied(MavCmd.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION, msg.originSystemId, msg.originComponentId))
                    }
                    return true
                }

                MavCmd.MAV_CMD_REQUEST_VIDEO_STREAM_STATUS -> {
                    if (payload.param1().roundToInt() == 0 || payload.param1().roundToInt() == 1) {
                        sendMessage(commandAccepted(MavCmd.MAV_CMD_REQUEST_VIDEO_STREAM_STATUS, msg.originSystemId, msg.originComponentId))
                        val videoStreamStatus = toMavlinkVideoStreamStatusMessage()
                        sendMessage(videoStreamStatus)
                    } else {
                        sendMessage(commandDenied(MavCmd.MAV_CMD_REQUEST_VIDEO_STREAM_STATUS, msg.originSystemId, msg.originComponentId))
                    }
                    return true
                }

                MavCmd.MAV_CMD_STORAGE_FORMAT -> {
                    if (payload.param2().toDouble() == 1.0) {
                        compositeDisposable.add(
                            djiPerformAction(KeyTools.createCameraKey(CameraKey.KeyFormatStorage, componentIndex, cameraLensType))
                                .subscribe(
                                    { sendMessage(commandAccepted(MavCmd.MAV_CMD_STORAGE_FORMAT, msg.originSystemId, msg.originComponentId)) },
                                    { error ->
                                        onError(
                                            error,
                                            StandardResponses::commandFailed,
                                            MavCmd.MAV_CMD_STORAGE_FORMAT,
                                            msg.originSystemId,
                                            msg.originComponentId
                                        )
                                    })
                        )
                    }

                    if (payload.param3().toDouble() == 1.0) {
                        capturedNumberOfImages = 0
                        sendMessage(commandAccepted(MavCmd.MAV_CMD_STORAGE_FORMAT, msg.originSystemId, msg.originComponentId))
                    }
                    return true
                }

                MavCmd.MAV_CMD_SET_CAMERA_MODE -> {
                    val cameraMode = EnumValue.create(io.dronefleet.mavlink.common.CameraMode::class.java, payload.param2().roundToInt()).entry()
                    val newCameraMode = when (cameraMode) {
                        io.dronefleet.mavlink.common.CameraMode.CAMERA_MODE_IMAGE -> CameraMode.PHOTO_INTERVAL
                        io.dronefleet.mavlink.common.CameraMode.CAMERA_MODE_VIDEO -> CameraMode.VIDEO_NORMAL
                        else -> CameraMode.UNKNOWN
                    }
                    compositeDisposable.add(
                        djiSetValue(KeyTools.createCameraKey(CameraKey.KeyCameraMode, componentIndex, cameraLensType), newCameraMode)
                            .subscribe(
                                { sendMessage(commandAccepted(MavCmd.MAV_CMD_SET_CAMERA_MODE)) },
                                { error ->
                                    onError(
                                        error,
                                        StandardResponses::commandFailed,
                                        MavCmd.MAV_CMD_SET_CAMERA_MODE,
                                        msg.originSystemId,
                                        msg.originComponentId
                                    )
                                })
                    )
                    return true
                }

                MavCmd.MAV_CMD_IMAGE_START_CAPTURE -> {
                    val count = payload.param3().roundToInt()
                    val interval = payload.param2()

                    if (count == 1) {
                        log.i("Capture single image")

                        compositeDisposable.add(djiGetValue(KeyTools.createCameraKey(CameraKey.KeyCameraMode, componentIndex, cameraLensType))
                            .flatMapCompletable { c: CameraMode ->
                                if (c != CameraMode.PHOTO_NORMAL) {
                                    return@flatMapCompletable djiSetValue<CameraMode>(
                                        KeyTools.createCameraKey<CameraMode>(
                                            CameraKey.KeyCameraMode,
                                            componentIndex,
                                            cameraLensType
                                        ), CameraMode.PHOTO_NORMAL
                                    )
                                }
                                Completable.complete()
                            }
                            .andThen(djiPerformAction(KeyTools.createCameraKey(CameraKey.KeyStartShootPhoto, componentIndex, cameraLensType)))
                            .subscribe(
                                {},
                                { error -> log.e("Error occurred: ${error.message}") }
                            )
                        )
                        return true
                    }

                    if (interval < 2.0) {
                        log.e("Cannot start image capture because interval should be larger than 2 seconds!")
                        sendMessage(commandDenied(MavCmd.MAV_CMD_IMAGE_START_CAPTURE, msg.originSystemId, msg.originComponentId))
                        return true
                    }

                    log.i("Start capturing $count images at interval $interval")
                    imageCapturingInterval = interval

                    val cameraSetIntervalKey = KeyTools.createCameraKey(CameraKey.KeyPhotoIntervalShootSettings, componentIndex, cameraLensType)

                    val photoIntervalShootSettings = PhotoIntervalShootSettings()
                    photoIntervalShootSettings.interval = interval.toDouble()
                    photoIntervalShootSettings.count = count

                    compositeDisposable.add(djiGetValue(KeyTools.createCameraKey(CameraKey.KeyCameraMode, componentIndex, cameraLensType))
                        .flatMapCompletable { c: CameraMode ->
                            if (c != CameraMode.PHOTO_INTERVAL) {
                                return@flatMapCompletable djiSetValue<CameraMode>(
                                    KeyTools.createCameraKey<CameraMode>(
                                        CameraKey.KeyCameraMode,
                                        componentIndex,
                                        cameraLensType
                                    ), CameraMode.PHOTO_INTERVAL
                                )
                            }
                            Completable.complete()
                        }
                        .andThen(
                            djiSetValue(cameraSetIntervalKey, photoIntervalShootSettings)
                                .andThen(djiPerformAction(KeyTools.createCameraKey(CameraKey.KeyStartShootPhoto, componentIndex, cameraLensType)))
                        )
                        .subscribe({
                            isCapturingImages = true
                            sendMessage(commandAccepted(MavCmd.MAV_CMD_IMAGE_START_CAPTURE, msg.originSystemId, msg.originComponentId))
                        }, { error ->
                            isCapturingImages = false
                            onError(
                                error,
                                StandardResponses::commandFailed,
                                MavCmd.MAV_CMD_IMAGE_START_CAPTURE,
                                msg.originSystemId,
                                msg.originComponentId
                            )
                        })
                    )
                    return true
                }

                MavCmd.MAV_CMD_IMAGE_STOP_CAPTURE -> {
                    if (isCapturingImages) {
                        log.i("Stop capturing images")
                        compositeDisposable.add(
                            djiPerformAction(KeyTools.createCameraKey(CameraKey.KeyStopShootPhoto, componentIndex, cameraLensType))
                                .subscribe({
                                    isCapturingImages = false
                                    sendMessage(commandAccepted(MavCmd.MAV_CMD_IMAGE_STOP_CAPTURE, msg.originSystemId, msg.originComponentId))
                                }, { error ->
                                    isCapturingImages = false
                                    onError(
                                        error,
                                        StandardResponses::commandFailed,
                                        MavCmd.MAV_CMD_IMAGE_STOP_CAPTURE,
                                        msg.originSystemId,
                                        msg.originComponentId
                                    )
                                })
                        )
                    }
                    return true
                }

                MavCmd.MAV_CMD_VIDEO_START_CAPTURE -> {
                    val statusFrequency = (1000 / payload.param2()).roundToInt()
                    compositeDisposable.add(djiGetValue(KeyTools.createCameraKey(CameraKey.KeyCameraMode, componentIndex, cameraLensType))
                        .flatMapCompletable { c: CameraMode ->
                            if (c != CameraMode.VIDEO_NORMAL) {
                                return@flatMapCompletable djiSetValue<CameraMode>(
                                    KeyTools.createCameraKey<CameraMode>(
                                        CameraKey.KeyCameraMode,
                                        componentIndex,
                                        cameraLensType
                                    ), CameraMode.VIDEO_NORMAL
                                )
                            }
                            Completable.complete()
                        }
                        .andThen(djiPerformAction(KeyTools.createCameraKey(CameraKey.KeyStartRecord, componentIndex, cameraLensType)))
                        .subscribe({
                            sendMessage(commandAccepted(MavCmd.MAV_CMD_VIDEO_START_CAPTURE, msg.originSystemId, msg.originComponentId))
                            if (statusFrequency != 0) {
                                addBroadcaster("camera_capture_status", statusFrequency) {
                                    sendMessage(toMavlinkCameraCaptureStatusMessage())
                                }
                            }
                        }, { error ->
                            stopBroadcaster("camera_capture_status")
                            onError(
                                error,
                                StandardResponses::commandFailed,
                                MavCmd.MAV_CMD_VIDEO_START_CAPTURE,
                                msg.originSystemId,
                                msg.originComponentId
                            )
                        })
                    )
                    return true
                }

                MavCmd.MAV_CMD_VIDEO_STOP_CAPTURE -> {
                    compositeDisposable.add(
                        djiPerformAction(KeyTools.createCameraKey(CameraKey.KeyStopRecord, componentIndex, cameraLensType))
                            .subscribe({
                                stopBroadcaster("camera_capture_status")
                                sendMessage(commandAccepted(MavCmd.MAV_CMD_VIDEO_STOP_CAPTURE, msg.originSystemId, msg.originComponentId))
                            }, { error ->
                                stopBroadcaster("camera_capture_status")
                                onError(
                                    error,
                                    StandardResponses::commandFailed,
                                    MavCmd.MAV_CMD_VIDEO_STOP_CAPTURE,
                                    msg.originSystemId,
                                    msg.originComponentId
                                )
                            })
                    )
                    return true
                }

                else -> {
                    log.i("Unsupported command: ${payload.command().entry()}")
                    sendMessage(commandUnsupported(payload.command().entry(), msg.originSystemId, msg.originComponentId))
                    return false
                }
            }
        }

        return false
    }

    override fun onLiveStreamStatusUpdate(status: LiveStreamStatus) {
        lastLiveStreamStatus = status
    }

    override fun onError(error: IDJIError) {
        log.w("Could not retrieve last livestream status information: " + error.description())
    }

    //region messages
    private fun toMavlinkCameraInformationMessage(): CameraInformation {
        val firmwareVersion = KeyManager.getInstance().getValue(KeyTools.createCameraKey(CameraKey.KeyFirmwareVersion, componentIndex, cameraLensType)) ?: ""
        val mavlinkFirmwareVersion = parseDJIFirmwareVersion(firmwareVersion)

        // Create camera settings file
        if (cameraDefinitionGenerator.shouldCreateNewFile()) {
            cameraDefinitionGenerator.writeCameraDefinition()
        }

        val cameraDefinitionURI = cameraDefinitionGenerator.cameraDefinitionsURI
        val cameraDetails = cameraDefinitionGenerator.cameraDetails

        return CameraInformation.builder()
            .timeBootMs(millisSinceBoot)
            .vendorName(cameraDetails!!.vendorName!!.toByteArray())
            .modelName(cameraDetails.modelName!!.toByteArray())
            .firmwareVersion(mavlinkFirmwareVersion)
            .focalLength(focalLength.toFloat())
            .sensorSizeH(cameraDetails.sensorHeight)
            .sensorSizeV(cameraDetails.sensorWidth)
            .resolutionH(cameraDetails.resolutionHeight)
            .resolutionV(cameraDetails.resolutionWidth)
            .flags(
                CameraCapFlags.CAMERA_CAP_FLAGS_HAS_MODES,
                CameraCapFlags.CAMERA_CAP_FLAGS_CAPTURE_IMAGE,
                CameraCapFlags.CAMERA_CAP_FLAGS_CAPTURE_VIDEO,
                CameraCapFlags.CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM,
                CameraCapFlags.CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS,
                CameraCapFlags.CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM
            )
            .camDefinitionVersion(CameraDefinitionGenerator.DEFINITIONS_VERSION)
            .camDefinitionUri(cameraDefinitionURI)
            .build()
    }

    private fun toMavlinkVideoStreamInformationMessage(): VideoStreamInformation {
        return if (LiveStreamManager.getInstance().isStreaming && LiveStreamManager.getInstance().cameraIndex == ComponentIndexType.FPV && lastLiveStreamStatus != null
        ) {
            VideoStreamInformation.builder()
                .streamId(1)
                .count(1)
                .type(VideoStreamType.VIDEO_STREAM_TYPE_RTSP)
                .flags(EnumValue.create(VideoStreamStatusFlags.VIDEO_STREAM_STATUS_FLAGS_RUNNING))
                .framerate(lastLiveStreamStatus!!.fps.toFloat())
                .resolutionH(lastLiveStreamStatus!!.resolution.height)
                .resolutionV(lastLiveStreamStatus!!.resolution.width)
                .bitrate(LiveStreamManager.getInstance().liveVideoBitrate.toLong())
                .name(cameraDefinitionGenerator.cameraDetails.toString())
                .uri(streamAddress)
                .build()
        } else {
            VideoStreamInformation.builder()
                .streamId(1)
                .count(1)
                .type(VideoStreamType.VIDEO_STREAM_TYPE_RTSP)
                .name(cameraDefinitionGenerator.cameraDetails.toString())
                .uri(streamAddress)
                .build()
        }
    }

    private fun toMavlinkVideoStreamStatusMessage(): VideoStreamStatus {
        if (lastLiveStreamStatus == null) {
            return VideoStreamStatus.builder()
                .streamId(1)
                .hfov(horizontalFOV)
                .build()
        }

        if (lastLiveStreamStatus!!.isStreaming && LiveStreamManager.getInstance().cameraIndex == componentIndex) {
            return VideoStreamStatus.builder()
                .streamId(1)
                .flags(VideoStreamStatusFlags.VIDEO_STREAM_STATUS_FLAGS_RUNNING)
                .framerate(lastLiveStreamStatus!!.fps.toFloat())
                .resolutionH(lastLiveStreamStatus!!.resolution.height)
                .resolutionV(lastLiveStreamStatus!!.resolution.width)
                .bitrate(LiveStreamManager.getInstance().liveVideoBitrate.toLong())
                .hfov(horizontalFOV)
                .build()
        }

        return VideoStreamStatus.builder()
            .streamId(1)
            .hfov(horizontalFOV)
            .build()
    }

    private fun toMavlinkCameraSettingsMessage(): CameraSettings {
        val mavlinkCameraMode = when (djiCameraMode) {
            CameraMode.PHOTO_NORMAL, CameraMode.PHOTO_INTERVAL -> io.dronefleet.mavlink.common.CameraMode.CAMERA_MODE_IMAGE
            CameraMode.VIDEO_NORMAL -> io.dronefleet.mavlink.common.CameraMode.CAMERA_MODE_VIDEO
            else -> io.dronefleet.mavlink.common.CameraMode.CAMERA_MODE_IMAGE_SURVEY
        }
        return CameraSettings.builder()
            .modeId(mavlinkCameraMode)
            .build()
    }

    private fun toMavlinkStorageInformationMessage(): StorageInformation {
        val cameraStorageInfos = KeyManager.getInstance().getValue(KeyTools.createCameraKey(CameraKey.KeyCameraStorageInfos, componentIndex, cameraLensType))
        if (cameraStorageInfos == null) {
            log.e("Cannot read storage information from camera!")
            return StorageInformation.builder()
                .timeBootMs(millisSinceBoot)
                .build()
        }
        val cameraStorageLocation = cameraStorageInfos.currentStorageType
        val cameraStorageInfo = cameraStorageInfos.getCameraStorageInfoByLocation(cameraStorageLocation)

        var storageStatus = StorageStatus.STORAGE_STATUS_NOT_SUPPORTED
        if (cameraStorageInfo.storageState == SDCardLoadState.INSERTED) {
            storageStatus = StorageStatus.STORAGE_STATUS_READY
        } else if (cameraStorageInfo.storageState == SDCardLoadState.NOT_INSERTED) {
            storageStatus = StorageStatus.STORAGE_STATUS_EMPTY
        }

        return StorageInformation.builder()
            .timeBootMs(millisSinceBoot)
            .storageId(1)
            .storageCount(1)
            .status(storageStatus)
            .totalCapacity(cameraStorageInfo.storageCapacity.toFloat())
            .usedCapacity((cameraStorageInfo.storageCapacity - cameraStorageInfo.storageLeftCapacity).toFloat())
            .availableCapacity(cameraStorageInfo.storageLeftCapacity.toFloat())
            .type(StorageType.STORAGE_TYPE_SD)
            .name("SD-card")
            .storageUsage(
                StorageUsageFlag.STORAGE_USAGE_FLAG_PHOTO,
                StorageUsageFlag.STORAGE_USAGE_FLAG_VIDEO
            )
            .build()
    }

    private fun toMavlinkCameraCaptureStatusMessage(): CameraCaptureStatus {
        // Current status of image capturing (0: idle, 1: capture in progress, 2: interval set
        // but idle, 3: interval set and capture in progress)
        val imageStatus = when {
            isCapturingImages && imageCapturingInterval > 0 -> 3
            imageCapturingInterval > 0 -> 2
            isCapturingImages -> 1
            else -> 0
        }

        val cameraStorageInfos = KeyManager.getInstance().getValue(KeyTools.createCameraKey(CameraKey.KeyCameraStorageInfos, componentIndex, cameraLensType))
        if (cameraStorageInfos == null) {
            log.e("Cannot read storage information from camera!")
            return CameraCaptureStatus.builder()
                .timeBootMs(millisSinceBoot)
                .build()
        }

        val cameraStorageLocation = cameraStorageInfos.currentStorageType
        val cameraStorageInfo = cameraStorageInfos.getCameraStorageInfoByLocation(cameraStorageLocation)

        return CameraCaptureStatus.builder()
            .timeBootMs(millisSinceBoot)
            .imageStatus(imageStatus)
            .videoStatus(if (videoIsRecording) 1 else 0)
            .imageInterval(imageCapturingInterval)
            .recordingTimeMs(videoRecordingTime * 1000L)
            .availableCapacity(cameraStorageInfo.availablePhotoCount.toFloat())
            .imageCount(capturedNumberOfImages)
            .build()
    }

    private fun toMavlinkCameraImageCapturedMessage(mediaFileInfo: GeneratedMediaFileInfo): CameraImageCaptured {
        // Resolve time when image was created
        val calendar = Calendar.getInstance(TimeZone.getDefault())
        calendar.clear()
        calendar[mediaFileInfo.createTime.year, mediaFileInfo.createTime.month - 1, mediaFileInfo.createTime.day, mediaFileInfo.createTime.hour, mediaFileInfo.createTime.minute] =
            mediaFileInfo.createTime.second

        val utcTime = calendar.timeInMillis

        // TODO: get GPS coordinates and gimbal angle
        // TODO: check image index > Zero based index of this image (i.e. a new image will have index CAMERA_CAPTURE_STATUS.image count -1)
        val ftpUrl =
            "mftp://[;comp=${EnumValue.create(componentID).value()}]images/${mediaFileInfo.index}.${mediaFileInfo.type.name.lowercase(Locale.getDefault())}"

        return CameraImageCaptured.builder()
            .timeBootMs(utcTime - bootMillis)
            .timeUtc(BigInteger.valueOf(utcTime / 1000L))
            .imageIndex(mediaFileInfo.index)
            .captureResult(1)
            .fileUrl(ftpUrl)
            .build()
    }

    private fun parseDJIFirmwareVersion(version: String): Long {
        val parts = version.substring(1).split("\\.".toRegex()).dropLastWhile { it.isEmpty() }.toTypedArray()
        if (parts.size != 4) {
            log.w("Could not parse camera firmware version!")
            return 0L
        }

        val dev = parts[3].toInt()
        val patch = parts[2].toInt()
        val minor = parts[1].toInt()
        val major = parts[0].toInt()

        return ((dev and 0xff) shl 24 or
                (patch and 0xff shl 16) or
                (minor and 0xff shl 8) or
                (major and 0xff)).toLong()
    }
}
