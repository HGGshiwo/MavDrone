package nl.wur.mavdrone.communication.components.impl

import java.lang.ref.WeakReference
import java.util.concurrent.TimeUnit

import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.common.callback.CommonCallbacks.CompletionCallback
import dji.v5.common.error.IDJIError
import dji.v5.manager.datacenter.livestream.LiveStreamManager
import dji.v5.manager.datacenter.livestream.LiveStreamSettings
import dji.v5.manager.datacenter.livestream.LiveStreamStatus
import dji.v5.manager.datacenter.livestream.LiveStreamStatusListener
import dji.v5.manager.datacenter.livestream.LiveStreamType
import dji.v5.manager.datacenter.livestream.StreamQuality
import dji.v5.manager.datacenter.livestream.settings.RtspSettings
import dji.v5.utils.common.ContextUtil

import io.dronefleet.mavlink.MavlinkMessage
import io.dronefleet.mavlink.common.CameraCapFlags
import io.dronefleet.mavlink.common.CameraInformation
import io.dronefleet.mavlink.common.CameraMode
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
import io.dronefleet.mavlink.util.EnumValue
import io.reactivex.rxjava3.android.schedulers.AndroidSchedulers
import io.reactivex.rxjava3.core.Completable
import io.reactivex.rxjava3.core.Observable
import io.reactivex.rxjava3.schedulers.Schedulers

import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.BaseComponent
import nl.wur.mavdrone.communication.components.Camera
import nl.wur.mavdrone.communication.components.Drone
import nl.wur.mavdrone.communication.utils.FileUtils.Companion.createFolderIfNeeded
import nl.wur.mavdrone.communication.utils.FileUtils.Companion.getAvailableDiskSpaceForFolder
import nl.wur.mavdrone.communication.utils.FileUtils.Companion.getExternalCacheDirPath
import nl.wur.mavdrone.communication.utils.FileUtils.Companion.getTotalDiskSpace
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandAccepted
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandDenied
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandFailed
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandUnsupported
import nl.wur.mavdrone.utils.NetworkUtils.Companion.getLocalWifiIpAddress
import nl.wur.mavdrone.utils.RXUtils.Companion.djiCompletableFunction
import kotlin.math.roundToInt


class OnBoardCamera(
    connectionWR: WeakReference<MavConnection>,
    private val droneWR: WeakReference<Drone>
) : BaseComponent(connectionWR, "MavOnboardCamera"), LiveStreamStatusListener {
    private val streamUsername = connectionWR.get()!!.streamInformation.user
    private val streamPassword = connectionWR.get()!!.streamInformation.password
    private val streamPort = connectionWR.get()!!.streamInformation.port
    private val streamAddress: String

    private val imageSaveFolder = getExternalCacheDirPath("OnboardCameraImages")
    private var lastLiveStreamStatus: LiveStreamStatus? = null

    init {
        val ip = getLocalWifiIpAddress(ContextUtil.getContext())
        streamAddress = "rtsp://$streamUsername:$streamPassword@$ip:$streamPort/streaming/live/1"

        LiveStreamManager.getInstance().addLiveStreamStatusListener(this)

        createFolderIfNeeded(imageSaveFolder)
    }

    override fun create() {
        droneWR.get()?.apply {
            addMavlinkMessageSender(CameraInformation::class.java) { this@OnBoardCamera.toMavlinkCameraInformationMessage() }
            addMavlinkMessageSender(VideoStreamInformation::class.java) { this@OnBoardCamera.toMavlinkVideoStreamInformationMessage() }
            addMavlinkMessageSender(VideoStreamStatus::class.java) { this@OnBoardCamera.toMavlinkVideoStreamStatusMessage() }
            addMavlinkMessageSender(CameraSettings::class.java) { this@OnBoardCamera.toMavlinkCameraSettingsMessage() }
            addMavlinkMessageSender(StorageInformation::class.java) { this@OnBoardCamera.toMavlinkCameraStorageInformation() }
        }

        // Start streaming when autostart is enabled and there is no camera connected. Wait for 3 seconds to give
        // some time to initialise the cameras.
        connectionWR.get()?.let { 
            if (it.streamInformation.autoStart) {
                compositeDisposable.add(
                    Observable.interval(3, 10, TimeUnit.SECONDS)
                        .subscribeOn(Schedulers.io())
                        .flatMap {
                            Observable.just<List<Camera>>(connectionWR.get()!!.mavlinkComponents!!.cameras)
                        }
                        .distinctUntilChanged()
                        .filter { obj: List<Camera> -> obj.isEmpty() }
                        .observeOn(AndroidSchedulers.mainThread())
                        .flatMapCompletable { enableLiveStream() }
                        .subscribe(
                            {},
                            { throwable: Throwable -> log.e("Could not auto-start camera: $throwable") })
                )
            }
        }
    }

    override fun destroy() {
        LiveStreamManager.getInstance().removeLiveStreamStatusListener(this)

        super.destroy()
    }

    @Suppress("deprecation")
    fun onCommandLong(msg: MavlinkMessage<*>): Boolean {
        val payload = msg.payload as CommandLong

        if (payload.targetSystem() == systemID
            && (payload.targetComponent() == EnumValue.create(droneWR.get()!!.componentID)
                .value() || payload.targetComponent() == 0)
        ) {
            when (payload.command().entry()) {
                MavCmd.MAV_CMD_SET_CAMERA_MODE -> {
                    droneWR.get()?.sendMessage(commandUnsupported(MavCmd.MAV_CMD_SET_CAMERA_MODE))
                    return true
                }

                MavCmd.MAV_CMD_VIDEO_START_STREAMING -> {
                    if (payload.param1().roundToInt() != 0 || payload.param1().roundToInt() == 1) {
                        log.e("Cannot start stream ${payload.param1().roundToInt()}, only stream with id 1 exists!")
                        droneWR.get()?.sendMessage(commandFailed(MavCmd.MAV_CMD_VIDEO_START_STREAMING))
                        return true
                    }

                    compositeDisposable.add(
                        enableLiveStream()
                            .subscribeOn(AndroidSchedulers.mainThread())
                            .observeOn(Schedulers.io())
                            .subscribe(
                                {
                                    droneWR.get()!!
                                        .sendMessage(commandAccepted(MavCmd.MAV_CMD_VIDEO_START_STREAMING))
                                },
                                { throwable: Throwable ->
                                    log.e("Could not start stream: " + throwable.message)
                                    droneWR.get()?.sendMessage(commandFailed(MavCmd.MAV_CMD_VIDEO_STOP_STREAMING))
                                })
                    )
                    return true
                }

                MavCmd.MAV_CMD_VIDEO_STOP_STREAMING -> {
                    if (payload.param1().roundToInt() != 0 || payload.param1().roundToInt() == 1) {
                        log.e("Cannot stop stream ${payload.param1().roundToInt()}, only stream with id 1 exists!")
                        droneWR.get()?.sendMessage(commandFailed(MavCmd.MAV_CMD_VIDEO_START_STREAMING))
                        return true
                    }

                    if (LiveStreamManager.getInstance().isStreaming
                        && LiveStreamManager.getInstance().cameraIndex == ComponentIndexType.FPV
                    ) {
                        compositeDisposable.add(djiCompletableFunction { callback: CompletionCallback? ->
                            LiveStreamManager.getInstance().stopStream(callback)
                        }
                            .subscribeOn(AndroidSchedulers.mainThread())
                            .observeOn(Schedulers.io())
                            .subscribe(
                                {
                                    droneWR.get()?.sendMessage(commandAccepted(MavCmd.MAV_CMD_VIDEO_STOP_STREAMING))
                                },
                                { throwable: Throwable ->
                                    log.e("Could not stop stream: " + throwable.message)
                                    droneWR.get()?.sendMessage(commandFailed(MavCmd.MAV_CMD_VIDEO_STOP_STREAMING))
                                })
                        )
                    } else {
                        droneWR.get()?.sendMessage(commandAccepted(MavCmd.MAV_CMD_VIDEO_STOP_STREAMING))
                    }
                    return true
                }

                MavCmd.MAV_CMD_REQUEST_CAMERA_INFORMATION -> {
                    droneWR.get()?.sendMessage(
                        commandAccepted(
                            MavCmd.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS,
                            msg.originSystemId,
                            msg.originComponentId
                        )
                    )
                    val cameraInformation = toMavlinkCameraInformationMessage()
                    droneWR.get()?.sendMessage(cameraInformation)
                    if (payload.param1().roundToInt() == 0 || payload.param1().roundToInt() == 1) {
                        droneWR.get()?.sendMessage(
                            commandAccepted(
                                MavCmd.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION,
                                msg.originSystemId,
                                msg.originComponentId
                            )
                        )
                        val videoStreamInformation = toMavlinkVideoStreamInformationMessage()
                        droneWR.get()?.sendMessage(videoStreamInformation)
                    } else {
                        droneWR.get()?.sendMessage(
                            commandDenied(
                                MavCmd.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION,
                                msg.originSystemId,
                                msg.originComponentId
                            )
                        )
                    }
                    return true
                }

                MavCmd.MAV_CMD_REQUEST_CAMERA_SETTINGS -> {
                    droneWR.get()?.sendMessage(
                        commandAccepted(
                            MavCmd.MAV_CMD_REQUEST_CAMERA_SETTINGS,
                            msg.originSystemId,
                            msg.originComponentId
                        )
                    )
                    val cameraSettingsInformation = toMavlinkCameraSettingsMessage()
                    droneWR.get()?.sendMessage(cameraSettingsInformation)
                    return true
                }

                MavCmd.MAV_CMD_REQUEST_STORAGE_INFORMATION -> {
                    droneWR.get()?.sendMessage(
                        commandAccepted(
                            MavCmd.MAV_CMD_REQUEST_STORAGE_INFORMATION,
                            msg.originSystemId,
                            msg.originComponentId
                        )
                    )
                    val storageInformation = toMavlinkCameraStorageInformation()
                    droneWR.get()?.sendMessage(storageInformation)
                    return true
                }

                MavCmd.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION -> {
                    if (payload.param1().roundToInt() == 0 || payload.param1().roundToInt() == 1) {
                        droneWR.get()?.sendMessage(
                            commandAccepted(
                                MavCmd.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION,
                                msg.originSystemId,
                                msg.originComponentId
                            )
                        )
                        val videoStreamInformation = toMavlinkVideoStreamInformationMessage()
                        droneWR.get()?.sendMessage(videoStreamInformation)
                    } else {
                        droneWR.get()?.sendMessage(
                            commandDenied(
                                MavCmd.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION,
                                msg.originSystemId,
                                msg.originComponentId
                            )
                        )
                    }
                    return true
                }

                MavCmd.MAV_CMD_REQUEST_VIDEO_STREAM_STATUS -> {
                    if (payload.param1().roundToInt() == 0 || payload.param1().roundToInt() == 1) {
                        droneWR.get()?.sendMessage(
                            commandAccepted(
                                MavCmd.MAV_CMD_REQUEST_VIDEO_STREAM_STATUS,
                                msg.originSystemId,
                                msg.originComponentId
                            )
                        )
                        val videoStreamStatus = toMavlinkVideoStreamStatusMessage()
                        droneWR.get()?.sendMessage(videoStreamStatus)
                    } else {
                        droneWR.get()?.sendMessage(
                            commandDenied(
                                MavCmd.MAV_CMD_REQUEST_VIDEO_STREAM_STATUS,
                                msg.originSystemId,
                                msg.originComponentId
                            )
                        )
                    }
                    return true
                }

                else -> {}
            }
        }

        return false
    }

    private fun toMavlinkCameraInformationMessage(): CameraInformation {
        return CameraInformation.builder()
            .timeBootMs(droneWR.get()?.millisSinceBoot ?: 0L)
            .vendorName(CAMERA_MANUFACTURER.toByteArray())
            .modelName(CAMERA_NAME.toByteArray())
            .flags(CameraCapFlags.CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM)
            .build()
    }

    private fun toMavlinkVideoStreamInformationMessage(): VideoStreamInformation {
        lastLiveStreamStatus?.let {
            if (LiveStreamManager.getInstance().isStreaming && LiveStreamManager.getInstance().cameraIndex == ComponentIndexType.FPV) {
                return VideoStreamInformation.builder()
                    .streamId(1)
                    .count(1)
                    .type(VideoStreamType.VIDEO_STREAM_TYPE_RTSP)
                    .flags(EnumValue.create(VideoStreamStatusFlags.VIDEO_STREAM_STATUS_FLAGS_RUNNING))
                    .framerate(it.fps.toFloat())
                    .resolutionH(it.resolution.height)
                    .resolutionV(it.resolution.width)
                    .bitrate(LiveStreamManager.getInstance().liveVideoBitrate.toLong())
                    .name(CAMERA_NAME)
                    .uri(streamAddress)
                    .build()
            }
        }

        return VideoStreamInformation.builder()
            .streamId(1)
            .count(1)
            .type(VideoStreamType.VIDEO_STREAM_TYPE_RTSP)
            .name(CAMERA_NAME)
            .uri(streamAddress)
            .build()
    }

    private fun toMavlinkVideoStreamStatusMessage(): VideoStreamStatus {
        lastLiveStreamStatus?.let {
            val bitrate = LiveStreamManager.getInstance().liveVideoBitrate

            return VideoStreamStatus.builder()
                .streamId(1)
                .flags(VideoStreamStatusFlags.VIDEO_STREAM_STATUS_FLAGS_RUNNING)
                .framerate(lastLiveStreamStatus!!.fps.toFloat())
                .resolutionH(lastLiveStreamStatus!!.resolution.height)
                .resolutionV(lastLiveStreamStatus!!.resolution.width)
                .bitrate(bitrate.toLong())
                .build()
        }

        return VideoStreamStatus.builder()
            .streamId(1)
            .build()
    }

    private fun toMavlinkCameraSettingsMessage(): CameraSettings {
        return CameraSettings.builder()
            .timeBootMs(droneWR.get()?.millisSinceBoot ?: 0L)
            .modeId(CameraMode.CAMERA_MODE_IMAGE)
            .zoomlevel(Float.NaN)
            .focuslevel(Float.NaN)
            .build();
    }

    private fun toMavlinkCameraStorageInformation(): StorageInformation {
        val totalCapacity = getTotalDiskSpace(imageSaveFolder).toFloat() / (1024 * 2024)
        val availableCapacity = getAvailableDiskSpaceForFolder(imageSaveFolder).toFloat() / (1024 * 1024)
        return StorageInformation.builder()
            .timeBootMs(droneWR.get()?.millisSinceBoot ?: 0L)
            .storageId(1)
            .storageCount(1)
            .status(StorageStatus.STORAGE_STATUS_READY)
            .totalCapacity(totalCapacity)
            .usedCapacity(totalCapacity - availableCapacity)
            .availableCapacity(availableCapacity)
            .type(StorageType.STORAGE_TYPE_OTHER)
            .name("Internal storage")
            .storageUsage(
                StorageUsageFlag.STORAGE_USAGE_FLAG_PHOTO,
                StorageUsageFlag.STORAGE_USAGE_FLAG_VIDEO
            )
            .build()
    }

    private fun enableLiveStream(): Completable {
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
            return djiCompletableFunction { callback: CompletionCallback? ->
                LiveStreamManager.getInstance().startStream(callback)
            }
                .andThen(Completable.fromRunnable {
                    LiveStreamManager.getInstance().liveStreamQuality = StreamQuality.FULL_HD
                    if (LiveStreamManager.getInstance().cameraIndex != ComponentIndexType.FPV) {
                        LiveStreamManager.getInstance().cameraIndex = ComponentIndexType.FPV
                    }
                    log.i("Started camera stream at $streamAddress")
                })
        }

        return Completable.fromRunnable {
            if (LiveStreamManager.getInstance().cameraIndex != ComponentIndexType.FPV) {
                LiveStreamManager.getInstance().cameraIndex = ComponentIndexType.FPV
                log.i("Changed camera stream to FPV")
            }
        }
    }
    
    //endregion
    override fun onLiveStreamStatusUpdate(status: LiveStreamStatus) {
        lastLiveStreamStatus = status
    }

    override fun onError(error: IDJIError) {
        log.w("Could not retrieve last livestream status information: " + error.description())
    }

    companion object {
        const val CAMERA_MANUFACTURER = "DJI"
        const val CAMERA_NAME = "Onboard camera"
    }
}
