package nl.wur.mavdrone.communication.components.impl

import dji.sdk.keyvalue.key.CameraKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.camera.CameraExposureCompensation
import dji.sdk.keyvalue.value.camera.CameraExposureMode
import dji.sdk.keyvalue.value.camera.CameraISO
import dji.sdk.keyvalue.value.camera.CameraMode
import dji.sdk.keyvalue.value.camera.CameraShutterSpeed
import dji.sdk.keyvalue.value.camera.CameraWhiteBalanceInfo
import dji.sdk.keyvalue.value.camera.CameraWhiteBalanceMode
import dji.sdk.keyvalue.value.camera.PhotoFileFormat
import dji.sdk.keyvalue.value.camera.PhotoRatio
import dji.sdk.keyvalue.value.camera.VideoFileFormat
import dji.sdk.keyvalue.value.common.CameraLensType
import dji.sdk.keyvalue.value.common.ComponentIndexType
import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.BaseComponent
import nl.wur.mavdrone.communication.components.Camera
import nl.wur.mavdrone.communication.utils.MavParameter
import java.lang.ref.WeakReference

class CameraParameters(
    connectionWR: WeakReference<MavConnection>,
    private val cameraWR: WeakReference<Camera>,
    private val componentIndex: ComponentIndexType,
    private val cameraLensType: CameraLensType
) : BaseComponent(connectionWR, "MavCameraParameters") {
    override fun create() {
        cameraWR.get()?.apply {
            addParameter(
                MavParameter.fromDJIParameter(
                    connectionWR,
                    cameraWR,
                    "CAM_MODE",
                    KeyTools.createCameraKey(CameraKey.KeyCameraMode, componentIndex, cameraLensType),
                    CameraMode.PHOTO_NORMAL,
                    CameraMode::class.java,
                    true,
                    null
                )
            )
            addParameter(
                MavParameter.fromDJIParameter(
                    connectionWR,
                    cameraWR,
                    "CAM_VIDFMT",
                    KeyTools.createCameraKey(
                        CameraKey.KeyVideoFileFormat,
                        componentIndex,
                        cameraLensType
                    ),
                    VideoFileFormat.MP4,
                    VideoFileFormat::class.java,
                    true,
                    null
                )
            )
            addParameter(
                MavParameter.fromDJIParameter(
                    connectionWR,
                    cameraWR,
                    "CAM_PHOTOFMT",
                    KeyTools.createCameraKey(
                        CameraKey.KeyPhotoFileFormat,
                        componentIndex,
                        cameraLensType
                    ),
                    PhotoFileFormat.JPEG,
                    PhotoFileFormat::class.java,
                    true,
                    null
                )
            )
            addParameter(
                MavParameter.fromDJIParameter(
                    connectionWR,
                    cameraWR,
                    "CAM_EXPMODE",
                    KeyTools.createCameraKey(CameraKey.KeyExposureMode, componentIndex, cameraLensType),
                    CameraExposureMode.SHUTTER_PRIORITY,
                    CameraExposureMode::class.java,
                    true,
                    null
                )
            )
            addParameter(
                MavParameter.fromDJIParameter(
                    connectionWR,
                    cameraWR,
                    "CAM_EV",
                    KeyTools.createCameraKey(
                        CameraKey.KeyExposureCompensation,
                        componentIndex,
                        cameraLensType
                    ),
                    CameraExposureCompensation.FIXED,
                    CameraExposureCompensation::class.java,
                    true,
                    null
                )
            )
            addParameter(
                MavParameter.fromDJIParameter(
                    connectionWR,
                    cameraWR,
                    "CAM_ISO",
                    KeyTools.createCameraKey(CameraKey.KeyISO, componentIndex, cameraLensType),
                    CameraISO.ISO_AUTO,
                    CameraISO::class.java,
                    true,
                    null
                )
            )
            addParameter(
                MavParameter.fromDJIParameter(
                    connectionWR,
                    cameraWR,
                    "CAM_SHUTTERSPD",
                    KeyTools.createCameraKey(CameraKey.KeyShutterSpeed, componentIndex, cameraLensType),
                    CameraShutterSpeed.SHUTTER_SPEED_AUTO,
                    CameraShutterSpeed::class.java,
                    true,
                    null
                )
            )
            addParameter(
                MavParameter.fromDJIParameter(
                    connectionWR,
                    cameraWR,
                    "CAM_PHOTORATIO",
                    KeyTools.createCameraKey(CameraKey.KeyPhotoRatio, componentIndex, cameraLensType),
                    PhotoRatio.RATIO_3COLON2,
                    PhotoRatio::class.java,
                    true,
                    null
                )
            )
        }
    }
}
