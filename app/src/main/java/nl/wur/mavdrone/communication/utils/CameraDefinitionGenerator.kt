package nl.wur.mavdrone.communication.utils

import androidx.annotation.Size
import dji.sdk.keyvalue.key.CameraKey
import dji.sdk.keyvalue.key.DJIKeyInfo
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.camera.CameraExposureCompensation
import dji.sdk.keyvalue.value.camera.CameraExposureMode
import dji.sdk.keyvalue.value.camera.CameraISO
import dji.sdk.keyvalue.value.camera.CameraMode
import dji.sdk.keyvalue.value.camera.CameraShutterSpeed
import dji.sdk.keyvalue.value.camera.CameraType
import dji.sdk.keyvalue.value.camera.CameraVideoStreamSourceType
import dji.sdk.keyvalue.value.camera.CameraWhiteBalanceMode
import dji.sdk.keyvalue.value.camera.PhotoFileFormat
import dji.sdk.keyvalue.value.camera.PhotoRatio
import dji.sdk.keyvalue.value.camera.VideoFileFormat
import dji.sdk.keyvalue.value.common.CameraLensType
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.KeyManager
import dji.v5.utils.common.ContextUtil
import dji.v5.utils.common.DiskUtil
import io.dronefleet.mavlink.standard.MavComponent
import io.dronefleet.mavlink.util.EnumValue
import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.BaseComponent
import nl.wur.mavdrone.communication.utils.FileUtils.Companion.writeFileUsingTemplate
import nl.wur.mavdrone.utils.RxException
import java.io.File
import java.lang.ref.WeakReference
import java.util.Locale

class CameraDefinitionGenerator(
    connectionWR: WeakReference<MavConnection>,
    private val componentIndex: ComponentIndexType,
    private val cameraLensType: CameraLensType,
    private val componentId: MavComponent
) : BaseComponent(connectionWR, "MavCameraDefWriter") {
    var cameraDetails: CameraDetails? = null
        private set

    private var cameraDefinitionsFile: String? = null

    override fun create() {
        prepareFolders()

        val cameraType = KeyManager.getInstance().getValue(
            KeyTools.createCameraKey(
                CameraKey.KeyCameraType,
                componentIndex,
                cameraLensType
            )
        )
        if (cameraType == null) {
            log.e("Could not retrieve camera type!")
        } else {
            cameraDetails = CameraDetails(cameraType)
//            cameraDefinitionsFile =
//                CAMERA_DEFINITIONS_FOLDER + "camera_definition_" + cameraDetails!!.vendorName + "_" + cameraDetails!!.modelName + ".xml"

            cameraDefinitionsFile =
                "[;comp=${EnumValue.of(componentId).value()}]camera_definition_${cameraDetails?.vendorName ?: "unknown"}_${cameraDetails?.modelName ?: "unknown"}.xml"
        }
    }

    val cameraDefinitionsURI: String?
        get() {
            if (cameraDefinitionsFile == null) {
                return null
            }
            return "mftp://$cameraDefinitionsFile"
        }

    fun shouldCreateNewFile(): Boolean {
        val cameraDefinitionsPath =
            DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), CAMERA_DEFINITIONS_FOLDER + cameraDefinitionsFile)
        val cameraDefinitionsFile = File(cameraDefinitionsPath)

        if (!cameraDefinitionsFile.exists()) {
            return true
        }

        // Also generate new file when it's one day old
        val oneDayInMillis = (24 * 60 * 60 * 1000).toLong()
        return cameraDefinitionsFile.lastModified() < (System.currentTimeMillis() - oneDayInMillis)
    }

    fun writeCameraDefinition() {
        val cameraDefinitionsPath =
            DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), CAMERA_DEFINITIONS_FOLDER + cameraDefinitionsFile)
        val cameraDefinitionsFile = File(cameraDefinitionsPath)
        if (cameraDefinitionsFile.exists()) {
            cameraDefinitionsFile.delete()
        }

        val content: MutableMap<String, Any> = HashMap()

        if (cameraDetails == null) {
            log.e("Camera details not available!")
            return
        }

        content["model"] = cameraDetails?.modelName ?: "Unknown"
        content["vendor"] = cameraDetails?.vendorName ?: "Unknown"

        val parameterObjectList: MutableList<Map<String, Any>> = ArrayList()
        parameterCameraMode(parameterObjectList)
        parameterExposureMode(parameterObjectList)
        parameterFromDJIRange(
            parameterObjectList,
            "CAM_PHOTOFMT",
            "Image Format",
            CameraKey.KeyPhotoFileFormatRange,
            PhotoFileFormat.JPEG
        )
        parameterFromDJIRange(
            parameterObjectList,
            "CAM_VIDFMT",
            "Video Format",
            CameraKey.KeyVideoFileFormatRange,
            VideoFileFormat.MP4
        )
        parameterFromDJIRange(
            parameterObjectList,
            "CAM_EV",
            "Exposure compensation",
            CameraKey.KeyExposureCompensationRange,
            CameraExposureCompensation.FIXED
        )
        parameterFromDJIRange(
            parameterObjectList,
            "CAM_ISO",
            "ISO",
            CameraKey.KeyISORange,
            CameraISO.ISO_AUTO
        )
//        TODO: create custom converter to enable whitebalance in CameraParameters.kt
//        parameterFromDJIRange(
//            parameterObjectList,
//            "CAM_WBMODE",
//            "White Balance Mode",
//            CameraKey.KeyCameraWhiteBalanceRange,
//            CameraWhiteBalanceMode.AUTO
//        )
//        TODO: create custom converter to enable cameraStreamSource in CameraParameters.kt
//        parameterFromDJIRange(
//            parameterObjectList,
//            "CAM_VIDSTREAMSRC",
//            "Video stream source",
//            CameraKey.KeyCameraVideoStreamSourceRange,
//            CameraVideoStreamSourceType.DEFAULT_CAMERA
//        )
        parameterFromDJIRange(
            parameterObjectList,
            "CAM_PHOTORATIO",
            "Photo ratio",
            CameraKey.KeyPhotoRatioRange,
            PhotoRatio.RATIO_16COLON9
        )
        parameterFromDJIRange(
            parameterObjectList,
            "CAM_SHUTTERSPD",
            "Shutter speed",
            CameraKey.KeyShutterSpeedRange,
            CameraShutterSpeed.SHUTTER_SPEED_AUTO
        )
        content["parameters"] = parameterObjectList

        try {
            writeFileUsingTemplate(
                "templates/camera_definition.template.xml",
                cameraDefinitionsPath,
                content,
                "camera definition",
                log
            )
        } catch (e: RxException) {
            log.e(e.message)
        }
    }

    private fun <T : Enum<T>> parameterFromDJIRange(
        list: MutableList<Map<String, Any>>,
        @Size(max = 16) name: String,
        description: String,
        rangeKey: DJIKeyInfo<List<T>>,
        defaultValue: T
    ) {
        val parameterObject: MutableMap<String, Any> = HashMap()
        parameterObject["name"] = name
        parameterObject["description"] = description
        parameterObject["defaultValue"] = defaultValue.ordinal

        val cameraKey = KeyTools.createCameraKey(rangeKey, componentIndex, cameraLensType);
        var enumValueList: List<Enum<*>>? = KeyManager.getInstance().getValue(cameraKey)
        if (enumValueList == null) {
            log.e("Could not retrieve $rangeKey")
            return
        }

        if (!enumValueList.contains(defaultValue)) {
            enumValueList = enumValueList + defaultValue
        }

        val allEnumValueObjects: MutableList<Map<String, Any>> = ArrayList()
        for (enumValue in enumValueList) {
            val enumValueObject: MutableMap<String, Any> = HashMap()
            enumValueObject["name"] = enumValue.name
            enumValueObject["value"] = enumValue.ordinal
            enumValueObject["hasExclusions"] = false
            allEnumValueObjects.add(enumValueObject)
        }

        parameterObject["options"] = allEnumValueObjects

        list.add(parameterObject)
    }

    private fun parameterCameraMode(list: MutableList<Map<String, Any>>) {
        val parameterObject: MutableMap<String, Any> = HashMap()
        parameterObject["name"] = "CAM_MODE"
        parameterObject["description"] = "Camera Mode"
        parameterObject["defaultValue"] = CameraMode.PHOTO_NORMAL.ordinal

        val cameraModeList = KeyManager.getInstance().getValue(
            KeyTools.createCameraKey(
                CameraKey.KeyCameraModeRange,
                componentIndex,
                cameraLensType
            )
        )
        if (cameraModeList == null) {
            log.e("Could not retrieve camera mode list!")
            return
        }

        val allCameraModeObjects: MutableList<Map<String, Any>> = ArrayList()
        for (cameraMode in cameraModeList) {
            val cameraModeObject: MutableMap<String, Any> = HashMap()
            cameraModeObject["name"] = cameraMode.name
            cameraModeObject["value"] = cameraMode.ordinal

            cameraModeObject["hasExclusions"] = false
            if (cameraMode == CameraMode.VIDEO_NORMAL) {
                cameraModeObject["hasExclusions"] = true
                cameraModeObject["exclusions"] = mutableListOf("CAM_PHOTOFMT", "CAM_PHOTORATIO")
            } else if (cameraMode == CameraMode.PHOTO_NORMAL) {
                cameraModeObject["hasExclusions"] = true
                cameraModeObject["exclusions"] = mutableListOf("CAM_VIDFMT")
            } else if (cameraMode.name.lowercase(Locale.getDefault()).startsWith("photo")) {
                cameraModeObject["hasExclusions"] = true
                cameraModeObject["exclusions"] = mutableListOf("CAM_VIDFMT", "CAM_PHOTORATIO")
            }
            allCameraModeObjects.add(cameraModeObject)
        }

        parameterObject["options"] = allCameraModeObjects

        list.add(parameterObject)
    }

    private fun parameterExposureMode(list: MutableList<Map<String, Any>>) {
        val parameterObject: MutableMap<String, Any> = HashMap()
        parameterObject["name"] = "CAM_EXPMODE"
        parameterObject["description"] = "Exposure Mode"
        parameterObject["defaultValue"] = CameraExposureMode.PROGRAM.ordinal

        val cameraModeList = KeyManager.getInstance().getValue(
            KeyTools.createCameraKey(
                CameraKey.KeyExposureModeRange,
                componentIndex,
                cameraLensType
            )
        )
        if (cameraModeList == null) {
            log.e("Could not retrieve camera exposure mode list!")
            return
        }

        val allCameraExposureModeObjects: MutableList<Map<String, Any>> = ArrayList()
        for (exposureMode in cameraModeList) {
            val exposureModeObject: MutableMap<String, Any> = HashMap()
            exposureModeObject["name"] = exposureMode.name
            exposureModeObject["value"] = exposureMode.ordinal

            exposureModeObject["hasExclusions"] = false
            if (exposureMode == CameraExposureMode.MANUAL) {
                exposureModeObject["hasExclusions"] = true
                exposureModeObject["exclusions"] = mutableListOf("CAM_EV")
            } else if (exposureMode == CameraExposureMode.PROGRAM) {
                exposureModeObject["hasExclusions"] = true
                exposureModeObject["exclusions"] = mutableListOf("CAM_ISO", "CAM_SHUTTERSPD")
            } else if (exposureMode == CameraExposureMode.SHUTTER_PRIORITY) {
                exposureModeObject["hasExclusions"] = true
                exposureModeObject["exclusions"] =
                    mutableListOf("CAM_EV", "CAM_ISO", "CAM_SHUTTERSPD")
            } else if (exposureMode == CameraExposureMode.APERTURE_PRIORITY) {
                exposureModeObject["hasExclusions"] = true
                exposureModeObject["exclusions"] =
                    mutableListOf("CAM_EV", "CAM_ISO", "CAM_SHUTTERSPD")
            }
            allCameraExposureModeObjects.add(exposureModeObject)
        }

        parameterObject["options"] = allCameraExposureModeObjects

        list.add(parameterObject)
    }

    private fun prepareFolders() {
        val cameraDefinitionsPath =
            DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), CAMERA_DEFINITIONS_FOLDER)
        val cameraDefinitionsFolder = File(cameraDefinitionsPath)
        if (!cameraDefinitionsFolder.exists()) {
            if (!cameraDefinitionsFolder.mkdir()) {
                log.w("Could not create folder '" + cameraDefinitionsFolder.path + "'")
            }
        }
    }

    class CameraDetails(cameraType: CameraType?) {
                var vendorName: String? = null
                var modelName: String? = null
                var sensorWidth: Float = 0f
                var sensorHeight: Float = 0f
                var resolutionWidth: Int = 0
                var resolutionHeight: Int = 0

        override fun toString(): String {
            return "$vendorName $modelName"
        }

        init {
            when (cameraType) {
                CameraType.M3E -> {
                    // TODO: check these values
                    vendorName = "DJI"
                    modelName = "M3E"
                    sensorWidth = 25.4f
                    sensorHeight = 43.18f
                    resolutionWidth = 640
                    resolutionHeight = 512
                }

                CameraType.M3T -> {
                    // TODO: check these values
                    vendorName = "DJI"
                    modelName = "M3T"
                    sensorWidth = 25.4f
                    sensorHeight = 43.18f
                    resolutionWidth = 640
                    resolutionHeight = 512
                }

                CameraType.M30 -> {
                    // TODO: check these values
                    vendorName = "DJI"
                    modelName = "M30"
                    sensorWidth = 12.7f
                    sensorHeight = 12.7f
                    resolutionWidth = 3840
                    resolutionHeight = 2160
                }

                CameraType.M30T -> {
                    // TODO: check these values
                    vendorName = "DJI"
                    modelName = "M30T"
                    sensorWidth = 25.4f
                    sensorHeight = 43.18f
                    resolutionWidth = 3840
                    resolutionHeight = 2160
                }

                CameraType.ZENMUSE_H20 -> {
                    // TODO: check these values
                    vendorName = "Zenmuse"
                    modelName = "H20"
                    sensorWidth = 25.4f
                    sensorHeight = 43.18f
                    resolutionWidth = 3840
                    resolutionHeight = 2160
                }

                CameraType.ZENMUSE_H20T -> {
                    // TODO: check these values
                    vendorName = "Zenmuse"
                    modelName = "H20T"
                    sensorWidth = 25.4f
                    sensorHeight = 43.18f
                    resolutionWidth = 3840
                    resolutionHeight = 2160
                }

                CameraType.ZENMUSE_P1 -> {
                    vendorName = "Zenmuse"
                    modelName = "P1"
                    sensorWidth = 35.9f
                    sensorHeight = 24.0f
                    resolutionWidth = 8192
                    resolutionHeight = 5460
                }

                CameraType.ZENMUSE_L1 -> {
                    // TODO: check these values
                    vendorName = "Zenmuse"
                    modelName = "L1"
                    sensorWidth = 25.4f
                    sensorHeight = 25.4f
                    resolutionWidth = 5472
                    resolutionHeight = 3078
                }

                else -> {
                    vendorName = "unknown"
                    modelName = "unknown"
                    sensorWidth = 0.0f
                    sensorHeight = 0.0f
                    resolutionWidth = 0
                    resolutionHeight = 0
                }
            }
        }
    } //endregion

    companion object {
        const val DEFINITIONS_VERSION: Int = 2
        private const val CAMERA_DEFINITIONS_FOLDER = "ComponentMetadata/"
    }
}
