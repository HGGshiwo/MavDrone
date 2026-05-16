package nl.wur.mavdrone.communication.utils

import dji.v5.utils.common.ContextUtil
import io.dronefleet.mavlink.common.CompMetadataType
import io.dronefleet.mavlink.standard.MavComponent
import io.dronefleet.mavlink.util.EnumValue
import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.BaseComponent
import nl.wur.mavdrone.communication.utils.FileUtils.Companion.writeFileUsingTemplate
import nl.wur.mavdrone.communication.utils.FileUtils.Companion.getExternalCacheDirPath
import nl.wur.mavdrone.utils.RxException
import java.io.BufferedInputStream
import java.io.File
import java.io.FileInputStream
import java.io.IOException
import java.io.InputStream
import java.lang.ref.WeakReference
import java.util.zip.CRC32

class ComponentInfoGenerator(
    connectionWR: WeakReference<MavConnection>,
    componentID: MavComponent
) : BaseComponent(connectionWR, "MavComponentInfoWriter") {
    private var componentMetadataFile: String = "[;comp=${EnumValue.of(componentID).value()}]component_metadata.json"
    private var parameterFile: String = "[;comp=${EnumValue.of(componentID).value()}]parameters.json"

    init {
        prepareFolders()
    }

    val componentMetadataURI: String
        get() = "mftp://$componentMetadataFile"


    val componentMetadataCRC: Long
        get() {
            try {
                val componentMetadataPath = getExternalCacheDirPath(COMPONENT_INFO_FOLDER + componentMetadataFile)
                return checksumBufferedInputStream(componentMetadataPath)
            } catch (e: IOException) {
                return 0L
            }
        }

    val parameterURI: String
        get() = "mftp://$parameterFile"

    val parameterCRC: Long
        get() {
            try {
                val parameterPath = getExternalCacheDirPath(COMPONENT_INFO_FOLDER + parameterFile)
                return checksumBufferedInputStream(parameterPath)
            } catch (e: IOException) {
                return 0L
            }
        }

    fun writeComponentInformation(
        vendorName: String,
        modelName: String,
        firmwareVersion: String,
        hardwareVersion: String
    ) {
        val componentMetadataPath = getExternalCacheDirPath(COMPONENT_INFO_FOLDER + componentMetadataFile)

        val content: MutableMap<String, Any> = HashMap()
        content["version"] = COMPONENT_INFO_VERSION
        content["vendorName"] = vendorName
        content["modelName"] = modelName
        content["firmwareVersion"] = firmwareVersion
        content["hardwareVersion"] = hardwareVersion

        val metadataTypes: MutableList<Map<String, Any>> = ArrayList()
        val paramCRC = parameterCRC
        if (paramCRC != 0L) {
            val paramType: MutableMap<String, Any> = HashMap()
            paramType["type"] = EnumValue.of(CompMetadataType.COMP_METADATA_TYPE_PARAMETER).value()
            paramType["uri"] = parameterURI
            paramType["fileCrc"] = paramCRC
            paramType["last"] = true
            metadataTypes.add(paramType)
        }
        content["metadataTypes"] = metadataTypes

        try {
            writeFileUsingTemplate(
                "templates/component_info.template.json",
                componentMetadataPath,
                content,
                "component metadata",
                log
            )
        } catch (e: RxException) {
            log.e(e.message)
        }
    }

    fun writeParameterMetadataFile(mavParameters: List<MavParameter<*>>) {
        val parameterMetadataPath = getExternalCacheDirPath(COMPONENT_INFO_FOLDER + parameterFile)

        val parameterList: MutableList<Map<String, Any>> = ArrayList()
        for (i in mavParameters.indices) {
            val parameter = mavParameters[i]

            // Don't store extended parameter values
            if (parameter.isExtParameter) {
                continue
            }

            val parameterObject: MutableMap<String, Any> = HashMap()
            parameterObject["category"] = parameter.category
            parameterObject["group"] = parameter.group
            parameterObject["longDesc"] = parameter.longDesc
            parameterObject["name"] = parameter.name
            parameterObject["shortDesc"] = parameter.shortDesc
            parameterObject["type"] = parameter.type

            // When parameter is an enum
            if (parameter.values.isNotEmpty()) {
                parameterObject["isEnum"] = true
                parameterObject["default"] = parameter.defaultIntValue
                parameterObject["max"] = parameter.values.size - 1
                parameterObject["min"] = 0
                parameterObject["values"] = getParameterValuesObject(parameter)
            } else {
                parameterObject["isEnum"] = false
                parameterObject["default"] = parameter.defaultValue!!
                parameterObject["max"] = parameter.maxValue!!
                parameterObject["min"] = parameter.minValue!!
                parameterObject["units"] = parameter.units
            }

            parameterObject["last"] = i == mavParameters.size - 1
            parameterList.add(parameterObject)
        }

        val content: MutableMap<String, Any> = HashMap()
        content["parameters"] = parameterList

        try {
            writeFileUsingTemplate(
                "templates/parameter_metadata.template.json",
                parameterMetadataPath,
                content,
                "parameter",
                log
            )
        } catch (e: RxException) {
            log.e(e.message)
        }
    }

    private fun getParameterValuesObject(parameter: MavParameter<*>): List<Map<String, Any>> {
        val parameterValuesObject: MutableList<Map<String, Any>> = ArrayList()
        for (i in parameter.values.indices) {
            val enumValue = parameter.values[i]
            val parameterSingleValueObject: MutableMap<String, Any> = HashMap()
            parameterSingleValueObject["description"] = enumValue.description
            parameterSingleValueObject["value"] = enumValue.value
            parameterSingleValueObject["lastValue"] = i == parameter.values.size - 1
            parameterValuesObject.add(parameterSingleValueObject)
        }
        return parameterValuesObject
    }

    private fun prepareFolders() {
        val componentMetadataPath = getExternalCacheDirPath(COMPONENT_INFO_FOLDER)
        val componentMetadataFolder = File(componentMetadataPath)
        if (!componentMetadataFolder.exists()) {
            if (!componentMetadataFolder.mkdir()) {
                log.w("Could not create folder '" + componentMetadataFolder.path + "'")
            }
        }
    } //endregion

    companion object {
        const val COMPONENT_INFO_VERSION: Int = 1
        const val COMPONENT_INFO_FOLDER: String = "ComponentMetadata/"

        //region: utils
        @Throws(IOException::class)
        private fun checksumBufferedInputStream(filename: String): Long {
            val `in`: InputStream = BufferedInputStream(FileInputStream(filename))
            val crc = CRC32()

            var c: Int
            while ((`in`.read().also { c = it }) != -1) crc.update(c)
            return crc.value
        }
    }
}