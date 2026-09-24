package nl.wur.mavdrone.communication.utils

import androidx.annotation.Size
import java.lang.ref.WeakReference
import java.util.concurrent.TimeUnit

import dji.sdk.keyvalue.key.DJIKey

import io.reactivex.rxjava3.core.Single
import io.reactivex.rxjava3.core.SingleEmitter

import io.dronefleet.mavlink.annotations.MavlinkEntryInfo
import io.dronefleet.mavlink.common.MavParamExtType
import io.dronefleet.mavlink.common.MavParamType
import io.dronefleet.mavlink.common.ParamAck
import io.dronefleet.mavlink.common.ParamExtAck
import io.dronefleet.mavlink.common.ParamExtSet
import io.dronefleet.mavlink.common.ParamExtValue
import io.dronefleet.mavlink.common.ParamSet
import io.dronefleet.mavlink.common.ParamValue
import io.dronefleet.mavlink.util.EnumValue

import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.BaseComponent
import nl.wur.mavdrone.communication.components.MavBaseComponent
import nl.wur.mavdrone.communication.utils.EnumUtils.Companion.normalizeMavlinkEnumName
import nl.wur.mavdrone.utils.ByteUtils.Companion.intFromBytes
import nl.wur.mavdrone.utils.ByteUtils.Companion.intToBytes
import nl.wur.mavdrone.utils.GenericPreferenceManager
import nl.wur.mavdrone.utils.RXUtils.Companion.djiGetValue
import nl.wur.mavdrone.utils.RXUtils.Companion.djiSetValue

class MavParameter<T : Any>(
    connectionWR: WeakReference<MavConnection>,
    private val componentWR: WeakReference<out MavBaseComponent>,
    @Size(max = 15) val name: String,
    private val javaMavlinkParameterConverter: JavaMavlinkParameterConverter<T>,
    private val parameterChangedCallback: ParameterChangedCallback<T>?,
    private val setParameterInterface: SetParameterInterface<T>,
    val isExtParameter: Boolean
) : BaseComponent(connectionWR, "MavParameter$name") {
    val index: Int

    var defaultValue: T? = null
    var maxValue: T? = null
    var minValue: T? = null
    var type: String = ""

    // Enum specific constants
    var values: MutableList<MavlinkParameterEnum> = ArrayList()
    var defaultIntValue: Int = 0

    // Metadata
    var category: String = ""
    var group: String = ""
    var longDesc: String = ""
    var shortDesc: String = ""
    var units: String = ""

    init {
        this.index = this.totalNumberOfParameters

        setParameterInterface.init(connectionWR, componentWR)

        if (isExtParameter) {
            this.type = parameterTypeToString(javaMavlinkParameterConverter.mavlinkParamExtType())
        } else {
            this.type = parameterTypeToString(javaMavlinkParameterConverter.mavlinkParamType())
        }
    }

    fun setMetadata(category: String, group: String, shortDesc: String, longDesc: String, units: String) {
        this.category = category
        this.group = group
        this.shortDesc = shortDesc
        this.longDesc = longDesc
        this.units = units
    }

    fun sendMavlinkParamValue() {
        // DJI key reads can hang or fail on some models; always answer with at least
        // the default value so the GCS parameter fetch completes.
        val valueSingle = setParameterInterface.value
            .timeout(PARAM_READ_TIMEOUT_MS.toLong(), TimeUnit.MILLISECONDS, Single.just(defaultValue!!))
            .onErrorResumeNext { Single.just(defaultValue!!) }

        if (isExtParameter) {
            compositeDisposable.add(
                valueSingle.subscribe({ value: T ->
                    val paramType = javaMavlinkParameterConverter.mavlinkParamExtType()
                    val mavlinkValue = javaMavlinkParameterConverter.toMavlinkExt(value)

                    val paramExtValue = ParamExtValue.builder()
                        .paramId(name)
                        .paramValue(mavlinkValue)
                        .paramType(paramType)
                        .paramIndex(index)
                        .paramCount(totalNumberOfParameters)
                        .build()
                    sendMessage(paramExtValue)
                }, { error -> log.w(error.message ?: "Cannot send value for parameter $name") })
            )
        } else {
            compositeDisposable.add(
                valueSingle.subscribe({ value: T ->
                    val paramType = javaMavlinkParameterConverter.mavlinkParamType()
                    val mavlinkValue = javaMavlinkParameterConverter.toMavlink(value)

                    val paramValue = ParamValue.builder()
                        .paramId(name)
                        .paramValue(mavlinkValue)
                        .paramType(paramType)
                        .paramIndex(index)
                        .paramCount(totalNumberOfParameters)
                        .build()
                    sendMessage(paramValue)
                }, { error -> log.w(error.message ?: "Cannot send value for parameter $name") })
            )
        }
    }

    fun updateFromMavlinkParamSetValue(msg: ParamSet) {
        if (msg.paramType().entry() != javaMavlinkParameterConverter.mavlinkParamType()) {
            log.e(
                "Could not update param $name because wrong param type is received (received: ${
                    msg.paramType().entry().name
                }, expected ${javaMavlinkParameterConverter.mavlinkParamType()})!"
            )
            return
        }

        val newValue = javaMavlinkParameterConverter.toJava(msg.paramValue())
        compositeDisposable.add(
            setParameterInterface.setValue(newValue)
                .subscribe({ value: T ->
                    sendMavlinkParamValue()
                    parameterChangedCallback?.parameterChanged(value)
                    connectionWR.get()!!.showToast("Updated param $name", 2)
                }, { error ->
                    log.w("Cannot update parameter value $name: ${error.message}")
                    // Respond with the unchanged value so the GCS does not time out
                    // waiting for the ack of a rejected set.
                    sendMavlinkParamValue()
                })
        )
    }

    fun updateFromMavlinkParamExtSetValue(msg: ParamExtSet) {
        if (msg.paramType().entry() != javaMavlinkParameterConverter.mavlinkParamExtType()) {
            log.e(
                "Could not update param $name because wrong param type is received (received: ${
                    msg.paramType().entry().name
                }, expected ${javaMavlinkParameterConverter.mavlinkParamExtType()})!"
            )

            val progressMessage = ParamExtAck.builder()
                .paramId(name)
                .paramValue(msg.paramValue())
                .paramType(javaMavlinkParameterConverter.mavlinkParamExtType())
                .paramResult(ParamAck.PARAM_ACK_VALUE_UNSUPPORTED)
                .build()
            sendMessage(progressMessage)
            return
        }

        val progressMessage = ParamExtAck.builder()
            .paramId(name)
            .paramValue(msg.paramValue())
            .paramType(javaMavlinkParameterConverter.mavlinkParamExtType())
            .paramResult(ParamAck.PARAM_ACK_IN_PROGRESS)
            .build()
        sendMessage(progressMessage)

        val newValue = javaMavlinkParameterConverter.toJava(msg.paramValue())

        compositeDisposable.add(
            setParameterInterface.setValue(newValue)
                .subscribe({ value: T ->
                    val ackMessage = ParamExtAck.builder()
                        .paramId(name)
                        .paramValue(msg.paramValue())
                        .paramType(javaMavlinkParameterConverter.mavlinkParamExtType())
                        .paramResult(ParamAck.PARAM_ACK_ACCEPTED)
                        .build()
                    sendMessage(ackMessage)

                    parameterChangedCallback?.parameterChanged(value)

                    connectionWR.get()?.showToast("Updated param $name", 2)
                    log.i("Updated param $name to $newValue")
                }, { error ->
                    log.w(error.message ?: "Cannot update parameter value $name")
                    // TODO: ignored paramValue here, not sure if this works
                    val failedMessage = ParamExtAck.builder()
                        .paramId(name)
                        .paramType(javaMavlinkParameterConverter.mavlinkParamExtType())
                        .paramResult(ParamAck.PARAM_ACK_FAILED)
                        .build()
                    sendMessage(failedMessage)
                })
        )
    }

    protected val totalNumberOfParameters: Int
        get() {
            return componentWR.get()?.mavlinkParameters?.values?.count { it.isExtParameter == isExtParameter } ?: 0
        }

    private fun sendMessage(msg: Any) {
        connectionWR.get()!!.sendMavlinkMessage(MavMessageWithComponentID(componentWR.get()!!.componentID, msg))
    }

    //endregion
    //region: utils
    private fun parameterTypeToString(paramType: MavParamType): String {
        when (paramType) {
            MavParamType.MAV_PARAM_TYPE_INT8 -> return "Int8"
            MavParamType.MAV_PARAM_TYPE_INT16 -> return "Int16"
            MavParamType.MAV_PARAM_TYPE_INT32 -> return "Int32"
            MavParamType.MAV_PARAM_TYPE_INT64 -> return "Int64"
            MavParamType.MAV_PARAM_TYPE_UINT8 -> return "Uint8"
            MavParamType.MAV_PARAM_TYPE_UINT16 -> return "Uint16"
            MavParamType.MAV_PARAM_TYPE_UINT32 -> return "Uint32"
            MavParamType.MAV_PARAM_TYPE_UINT64 -> return "Uint64"
            MavParamType.MAV_PARAM_TYPE_REAL32 -> return "Float"
            MavParamType.MAV_PARAM_TYPE_REAL64 -> return "Double"
            else -> {
                log.w("Cannot set parameter type for $paramType")
                return ""
            }
        }
    }

    private fun parameterTypeToString(paramExtType: MavParamExtType): String {
        when (paramExtType) {
            MavParamExtType.MAV_PARAM_EXT_TYPE_UINT8 -> return "Uint8"
            MavParamExtType.MAV_PARAM_EXT_TYPE_INT8 -> return "Int8"
            MavParamExtType.MAV_PARAM_EXT_TYPE_UINT16 -> return "Uint16"
            MavParamExtType.MAV_PARAM_EXT_TYPE_INT16 -> return "Int16"
            MavParamExtType.MAV_PARAM_EXT_TYPE_UINT32 -> return "Uint32"
            MavParamExtType.MAV_PARAM_EXT_TYPE_INT32 -> return "Int32"
            MavParamExtType.MAV_PARAM_EXT_TYPE_UINT64 -> return "Uint64"
            MavParamExtType.MAV_PARAM_EXT_TYPE_INT64 -> return "Int64"
            MavParamExtType.MAV_PARAM_EXT_TYPE_REAL32 -> return "Float"
            MavParamExtType.MAV_PARAM_EXT_TYPE_REAL64 -> return "Double"
            MavParamExtType.MAV_PARAM_EXT_TYPE_CUSTOM -> return "Custom"
            else -> {
                log.w("Cannot set parameter type for $paramExtType")
                return ""
            }
        }
    }

    class MavlinkParameterEnum(var description: String, var value: Int)

    interface SetParameterInterface<T : Any> {
        fun init(connectionWR: WeakReference<MavConnection>, componentWR: WeakReference<out MavBaseComponent>)
        val value: Single<T>
        fun setValue(value: T): Single<T>
    }

    private class SetDJIParameters<T : Any>(private val djiKey: DJIKey<T>) : SetParameterInterface<T> {
        override fun init(connectionWR: WeakReference<MavConnection>, componentWR: WeakReference<out MavBaseComponent>) {
        }

        override val value: Single<T>
            get() = djiGetValue(djiKey)

        override fun setValue(value: T): Single<T> {
            return djiSetValue(djiKey, value)
                .andThen(Single.just(value))
        }
    } //endregion

    companion object {
        private const val PARAM_READ_TIMEOUT_MS = 300
        //region: class methods
        fun fromDJIParameter(
            connectionWR: WeakReference<MavConnection>,
            componentWR: WeakReference<out MavBaseComponent>,
            @Size(max = 15) name: String,
            key: DJIKey<Boolean>,
            defaultValue: Boolean,
            extended: Boolean,
            parameterChangedCallback: ParameterChangedCallback<Boolean>?
        ): MavParameter<Boolean> {
            val setDJIParameters = SetDJIParameters(key)

            val mavParameter = MavParameter(
                connectionWR,
                componentWR,
                name,
                ParameterConverter.booleanConverter,
                parameterChangedCallback,
                setDJIParameters,
                extended
            )
            mavParameter.defaultValue = defaultValue

            mavParameter.values.add(MavlinkParameterEnum("False", 0))
            mavParameter.values.add(MavlinkParameterEnum("True", 1))
            mavParameter.defaultIntValue = if ((defaultValue)) 1 else 0

            return mavParameter
        }

        fun <T : Any> fromDJIParameter(
            connectionWR: WeakReference<MavConnection>,
            componentWR: WeakReference<out MavBaseComponent>,
            @Size(max = 15) name: String,
            key: DJIKey<T>,
            defaultValue: T,
            max: T,
            min: T,
            javaMavlinkParameterConverter: JavaMavlinkParameterConverter<T>,
            extended: Boolean,
            parameterChangedCallback: ParameterChangedCallback<T>?
        ): MavParameter<T> {
            val setDJIParameters: SetParameterInterface<T> = SetDJIParameters(key)

            val mavParameter = MavParameter(
                connectionWR,
                componentWR,
                name,
                javaMavlinkParameterConverter,
                parameterChangedCallback,
                setDJIParameters,
                extended
            )
            mavParameter.defaultValue = defaultValue
            mavParameter.maxValue = max
            mavParameter.minValue = min
            return mavParameter
        }

        // DJI parameter with ENUM value
        fun <T : Enum<*>> fromDJIParameter(
            connectionWR: WeakReference<MavConnection>,
            componentWR: WeakReference<out MavBaseComponent>,
            @Size(max = 15) name: String,
            key: DJIKey<T>,
            defaultValue: T,
            enumType: Class<T>,
            extended: Boolean,
            parameterChangedCallback: ParameterChangedCallback<T>?
        ): MavParameter<T> {
            val allEnumFields: MutableList<T> = ArrayList()
            val enumArray = enumType.enumConstants
            enumArray?.let {
                it.sortBy { obj -> obj.ordinal }
                allEnumFields.addAll(it)
            }

            val setDJIParameters: SetParameterInterface<T> = SetDJIParameters(key)

            val converter: JavaMavlinkParameterConverter<T> = object : JavaMavlinkParameterConverter<T> {
                override fun mavlinkParamType(): MavParamType {
                    return MavParamType.MAV_PARAM_TYPE_INT32
                }

                override fun mavlinkParamExtType(): MavParamExtType {
                    return MavParamExtType.MAV_PARAM_EXT_TYPE_INT32
                }

                override fun toJava(paramValue: Float): T {
                    return allEnumFields[paramValue.toRawBits()]
                }

                override fun toJava(paramValue: String): T {
                    return allEnumFields[intFromBytes(paramValue.toByteArray())]
                }

                override fun toMavlink(paramValue: T): Float {
                    return Float.fromBits(allEnumFields.indexOf(paramValue))
                }

                override fun toMavlinkExt(paramValue: T): String {
                    return String(intToBytes(allEnumFields.indexOf(paramValue)))
                }
            }

            val mavParameter = MavParameter(
                connectionWR,
                componentWR,
                name,
                converter,
                parameterChangedCallback,
                setDJIParameters,
                extended
            )

            mavParameter.defaultValue = defaultValue

            allEnumFields.forEachIndexed { i, v ->
                mavParameter.values.add(MavlinkParameterEnum(v.name, i))
            }
            mavParameter.defaultIntValue = allEnumFields.indexOf(defaultValue)

            return mavParameter
        }

        fun <T : Any> fromAndroidParameter(
            connectionWR: WeakReference<MavConnection>,
            componentWR: WeakReference<out MavBaseComponent>,
            @Size(max = 15) name: String,
            defaultValue: T,
            min: T,
            max: T,
            javaMavlinkParameterConverter: JavaMavlinkParameterConverter<T>,
            extended: Boolean,
            parameterChangedCallback: ParameterChangedCallback<T>?
        ): MavParameter<T> {
            val setAndroidParameters: SetParameterInterface<T> = object : SetParameterInterface<T> {
                private var genericPreferenceManager: GenericPreferenceManager<T>? = null

                override fun init(connectionWR: WeakReference<MavConnection>, componentWR: WeakReference<out MavBaseComponent>) {
                    genericPreferenceManager = GenericPreferenceManager(connectionWR.get()!!.androidPreferences)
                }

                override val value: Single<T>
                    get() {
                        val value = genericPreferenceManager!!.get(name, defaultValue)
                        return Single.just(value)
                    }

                override fun setValue(value: T): Single<T> {
                    genericPreferenceManager?.set(name, value)
                    return Single.just(value)
                }
            }

            val mavParameter = MavParameter(
                connectionWR,
                componentWR,
                name,
                javaMavlinkParameterConverter,
                parameterChangedCallback,
                setAndroidParameters,
                extended
            )
            mavParameter.defaultValue = defaultValue
            mavParameter.maxValue = max
            mavParameter.minValue = min
            return mavParameter
        }

        fun <T : Enum<*>> fromAndroidParameter(
            connectionWR: WeakReference<MavConnection>,
            componentWR: WeakReference<out MavBaseComponent>,
            @Size(max = 15) name: String,
            defaultValue: T,
            enumType: Class<T>,
            extended: Boolean,
            parameterChangedCallback: ParameterChangedCallback<T>?
        ): MavParameter<T> {
            val allEnumFields: MutableList<T> = ArrayList()
            val enumFields = enumType.fields
            for (enumField in enumFields) {
                if (!enumField.isEnumConstant) {
                    continue
                }

                val mavlinkEntryInfo = enumField.getAnnotation(MavlinkEntryInfo::class.java) ?: continue

                val enumValue = EnumValue.create(enumType, mavlinkEntryInfo.value)
                allEnumFields.add(enumValue.entry())
            }

            // Sort by value
            allEnumFields.sortBy { EnumValue.of(it).value() }

            val setAndroidParameters: SetParameterInterface<T> = object : SetParameterInterface<T> {
                private var genericPreferenceManager: GenericPreferenceManager<Int>? = null
                var defaultIntValue: Int = 0

                override fun init(connectionWR: WeakReference<MavConnection>, componentWR: WeakReference<out MavBaseComponent>) {
                    genericPreferenceManager = GenericPreferenceManager(connectionWR.get()!!.androidPreferences)
                    defaultIntValue = EnumValue.of(defaultValue).value()
                }

                override val value: Single<T>
                    get() = Single.create { subscriber: SingleEmitter<T> ->
                        val value = genericPreferenceManager!!.get(name, defaultIntValue)
                        subscriber.onSuccess(EnumValue.create(enumType, value).entry())
                    }

                override fun setValue(value: T): Single<T> {
                    return Single.create { subscriber: SingleEmitter<T> ->
                        genericPreferenceManager!!.set(name, EnumValue.of(value).value())
                        subscriber.onSuccess(value)
                    }
                }
            }

            val converter: JavaMavlinkParameterConverter<T> = object : JavaMavlinkParameterConverter<T> {
                override fun mavlinkParamType(): MavParamType {
                    return MavParamType.MAV_PARAM_TYPE_INT32
                }

                override fun mavlinkParamExtType(): MavParamExtType {
                    return MavParamExtType.MAV_PARAM_EXT_TYPE_UINT8
                }

                override fun toJava(paramValue: Float): T {
                    return allEnumFields[paramValue.toRawBits()]
                }

                override fun toJava(paramValue: String): T {
                    return allEnumFields[intFromBytes(paramValue.toByteArray())]
                }

                override fun toMavlink(paramValue: T): Float {
                    return Float.fromBits(allEnumFields.indexOf(paramValue))
                }

                override fun toMavlinkExt(paramValue: T): String {
                    return String(intToBytes(allEnumFields.indexOf(paramValue)))
                }
            }

            val mavParameter = MavParameter(
                connectionWR,
                componentWR,
                name,
                converter,
                parameterChangedCallback,
                setAndroidParameters,
                extended
            )
            mavParameter.defaultValue = defaultValue

            allEnumFields.forEach {
                val description = normalizeMavlinkEnumName(it)
                mavParameter.values.add(MavlinkParameterEnum(description, EnumValue.of(it).value()))
            }
            mavParameter.defaultIntValue = EnumValue.of(defaultValue).value()

            return mavParameter
        }

        fun fromAndroidParameter(
            connectionWR: WeakReference<MavConnection>,
            componentWR: WeakReference<out MavBaseComponent>,
            @Size(max = 15) name: String,
            defaultValue: Boolean,
            extended: Boolean,
            parameterChangedCallback: ParameterChangedCallback<Boolean>?
        ): MavParameter<Boolean> {
            val setAndroidParameters: SetParameterInterface<Boolean> = object : SetParameterInterface<Boolean> {
                private var genericPreferenceManager: GenericPreferenceManager<Boolean>? = null

                override fun init(connectionWR: WeakReference<MavConnection>, componentWR: WeakReference<out MavBaseComponent>) {
                    genericPreferenceManager = GenericPreferenceManager(connectionWR.get()!!.androidPreferences)
                }

                override val value: Single<Boolean>
                    get() {
                        val value = genericPreferenceManager!!.get(name, defaultValue)
                        return Single.just(value)
                    }

                override fun setValue(value: Boolean): Single<Boolean> {
                    genericPreferenceManager!!.set(name, value)
                    return Single.just(value)
                }
            }

            val mavParameter = MavParameter(
                connectionWR,
                componentWR,
                name,
                ParameterConverter.booleanConverter,
                parameterChangedCallback,
                setAndroidParameters,
                extended
            )
            mavParameter.defaultValue = defaultValue

            mavParameter.values.add(MavlinkParameterEnum("False", 0))
            mavParameter.values.add(MavlinkParameterEnum("True", 1))
            mavParameter.defaultIntValue = if ((defaultValue)) 1 else 0

            return mavParameter
        }
    }
}
