package nl.wur.mavdrone.communication.utils

import java.nio.ByteBuffer
import java.nio.charset.StandardCharsets

import io.dronefleet.mavlink.common.MavParamExtType
import io.dronefleet.mavlink.common.MavParamType

import nl.wur.mavdrone.utils.ByteUtils.Companion.floatFromBytes
import nl.wur.mavdrone.utils.ByteUtils.Companion.floatToBytes
import nl.wur.mavdrone.utils.ByteUtils.Companion.intFromBytes
import nl.wur.mavdrone.utils.ByteUtils.Companion.intToBytes

class ParameterConverter {
    companion object {
                var intConverter: JavaMavlinkParameterConverter<Int> =
            object : JavaMavlinkParameterConverter<Int> {
                override fun mavlinkParamType(): MavParamType {
                    return MavParamType.MAV_PARAM_TYPE_INT32
                }

                override fun mavlinkParamExtType(): MavParamExtType {
                    return MavParamExtType.MAV_PARAM_EXT_TYPE_INT32
                }

                override fun toJava(paramValue: Float): Int {
                    return paramValue.toRawBits()
                }

                override fun toJava(paramValue: String): Int {
                    return intFromBytes(paramValue.toByteArray())
                }

                override fun toMavlink(paramValue: Int): Float {
                    return Float.fromBits(paramValue)
                }

                override fun toMavlinkExt(paramValue: Int): String {
                    return String(intToBytes(paramValue))
                }
            }

                var floatConverter: JavaMavlinkParameterConverter<Float> =
            object : JavaMavlinkParameterConverter<Float> {
                override fun mavlinkParamType(): MavParamType {
                    return MavParamType.MAV_PARAM_TYPE_REAL32
                }

                override fun mavlinkParamExtType(): MavParamExtType {
                    return MavParamExtType.MAV_PARAM_EXT_TYPE_REAL32
                }

                override fun toJava(paramValue: Float): Float {
                    return paramValue
                }

                override fun toJava(paramValue: String): Float {
                    return floatFromBytes(paramValue.toByteArray())
                }

                override fun toMavlink(paramValue: Float): Float {
                    return paramValue
                }

                override fun toMavlinkExt(paramValue: Float): String {
                    return String(floatToBytes(paramValue))
                }
            }

                var doubleConverter: JavaMavlinkParameterConverter<Double> =
            object : JavaMavlinkParameterConverter<Double> {
                override fun mavlinkParamType(): MavParamType {
                    return MavParamType.MAV_PARAM_TYPE_REAL32
                }

                override fun mavlinkParamExtType(): MavParamExtType {
                    return MavParamExtType.MAV_PARAM_EXT_TYPE_REAL32
                }

                override fun toJava(paramValue: Float): Double {
                    return paramValue.toDouble()
                }

                override fun toJava(paramValue: String): Double {
                    return floatFromBytes(paramValue.toByteArray()).toDouble()
                }

                override fun toMavlink(paramValue: Double): Float {
                    return paramValue.toFloat()
                }

                override fun toMavlinkExt(paramValue: Double): String {
                    return String(floatToBytes(paramValue.toFloat()))
                }
            }

                var booleanConverter: JavaMavlinkParameterConverter<Boolean> =
            object : JavaMavlinkParameterConverter<Boolean> {
                override fun mavlinkParamType(): MavParamType {
                    // This value must be INT32 otherwise it's difficult to parse it to a boolean!
                    return MavParamType.MAV_PARAM_TYPE_INT32
                }

                override fun mavlinkParamExtType(): MavParamExtType {
                    // This value must be INT32 otherwise it's difficult to parse it to a boolean!
                    return MavParamExtType.MAV_PARAM_EXT_TYPE_INT32
                }

                override fun toJava(paramValue: Float): Boolean {
                    return (paramValue.toRawBits() >= 1)
                }

                override fun toJava(paramValue: String): Boolean {
                    return ByteBuffer.wrap(paramValue.toByteArray()).getInt() >= 1
                }

                override fun toMavlink(paramValue: Boolean): Float {
                    return if ((paramValue)) Float.fromBits(1) else Float.fromBits(0)
                }

                override fun toMavlinkExt(paramValue: Boolean): String {
                    return String(
                        ByteBuffer.allocate(4).putInt(if (paramValue) 1 else 0).array(),
                        StandardCharsets.ISO_8859_1
                    )
                }
            }
    }
}
