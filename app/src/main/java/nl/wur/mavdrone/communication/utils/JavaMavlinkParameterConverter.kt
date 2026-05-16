package nl.wur.mavdrone.communication.utils

import io.dronefleet.mavlink.common.MavParamExtType
import io.dronefleet.mavlink.common.MavParamType

interface JavaMavlinkParameterConverter<T> {
    fun mavlinkParamType(): MavParamType

    fun mavlinkParamExtType(): MavParamExtType

    fun toJava(paramValue: Float): T

    fun toJava(paramValue: String): T

    fun toMavlink(paramValue: T): Float

    fun toMavlinkExt(paramValue: T): String
}