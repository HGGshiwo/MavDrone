package nl.wur.mavdrone.communication.utils

import io.dronefleet.mavlink.util.EnumValue
import io.dronefleet.mavlink.util.reflection.MavlinkReflection

class EnumUtils {
    companion object {
        fun <T : Enum<*>?> listToEnum(flagList: List<T>): EnumValue<T> {
            var bitmask = 0
            flagList.forEach {
                bitmask = bitmask or MavlinkReflection.getEnumValue(it) // Assuming getEnumValue returns an int
            }
            return EnumValue.create(bitmask)
        }

        fun <T : Enum<T>?> enumToList(bitmask: Int, enumClass: Class<T>): List<T> {
            val flagList: MutableList<T> = ArrayList()

            enumClass.enumConstants?.forEach {
                val flagValue = MavlinkReflection.getEnumValue(it)
                if ((bitmask and flagValue) == flagValue) {
                    flagList.add(it)
                }
            }

            return flagList
        }

        fun <T : Enum<*>?> enumToList(enumValue: EnumValue<T>, enumClass: Class<T>): List<T> {
            val flagList: MutableList<T> = ArrayList()
            enumClass.enumConstants?.filter { enumValue.flagsEnabled(it) }?.let { flagList.addAll(it) }
            return flagList
        }

        fun normalizeMavlinkEnumName(value: Enum<*>): String {
            var description = StringBuilder(value.name)
            val parts = listOf(
                *description.toString().split("_".toRegex()).dropLastWhile { it.isEmpty() }
                    .toTypedArray())
            if (parts.size <= 2) {
                return description.toString()
            }

            description = StringBuilder()
            for (i in 2 until parts.size) {
                description.append(parts[i])
            }

            description.append(" (").append(EnumValue.of(value).value()).append(")")
            return description.toString()
        }
    }
}
