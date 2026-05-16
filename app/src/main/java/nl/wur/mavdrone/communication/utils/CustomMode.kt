package nl.wur.mavdrone.communication.utils

import kotlin.jvm.Throws

// Source: https://github.com/mavlink/MAVSDK/blob/main/src/mavsdk/core/px4_custom_mode.h
class CustomMode {
    enum class MAIN_MODE(val value: Int) {
        INVALID(0),
        PX4_CUSTOM_MAIN_MODE_MANUAL(1),
        PX4_CUSTOM_MAIN_MODE_ALTCTL(2),
        PX4_CUSTOM_MAIN_MODE_POSCTL(3),
        PX4_CUSTOM_MAIN_MODE_AUTO(4),
        PX4_CUSTOM_MAIN_MODE_ACRO(5),
        PX4_CUSTOM_MAIN_MODE_OFFBOARD(6),
        PX4_CUSTOM_MAIN_MODE_STABILIZED(7),
        PX4_CUSTOM_MAIN_MODE_RATTITUDE(8);

        companion object {
            @Throws(IllegalArgumentException::class)
            fun fromValue(value: Int): MAIN_MODE {
                values().forEach {
                    if (it.value == value) {
                        return it
                    }
                }

                throw IllegalArgumentException("No enum constant with value $value")
            }
        }
    }

    enum class SUB_MODE(val value: Int) {
        INVALID(0),
        PX4_CUSTOM_SUB_MODE_AUTO_READY(1),
        PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF(2),
        PX4_CUSTOM_SUB_MODE_AUTO_LOITER(3),
        PX4_CUSTOM_SUB_MODE_AUTO_MISSION(4),
        PX4_CUSTOM_SUB_MODE_AUTO_RTL(5),
        PX4_CUSTOM_SUB_MODE_AUTO_LAND(6),
        PX4_CUSTOM_SUB_MODE_AUTO_RTGS(7),
        PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET(8);

        companion object {
            @Throws(IllegalArgumentException::class)
            fun fromValue(value: Int): SUB_MODE {
                values().forEach {
                    if (it.value == value) {
                        return it
                    }
                }

                throw IllegalArgumentException("No enum constant with value $value")
            }
        }
    }
}
