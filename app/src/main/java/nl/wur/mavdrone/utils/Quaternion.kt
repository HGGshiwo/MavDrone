package nl.wur.mavdrone.utils

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.asin


class Quaternion(
    private var w: Double,
    private var x: Double,
    private var y: Double,
    private var z: Double
) {

    // Convert quaternion to a list of Float values [w, x, y, z]
    fun toList(): List<Float> {
        return listOf(w.toFloat(), x.toFloat(), y.toFloat(), z.toFloat())
    }

    fun toEuler(): Triple<Double, Double, Double> {
        val rollX = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        val pitchY = asin(2.0 * (w * y - z * x))
        val yawZ = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        return Triple(rollX, pitchY, yawZ)
    }

    fun toEulerDegrees(): Triple<Double, Double, Double> {
        val (rollRad, pitchRad, yawRad) = toEuler()

        val rollDeg = rollRad.toDegrees()
        val pitchDeg = pitchRad.toDegrees()
        val yawDeg = yawRad.toDegrees()

        return Triple(rollDeg, pitchDeg, yawDeg)
    }

    override fun toString(): String {
        return "Quaternion(w=$w, x=$x, y=$y, z=$z)"
    }

    companion object {
        fun fromEuler(roll: Double, pitch: Double, yaw: Double): Quaternion {
            val cy = cos(yaw * 0.5)
            val sy = sin(yaw * 0.5)
            val cp = cos(pitch * 0.5)
            val sp = sin(pitch * 0.5)
            val cr = cos(roll * 0.5)
            val sr = sin(roll * 0.5)

            val w = cr * cp * cy + sr * sp * sy
            val x = sr * cp * cy - cr * sp * sy
            val y = cr * sp * cy + sr * cp * sy
            val z = cr * cp * sy - sr * sp * cy

            return Quaternion(w, x, y, z)
        }

        fun fromEulerDegrees(roll: Double, pitch: Double, yaw: Double): Quaternion {
            val rollRad = roll.toRadians()
            val pitchRad = pitch.toRadians()
            val yawRad = yaw.toRadians()

            return fromEuler(rollRad, pitchRad, yawRad)
        }

        fun fromList(quaternionList: List<Float>): Quaternion {
            if (quaternionList.size != 4) {
                throw IllegalArgumentException("Quaternion list must have exactly 4 elements")
            }
            return Quaternion(
                quaternionList[0].toDouble(), quaternionList[1].toDouble(),
                quaternionList[2].toDouble(), quaternionList[3].toDouble()
            )
        }
    }
}