package nl.wur.mavdrone.utils

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class CoordinateUtils {
    companion object {
        fun getDistanceInMeterFromTwoGPSLocations(
            firstLat: Double,
            firstLong: Double,
            secondLat: Double,
            secondLong: Double
        ): Double {
            // Source: https://sdk-forum.dji.net/hc/en-us/articles/12967095303961-How-to-calculate-the-distance-between-two-coordinates-
            // Radius of the Earth in m
            val R = 6371000.0

            val dlon = (secondLong - firstLong).toRadians()
            val dlat = (secondLat - firstLat).toRadians()
            val a = sin(dlat / 2.0) * sin(dlat / 2.0) + cos(firstLat).toRadians() * cos(secondLat.toRadians()) * sin(dlon / 2.0) * sin(dlon / 2.0)
            val c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a))
            return R * c
        }

        fun getAngleBetweenTwoGPSLocations(
            firstLat: Double,
            firstLong: Double,
            secondLat: Double,
            secondLong: Double
        ): Double {
            val lat1: Double = firstLat.toRadians()
            val lon1: Double = firstLong.toRadians()
            val lat2: Double = secondLat.toRadians()
            val lon2: Double = secondLong.toRadians()

            val dLon = lon2 - lon1
            val y = sin(dLon) * cos(lat2)
            val x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon)
            val angle = atan2(y, x)
            return ((angle.toDegrees() + 180) % 360 - 180)  // [-180, 180]
        }

    }
}
