package nl.wur.mavdrone.communication.components.impl

import java.lang.ref.WeakReference
import java.math.BigInteger
import kotlin.math.ceil
import kotlin.math.min

import dji.v5.manager.aircraft.perception.PerceptionManager
import dji.v5.manager.aircraft.perception.data.ObstacleData
import dji.v5.manager.aircraft.perception.listener.ObstacleDataListener
import io.dronefleet.mavlink.common.MavDistanceSensor
import io.dronefleet.mavlink.common.MavFrame
import io.dronefleet.mavlink.common.ObstacleDistance
import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.BaseComponent
import nl.wur.mavdrone.communication.components.MavBaseComponent

class Radar(connectionWR: WeakReference<MavConnection>,
    private val componentWR: WeakReference<out MavBaseComponent>
) : BaseComponent(connectionWR, "MavRadar"), ObstacleDataListener {
    private var obstacleData: ObstacleData? = null

    init {
        PerceptionManager.getInstance().radarManager.addObstacleDataListener(this)
    }

    override fun create() {
        componentWR.get()?.apply {
            addMavlinkMessageSender(ObstacleDistance::class.java, this@Radar::toMavlinkObstacleDistanceMessage, 500, true)
        }
    }

    override fun destroy() {
        PerceptionManager.getInstance().radarManager.removeObstacleDataListener(this)

        super.destroy()
    }

    override fun onUpdate(obstacleData: ObstacleData) {
        this.obstacleData = obstacleData
    }

    private fun toMavlinkObstacleDistanceMessage(): ObstacleDistance? {
        obstacleData?.let {
            // Mavlink only holds 72 data points so reduce if needed
            val distancesArray = obstacleData!!.horizontalObstacleDistance.stream()
                .mapToInt { d: Int? -> d!! }
                .toArray()

            val distancesList: MutableList<Int> = ArrayList()

            if (distancesArray.size > 72) {
                val collapseSize = ceil(distancesArray.size / 72.0).toInt()
                var i = 0
                while (i < distancesArray.size) {
                    var distanceSum = 0
                    val endIndex = min((i + collapseSize).toDouble(), distancesArray.size.toDouble())
                        .toInt()
                    for (j in i until endIndex) {
                        distanceSum += distancesArray[j]
                    }
                    distancesList.add(distanceSum / collapseSize / 10)
                    i += collapseSize
                }
            } else {
                distancesList.addAll(distancesArray.map { it / 10 })
            }
            return ObstacleDistance.builder()
                .timeUsec(BigInteger.valueOf((System.currentTimeMillis() * 1000L)))
                .sensorType(MavDistanceSensor.MAV_DISTANCE_SENSOR_RADAR)
                .distances(distancesList)
                .increment(360 / distancesList.size)
                .minDistance(70)
                .maxDistance(4000)
                .angleOffset(0f)
                .frame(MavFrame.MAV_FRAME_BODY_FRD)
                .build()
        }

        return null
    }
}
