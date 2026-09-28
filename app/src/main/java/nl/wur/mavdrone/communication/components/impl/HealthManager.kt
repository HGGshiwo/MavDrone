package nl.wur.mavdrone.communication.components.impl

import java.lang.ref.WeakReference
import java.util.Locale
import java.util.concurrent.TimeUnit
import java.util.function.Consumer

import dji.v5.common.callback.CommonCallbacks.CompletionCallbackWithParam
import dji.v5.manager.aircraft.perception.PerceptionManager
import dji.v5.manager.aircraft.perception.data.ObstacleAvoidanceType
import dji.v5.manager.diagnostic.DJIDeviceHealthInfo
import dji.v5.manager.diagnostic.DJIDeviceHealthInfoChangeListener
import dji.v5.manager.diagnostic.DeviceHealthManager

import io.reactivex.rxjava3.android.schedulers.AndroidSchedulers
import io.reactivex.rxjava3.core.Observable
import io.reactivex.rxjava3.schedulers.Schedulers

import io.dronefleet.mavlink.common.MavSysStatusSensor
import io.dronefleet.mavlink.util.EnumValue

import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.BaseComponent
import nl.wur.mavdrone.communication.utils.EnumUtils.Companion.listToEnum
import nl.wur.mavdrone.utils.RXUtils
import nl.wur.mavdrone.utils.RXUtils.Companion.djiGetValue

class HealthManager(
    connectionWR: WeakReference<MavConnection>,
    private val missionManagerWR: WeakReference<MissionManager>,
    private val rtkWR: WeakReference<RTK>
) : BaseComponent(connectionWR, "MavHealthManager"), DJIDeviceHealthInfoChangeListener {
    var healthySensors: EnumValue<MavSysStatusSensor?> = AVAILABLE_SENSORS
        private set
    var enabledSensors: EnumValue<MavSysStatusSensor?> = AVAILABLE_SENSORS
        private set

    private var prevHealthySensorsList: List<MavSysStatusSensor?> = ArrayList(ALL_SENSORS)
    private val enabledSensorsList: MutableList<MavSysStatusSensor?> = ArrayList(ALL_SENSORS)
    private val keywordMap: MutableMap<String, MavSysStatusSensor> = LinkedHashMap()

    private var lastUpdateTime = System.currentTimeMillis()

    init {
        DeviceHealthManager.getInstance().addDJIDeviceHealthInfoChangeListener(this)

        // Define keyword map, when a key is in the DJI error description, the corresponding value status bit will be
        // removed from the list
        keywordMap["rtk"] = MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_GPS
        keywordMap["interference"] = MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_GPS
        keywordMap["compass"] = MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_3D_GYRO
        keywordMap["remote controller"] = MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_RC_RECEIVER
        keywordMap["battery"] = MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_BATTERY
        keywordMap["obstacle"] = MavSysStatusSensor.MAV_SYS_STATUS_OBSTACLE_AVOIDANCE

        // Check every 5 seconds whether some sensors are active
        compositeDisposable.add(
            Observable.interval(5, TimeUnit.SECONDS)
                .observeOn(Schedulers.newThread())
                .subscribe { t: Long -> this.updateEnabledSensors(t) }
        )
    }

    override fun destroy() {
        DeviceHealthManager.getInstance().removeDJIDeviceHealthInfoChangeListener(this)

        super.destroy()
    }

    override fun onDeviceHealthInfoUpdate(infoList: List<DJIDeviceHealthInfo>) {
        if (lastUpdateTime + MIN_UPDATE_TIME > System.currentTimeMillis()) {
            return
        }

        // TODO: update this method when better way is available: https://github.com/dji-sdk/Mobile-SDK-Android-V5/issues/61
        // All messages are probably defined in: https://github.com/dji-sdk/Mobile-SDK-Android/blob/master/hms.json
        val healthySensorsList: MutableList<MavSysStatusSensor?> = ArrayList(ALL_SENSORS)
        for (info in infoList) {
            val description = info.description().lowercase(Locale.getDefault())
            for (key in keywordMap.keys) {
                if (description.contains(key)) {
                    healthySensorsList.remove(keywordMap[key])
                    break
                }
            }
        }

        // Set Geofence unhealthy when a fence is loaded but not activated
        missionManagerWR.get()?.let {
            if (it.currentGeofenceMission != null && !it.geofenceManager.isEnabled) {
                healthySensorsList.remove(MavSysStatusSensor.MAV_SYS_STATUS_GEOFENCE)
            }
        }

        // Set GPS unhealthy when RTK is enabled but not healthy
        rtkWR.get()?.let {
            if (it.isRtkEnabled && !it.isRtkHealthy) {
                healthySensorsList.remove(MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_GPS)
            }
        }

        if (prevHealthySensorsList != healthySensorsList) {
            healthySensors = listToEnum(healthySensorsList)
            prevHealthySensorsList = healthySensorsList
        }

        lastUpdateTime = System.currentTimeMillis()
    }

    fun getUnhealthySensors(): List<MavSysStatusSensor> {
        return ALL_SENSORS.filterIsInstance<MavSysStatusSensor>().filter { !healthySensors.flagsEnabled(it) }
    }

    // GCS implementations (PX4-style) gate prearm checks on the MAV_SYS_STATUS_PREARM_CHECK
    // bit (0x10000000) of SYS_STATUS.onboard_control_sensors_health, so it must reflect the
    // live prearm result rather than only the DJI device health.
    fun getHealthFlags(prearmCheckPassed: Boolean): EnumValue<MavSysStatusSensor?> {
        val healthyValue = healthySensors.value()
        val prearmBit = EnumValue.create(MavSysStatusSensor.MAV_SYS_STATUS_PREARM_CHECK).value()
        return EnumValue.create(
            if (prearmCheckPassed) healthyValue or prearmBit else healthyValue and prearmBit.inv()
        )
    }

    private fun updateEnabledSensors(t: Long) {
        // Check whether obstacle avoidance is enabled
        compositeDisposable.add(
            djiGetValue(Consumer { callback: CompletionCallbackWithParam<ObstacleAvoidanceType> ->
                PerceptionManager.getInstance().getObstacleAvoidanceType(callback)
            })
                .observeOn(AndroidSchedulers.mainThread())
                .subscribe(
                    { value: ObstacleAvoidanceType ->
                        obstacleAvoidanceQueryFailed = false
                        var enabled = false
                        when (value) {
                            ObstacleAvoidanceType.BRAKE -> enabled = true
                            ObstacleAvoidanceType.BYPASS -> enabled = true
                            else -> {}
                        }
                        if (enabled && !enabledSensorsList.contains(MavSysStatusSensor.MAV_SYS_STATUS_OBSTACLE_AVOIDANCE)) {
                            enabledSensorsList.add(MavSysStatusSensor.MAV_SYS_STATUS_OBSTACLE_AVOIDANCE)
                            enabledSensors = listToEnum(enabledSensorsList)
                        }
                        if (!enabled && enabledSensorsList.contains(MavSysStatusSensor.MAV_SYS_STATUS_OBSTACLE_AVOIDANCE)) {
                            enabledSensorsList.remove(MavSysStatusSensor.MAV_SYS_STATUS_OBSTACLE_AVOIDANCE)
                            enabledSensors = listToEnum(enabledSensorsList)
                        }
                    },
                    { error ->
                        // This query repeats every 5 s; only report the first failure until it succeeds again
                        if (!obstacleAvoidanceQueryFailed) {
                            obstacleAvoidanceQueryFailed = true
                            RXUtils.handleError(error)
                        }
                    })
        )

        // Check if geofence is enabled (e.g. has a fence loaded)
        missionManagerWR.get()?.let {
            val geofenceEnabled = it.currentGeofenceMission != null

            if (geofenceEnabled && !enabledSensorsList.contains(MavSysStatusSensor.MAV_SYS_STATUS_GEOFENCE)) {
                enabledSensorsList.add(MavSysStatusSensor.MAV_SYS_STATUS_GEOFENCE)
                enabledSensors = listToEnum(enabledSensorsList)
            } else if (!geofenceEnabled && enabledSensorsList.contains(MavSysStatusSensor.MAV_SYS_STATUS_GEOFENCE)) {
                enabledSensorsList.remove(MavSysStatusSensor.MAV_SYS_STATUS_GEOFENCE)
                enabledSensors = listToEnum(enabledSensorsList)
            }
        }
    }

    companion object {
        private const val MIN_UPDATE_TIME = 2000
        private var obstacleAvoidanceQueryFailed = false
        private val ALL_SENSORS: List<MavSysStatusSensor?> = listOf(
            MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_3D_GYRO,
            MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_GPS,
            MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_YAW_POSITION,
            MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_RC_RECEIVER,
            MavSysStatusSensor.MAV_SYS_STATUS_GEOFENCE,
            MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_BATTERY,
            MavSysStatusSensor.MAV_SYS_STATUS_OBSTACLE_AVOIDANCE,
            MavSysStatusSensor.MAV_SYS_STATUS_PREARM_CHECK
        )
        val AVAILABLE_SENSORS: EnumValue<MavSysStatusSensor?> = listToEnum(ALL_SENSORS)
    }
}
