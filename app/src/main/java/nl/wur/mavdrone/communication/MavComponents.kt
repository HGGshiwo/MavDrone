package nl.wur.mavdrone.communication

import java.lang.ref.WeakReference

import timber.log.Timber

import dji.sdk.keyvalue.value.common.CameraLensType
import dji.sdk.keyvalue.value.common.ComponentIndexType

import io.dronefleet.mavlink.standard.MavComponent

import nl.wur.mavdrone.communication.components.Battery
import nl.wur.mavdrone.communication.components.Camera
import nl.wur.mavdrone.communication.components.Drone
import nl.wur.mavdrone.communication.components.Gimbal
import nl.wur.mavdrone.communication.components.MavBaseComponent


class MavComponents(mavConnection: MavConnection) {
    val allComponents: MutableMap<MavComponent, MavBaseComponent> = LinkedHashMap()
    private val connectionWR = WeakReference(mavConnection)

    fun createAllComponents() {
        allComponents[MavComponent.MAV_COMP_ID_AUTOPILOT1] = Drone(connectionWR)
        allComponents[MavComponent.MAV_COMP_ID_CAMERA] = Camera(
            connectionWR,
            MavComponent.MAV_COMP_ID_CAMERA,
            ComponentIndexType.LEFT_OR_MAIN,
            CameraLensType.CAMERA_LENS_DEFAULT,
        )
        allComponents[MavComponent.MAV_COMP_ID_CAMERA2] = Camera(
            connectionWR,
            MavComponent.MAV_COMP_ID_CAMERA2,
            ComponentIndexType.RIGHT,
            CameraLensType.CAMERA_LENS_DEFAULT,
        )
        allComponents[MavComponent.MAV_COMP_ID_CAMERA3] = Camera(
            connectionWR,
            MavComponent.MAV_COMP_ID_CAMERA3,
            ComponentIndexType.UP,
            CameraLensType.CAMERA_LENS_DEFAULT,
        )
        allComponents[MavComponent.MAV_COMP_ID_GIMBAL] = Gimbal(
            connectionWR,
            MavComponent.MAV_COMP_ID_GIMBAL,
            ComponentIndexType.LEFT_OR_MAIN
        )
        allComponents[MavComponent.MAV_COMP_ID_GIMBAL2] = Gimbal(
            connectionWR,
            MavComponent.MAV_COMP_ID_GIMBAL2,
            ComponentIndexType.RIGHT
        )
        allComponents[MavComponent.MAV_COMP_ID_GIMBAL3] = Gimbal(
            connectionWR,
            MavComponent.MAV_COMP_ID_GIMBAL3,
            ComponentIndexType.UP
        )
        allComponents[MavComponent.MAV_COMP_ID_BATTERY] =
            Battery(connectionWR, MavComponent.MAV_COMP_ID_BATTERY, 0)
        allComponents[MavComponent.MAV_COMP_ID_BATTERY2] =
            Battery(connectionWR, MavComponent.MAV_COMP_ID_BATTERY2, 1)
    }

    fun onGCSConnected() {
        allComponents.values.forEach { it.onGCSConnected() }
    }

    fun onGCSDisconnected() {
        Timber.tag("MavConnection").i("Disconnected from all streams!")
        allComponents.values.forEach { it.onGCSDisconnected() }
    }

    fun destroyAll() {
        val iterator: MutableIterator<Map.Entry<MavComponent, MavBaseComponent>> = allComponents.entries.iterator()
        while (iterator.hasNext()) {
            val entry = iterator.next()
            entry.value.destroy()
            iterator.remove()
        }
    }

    val drone: Drone
        get() = allComponents[MavComponent.MAV_COMP_ID_AUTOPILOT1] as Drone

    val batteries: List<Battery>
        get() = listOf(
            MavComponent.MAV_COMP_ID_BATTERY,
            MavComponent.MAV_COMP_ID_BATTERY2
        )
            .mapNotNull { allComponents[it] as? Battery }
            .filter { it.isConnected }

    val cameras: List<Camera>
        get() = listOf(
            MavComponent.MAV_COMP_ID_CAMERA,
            MavComponent.MAV_COMP_ID_CAMERA2,
            MavComponent.MAV_COMP_ID_CAMERA3
        )
            .mapNotNull { allComponents[it] as? Camera }
            .filter { it.isConnected }

    val gimbals: List<Gimbal>
        get() = listOf(
            MavComponent.MAV_COMP_ID_GIMBAL,
            MavComponent.MAV_COMP_ID_GIMBAL2,
            MavComponent.MAV_COMP_ID_GIMBAL3
        )
            .mapNotNull { allComponents[it] as? Gimbal }
            .filter { it.isConnected }

    val cameraToGimbal: Map<Camera, Gimbal?>
        get() = listOf(
            MavComponent.MAV_COMP_ID_CAMERA to MavComponent.MAV_COMP_ID_GIMBAL,
            MavComponent.MAV_COMP_ID_CAMERA2 to MavComponent.MAV_COMP_ID_GIMBAL2,
            MavComponent.MAV_COMP_ID_CAMERA3 to MavComponent.MAV_COMP_ID_GIMBAL3,
            MavComponent.MAV_COMP_ID_CAMERA4 to null // No matching gimbal
        ).mapNotNull { (camId, gimId) ->
            val camera = allComponents[camId] as? Camera
            if (camera?.isConnected == true) {
                val gimbal = gimId?.let { allComponents[it] as? Gimbal }?.takeIf { it.isConnected }
                camera to gimbal
            } else null
        }.toMap()

}
