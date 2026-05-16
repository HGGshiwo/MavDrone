package nl.wur.mavdrone.communication.utils

import io.dronefleet.mavlink.standard.MavComponent
import io.dronefleet.mavlink.util.EnumValue

class MavMessageWithComponentID(private val componentId: MavComponent, val message: Any?) {
    fun getComponentId(): Int {
        return EnumValue.create(componentId).value()
    }
}
