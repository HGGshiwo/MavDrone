package nl.wur.mavdrone.communication.utils

class GCSConnection(
val systemId: Int, 
val componentId: Int) {
    var isConnected: Boolean = false
    var missedHeartbeats: Int = 0
        private set
    var lastHeartbeat: Long = 0
        private set

    override fun toString(): String {
        return "GCS [sysid=$systemId,compid=$componentId]"
    }

    fun updateLastHeartbeatTime() {
        lastHeartbeat = System.currentTimeMillis()
    }

    fun addMissedHeartbeats() {
        missedHeartbeats += 1
    }

    fun resetMissedHeartbeats() {
        missedHeartbeats = 0
    }
}
