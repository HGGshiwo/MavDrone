package nl.wur.mavdrone.communication.components.impl

import dji.v5.manager.diagnostic.DJIDeviceHealthInfo
import dji.v5.manager.diagnostic.DJIDeviceHealthInfoChangeListener
import dji.v5.manager.diagnostic.DeviceHealthManager
import dji.v5.manager.diagnostic.WarningLevel
import io.dronefleet.mavlink.common.MavSeverity
import io.dronefleet.mavlink.common.Statustext
import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.BaseComponent
import nl.wur.mavdrone.communication.components.MavBaseComponent
import java.lang.ref.WeakReference
import kotlin.math.min
import kotlin.math.roundToInt

class DeviceHealth(
    connectionWR: WeakReference<MavConnection>,
    private val componentWR: WeakReference<out MavBaseComponent>
) : BaseComponent(connectionWR, "MavDeviceHealth"), DJIDeviceHealthInfoChangeListener {
    private val lastMessages: MutableMap<String, Long> = HashMap()
    private var messageID = 1

    override fun create() {
        DeviceHealthManager.getInstance().addDJIDeviceHealthInfoChangeListener(this)
    }

    override fun destroy() {
        DeviceHealthManager.getInstance().removeDJIDeviceHealthInfoChangeListener(this)

        super.destroy()
    }

    private fun sendDeviceHealthUpdate(info: DJIDeviceHealthInfo) {
        connectionWR.get()?.let {
            if (it.isNotConnectedToGCS) return
        }

        val now = System.currentTimeMillis()
        lastMessages[info.informationCode()]?.let {
            if (now - it < MIN_TIME_BETWEEN_MESSAGES_MS) return
        }

        lastMessages[info.informationCode()] = now
        val severity = when (info.warningLevel()) {
            WarningLevel.SERIOUS_WARNING -> MavSeverity.MAV_SEVERITY_ALERT
            WarningLevel.WARNING -> MavSeverity.MAV_SEVERITY_ERROR
            WarningLevel.CAUTION -> MavSeverity.MAV_SEVERITY_WARNING
            else -> return
        }

        // Split text in pieces of 50 characters
        info.description().chunked(CHARACTERS_PER_MESSAGE).forEachIndexed { index, chunk ->
            val statusText = Statustext.builder()
                .severity(severity)
                .text(chunk)
                .id(if (info.description().length <= CHARACTERS_PER_MESSAGE) 0 else messageID)
                .chunkSeq(index)
                .build()

            componentWR.get()?.sendMessage(statusText)
        }

        if (info.description().length > CHARACTERS_PER_MESSAGE) {
            messageID += 1
        }
    }

    override fun onDeviceHealthInfoUpdate(messages: List<DJIDeviceHealthInfo>) {
        messages.forEach { sendDeviceHealthUpdate(it) }
    }

    companion object {
        const val MIN_TIME_BETWEEN_MESSAGES_MS: Int = 60000
        const val CHARACTERS_PER_MESSAGE: Int = 50
    }
}
