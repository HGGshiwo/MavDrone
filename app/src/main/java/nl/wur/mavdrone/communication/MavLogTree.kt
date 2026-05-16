package nl.wur.mavdrone.communication

import java.lang.ref.WeakReference
import kotlin.math.min
import kotlin.math.roundToInt
import android.util.Log

import io.dronefleet.mavlink.common.MavSeverity
import io.dronefleet.mavlink.common.Statustext
import io.dronefleet.mavlink.standard.MavComponent

import nl.wur.mavdrone.communication.utils.MavMessageWithComponentID

import timber.log.Timber

class MavLogTree(mavConnection: MavConnection) : Timber.Tree() {
    var connectionWR: WeakReference<MavConnection> = WeakReference(mavConnection)

    private val lastMessages: MutableMap<String, Long> = HashMap()
    private val logLevelMap: MutableMap<Int, MavSeverity> = HashMap()
    private var messageID = 1

    init {
        logLevelMap[6] = MavSeverity.MAV_SEVERITY_CRITICAL
        logLevelMap[5] = MavSeverity.MAV_SEVERITY_WARNING
        logLevelMap[4] = MavSeverity.MAV_SEVERITY_INFO
        logLevelMap[3] = MavSeverity.MAV_SEVERITY_DEBUG
        logLevelMap[2] = MavSeverity.MAV_SEVERITY_DEBUG
    }

    override fun log(priority: Int, tag: String?, message: String, t: Throwable?) {
        if (connectionWR.get()!!.isNotConnectedToGCS) {
            return
        }

        // Only send errors related to Mavlink part as message, since it takes up quite a bit of
        // bandwidth, especially longer messages
        if (priority < Log.WARN || tag == null || !tag.startsWith("Mav")) {
            return
        }

        val now = System.currentTimeMillis()
        if (lastMessages[message]?.let { now - it < MIN_TIME_BETWEEN_MESSAGES_MS } == true) return

        lastMessages[message] = now

        val severity = logLevelMap[priority]

        // Remove the hex error code
//        Pattern pattern = Pattern.compile(" (.*)\\(0x.+\\)");
//        Matcher matcher = pattern.matcher(logRecord.getMessage());
//        String msg = matcher.group(1);

        // Split text in pieces of 50 characters
        message.chunked(CHARACTERS_PER_MESSAGE).forEachIndexed { index, chunk ->
            val statusText = Statustext.builder()
                .severity(severity)
                .text(chunk)
                .id(if (message.length > CHARACTERS_PER_MESSAGE) messageID else 0)
                .chunkSeq(index)
                .build()

            connectionWR.get()?.sendMavlinkMessage(
                MavMessageWithComponentID(
                    MavComponent.MAV_COMP_ID_AUTOPILOT1,
                    statusText
                )
            )
        }

        if (message.length > CHARACTERS_PER_MESSAGE) {
            messageID += 1
        }
    }

    companion object {
        const val MIN_TIME_BETWEEN_MESSAGES_MS: Int = 60000
        const val CHARACTERS_PER_MESSAGE: Int = 50
    }
}
