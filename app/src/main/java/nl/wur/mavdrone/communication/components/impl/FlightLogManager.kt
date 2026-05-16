package nl.wur.mavdrone.communication.components.impl

import java.io.File
import java.io.IOException
import java.io.RandomAccessFile
import java.lang.ref.WeakReference
import java.util.Calendar
import java.util.Locale
import java.util.TimeZone
import java.util.regex.Pattern
import kotlin.math.min

import dji.v5.manager.aircraft.flightrecord.FlightLogManager

import io.dronefleet.mavlink.MavlinkMessage
import io.dronefleet.mavlink.common.LogData
import io.dronefleet.mavlink.common.LogEntry
import io.dronefleet.mavlink.common.LogErase
import io.dronefleet.mavlink.common.LogRequestData
import io.dronefleet.mavlink.common.LogRequestEnd
import io.dronefleet.mavlink.common.LogRequestList
import io.dronefleet.mavlink.util.EnumValue

import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.BaseComponent
import nl.wur.mavdrone.communication.components.MavBaseComponent

class FlightLogManager(
    connectionWR: WeakReference<MavConnection>,
    private val componentWR: WeakReference<out MavBaseComponent>
) : BaseComponent(connectionWR, "MavFlightLogManager") {
    private var logFolderTXT: File = File(FlightLogManager.getInstance().flightRecordPath)
    private var logFiles: MutableList<File> = ArrayList()

    private var pattern: Pattern =
        Pattern.compile("DJIFlightRecord_(\\d{4})-(\\d{2})-(\\d{2})_\\[(\\d{2})-(\\d{2})-(\\d{2})]")

    private var currentLogID: Int = -1
    private var currentOffset: Int = 0
    private var currentLogFile: RandomAccessFile? = null

    override fun create() {
        componentWR.get()?.apply {
            addMavlinkListener(LogRequestList::class.java, this@FlightLogManager::onLogRequestList)
            addMavlinkListener(LogRequestData::class.java, this@FlightLogManager::onLogRequestData)
            addMavlinkListener(LogErase::class.java, this@FlightLogManager::onLogErase)
            addMavlinkListener(LogRequestEnd::class.java, this@FlightLogManager::onLogRequestEnd)
        }
    }

    private fun onLogRequestList(msg: MavlinkMessage<*>) {
        val payload = msg.payload as LogRequestList

        if (payload.targetSystem() == systemID && (payload.targetComponent() == EnumValue.create(
                componentWR.get()!!.componentID
            ).value() || payload.targetComponent() == 0)
        ) {
            log.d("onLogRequestList")

            loadFiles()

            if (logFiles.isEmpty()) {
                val logEntry = LogEntry.builder()
                    .id(0)
                    .numLogs(0)
                    .lastLogNum(0)
                    .timeUtc(0)
                    .size(0)
                    .build()
                componentWR.get()!!.sendMessage(logEntry)
                return
            }

            var endID = payload.end()
            if (endID == 0XFFFF) {
                endID = logFiles.size
            }
            endID = min(logFiles.size.toDouble(), endID.toDouble()).toInt()

            for (i in payload.start() until endID) {
                val logFile = logFiles[i]

                val timeUTC = fileNameToUTCTime(logFile.name) ?: continue

                val logEntry = LogEntry.builder()
                    .id(i)
                    .numLogs(logFiles.size)
                    .lastLogNum(logFiles.size - 1)
                    .timeUtc(timeUTC)
                    .size(logFile.length())
                    .build()
                componentWR.get()?.sendMessage(logEntry)
            }
        }
    }

    @Throws(IOException::class)
    private fun onLogRequestData(msg: MavlinkMessage<*>) {
        val payload = msg.payload as LogRequestData

        if (payload.targetSystem() == systemID && (payload.targetComponent() == EnumValue.create(
                componentWR.get()!!.componentID
            ).value() || payload.targetComponent() == 0)
        ) {
            log.d("onLogRequestPayload")

            if (payload.ofs() == 0L && currentLogID == -1) {
                // Open log file
                currentLogID = payload.id()
                currentLogFile = RandomAccessFile(logFiles[currentLogID], "r")
            } else if (currentLogID != payload.id()) {
                log.e("Cannot download log ${payload.id()} because another file is opened!")
                return
            } else if (currentLogFile == null) {
                log.e("Cannot download log ${payload.id()} because no log file is opened!")
                return
            }

            if (currentOffset.toLong() != payload.ofs()) {
                currentLogFile!!.seek(payload.ofs())
            }
            val data = ByteArray(
                min(payload.count().toDouble(), 90.0).toInt()
            )
            val totalNumberOfBytes = currentLogFile!!.read(data)

            currentOffset += totalNumberOfBytes

            val logData = LogData.builder()
                .id(currentLogID)
                .ofs(payload.ofs())
                .count(totalNumberOfBytes)
                .data(data)
                .build()
            componentWR.get()!!.sendMessage(logData)

            if (payload.count() - totalNumberOfBytes <= 0) {
                closeCurrentLogFile()
            }
        }
    }

    private fun onLogErase(msg: MavlinkMessage<*>) {
        val payload = msg.payload as LogErase

        if (payload.targetSystem() == systemID && (payload.targetComponent() == EnumValue.create(
                componentWR.get()!!.componentID
            ).value() || payload.targetComponent() == 0)
        ) {
            log.d("onLogErase")

            // TODO: Ask confirmation
            logFiles.forEach {
                if (!it.delete()) {
                    log.w("Could not delete ${it.path}")
                }
            }

            // Reload log files
            loadFiles()
        }
    }

    private fun onLogRequestEnd(msg: MavlinkMessage<*>) {
        val payload = msg.payload as LogRequestEnd

        if (payload.targetSystem() == systemID && (payload.targetComponent() == EnumValue.create(
                componentWR.get()?.componentID
            ).value() || payload.targetComponent() == 0)
        ) {
            log.d("onLogRequestEnd")

            closeCurrentLogFile()
        }
    }

    //region: utils
    private fun closeCurrentLogFile() {
        try {
            log.i("Closing log file $currentLogID")

            currentLogFile?.close()

            currentLogID = -1
            currentOffset = 0
        } catch (e: IOException) {
            log.w("Cannot close current log file: ${e.message}")
        }
    }

    private fun loadFiles() {
        logFiles.clear()

        logFolderTXT.listFiles { _, name -> name.lowercase(Locale.getDefault()).endsWith(".txt") }
            ?.let { logFiles.addAll(it) }
    }

    private fun fileNameToUTCTime(fileName: String): Long? {
        val m = pattern.matcher(fileName)

        if (m.find()) {
            val year = m.group(1)?.toInt() ?: -1
            val month = m.group(2)?.toInt()?.minus(1) ?: -1
            val day = m.group(3)?.toInt() ?: -1
            val hour = m.group(4)?.toInt() ?: -1
            val minute = m.group(5)?.toInt() ?: -1
            val second = m.group(6)?.toInt() ?: -1

            val calendar = Calendar.getInstance(TimeZone.getDefault())
            calendar.clear()
            calendar.set(year, month, day, hour, minute, second)

            return calendar.timeInMillis / 1000L
        }

        log.w("Could not parse file name: $fileName")
        return null
    } //endregion
}
