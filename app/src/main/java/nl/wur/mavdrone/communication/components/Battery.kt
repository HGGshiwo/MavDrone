package nl.wur.mavdrone.communication.components

import java.lang.ref.WeakReference
import java.util.concurrent.TimeUnit
import kotlin.math.absoluteValue
import kotlin.math.roundToInt
import android.annotation.SuppressLint

import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.v5.manager.KeyManager

import io.reactivex.rxjava3.core.Completable

import io.dronefleet.mavlink.common.BatteryStatus
import io.dronefleet.mavlink.common.MavBatteryChargeState
import io.dronefleet.mavlink.common.MavBatteryFunction
import io.dronefleet.mavlink.common.MavBatteryMode
import io.dronefleet.mavlink.common.MavBatteryType
import io.dronefleet.mavlink.common.SmartBatteryInfo
import io.dronefleet.mavlink.standard.MavAutopilot
import io.dronefleet.mavlink.standard.MavComponent
import io.dronefleet.mavlink.standard.MavType

import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.utils.RXUtils

class Battery(connectionWR: WeakReference<MavConnection>, componentId: MavComponent, private val componentIndex: Int) : MavBaseComponent(
    connectionWR,
    KeyTools.createKey(BatteryKey.KeyConnection, componentIndex),
    componentId,
    MavType.MAV_TYPE_BATTERY,
    MavAutopilot.MAV_AUTOPILOT_INVALID,
    "MavBattery$componentIndex",
    true
) {
    private var batterySpecification: BatterySpecification? = null
    private var batteryTemperature = UINT16_MAX
    private val batteryVoltages = arrayOfNulls<Int>(10)
    private val batteryVoltagesExt = arrayOfNulls<Int>(4)
    var current: Int = UINT16_MAX
        private set
    var voltage: Int = UINT16_MAX
        private set
    private var batteryFullChargeCapacity = UINT16_MAX
    private var batteryChargeRemaining = UINT16_MAX
    var remainingPercent: Int = UINT16_MAX
        private set
    private var batteryRemainingSeconds = UINT16_MAX

    private var lowWarningThreshold = 20
    private var criticalWarningThreshold = 5

    init {
        connectToProduct()
    }

    override fun create() {
        batteryVoltages.fill(UINT16_MAX)
        batteryVoltagesExt.fill(0)

        batterySpecification = getBatterySpecification()
        batterySpecification?.gatherCurrentBatterySpecification(componentIndex)

        //region: DJI listeners
        addDJIListener<List<Int>>(KeyTools.createKey(BatteryKey.KeyCellVoltages, componentIndex)) { value ->
            batteryVoltages.fill(UINT16_MAX)
            batteryVoltagesExt.fill(0)

            value.take(10).forEachIndexed { i, voltage -> batteryVoltages[i] = voltage }
            value.drop(10).take(4).forEachIndexed { i, voltage -> batteryVoltagesExt[i] = voltage }
        }
        addDJIListener(KeyTools.createKey(FlightControllerKey.KeyLowBatteryWarningThreshold)) { value -> lowWarningThreshold = value }
        addDJIListener(KeyTools.createKey(FlightControllerKey.KeySeriousLowBatteryWarningThreshold)) { value -> criticalWarningThreshold = value }
        addDJIListener(KeyTools.createKey(BatteryKey.KeyBatteryTemperature, componentIndex)) { value -> batteryTemperature = (value * 100).roundToInt() }
        addDJIListener(KeyTools.createKey(BatteryKey.KeyCurrent, componentIndex)) { value -> current = (value * 10).absoluteValue }
        addDJIListener(KeyTools.createKey(BatteryKey.KeyVoltage, componentIndex)) { value -> voltage = value }
        addDJIListener(KeyTools.createKey(BatteryKey.KeyFullChargeCapacity, componentIndex)) { value -> batteryFullChargeCapacity = value }
        addDJIListener(KeyTools.createKey(BatteryKey.KeyChargeRemaining, componentIndex)) { value -> batteryChargeRemaining = value }
        addDJIListener(KeyTools.createKey(BatteryKey.KeyChargeRemainingInPercent, componentIndex)) { value -> remainingPercent = value }
        addDJIListener(KeyTools.createKey(FlightControllerKey.KeyLowBatteryRTHInfo)) { value -> batteryRemainingSeconds = value.remainingFlightTime }

        //endregion

        // Add message senders
        addMavlinkMessageSender(BatteryStatus::class.java, { this.toMavlinkBatteryStatusMessage() }, 1000)
        addMavlinkMessageSender(SmartBatteryInfo::class.java) { this.toMavlinkSmartBatteryInfoMessage() }

        // Make sure that the second battery only starts emitting messages after the first
        // battery. Otherwise, QGC will fail because it expects the battery with ID 1 to be the
        // first message it receives.
        compositeDisposable.add(
            Completable
                .timer(componentIndex * 1000L, TimeUnit.MILLISECONDS)
                .subscribe(
                    { startMavlinkMessageSender(BatteryStatus::class.java) },
                    { error -> RXUtils.handleError(error) })
        )
    }


    private fun getBatterySpecification(): BatterySpecification {
        val serialNumber = KeyManager.getInstance().getValue(KeyTools.createKey(BatteryKey.KeySerialNumber, componentIndex))
        for ((key, value) in BATTERY_SPECIFICATIONS) {
            if (serialNumber != null && serialNumber.startsWith(key)) {
                return value
            }
        }

        return BatterySpecification("0_0", -1, 0)
    }

    private fun toMavlinkSmartBatteryInfoMessage(): SmartBatteryInfo {
        //TODO: replace with BATTERY_INFO message
        return SmartBatteryInfo.builder()
            .id(componentIndex)
            .batteryFunction(MavBatteryFunction.MAV_BATTERY_FUNCTION_ALL)
            .type(MavBatteryType.MAV_BATTERY_TYPE_LIPO)
            .capacityFullSpecification(batterySpecification!!.capacityFullSpecification)
            .capacityFull(batteryFullChargeCapacity)
            .cycleCount(batterySpecification!!.cycleCount)
            .serialNumber(batterySpecification!!.serialNumber)
            .deviceName(batterySpecification!!.deviceName)
            .weight(batterySpecification!!.weight)
            .dischargeMinimumVoltage(UINT16_MAX)
            .chargingMinimumVoltage(UINT16_MAX)
            .restingMinimumVoltage(UINT16_MAX)
            .chargingMaximumVoltage(0)
            .cellsInSeries(batterySpecification!!.numberOfCells)
            .dischargeMaximumCurrent(0)
            .dischargeMaximumBurstCurrent(0)
            .manufactureDate(batterySpecification!!.manufactureDate)
            .build()
    }

    private fun toMavlinkBatteryStatusMessage(): BatteryStatus {
        val chargeState = if (remainingPercent < criticalWarningThreshold) {
            MavBatteryChargeState.MAV_BATTERY_CHARGE_STATE_CRITICAL
        } else if (remainingPercent < lowWarningThreshold) {
            MavBatteryChargeState.MAV_BATTERY_CHARGE_STATE_LOW
        } else {
            MavBatteryChargeState.MAV_BATTERY_CHARGE_STATE_OK
        }

        return BatteryStatus.builder()
            .id(componentIndex)
            .batteryFunction(MavBatteryFunction.MAV_BATTERY_FUNCTION_ALL)
            .type(MavBatteryType.MAV_BATTERY_TYPE_LIPO)
            .temperature(batteryTemperature)
            .voltages(listOf(*batteryVoltages))
            .currentBattery(current)
            .currentConsumed(batteryFullChargeCapacity - batteryChargeRemaining)
            .batteryRemaining(remainingPercent)
            .timeRemaining(batteryRemainingSeconds)
            .chargeState(chargeState)
            .voltagesExt(listOf(*batteryVoltagesExt))
            .mode(MavBatteryMode.MAV_BATTERY_MODE_UNKNOWN)
            .build()
    }

    private class BatterySpecification(val deviceName: String, val capacityFullSpecification: Int, val weight: Int) {
        var numberOfCells: Int = 0
        var cycleCount: Int = 0
        var serialNumber: String? = null
        var manufactureDate: String? = null

        @SuppressLint("DefaultLocale")
        fun gatherCurrentBatterySpecification(componentIndex: Int) {
            numberOfCells = KeyManager.getInstance().getValue(KeyTools.createKey(BatteryKey.KeyNumberOfCells, componentIndex), 0)
            cycleCount = KeyManager.getInstance().getValue(KeyTools.createKey(BatteryKey.KeyNumberOfDischarges, componentIndex), UINT16_MAX)
            serialNumber = KeyManager.getInstance().getValue(KeyTools.createKey(BatteryKey.KeySerialNumber, componentIndex))

            manufactureDate = "00/00/0000"
            KeyManager.getInstance().getValue(KeyTools.createKey(BatteryKey.KeyBatteryManufacturedDate, componentIndex))?.let {
                manufactureDate = String.format("%02d/%02d/%04d", it.day, it.month, it.year)
            }
        }
    }

    companion object {
        // TODO: add more battery specifications here
        private val BATTERY_SPECIFICATIONS: Map<String, BatterySpecification> = mapOf(
            "1Z6" to BatterySpecification("DJI_TB60", 5935, 1350),
            "6JY" to BatterySpecification("DJI_TB65", 5880, 1350),
            "4ER" to BatterySpecification("DJI_MAVIC3BATTERY", 5000, 335)
        )
    }
}
