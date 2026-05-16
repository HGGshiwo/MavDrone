package nl.wur.mavdrone.communication.components.impl.mission

import java.lang.ref.WeakReference

import io.reactivex.rxjava3.core.Completable
import io.reactivex.rxjava3.core.Single

import io.dronefleet.mavlink.MavlinkMessage
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.MissionItemInt

import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.BaseComponent
import nl.wur.mavdrone.communication.components.impl.MissionManager
import nl.wur.mavdrone.communication.utils.KMZ
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.missionUnsupported

abstract class BaseMission(
    connectionWR: WeakReference<MavConnection>,
    
protected val missionManagerWR: WeakReference<MissionManager>,
    logName: String
) : BaseComponent(connectionWR, logName) {
        val missionItems: MutableList<MissionItemInt> = ArrayList()

    abstract fun copy(): BaseMission

    open fun clear() {
        log.i("Clearing ${this.javaClass.simpleName}")
        missionItems.clear()
    }

    fun size(): Int {
        return missionItems.size
    }

    fun get(i: Int): MissionItemInt {
        return missionItems[i]
    }

    open fun addItem(msg: MavlinkMessage<*>): Boolean {
        val payload = msg.payload as MissionItemInt

        return addItem(payload, msg.originSystemId, msg.originComponentId)
    }

    fun addItem(payload: MissionItemInt, msgSystemId: Int, msgComponentId: Int): Boolean {
        if (!allowedCommands.contains(payload.command().entry())) {
            log.e("${this.javaClass.simpleName} does not support ${payload.command().entry()}")
            missionManagerWR.get()
                ?.sendMessage(
                    missionUnsupported(
                        payload.missionType().entry(),
                        msgSystemId,
                        msgComponentId
                    )
                )
            return false
        }

        missionItems.add(payload)
        return true
    }

    fun addItems(missionItems: List<MissionItemInt>) {
        this.missionItems.addAll(missionItems)
    }

    abstract fun convertFromMavlink(): Completable

    abstract fun convertToMavlink(targetSystem: Int, targetComponent: Int): Completable

    abstract fun writeToFile(kmz: KMZ): Completable

    abstract val allowedCommands: List<MavCmd>

    companion object {
        fun <T : BaseMission> fromKMZ(kmz: KMZ): Single<T> {
            throw UnsupportedOperationException("Cannot call fromKMZ on BaseMission directly. Override in a subclass.")
        }
    }
}
