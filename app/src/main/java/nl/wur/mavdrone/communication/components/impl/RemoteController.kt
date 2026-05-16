package nl.wur.mavdrone.communication.components.impl

import dji.sdk.keyvalue.key.AirLinkKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.RemoteControllerKey
import dji.v5.manager.KeyManager
import io.dronefleet.mavlink.common.RcChannels
import io.dronefleet.mavlink.standard.MavModeFlag
import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.BaseComponent
import nl.wur.mavdrone.communication.components.MavBaseComponent
import java.lang.ref.WeakReference

class RemoteController(
    connectionWR: WeakReference<MavConnection>,
    private val componentWR: WeakReference<out MavBaseComponent>
) : BaseComponent(connectionWR, "MavRemoteController") {
    private var airlinkQuality = 0

    private var remoteLeftVerticalStick = 0
    private var remoteLeftHorizontalStick = 0
    private var remoteRightVerticalStick = 0
    private var remoteRightHorizontalStick = 0
    private var remoteLeftDial = 0
    private var remoteRightDial = 0
    private var remoteShutterButton = false
    private var remoteRecordButton = false

    override fun create() {
        componentWR.get()?.addDJIListener(KeyTools.createKey(AirLinkKey.KeyConnection))
        { _: Boolean?, newValue: Boolean? ->
            newValue?.let { if (it) onAirlinkConnected() else onAirlinkDisconnected() } ?: onAirlinkDisconnected()
        }

        componentWR.get()?.addDJIListener(KeyTools.createKey(RemoteControllerKey.KeyConnection))
        { _: Boolean?, newValue: Boolean? ->
            newValue?.let { if (it) onRemoteControllerConnected() else onRemoteControllerDisconnected() } ?: onRemoteControllerDisconnected()
        }

        componentWR.get()?.addMavlinkMessageSender(RcChannels::class.java, { this.toMavlinkRcChannelsMessage() }, true)
    }

    override fun destroy() {
        onAirlinkDisconnected()
        onRemoteControllerDisconnected()

        super.destroy()
    }

    private fun onAirlinkConnected() {
        KeyManager.getInstance().listen(KeyTools.createKey(AirLinkKey.KeySignalQuality), this)
        { _: Int?, newValue: Int? -> newValue?.let { airlinkQuality = it } }
    }

    private fun onAirlinkDisconnected() {
        KeyManager.getInstance().cancelListen(KeyTools.createKey(AirLinkKey.KeySignalQuality), this)
    }

    private fun onRemoteControllerConnected() {
        componentWR.get()?.addMavModeFlag(MavModeFlag.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)

        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyStickLeftVertical), this)
        { _: Int?, newValue: Int? -> newValue?.let { remoteLeftVerticalStick = it } }

        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyStickLeftHorizontal), this)
        { _: Int?, newValue: Int? -> newValue?.let { remoteLeftHorizontalStick = it } }

        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyStickRightVertical), this)
        { _: Int?, newValue: Int? -> newValue?.let { remoteRightVerticalStick = it } }

        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyStickRightHorizontal), this)
        { _: Int?, newValue: Int? -> newValue?.let { remoteRightHorizontalStick = it } }

        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyLeftDial), this)
        { _: Int?, newValue: Int? -> newValue?.let { remoteLeftDial = it } }

        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyRightDial), this)
        { _: Int?, newValue: Int? -> newValue?.let { remoteRightDial = it } }

        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyShutterButtonDown), this)
        { _: Boolean?, newValue: Boolean? -> newValue?.let { remoteShutterButton = it } }

        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyRecordButtonDown), this)
        { _: Boolean?, newValue: Boolean? -> newValue?.let { remoteRecordButton = it } }
    }

    private fun onRemoteControllerDisconnected() {
        componentWR.get()?.removeMavModeFlag(MavModeFlag.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)

        KeyManager.getInstance().cancelListen(KeyTools.createKey(RemoteControllerKey.KeyStickLeftVertical), this)
        KeyManager.getInstance().cancelListen(KeyTools.createKey(RemoteControllerKey.KeyStickLeftHorizontal), this)
        KeyManager.getInstance().cancelListen(KeyTools.createKey(RemoteControllerKey.KeyStickRightVertical), this)
        KeyManager.getInstance().cancelListen(KeyTools.createKey(RemoteControllerKey.KeyStickRightHorizontal), this)
        KeyManager.getInstance().cancelListen(KeyTools.createKey(RemoteControllerKey.KeyLeftDial), this)
        KeyManager.getInstance().cancelListen(KeyTools.createKey(RemoteControllerKey.KeyRightDial), this)
        KeyManager.getInstance().cancelListen(KeyTools.createKey(RemoteControllerKey.KeyShutterButtonDown), this)
        KeyManager.getInstance().cancelListen(KeyTools.createKey(RemoteControllerKey.KeyRecordButtonDown), this)
    }

    private fun toMavlinkRcChannelsMessage(): RcChannels {
        // 1000 = 0%, 2000 = 100%, 0 = -100%
        return RcChannels.builder()
            .timeBootMs(componentWR.get()!!.millisSinceBoot)
            .chancount(8)
            .chan1Raw(scaleStick(remoteLeftVerticalStick))
            .chan2Raw(scaleStick(remoteLeftHorizontalStick))
            .chan3Raw(scaleStick(remoteRightVerticalStick))
            .chan4Raw(scaleStick(remoteRightHorizontalStick))
            .chan5Raw(scaleStick(remoteLeftDial))
            .chan6Raw(scaleStick(remoteRightDial))
            .chan7Raw(if (remoteShutterButton) 2000 else 1000)
            .chan8Raw(if (remoteRecordButton) 2000 else 1000)
            .rssi(airlinkQuality)
            .build()
    }

    private fun scaleStick(value: Int): Int = ((value / STICK_MAX_VALUE) * 1000 + 1000).toInt()

    companion object {
        private const val STICK_MAX_VALUE: Float = 660f
    }
}
