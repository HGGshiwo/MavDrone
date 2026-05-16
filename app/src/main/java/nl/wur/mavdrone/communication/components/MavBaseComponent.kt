package nl.wur.mavdrone.communication.components

import java.lang.ref.WeakReference
import java.math.BigInteger
import java.time.Clock
import java.time.Duration
import java.util.concurrent.TimeUnit
import java.util.function.BiFunction
import java.util.function.Function
import kotlin.math.roundToInt
import android.os.Build

import dji.sdk.keyvalue.key.DJIKey
import dji.v5.common.callback.CommonCallbacks
import dji.v5.manager.KeyManager

import io.reactivex.rxjava3.core.Observable
import io.reactivex.rxjava3.disposables.CompositeDisposable
import io.reactivex.rxjava3.disposables.Disposable
import io.reactivex.rxjava3.functions.Consumer
import io.reactivex.rxjava3.schedulers.Schedulers

import io.dronefleet.mavlink.MavlinkMessage
import io.dronefleet.mavlink.annotations.MavlinkMessageInfo
import io.dronefleet.mavlink.common.CommandAck
import io.dronefleet.mavlink.common.CommandLong
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.MessageInterval
import io.dronefleet.mavlink.common.MissionAck
import io.dronefleet.mavlink.common.ParamExtRequestList
import io.dronefleet.mavlink.common.ParamExtRequestRead
import io.dronefleet.mavlink.common.ParamExtSet
import io.dronefleet.mavlink.common.ParamRequestList
import io.dronefleet.mavlink.common.ParamRequestRead
import io.dronefleet.mavlink.common.ParamSet
import io.dronefleet.mavlink.common.Ping
import io.dronefleet.mavlink.common.SystemTime
import io.dronefleet.mavlink.standard.Heartbeat
import io.dronefleet.mavlink.standard.MavAutopilot
import io.dronefleet.mavlink.standard.MavComponent
import io.dronefleet.mavlink.standard.MavModeFlag
import io.dronefleet.mavlink.standard.MavState
import io.dronefleet.mavlink.standard.MavType
import io.dronefleet.mavlink.standard.ProtocolVersion
import io.dronefleet.mavlink.util.EnumValue

import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.utils.CustomMode.MAIN_MODE
import nl.wur.mavdrone.communication.utils.CustomMode.MAIN_MODE.Companion.fromValue
import nl.wur.mavdrone.communication.utils.EnumUtils.Companion.listToEnum
import nl.wur.mavdrone.communication.utils.MavMessageWithComponentID
import nl.wur.mavdrone.communication.utils.MavParameter
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandAccepted
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandDenied
import nl.wur.mavdrone.communication.utils.StandardResponses.Companion.commandFailed
import nl.wur.mavdrone.utils.RXUtils.Companion.handleError
import nl.wur.mavdrone.utils.RxException
import nl.wur.mavdrone.utils.TriFunction

@Suppress("deprecation")
abstract class MavBaseComponent(
    connectionWR: WeakReference<MavConnection>,
    private val connectionKey: DJIKey<Boolean>?,
    
val componentID: MavComponent,
    private val mavType: MavType,
    private val mavAutopilot: MavAutopilot,
    logName: String,
    private val allowReconnect: Boolean
) : BaseComponent(connectionWR, logName) {
    private var mavState = MavState.MAV_STATE_UNINIT
    private val mavModeFlags: MutableList<MavModeFlag> = ArrayList()
    protected var customMode = 0

    private val usedKeys: MutableList<DJIKey<*>> = ArrayList()
    private val messageSenders: MutableMap<Int, MessageSender<*>> = LinkedHashMap()
    private val broadcasterDisposables: MutableMap<String, Disposable> = LinkedHashMap()
        val mavlinkParameters: MutableMap<String, MavParameter<*>> = LinkedHashMap()

    var isConnected: Boolean = false
        protected set

    private var bootClock: Clock? = null
    var bootMillis: Long = 0
        private set

    override fun destroy() {
        // Dispose broadcasters
        broadcasterDisposables.values.forEach { it.dispose() }
        broadcasterDisposables.clear()
        messageSenders.clear()
        mavlinkParameters.values.forEach { it.destroy() }
        mavlinkParameters.clear()

        destroyListeners()

        super.destroy()
    }

    fun connectToProduct() {
        connectionKey?.let {
            addDJIListener(connectionKey) { _: Boolean?, newValue: Boolean? ->
                if (newValue != null && newValue) {
                    onConnected()
                } else {
                    onDisconnected()
                }
            }
        } ?: onConnected()
    }

    abstract override fun create()

    protected open fun destroyListeners() {
        usedKeys.forEach { KeyManager.getInstance().cancelListen(it, this) }
        usedKeys.clear()
    }

    protected fun setState(state: MavState) {
        this.mavState = state
    }

    private fun onConnected() {
        compositeDisposable = CompositeDisposable()

        bootMillis = System.currentTimeMillis()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            bootClock = Clock.offset(Clock.systemUTC(), Duration.ofMillis(bootMillis))
        }

        // Add message senders
        addMavlinkMessageSender(Heartbeat::class.java, { this.toMavlinkHeartBeatMessage() }, 1000, true)
        addMavlinkMessageSender(ProtocolVersion::class.java) { this.toMavlinkProtocolVersionMessage() }
        addMavlinkMessageSender(SystemTime::class.java) { this.toMavlinkSystemTimeMessage() }

        log.i("Start sending Mavlink messages for ${componentID.name}")

        // Add callbacks for default messages
        addMavlinkListener(CommandLong::class.java) { msg -> this.onCommandLong(msg) }
        addMavlinkListener(ParamRequestList::class.java) { msg -> this.onParamRequestList(msg) }
        addMavlinkListener(ParamRequestRead::class.java) { msg -> this.onParamRequestRead(msg) }
        addMavlinkListener(ParamExtRequestList::class.java) { msg -> this.onParamExtRequestList(msg) }
        addMavlinkListener(ParamExtRequestRead::class.java) { msg -> this.onParamExtRequestRead(msg) }
        addMavlinkListener(ParamExtSet::class.java) { msg -> this.onParamExtSet(msg) }
        addMavlinkListener(ParamSet::class.java) { msg -> this.onParamSet(msg) }
        addMavlinkListener(Ping::class.java) { msg -> this.onPing(msg) }

        create()

        isConnected = true
    }

    private fun onDisconnected() {
        log.e("Lost connection to ${componentID.name}!")

        isConnected = false

        messageSenders.clear()

        // Dispose broadcasters
        broadcasterDisposables.values.forEach { it.dispose() }
        broadcasterDisposables.clear()

        compositeDisposable.dispose()
        destroyListeners()

        if (allowReconnect) { connectToProduct() }
    }

    fun sendMessage(message: Any?) {
        connectionWR.get()?.sendMavlinkMessage(MavMessageWithComponentID(componentID, message))
    }

    val millisSinceBoot: Long
        //region: generic Mavlink utils
        get() = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            bootClock?.millis() ?: (System.currentTimeMillis() - bootMillis)
        } else {
            System.currentTimeMillis() - bootMillis
        }


    fun addBroadcaster(key: String, intervalMs: Int, function: Consumer<Long>) {
        val broadcaster = Observable.interval(intervalMs.toLong(), TimeUnit.MILLISECONDS)
            .subscribeOn(Schedulers.newThread())
            .subscribe(function)

        broadcasterDisposables[key] = broadcaster
    }

    fun addBroadcaster(key: String, delayMs: Int, intervalMs: Int, function: Consumer<Long>) {
        val broadcaster = Observable.interval(delayMs.toLong(), intervalMs.toLong(), TimeUnit.MILLISECONDS)
            .subscribeOn(Schedulers.newThread())
            .subscribe(function)

        broadcasterDisposables[key] = broadcaster
    }

    fun stopBroadcaster(key: String) {
        val broadcaster = broadcasterDisposables.remove(key)
        broadcaster?.dispose()
    }

    fun isBroadcasterRunning(key: String): Boolean {
        return broadcasterDisposables.containsKey(key)
    }

    fun <MessageType> addMavlinkMessageSender(messageType: Class<MessageType>, messageSupplier: () -> MessageType?) {
        addMavlinkMessageSender(messageType, messageSupplier, 1000)
    }

    fun <MessageType> addMavlinkMessageSender(messageType: Class<MessageType>, messageSupplier: () -> MessageType?, defaultMs: Int) {
        val messageInfo = messageType.getAnnotation(MavlinkMessageInfo::class.java)

        if (messageInfo == null) {
            log.e("Cannot add message sender because $messageType is not a Mavlink message")
            return
        }

        val messageId = messageInfo.id

        val messageSender = MessageSender(messageSupplier, messageId, defaultMs)
        messageSenders[messageId] = messageSender
    }

    fun <MessageType> addMavlinkMessageSender(messageType: Class<MessageType>, messageSupplier: () -> MessageType?, start: Boolean) {
        addMavlinkMessageSender(messageType, messageSupplier)

        if (start) {
            startMavlinkMessageSender(messageType)
        }
    }

    fun <MessageType> addMavlinkMessageSender(messageType: Class<MessageType>, messageSupplier: () -> MessageType?, defaultMs: Int, start: Boolean) {
        addMavlinkMessageSender(messageType, messageSupplier, defaultMs)

        if (start) {
            startMavlinkMessageSender(messageType)
        }
    }

    fun startMavlinkMessageSender(messageType: Class<*>) {
        val messageInfo = messageType.getAnnotation(MavlinkMessageInfo::class.java)

        if (messageInfo == null) {
            log.e("Cannot start sender because $messageType is not a Mavlink message")
            return
        }

        val messageId = messageInfo.id
        val messageSender = messageSenders[messageId]

        if (messageSender == null) {
            log.e("Cannot start sender because $messageType is not added to this class. Add sender using the addMavlinkMessageSender() function!")
            return
        }

        messageSender.startMessageBroadcaster()
    }

    fun <MessageType: Any> addMavlinkListener(messageType: Class<MessageType>, callback: Consumer<MavlinkMessage<*>>) {
        compositeDisposable.add(
            connectionWR.get()!!.mavlinkIncomingMessagesObservable!!
                .filter { mavlinkMessage -> messageType.isInstance(mavlinkMessage.payload) }
                .subscribe(callback) { error -> log.e("Error in processing ${messageType.simpleName} message: ${error.message}") }
        )
    }

    fun <ResultType> addDJIListener(key: DJIKey<ResultType>, callback: CommonCallbacks.KeyListener<ResultType>) {
        KeyManager.getInstance().listen(key, this, callback)
        usedKeys.add(key)
    }

    fun <ResultType> addDJIListener(key: DJIKey<ResultType>, consumer: java.util.function.Consumer<ResultType>) {
        addDJIListener(key) { _: ResultType?, newValue: ResultType? ->
            newValue?.let { consumer.accept(it) }
        }
    }

    fun addParameter(parameter: MavParameter<*>) {
        mavlinkParameters[parameter.name] = parameter
    }

    protected fun isCommandForMe(payload: CommandLong) : Boolean {
        return payload.targetSystem() == systemID
                && (payload.targetComponent() == EnumValue.create(componentID).value() || payload.targetComponent() == 0)
    }

    //endregion
    //region: handle messages
    protected open fun onCommandLong(msg: MavlinkMessage<*>): Boolean {
        val payload = msg.payload as CommandLong

        if (isCommandForMe(payload)) {
            val index: Int
            when (payload.command().entry()) {
                MavCmd.MAV_CMD_REQUEST_MESSAGE -> {
                    val requestedMessageID = payload.param1().roundToInt()

                    if (requestedMessageID == 244) { // MESSAGE_INTERVAL
                        // This message is an exception since it uses param2 as id
                        sendMessage(commandAccepted(MavCmd.MAV_CMD_REQUEST_MESSAGE, msg.originSystemId, msg.originComponentId))
                        val messageInterval = toMavlinkMessageIntervalMessage(payload.param2().roundToInt())
                        sendMessage(messageInterval)
                    } else {
                        messageSenders[requestedMessageID]?.let {
                            sendMessage(commandAccepted(MavCmd.MAV_CMD_REQUEST_MESSAGE, msg.originSystemId, msg.originComponentId))
                            try {
                                sendMessage(it.messageSupplier())
                                log.d("Send message $requestedMessageID from component $componentID (target component ${payload.targetComponent()})")
                            } catch (e: Throwable) {
                                log.w("Cannot send message $requestedMessageID, unknown error: ${e.message}")
                                sendMessage(commandFailed(MavCmd.MAV_CMD_REQUEST_MESSAGE, msg.originSystemId, msg.originComponentId))
                            }
                        } ?: run {
                            // Debug this while it's an error. MAVSDK just keeps on spamming by
                            // asking messages that do not exist on some components
                            log.d("Cannot send message $requestedMessageID, message is not specified for component $componentID (target component ${payload.targetComponent()})")
                            sendMessage(commandDenied(MavCmd.MAV_CMD_REQUEST_MESSAGE, msg.originSystemId, msg.originComponentId))
                        }
                    }
                    return true
                }

                MavCmd.MAV_CMD_SET_MESSAGE_INTERVAL -> {
                    index = payload.param1().roundToInt()
                    val messageIntervalUs = payload.param2().roundToInt()

                    if (setMessageInterval(index, messageIntervalUs)) {
                        sendMessage(commandAccepted(MavCmd.MAV_CMD_SET_MESSAGE_INTERVAL, msg.originSystemId, msg.originComponentId))
                    } else {
                        sendMessage(commandFailed(MavCmd.MAV_CMD_SET_MESSAGE_INTERVAL, msg.originSystemId, msg.originComponentId))
                    }
                    return true
                }

                MavCmd.MAV_CMD_GET_MESSAGE_INTERVAL -> {
                    index = payload.param1().roundToInt()
                    sendMessage(commandAccepted(MavCmd.MAV_CMD_SET_MESSAGE_INTERVAL))

                    val messageInterval = toMavlinkMessageIntervalMessage(index)
                    sendMessage(messageInterval)
                    return true
                }

                else -> {}
            }
        }

        return false
    }

    private fun onPing(msg: MavlinkMessage<*>) {
        val payload = msg.payload as Ping
        if (payload.targetSystem() == 0 && payload.targetComponent() == 0) {
            val ping = toMavlinkPingMessage(msg.originSystemId, msg.originComponentId, payload.timeUsec(), payload.seq())
            sendMessage(ping)
        }
    }

    //endregion
    //region: handle commands
    private fun setMessageInterval(messageId: Int, messageIntervalUs: Int): Boolean {
        var interval = messageIntervalUs
        val messageSender = messageSenders[messageId]

        if (messageSender == null) {
            log.e("Cannot set message interval for id $messageId because the message sender is not defined on component $componentID!")
            return false
        }

        messageSender.stopMessageBroadcaster()

        if (interval >= 0) {
            if (interval == 0) {
                interval = messageSender.defaultIntervalMs * 1000
            }

            messageSender.intervalMs = interval / 1000
            messageSender.startMessageBroadcaster()
        }
        return true
    }

    fun addMavModeFlag(modeFlag: MavModeFlag) {
        if (!mavModeFlags.contains(modeFlag)) {
            mavModeFlags.add(modeFlag)
        }
    }

    fun getMavModeFlags(): List<MavModeFlag> {
        return mavModeFlags
    }

    fun containsMavModeFlag(modeFlag: MavModeFlag): Boolean {
        return mavModeFlags.contains(modeFlag)
    }

    fun removeMavModeFlag(modeFlag: MavModeFlag) {
        mavModeFlags.remove(modeFlag)
    }

    protected fun clearMavModeFlags() {
        mavModeFlags.clear()
    }

    fun setCustomMode(customMode: MAIN_MODE) {
        this.customMode = customMode.value
    }

    fun clearCustomMode() {
        this.customMode = 0
    }

    fun getCustomMode(): MAIN_MODE {
        return fromValue(customMode)
    }

    //region: parameters
    private fun onParamRequestList(msg: MavlinkMessage<*>) {
        val payload = msg.payload as ParamRequestList
        if (payload.targetSystem() == systemID && (payload.targetComponent() == EnumValue.create(componentID).value() || payload.targetComponent() == 0)) {
            log.d("onParamRequestList")

            mavlinkParameters.keys.forEach { sendParameter(it) }
        }
    }

    private fun onParamRequestRead(msg: MavlinkMessage<*>) {
        val payload = msg.payload as ParamRequestRead
        if (payload.targetSystem() == systemID && (payload.targetComponent() == EnumValue.create(componentID).value() || payload.targetComponent() == 0)) {
            log.d("onParamRequestRead")

            // param id -1 to use the param ID field as identifier (else the param id will be ignored)
            if (payload.paramIndex() == -1) {
                sendParameter(payload.paramId())
            } else {
                sendParameter(payload.paramIndex())
            }
        }
    }

    private fun onParamSet(msg: MavlinkMessage<*>) {
        val payload = msg.payload as ParamSet
        if (payload.targetSystem() == systemID && (payload.targetComponent() == EnumValue.create(componentID).value() || payload.targetComponent() == 0)) {
            log.d("onParamSet")

            val param = mavlinkParameters[payload.paramId()]

            if (param == null || param.isExtParameter) {
                log.e("Could not set parameter value for unknown parameter " + payload.paramId())
                return
            }

            param.updateFromMavlinkParamSetValue(payload)
        }
    }

    private fun sendParameter(paramIndex: Int) {
        if (paramIndex >= mavlinkParameters.size) {
            log.e("Cannot find parameter with id $paramIndex")
            return
        }

        mavlinkParameters.values.firstOrNull {
            param -> param.index == paramIndex && !param.isExtParameter
        }?.sendMavlinkParamValue()
    }

    private fun sendParameter(paramKey: String) {
        val param = mavlinkParameters[paramKey]

        if (param == null) {
            log.w("Parameter '$paramKey' does not exist!")
            return
        }

        if (!param.isExtParameter) {
            param.sendMavlinkParamValue()
        }
    }

    //endregion
    //region: ext parameters
    private fun onParamExtRequestList(msg: MavlinkMessage<*>) {
        val payload = msg.payload as ParamExtRequestList

        if (payload.targetSystem() == systemID && (payload.targetComponent() == EnumValue.create(componentID).value() || payload.targetComponent() == 0)) {
            log.d("onParamExtRequestList")

            mavlinkParameters.keys.forEach { sendExtParameter(it) }
        }
    }

    private fun onParamExtRequestRead(msg: MavlinkMessage<*>) {
        val payload = msg.payload as ParamExtRequestRead

        if (payload.targetSystem() == systemID && (payload.targetComponent() == EnumValue.create(componentID).value() || payload.targetComponent() == 0)) {
            // param id -1 to use the param ID field as identifier (else the param id will be ignored)
            if (payload.paramIndex() == -1) {
                log.d("onParamExtRequestRead ${payload.paramId()}")
                sendExtParameter(payload.paramId())
            } else {
                log.d("onParamExtRequestRead ${payload.paramIndex()}")
                sendExtParameter(payload.paramIndex())
            }
        }
    }

    private fun onParamExtSet(msg: MavlinkMessage<*>) {
        val payload = msg.payload as ParamExtSet

        if (payload.targetSystem() == systemID && (payload.targetComponent() == EnumValue.create(componentID).value() || payload.targetComponent() == 0)) {
            log.d("onParamExtSet")

            val param = mavlinkParameters[payload.paramId()]

            if (param == null || !param.isExtParameter) {
                log.e("Could not set extended parameter value for unknown parameter " + payload.paramId())
                return
            }

            param.updateFromMavlinkParamExtSetValue(payload)
        }
    }

    private fun sendExtParameter(paramIndex: Int) {
        if (paramIndex >= mavlinkParameters.size) {
            log.e("Cannot find parameter with id $paramIndex")
            return
        }

        mavlinkParameters.values.firstOrNull { param ->
            param.index == paramIndex && param.isExtParameter
        }?.sendMavlinkParamValue()
    }

    private fun sendExtParameter(paramKey: String) {
        val param = mavlinkParameters[paramKey]

        if (param != null && param.isExtParameter) {
            param.sendMavlinkParamValue()
        } else {
            log.d("Could not find parameter $paramKey")
        }
    }

    //endregion
    //region: messages
    private fun toMavlinkPingMessage(targetSystem: Int, targetComponent: Int, timeUs: BigInteger, seq: Long): Ping {
        return Ping.builder()
            .targetSystem(targetSystem)
            .targetComponent(targetComponent)
            .timeUsec(timeUs)
            .seq(seq)
            .build()
    }

    private fun toMavlinkMessageIntervalMessage(messageId: Int): MessageInterval {
        val messageSender = messageSenders[messageId]

        val messageIntervalUs: Int
        if (messageSender == null) {
            log.e("Cannot set message interval for id $messageId because the message sender is not defined!")
            messageIntervalUs = 0
        } else {
            messageIntervalUs = messageSender.intervalMs * 1000
        }

        return MessageInterval.builder()
            .messageId(messageId)
            .intervalUs(messageIntervalUs)
            .build()
    }

    private fun toMavlinkHeartBeatMessage(): Heartbeat {
        val currentMavModeFlags = listToEnum(mavModeFlags)

        return Heartbeat.builder()
            .type(mavType)
            .autopilot(mavAutopilot)
            .systemStatus(mavState)
            .baseMode(currentMavModeFlags)
            .customMode(customMode.toLong())
            .build()
    }

    private fun toMavlinkSystemTimeMessage(): SystemTime {
        return SystemTime.builder()
            .timeUnixUsec(BigInteger.valueOf(System.currentTimeMillis() / 1000L))
            .timeBootMs(millisSinceBoot)
            .build()
    }

    private fun toMavlinkProtocolVersionMessage(): ProtocolVersion {
        return ProtocolVersion.builder()
            .version(200)
            .minVersion(100)
            .maxVersion(200)
            .specVersionHash(PROTOCOL_VERSION_HASH)
            .build()
    }

    //endregion
    //region: utils
    protected fun <MessageType> isMessageOfType(messageId: Int, messageType: Class<MessageType>): Boolean {
        val messageInfo = messageType.getAnnotation(MavlinkMessageInfo::class.java)
        if (messageInfo == null) {
            log.e("Cannot compare because $messageType is not a Mavlink message")
            return false
        }
        return messageId == messageInfo.id
    }

    fun onError(throwable: Throwable, standardResponse: TriFunction<MavCmd, Int, Int, CommandAck>, command: MavCmd, systemId: Int, componentId: Int) {
        sendMessage(standardResponse.apply(command, systemId, componentId))
        if (throwable is RxException) {
            if (throwable.logTag.isEmpty()) {
                handleError(throwable)
            } else {
                handleError(throwable.setLogger(log))
            }
        } else {
            handleError(throwable)
        }
    }

    fun onError(throwable: Throwable, standardResponse: Function<MavCmd, CommandAck>, command: MavCmd) {
        sendMessage(standardResponse.apply(command))
        if (throwable is RxException) {
            handleError(throwable.setLogger(log))
        } else {
            handleError(throwable)
        }
    }

    fun onError(throwable: Throwable, standardResponse: BiFunction<Int, Int, MissionAck>, systemId: Int, componentId: Int) {
        sendMessage(standardResponse.apply(systemId, componentId))
        if (throwable is RxException) {
            handleError(throwable.setLogger(log))
        } else {
            handleError(throwable)
        }
    }

    private inner class MessageSender<MessageType>(val messageSupplier: () -> MessageType?, private val messageId: Int, val defaultIntervalMs: Int) {
        var intervalMs: Int = defaultIntervalMs

        fun startMessageBroadcaster() {
            val messageBroadcaster = Consumer { _: Long ->
                try {
                    val msg = messageSupplier()
                    sendMessage(msg)
                } catch (_: Throwable) { }
            }
            addBroadcaster("message_$messageId", intervalMs, messageBroadcaster)
        }

        fun stopMessageBroadcaster() {
            stopBroadcaster("message_$messageId")
        }
    } //endregion

    companion object {
        private const val PROTOCOL_VERSION_TAG = "817c44d8b2b2b0db33d4e0fd4f65da9452b6cf63"
        val PROTOCOL_VERSION_HASH = PROTOCOL_VERSION_TAG.toByteArray().copyOfRange(0, 8)
    }
}
