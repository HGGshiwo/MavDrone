package nl.wur.mavdrone.widgets

import android.annotation.SuppressLint
import android.app.AlertDialog
import android.content.Context
import android.content.SharedPreferences
import android.content.res.ColorStateList
import android.graphics.drawable.Drawable
import android.text.InputFilter
import android.util.AttributeSet
import android.view.LayoutInflater
import android.view.View
import android.widget.CompoundButton
import android.widget.Toast
import androidx.annotation.DrawableRes
import androidx.annotation.StyleRes
import androidx.core.content.res.use
import dji.v5.utils.common.DisplayUtil
import dji.v5.ux.core.base.SchedulerProvider
import dji.v5.ux.core.base.widget.ConstraintLayoutWidget
import dji.v5.ux.core.extension.getColor
import dji.v5.ux.core.extension.getColorAndUse
import dji.v5.ux.core.extension.getColorStateListAndUse
import dji.v5.ux.core.extension.getDimensionAndUse
import dji.v5.ux.core.extension.getDrawable
import dji.v5.ux.core.extension.getDrawableAndUse
import dji.v5.ux.core.extension.getResourceIdAndUse
import dji.v5.ux.core.extension.getString
import dji.v5.ux.core.extension.textColor
import dji.v5.ux.core.extension.textColorStateList
import dji.v5.ux.core.util.EditTextNumberInputFilter
import io.reactivex.rxjava3.core.Flowable
import io.reactivex.rxjava3.disposables.CompositeDisposable
import io.reactivex.rxjava3.processors.PublishProcessor
import io.reactivex.rxjava3.schedulers.Schedulers
import nl.wur.mavdrone.MavManager
import nl.wur.mavdrone.MavManager.MavlinkStatus
import nl.wur.mavdrone.R
import nl.wur.mavdrone.databinding.WidgetMavinkControlBinding
import nl.wur.mavdrone.utils.Logger
import nl.wur.mavdrone.utils.RxException
import nl.wur.mavdrone.widgets.MavControlWidget.ModelState.GCSConnected
import nl.wur.mavdrone.widgets.MavControlWidget.ModelState.ServerRunning
import nl.wur.mavdrone.widgets.MavControlWidget.UIState.MavlinkSwitchTap
import nl.wur.mavdrone.widgets.MavControlWidget.UIState.VisibilityUpdated


open class MavControlWidget @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : ConstraintLayoutWidget<Any>(
    context,
    attrs,
    defStyleAttr
), MavManager.MavlinkStatusListener {

    //region Fields
    private lateinit var binding: WidgetMavinkControlBinding

    private var shouldReactToCheckChange = false
    private var lastMavlinkStatus: MavlinkStatus? = null

    private val uiUpdateStateProcessor: PublishProcessor<UIState> = PublishProcessor.create()
    private var compositeDisposable: CompositeDisposable? = null

    private val androidPreferences: SharedPreferences
        get() = context.getSharedPreferences("default_preferences", Context.MODE_PRIVATE)

    //endregion

    //region: init
    var mavlinkConnectedIcon: Drawable? = getDrawable(R.drawable.ic_mavlink_connected)
        set(value) {
            field = value
            checkAndUpdateState()
        }

    var mavlinkServerRunningIcon: Drawable? = getDrawable(R.drawable.ic_mavlink_server_running)
        set(value) {
            field = value
            checkAndUpdateState()
        }

    var mavlinkInactiveIcon: Drawable? = getDrawable(R.drawable.ic_mavlink_inactive)
        set(value) {
            field = value
            checkAndUpdateState()
        }

    var titleBackground: Drawable?
        get() = binding.textviewMavlinkTitle.background
        set(value) {
            binding.textviewMavlinkTitle.background = value
        }

    var titleTextColors: ColorStateList?
        get() = binding.textviewMavlinkTitle.textColorStateList
        set(value) {
            binding.textviewMavlinkTitle.textColorStateList = value
        }

    var titleTextColor: Int
        get() = binding.textviewMavlinkTitle.textColor
        set(value) {
            binding.textviewMavlinkTitle.textColor = value
        }

    var titleTextSize: Float
        get() = binding.textviewMavlinkTitle.textSize
        set(textSize) {
            binding.textviewMavlinkTitle.textSize = textSize
        }

    var valueTextColors: ColorStateList?
        get() = binding.textviewMavlinkPortValue.textColorStateList
        set(value) {
            listOf(
                binding.textviewMavlinkPortValue,
                binding.textviewMavlinkSystemIdValue,
                binding.textviewMavlinkMaxMissedHeartbeatsValue,
                binding.textviewMavlinkExpectedHeartbeatIntervalValue,
                binding.textviewStreamPortValue,
                binding.textviewStreamUsernameValue,
                binding.textviewStreamPasswordValue,
            ).forEach { it.textColorStateList = value }
        }

    var valueTextColor: Int
        get() = binding.textviewMavlinkPortValue.textColor
        set(value) {
            listOf(
                binding.textviewMavlinkPortValue,
                binding.textviewMavlinkSystemIdValue,
                binding.textviewMavlinkMaxMissedHeartbeatsValue,
                binding.textviewMavlinkExpectedHeartbeatIntervalValue,
                binding.textviewStreamPortValue,
                binding.textviewStreamUsernameValue,
                binding.textviewStreamPasswordValue,
            ).forEach { it.textColor = value }
        }

    var valueTextSize: Float
        get() = binding.textviewMavlinkPortValue.textSize
        set(value) {
            listOf(
                binding.textviewMavlinkPortValue,
                binding.textviewMavlinkSystemIdValue,
                binding.textviewMavlinkMaxMissedHeartbeatsValue,
                binding.textviewMavlinkExpectedHeartbeatIntervalValue,
                binding.textviewStreamPortValue,
                binding.textviewStreamUsernameValue,
                binding.textviewStreamPasswordValue,
            ).forEach { it.textSize = value }
        }

    var valueBackground: Drawable?
        get() = binding.textviewMavlinkPortValue.background
        set(value) {
            listOf(
                binding.textviewMavlinkPortValue,
                binding.textviewMavlinkSystemIdValue,
                binding.textviewMavlinkMaxMissedHeartbeatsValue,
                binding.textviewMavlinkExpectedHeartbeatIntervalValue,
                binding.textviewStreamPortValue,
                binding.textviewStreamUsernameValue,
                binding.textviewStreamPasswordValue,
            ).forEach { it.background = value }
        }

    var labelTextSize: Float
        get() = binding.textviewMavlinkPortLabel.textSize
        set(value) {
            listOf(
                binding.textviewMavlinkPortLabel,
                binding.textviewMavlinkSystemIdLabel,
                binding.textviewMavlinkMaxMissedHeartbeatsLabel,
                binding.textviewMavlinkExpectedHeartbeatIntervalLabel,
                binding.textviewStreamPortLabel,
                binding.textviewStreamUsernameLabel,
                binding.textviewStreamPasswordLabel,
                binding.textviewStreamAutosaveLabel,
            ).forEach { it.textSize = value }
        }

    var labelTextColors: ColorStateList?
        get() = binding.textviewMavlinkPortLabel.textColorStateList
        set(value) {
            listOf(
                binding.textviewMavlinkPortLabel,
                binding.textviewMavlinkSystemIdLabel,
                binding.textviewMavlinkMaxMissedHeartbeatsLabel,
                binding.textviewMavlinkExpectedHeartbeatIntervalLabel,
                binding.textviewStreamPortLabel,
                binding.textviewStreamUsernameLabel,
                binding.textviewStreamPasswordLabel,
                binding.textviewStreamAutosaveLabel,
            ).forEach { it.textColorStateList = value }
        }

    var labelTextColor: Int
        get() = binding.textviewMavlinkPortLabel.textColor
        set(value) {
            listOf(
                binding.textviewMavlinkPortLabel,
                binding.textviewMavlinkSystemIdLabel,
                binding.textviewMavlinkMaxMissedHeartbeatsLabel,
                binding.textviewMavlinkExpectedHeartbeatIntervalLabel,
                binding.textviewStreamPortLabel,
                binding.textviewStreamUsernameLabel,
                binding.textviewStreamPasswordLabel,
                binding.textviewStreamAutosaveLabel,
            ).forEach { it.textColor = value }
        }

    var labelBackground: Drawable?
        get() = binding.textviewMavlinkPortLabel.background
        set(value) {
            listOf(
                binding.textviewMavlinkPortLabel,
                binding.textviewMavlinkSystemIdLabel,
                binding.textviewMavlinkMaxMissedHeartbeatsLabel,
                binding.textviewMavlinkExpectedHeartbeatIntervalLabel,
                binding.textviewStreamPortLabel,
                binding.textviewStreamUsernameLabel,
                binding.textviewStreamPasswordLabel,
                binding.textviewStreamAutosaveLabel,
            ).forEach { it.background = value }
        }

    var inputTextSize: Float
        get() = binding.editTextMavlinkPort.textSize
        set(value) {
            listOf(
                binding.editTextMavlinkPort,
                binding.editTextMavlinkSystemId,
                binding.editTextMavlinkMaxMissedHeartbeats,
                binding.editTextMavlinkExpectedHeartbeatInterval,
                binding.editTextStreamPort,
                binding.editTextStreamUsername,
                binding.editTextStreamPassword,
            ).forEach { it.textSize = value }
        }

    var inputTextColors: ColorStateList?
        get() = binding.editTextMavlinkPort.textColorStateList
        set(value) {
            listOf(
                binding.editTextMavlinkPort,
                binding.editTextMavlinkSystemId,
                binding.editTextMavlinkMaxMissedHeartbeats,
                binding.editTextMavlinkExpectedHeartbeatInterval,
                binding.editTextStreamPort,
                binding.editTextStreamUsername,
                binding.editTextStreamPassword,
            ).forEach { it.textColorStateList = value }
        }

    var inputTextColor: Int
        get() = binding.editTextMavlinkPort.textColor
        set(value) {
            listOf(
                binding.editTextMavlinkPort,
                binding.editTextMavlinkSystemId,
                binding.editTextMavlinkMaxMissedHeartbeats,
                binding.editTextMavlinkExpectedHeartbeatInterval,
                binding.editTextStreamPort,
                binding.editTextStreamUsername,
                binding.editTextStreamPassword,
            ).forEach { it.textColor = value }
        }

    var inputBackground: Drawable?
        get() = binding.editTextMavlinkPort.background
        set(value) {
            listOf(
                binding.editTextMavlinkPort,
                binding.editTextMavlinkSystemId,
                binding.editTextMavlinkMaxMissedHeartbeats,
                binding.editTextMavlinkExpectedHeartbeatInterval,
                binding.editTextStreamPort,
                binding.editTextStreamUsername,
                binding.editTextStreamPassword,
            ).forEach { it.background = value }
        }

    var buttonTextSize: Float
        get() = binding.textviewSave.textSize
        set(textSize) {
            binding.textviewSave.textSize = textSize
        }

    var buttonTextColors: ColorStateList?
        get() = binding.textviewSave.textColorStateList
        set(value) {
            binding.textviewSave.textColorStateList = value
        }

    var buttonTextColor: Int
        get() = binding.textviewSave.textColor
        set(value) {
            binding.textviewSave.textColor = value
        }

    var buttonBackground: Drawable?
        get() = binding.textviewSave.background
        set(value) {
            binding.textviewSave.background = value
        }

    var headerTextSize: Float
        get() = binding.textviewConnectionSectionHeader.textSize
        set(value) {
            listOf(
                binding.textviewConnectionSectionHeader,
                binding.textviewStreamSectionHeader,
            ).forEach { it.textSize = value }
        }

    var headerTextColors: ColorStateList?
        get() = binding.textviewConnectionSectionHeader.textColorStateList
        set(value) {
            listOf(
                binding.textviewConnectionSectionHeader,
                binding.textviewStreamSectionHeader,
            ).forEach { it.textColorStateList = value }
        }

    var headerBackground: Drawable?
        get() = binding.textviewConnectionSectionHeader.background
        set(value) {
            listOf(
                binding.textviewConnectionSectionHeader,
                binding.textviewStreamSectionHeader,
            ).forEach { it.background = value }
        }

    var headerTextColor: Int
        get() = binding.textviewConnectionSectionHeader.textColor
        set(value) {
            listOf(
                binding.textviewConnectionSectionHeader,
                binding.textviewStreamSectionHeader,
            ).forEach { it.textColor = value }
        }

    //endregion

    //region Lifecycle
    override fun initView(context: Context, attrs: AttributeSet?, defStyleAttr: Int) {
        binding = WidgetMavinkControlBinding.inflate(LayoutInflater.from(context), this)
    }

    init {
        setBackgroundResource(R.drawable.uxsdk_background_black_rectangle)
        initViewElements()
        attrs?.let { initAttributes(context, it) }
    }

    override fun reactToModelChanges() {
    }

    override fun getIdealDimensionRatioString(): String {
        return getString(R.string.mavlink_control_ratio)
    }

    override fun onAttachedToWindow() {
        super.onAttachedToWindow()
        compositeDisposable = CompositeDisposable()
        if (!isInEditMode) {
            MavManager.getInstance().addMavlinkStatusListener(this)
        }
    }

    @SuppressLint("CheckResult")
    override fun onDetachedFromWindow() {
        compositeDisposable?.dispose()
        if (!isInEditMode) {
            MavManager.getInstance().removeMavlinkStatusListener(this)
        }

        if (lastMavlinkStatus?.isServerRunning == true) {
            MavManager.getInstance().stopServer(context)
                .subscribe(
                    { log.i("Stopped MavlinkServer") },
                    { throwable -> log.e("Could not stop MavlinkServer: $throwable") }
                )
        }

        super.onDetachedFromWindow()
    }

    override fun onVisibilityChanged(changedView: View, visibility: Int) {
        super.onVisibilityChanged(changedView, visibility)
        uiUpdateStateProcessor.onNext(VisibilityUpdated(visibility == View.VISIBLE))
    }

    override fun onStatusChanged(status: MavlinkStatus) {
        lastMavlinkStatus = status
        checkAndUpdateState()
    }

    override fun onError(error: RxException) {
        log.e("Exception in subscribing MavlinkThreadCommunication: $error")
    }

    @SuppressLint("Recycle")
    private fun initAttributes(context: Context, attrs: AttributeSet) {
        context.obtainStyledAttributes(attrs, R.styleable.MavlinkControlWidget).use { typedArray ->
            typedArray.getDrawableAndUse(R.styleable.MavlinkControlWidget_mavlinkConnectedIcon) {
                mavlinkConnectedIcon = it
            }
            typedArray.getDrawableAndUse(R.styleable.MavlinkControlWidget_mavlinkServerRunningDrawable) {
                mavlinkServerRunningIcon = it
            }
            typedArray.getDrawableAndUse(R.styleable.MavlinkControlWidget_mavlinkInactiveDrawable) {
                mavlinkInactiveIcon = it
            }
            typedArray.getDrawableAndUse(R.styleable.MavlinkControlWidget_buttonBackground) {
                buttonBackground = it
            }
            typedArray.getColorStateListAndUse(R.styleable.MavlinkControlWidget_buttonTextColor) {
                buttonTextColors = it
            }
            typedArray.getColorAndUse(R.styleable.MavlinkControlWidget_buttonTextColor) {
                buttonTextColor = it
            }
            typedArray.getDimensionAndUse(R.styleable.MavlinkControlWidget_buttonTextSize) {
                buttonTextSize = DisplayUtil.pxToSp(context, it)
            }
            typedArray.getResourceIdAndUse(R.styleable.MavlinkControlWidget_buttonTextAppearance) {
                setButtonTextAppearance(it)
            }
            typedArray.getDrawableAndUse(R.styleable.MavlinkControlWidget_labelsBackground) {
                labelBackground = it
            }
            typedArray.getColorStateListAndUse(R.styleable.MavlinkControlWidget_labelsTextColor) {
                labelTextColors = it
            }
            typedArray.getColorAndUse(R.styleable.MavlinkControlWidget_labelsTextColor) {
                labelTextColor = it
            }
            typedArray.getDimensionAndUse(R.styleable.MavlinkControlWidget_labelsTextSize) {
                labelTextSize = it
            }
            typedArray.getResourceIdAndUse(R.styleable.MavlinkControlWidget_labelsTextAppearance) {
                setLabelTextAppearance(it)
            }
            typedArray.getDrawableAndUse(R.styleable.MavlinkControlWidget_inputBackground) {
                inputBackground = it
            }
            typedArray.getColorStateListAndUse(R.styleable.MavlinkControlWidget_inputTextColor) {
                inputTextColors = it
            }
            typedArray.getColorAndUse(R.styleable.MavlinkControlWidget_inputTextColor) {
                inputTextColor = it
            }
            typedArray.getDimensionAndUse(R.styleable.MavlinkControlWidget_inputTextSize) {
                inputTextSize = DisplayUtil.pxToSp(context, it)
            }
            typedArray.getResourceIdAndUse(R.styleable.MavlinkControlWidget_inputTextAppearance) {
                setInputTextAppearance(it)
            }
            typedArray.getDrawableAndUse(R.styleable.MavlinkControlWidget_valueBackground) {
                valueBackground = it
            }
            typedArray.getColorStateListAndUse(R.styleable.MavlinkControlWidget_valueTextColor) {
                valueTextColors = it
            }

            typedArray.getColorAndUse(R.styleable.MavlinkControlWidget_valueTextColor) {
                valueTextColor = it
            }
            typedArray.getDimensionAndUse(R.styleable.MavlinkControlWidget_valueTextSize) {
                valueTextSize = DisplayUtil.pxToSp(context, it)
            }
            typedArray.getResourceIdAndUse(R.styleable.MavlinkControlWidget_valueTextAppearance) {
                setValueTextAppearance(it)
            }
            typedArray.getDrawableAndUse(R.styleable.MavlinkControlWidget_headerBackground) {
                headerBackground = it
            }
            typedArray.getColorStateListAndUse(R.styleable.MavlinkControlWidget_headerTextColor) {
                headerTextColors = it
            }
            typedArray.getColorAndUse(R.styleable.MavlinkControlWidget_headerTextColor) {
                headerTextColor = it
            }
            typedArray.getDimensionAndUse(R.styleable.MavlinkControlWidget_headerTextSize) {
                headerTextSize = DisplayUtil.pxToSp(context, it)
            }
            typedArray.getResourceIdAndUse(R.styleable.MavlinkControlWidget_headerTextAppearance) {
                setHeaderTextAppearance(it)
            }
            typedArray.getDrawableAndUse(R.styleable.MavlinkControlWidget_widgetTitleBackground) {
                titleBackground = it
            }
            typedArray.getColorStateListAndUse(R.styleable.MavlinkControlWidget_widgetTitleTextColor) {
                titleTextColors = it
            }
            typedArray.getColorAndUse(R.styleable.MavlinkControlWidget_widgetTitleTextColor) {
                titleTextColor = it
            }
            typedArray.getDimensionAndUse(R.styleable.MavlinkControlWidget_widgetTitleTextSize) {
                titleTextSize = DisplayUtil.pxToSp(context, it)
            }
            typedArray.getResourceIdAndUse(R.styleable.MavlinkControlWidget_widgetTitleTextAppearance) {
                setTitleTextAppearance(it)
            }
        }
    }


    private fun updateUI(status: MavlinkStatus) {
        widgetStateDataProcessor.onNext(GCSConnected(status.gcsConnected))
        widgetStateDataProcessor.onNext(ServerRunning(status.isServerRunning))
        shouldReactToCheckChange = false
        if (status.isServerRunning) {
            updateWidgetToStartedState()
        } else {
            updateWidgetToStoppedState()
        }

        binding.textviewDetailsHostIp.text = status.serverIPAddress
        binding.textviewDetailsGcsIps.text = status.gcsIPAddress ?: getString(R.string.mavlink_widget_null_string)
        binding.textviewDetailsWaypointMission.text = status.loadedWaypointMission?: getString(R.string.mavlink_widget_null_string)
        binding.textviewDetailsConnectedComponents.text =
            if (status.connectedComponents.isEmpty()) getString(R.string.mavlink_widget_null_string)
            else status.connectedComponents.joinToString(separator = "\n") { "SystemID=${it.systemId}, ComponentID=${it.componentID}" }
        binding.textviewDetailsGeofenceMission.text = if (status.geoFenceMissionLoaded) "Yes" else "No"
        binding.textviewDetailsStatus.text = status.missionState.toString().replace("_", " ")

        val icon = when {
            status.gcsConnected -> mavlinkConnectedIcon
            status.isServerRunning -> mavlinkServerRunningIcon
            else -> mavlinkInactiveIcon
        }
        binding.textviewMavlinkTitle.setCompoundDrawablesWithIntrinsicBounds(icon, null, null, null)

        shouldReactToCheckChange = true
    }

    private fun checkAndUpdateState() {
        if (!isInEditMode) {
            lastMavlinkStatus?.let { updateUI(it) }
        }
    }

    private fun initViewElements() {
        binding.editTextMavlinkPort.filters = arrayOf<InputFilter>(EditTextNumberInputFilter("1", "49151"))
        binding.editTextMavlinkSystemId.filters = arrayOf<InputFilter>(EditTextNumberInputFilter("1", "255"))
        binding.editTextMavlinkMaxMissedHeartbeats.filters = arrayOf<InputFilter>(EditTextNumberInputFilter("0", "10"))
        binding.editTextMavlinkExpectedHeartbeatInterval.filters = arrayOf<InputFilter>(EditTextNumberInputFilter("100", "10000"))
        binding.editTextMavlinkPort.filters = arrayOf<InputFilter>(EditTextNumberInputFilter("1", "49151"))

        binding.textviewSave.setOnClickListener { saveSettings() }

        binding.switchMavlink.setOnCheckedChangeListener { _: CompoundButton?, isChecked: Boolean -> handleSwitchChange(isChecked) }
        valueTextColor = getColor(R.color.uxsdk_blue)
        titleTextColor = getColor(R.color.uxsdk_white)
        headerTextColor = getColor(R.color.uxsdk_white)
        inputTextColor = getColor(R.color.uxsdk_black)

        // Load settings
        binding.editTextMavlinkPort.setText(androidPreferences.getInt("SYS_MAV_PORT", MavManager.ConnectionInformation().port).toString())
        binding.editTextMavlinkSystemId.setText(androidPreferences.getInt("SYS_MAV_ID", MavManager.ConnectionInformation().systemId).toString())
        binding.editTextMavlinkMaxMissedHeartbeats.setText(androidPreferences.getInt("SYS_GCS_HB_MISS", MavManager.ConnectionInformation().maxMissedHeartBeats).toString())
        binding.editTextMavlinkExpectedHeartbeatInterval.setText(androidPreferences.getInt("SYS_GCS_HB_INT", MavManager.ConnectionInformation().expectedHeartBeatInterval).toString())

        binding.editTextStreamPort.setText(androidPreferences.getInt("STREAM_PORT", MavManager.StreamInformation().port).toString())
        binding.editTextStreamUsername.setText(androidPreferences.getString("STREAM_USER", MavManager.StreamInformation().user))
        binding.editTextStreamPassword.setText(androidPreferences.getString("STREAM_PASSWORD", MavManager.StreamInformation().password))
        binding.switchStreamAutostart.isChecked = androidPreferences.getBoolean("STREAM_START", MavManager.StreamInformation().autoStart)
    }

    private fun handleSwitchChange(isChecked: Boolean) {
        uiUpdateStateProcessor.onNext(MavlinkSwitchTap(isChecked))
        if (shouldReactToCheckChange) {
            if (isChecked) {
                log.i("Start Mavlink")
                startMavlink()
            } else {
                log.i("Stopping Mavlink")
                stopMavlink()
            }
        }
    }

    private fun saveSettings() {
        val tmpConnectionInformation = connectionInformation
        val tmpStreamInformation = streamInformation

        if (tmpConnectionInformation !=  null && tmpStreamInformation != null) {
            val editor = androidPreferences.edit()
            editor.putInt("SYS_MAV_PORT", tmpConnectionInformation.port)
            editor.putInt("SYS_MAV_ID", tmpConnectionInformation.systemId)
            editor.putInt("SYS_GCS_HB_MISS", tmpConnectionInformation.maxMissedHeartBeats)
            editor.putInt("SYS_GCS_HB_INT", tmpConnectionInformation.expectedHeartBeatInterval)
            editor.putInt("STREAM_PORT", tmpStreamInformation.port)
            editor.putString("STREAM_USER", tmpStreamInformation.user)
            editor.putString("STREAM_PASSWORD", tmpStreamInformation.password)
            editor.putBoolean("STREAM_START", tmpStreamInformation.autoStart)
            editor.apply()

            AlertDialog.Builder(context, R.style.DialogTheme)
                .setMessage(R.string.mavlink_widget_save_success)
                .setPositiveButton("OK") { dialog, _ -> dialog.dismiss() }
                .show()
        } else {
            Toast.makeText(
                context,
                getString(R.string.mavlink_widget_input_error),
                Toast.LENGTH_SHORT
            ).show()
        }
    }

    private fun startMavlink() {
        val tmpConnectionInformation = connectionInformation
        val tmpStreamInformation = streamInformation

        if (tmpConnectionInformation !=  null && tmpStreamInformation != null) {
            setMavlinkStatus(true)

            MavManager.getInstance().connectionInformation = tmpConnectionInformation
            MavManager.getInstance().streamInformation = tmpStreamInformation

            addDisposable(MavManager.getInstance()
                .startServer(context)
                .subscribeOn(Schedulers.io())
                .observeOn(SchedulerProvider.ui())
                .subscribe(
                    {
                        reactToModelChanges()
                    },
                    { throwable ->
                        setMavlinkStatus(false)
                        log.e("Error starting Mavlink: ${throwable.message}", throwable)
                    }
                )
            )
        } else {
            setMavlinkStatus(false)
            Toast.makeText(
                context,
                getString(R.string.mavlink_widget_input_error),
                Toast.LENGTH_SHORT
            ).show()
        }
    }

    private fun stopMavlink() {
        addDisposable(MavManager.getInstance()
            .stopServer(context)
            .subscribeOn(SchedulerProvider.ui())
            .subscribe(
                { log.i("Stopped Mavlink") },
                { throwable -> log.e("Error stopping Mavlink: ${throwable.message}", throwable) }
            )
        )
    }

    private fun setMavlinkStatus(mavlinkActive: Boolean) {
        shouldReactToCheckChange = false
        if (mavlinkActive) {
            updateWidgetToStartedState()
        } else {
            updateWidgetToStoppedState()
        }
        shouldReactToCheckChange = true
    }

    private fun updateWidgetToStartedState() {
        binding.switchMavlink.isChecked = true
        binding.switchStreamAutostart.isEnabled = false

        binding.textviewMavlinkPortValue.text = when {
            binding.editTextMavlinkPort.text.toString().isNotEmpty() -> {
                binding.editTextMavlinkPort.text.toString()
            }
            else -> {
                getString(R.string.mavlink_widget_null_string)
            }
        }
        binding.textviewMavlinkSystemIdValue.text = when {
            binding.editTextMavlinkSystemId.text.toString().isNotEmpty() -> {
                binding.editTextMavlinkSystemId.text.toString()
            }
            else -> {
                getString(R.string.mavlink_widget_null_string)
            }
        }
        binding.textviewMavlinkMaxMissedHeartbeatsValue.text = when {
            binding.editTextMavlinkMaxMissedHeartbeats.text.toString().isNotEmpty() -> {
                binding.editTextMavlinkMaxMissedHeartbeats.text.toString()
            }
            else -> {
                getString(R.string.mavlink_widget_null_string)
            }
        }

        binding.textviewMavlinkExpectedHeartbeatIntervalValue.text = when {
            binding.editTextMavlinkExpectedHeartbeatInterval.text.toString().isNotEmpty() -> {
                binding.editTextMavlinkExpectedHeartbeatInterval.text.toString()
            }
            else -> {
                getString(R.string.mavlink_widget_null_string)
            }
        }

        binding.textviewStreamPortValue.text = when {
            binding.editTextStreamPort.text.toString().isNotEmpty() -> {
                binding.editTextStreamPort.text.toString()
            }
            else -> {
                getString(R.string.mavlink_widget_null_string)
            }
        }

        binding.textviewStreamUsernameValue.text = when {
            binding.editTextStreamUsername.text.toString().isNotEmpty() -> {
                binding.editTextStreamUsername.text.toString()
            }
            else -> {
                getString(R.string.mavlink_widget_null_string)
            }
        }

        binding.textviewStreamPasswordValue.text = when {
            binding.editTextStreamPassword.text.toString().isNotEmpty() -> {
                binding.editTextStreamPassword.text.toString()
            }
            else -> {
                getString(R.string.mavlink_widget_null_string)
            }
        }

        listOf(
            binding.editTextMavlinkPort,
            binding.editTextMavlinkSystemId,
            binding.editTextMavlinkMaxMissedHeartbeats,
            binding.editTextMavlinkExpectedHeartbeatInterval,
            binding.editTextStreamPort,
            binding.editTextStreamUsername,
            binding.editTextStreamPassword,
        ).forEach { it.visibility = View.INVISIBLE }

        listOf(
            binding.tableLayoutMavlinkConnectionDetails,
            binding.mavlinkConnectionDetailsFooter,
            binding.textviewMavlinkPortValue,
            binding.textviewMavlinkSystemIdValue,
            binding.textviewMavlinkMaxMissedHeartbeatsValue,
            binding.textviewMavlinkExpectedHeartbeatIntervalValue,
            binding.textviewStreamPortValue,
            binding.textviewStreamUsernameValue,
            binding.textviewStreamPasswordValue,
        ).forEach { it.visibility = View.VISIBLE }
    }

    private fun updateWidgetToStoppedState() {
        binding.switchMavlink.isChecked = false
        binding.switchStreamAutostart.isEnabled = true

        listOf(
            binding.editTextMavlinkPort,
            binding.editTextMavlinkSystemId,
            binding.editTextMavlinkMaxMissedHeartbeats,
            binding.editTextMavlinkExpectedHeartbeatInterval,
            binding.editTextStreamPort,
            binding.editTextStreamUsername,
            binding.editTextStreamPassword,
        ).forEach { it.visibility = View.VISIBLE }

        listOf(
            binding.textviewMavlinkPortValue,
            binding.textviewMavlinkSystemIdValue,
            binding.textviewMavlinkMaxMissedHeartbeatsValue,
            binding.textviewMavlinkExpectedHeartbeatIntervalValue,
            binding.textviewStreamPortValue,
            binding.textviewStreamUsernameValue,
            binding.textviewStreamPasswordValue,
        ).forEach { it.visibility = View.INVISIBLE }

        listOf(
            binding.tableLayoutMavlinkConnectionDetails,
            binding.mavlinkConnectionDetailsFooter,
        ).forEach { it.visibility = View.GONE }
    }

    private val connectionInformation : MavManager.ConnectionInformation?
        get() {
            if (binding.editTextMavlinkPort.text.toString().isNotEmpty()
                && binding.editTextMavlinkSystemId.text.toString().isNotEmpty()
                && binding.editTextMavlinkMaxMissedHeartbeats.text.toString().isNotEmpty()
                && binding.editTextMavlinkExpectedHeartbeatInterval.text.toString().isNotEmpty()) {
                val port = binding.editTextMavlinkPort.text.toString().toInt()
                val systemId = binding.editTextMavlinkSystemId.text.toString().toInt()
                val maxMissedHeartBeats = binding.editTextMavlinkMaxMissedHeartbeats.text.toString().toInt()
                val expHeartBeatInterval = binding.editTextMavlinkExpectedHeartbeatInterval.text.toString().toInt()
                return MavManager.ConnectionInformation(port, systemId, maxMissedHeartBeats, expHeartBeatInterval)
            }
            return null
        }

    private val streamInformation : MavManager.StreamInformation?
        get() {
            if (binding.editTextStreamPort.text.toString().isNotEmpty()
                && binding.editTextStreamUsername.text.toString().isNotEmpty()
                && binding.editTextStreamPassword.text.toString().isNotEmpty()) {
                val port = binding.editTextStreamPort.text.toString().toInt()
                val username = binding.editTextStreamUsername.text.toString()
                val password = binding.editTextStreamPassword.text.toString()
                val autostart = binding.switchStreamAutostart.isChecked
                return MavManager.StreamInformation(port, username, password, autostart)
            }
            return null
        }

    override fun setEnabled(enabled: Boolean) {
        super.setEnabled(enabled)
        binding.switchMavlink.isEnabled = enabled
    }
    //endregion

    //region customizations
    fun setTitleBackground(@DrawableRes resourceId: Int) {
        titleBackground = getDrawable(resourceId)
    }

    fun setMavlinkConnectedIcon(@DrawableRes resourceId: Int) {
        mavlinkConnectedIcon = getDrawable(resourceId)
    }

    fun setMavlinkServerRunningIcon(@DrawableRes resourceId: Int) {
        mavlinkServerRunningIcon = getDrawable(resourceId)
    }

    fun setMavlinkInactiveIcon(@DrawableRes resourceId: Int) {
        mavlinkInactiveIcon = getDrawable(resourceId)
    }

    fun setTitleTextAppearance(@StyleRes textAppearance: Int) {
        binding.textviewMavlinkTitle.setTextAppearance(textAppearance)
    }

    fun setValueTextAppearance(@StyleRes textAppearance: Int) {
        listOf(
            binding.textviewMavlinkPortValue,
            binding.textviewMavlinkSystemIdValue,
            binding.textviewMavlinkMaxMissedHeartbeatsValue,
            binding.textviewMavlinkExpectedHeartbeatIntervalValue,
            binding.textviewStreamPortValue,
            binding.textviewStreamUsernameValue,
            binding.textviewStreamPasswordValue
        ).forEach { it.setTextAppearance(textAppearance) }
    }

    fun setValueBackground(@DrawableRes resourceId: Int) {
        valueBackground = getDrawable(resourceId)
    }

    fun setLabelTextAppearance(@StyleRes textAppearance: Int) {
        listOf(
            binding.textviewMavlinkPortLabel,
            binding.textviewMavlinkSystemIdLabel,
            binding.textviewMavlinkMaxMissedHeartbeatsLabel,
            binding.textviewMavlinkExpectedHeartbeatIntervalLabel,
            binding.textviewStreamPortLabel,
            binding.textviewStreamUsernameLabel,
            binding.textviewStreamPasswordLabel,
            binding.textviewStreamAutosaveLabel,
        ).forEach { it.setTextAppearance(textAppearance) }
    }

    fun setLabelBackground(@DrawableRes resourceId: Int) {
        labelBackground = getDrawable(resourceId)
    }

    fun setInputTextAppearance(@StyleRes textAppearance: Int) {
        listOf(
            binding.editTextMavlinkPort,
            binding.editTextMavlinkSystemId,
            binding.editTextMavlinkMaxMissedHeartbeats,
            binding.editTextMavlinkExpectedHeartbeatInterval,
            binding.editTextStreamPort,
            binding.editTextStreamUsername,
            binding.editTextStreamPassword,
        ).forEach { it.setTextAppearance(textAppearance) }
    }

    fun setInputBackground(@DrawableRes resourceId: Int) {
        inputBackground = getDrawable(resourceId)
    }

    fun setButtonTextAppearance(@StyleRes textAppearance: Int) {
        binding.textviewSave.setTextAppearance(textAppearance)
    }

    fun setButtonBackground(@DrawableRes resourceId: Int) {
        buttonBackground = getDrawable(resourceId)
    }

    fun setHeaderTextAppearance(@StyleRes textAppearance: Int) {
        listOf(
            binding.textviewConnectionSectionHeader,
            binding.textviewStreamSectionHeader,
        ).forEach { it.setTextAppearance(textAppearance) }
    }

    fun setHeaderBackground(@DrawableRes resourceId: Int) {
        headerBackground = getDrawable(resourceId)
    }
    //endregion

    //region Hooks
    fun getUIStateUpdates(): Flowable<UIState> {
        return uiUpdateStateProcessor.onBackpressureBuffer()
    }

    sealed class ModelState {
        data class GCSConnected(val isConnected: Boolean) : ModelState()
        data class ServerRunning(val isRunning: Boolean) : ModelState()
    }

    sealed class UIState {
        data class VisibilityUpdated(val isVisible: Boolean) : UIState()
        data class MavlinkSwitchTap(val isChecked: Boolean) : UIState()
    }
    //endregion

    companion object {
        private val log = Logger("MavControlWidget")
    }

    //endregion
}