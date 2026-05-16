package nl.wur.mavdrone.widgets

import android.annotation.SuppressLint
import android.app.AlertDialog
import android.content.Context
import android.content.SharedPreferences
import android.content.res.ColorStateList
import android.graphics.drawable.Drawable
import android.util.AttributeSet
import android.view.LayoutInflater
import android.view.View
import android.widget.ProgressBar
import androidx.annotation.DrawableRes
import androidx.core.content.res.use
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.RemoteControllerKey
import dji.v5.manager.KeyManager
import dji.v5.manager.aircraft.waypoint3.WaypointMissionManager
import dji.v5.ux.core.base.widget.ConstraintLayoutWidget
import dji.v5.ux.core.extension.getColor
import dji.v5.ux.core.extension.getColorAndUse
import dji.v5.ux.core.extension.getDrawable
import dji.v5.ux.core.extension.getDrawableAndUse
import io.reactivex.rxjava3.android.schedulers.AndroidSchedulers
import io.reactivex.rxjava3.core.Observable
import io.reactivex.rxjava3.disposables.CompositeDisposable
import io.reactivex.rxjava3.schedulers.Schedulers
import nl.wur.mavdrone.MavManager
import nl.wur.mavdrone.R
import nl.wur.mavdrone.databinding.WidgetMissionControlBinding
import nl.wur.mavdrone.utils.CurrentMissionState
import nl.wur.mavdrone.utils.Logger
import nl.wur.mavdrone.utils.RXUtils
import nl.wur.mavdrone.utils.RXUtils.Companion.djiCompletableFunction
import nl.wur.mavdrone.utils.RxException
import java.util.concurrent.TimeUnit


class MissionControlWidget @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : ConstraintLayoutWidget<Any>(
    context,
    attrs,
    defStyleAttr
), View.OnClickListener, View.OnLongClickListener, MavManager.MavlinkStatusListener, MavManager.MavlinkGUIListener {
    //region Fields
    private lateinit var binding: WidgetMissionControlBinding

    var enableColor: Int = getColor(R.color.enable)
    var disableColor: Int = getColor(R.color.disable)

    var playDrawable: Drawable? = getDrawable(R.drawable.play)
    var pauseDrawable: Drawable? = getDrawable(R.drawable.pause)
    var resumeDrawable: Drawable? = getDrawable(R.drawable.resume)

    private val androidPreferences: SharedPreferences
        get() = context.getSharedPreferences("default_preferences", Context.MODE_PRIVATE)

    private var remoteLeftVerticalStick = 0
    private var remoteLeftHorizontalStick = 0
    private var remoteRightVerticalStick = 0
    private var remoteRightHorizontalStick = 0
    private var remoteSafetyTriggerValue = 20

    private var missionState: CurrentMissionState = CurrentMissionState.NO_MISSION
    private var compositeDisposable: CompositeDisposable? = null

    private var isCurrentlyPausingMission = false;

    private var alertDialog: AlertDialog? = null
    //endregion

    override fun initView(context: Context, attrs: AttributeSet?, defStyleAttr: Int) {
        binding = WidgetMissionControlBinding.inflate(LayoutInflater.from(context), this)
    }

    init {
        setOnClickListener(this)
        attrs?.let { initAttributes(context, it) }
    }

    override fun onAttachedToWindow() {
        super.onAttachedToWindow()

        compositeDisposable = CompositeDisposable()

        MavManager.getInstance().addMavlinkStatusListener(this)
        MavManager.getInstance().addMavlinkGUIListener(this)

        remoteSafetyTriggerValue = androidPreferences.getInt("RC_STICK_TRIGG", 20)

        if (remoteSafetyTriggerValue > 250) {
            log.w("Remote safety trigger value is very high: $remoteSafetyTriggerValue")
        }

        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyStickLeftVertical), this) { _, newValue ->
            newValue?.let {
                remoteLeftVerticalStick = it
            }
        }

        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyStickLeftHorizontal), this) { _, newValue ->
            newValue?.let {
                remoteLeftHorizontalStick = it
            }
        }

        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyStickRightVertical), this) { _, newValue ->
            newValue?.let {
                remoteRightVerticalStick = it
            }
        }

        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyStickRightHorizontal), this) { _, newValue ->
            newValue?.let {
                remoteRightHorizontalStick = it
            }
        }

        // Check every 500 ms if the remote controller is not moved. If so, pause the mission
        compositeDisposable?.add(Observable.interval(500, TimeUnit.MILLISECONDS)
            .subscribeOn(Schedulers.io())
            .observeOn(AndroidSchedulers.mainThread())
            .filter { missionState == CurrentMissionState.ACTIVE }
            .subscribe(
                { checkTriggerMovement() },
                { error -> log.e("Cannot listen for remote controller triggers: $error") }
            )
        )
    }

    override fun onDetachedFromWindow() {
        MavManager.getInstance().removeMavlinkStatusListener(this)
        MavManager.getInstance().removeMavlinkGUIListener(this)

        KeyManager.getInstance().cancelListen(KeyTools.createKey(RemoteControllerKey.KeyStickLeftVertical), this)
        KeyManager.getInstance().cancelListen(KeyTools.createKey(RemoteControllerKey.KeyStickLeftHorizontal), this)
        KeyManager.getInstance().cancelListen(KeyTools.createKey(RemoteControllerKey.KeyStickRightVertical), this)
        KeyManager.getInstance().cancelListen(KeyTools.createKey(RemoteControllerKey.KeyStickRightHorizontal), this)

        compositeDisposable?.dispose()

        super.onDetachedFromWindow()
    }

    override fun onClick(v: View?) {
        when (missionState) {
            CurrentMissionState.NOT_STARTED -> MavManager.getInstance().startMission()
            CurrentMissionState.ACTIVE -> MavManager.getInstance().pauseMission()
            CurrentMissionState.PAUSED -> MavManager.getInstance().resumeMission()
            CurrentMissionState.NO_MISSION -> {}
        }
    }

    override fun onLongClick(v: View?): Boolean {
        if (missionState == CurrentMissionState.ACTIVE || missionState == CurrentMissionState.PAUSED) {
            MavManager.getInstance().stopMission()
        }
        return true
    }

    override fun reactToModelChanges() {
        when (missionState) {
            CurrentMissionState.NO_MISSION -> {
                binding.startButton.isEnabled = false
                binding.startButton.backgroundTintList = ColorStateList.valueOf(disableColor)
            }
            CurrentMissionState.NOT_STARTED -> {
                binding.startButton.isEnabled = true
                binding.startButton.backgroundTintList = ColorStateList.valueOf(enableColor)
                binding.startButton.setImageDrawable(playDrawable)
            }
            CurrentMissionState.ACTIVE -> {
                binding.startButton.isEnabled = true
                binding.startButton.backgroundTintList = ColorStateList.valueOf(enableColor)
                binding.startButton.setImageDrawable(pauseDrawable)
            }
            CurrentMissionState.PAUSED -> {
                binding.startButton.isEnabled = true
                binding.startButton.backgroundTintList = ColorStateList.valueOf(enableColor)
                binding.startButton.setImageDrawable(resumeDrawable)
            }
        }
    }

    @SuppressLint("Recycle")
    private fun initAttributes(context: Context, attrs: AttributeSet) {
        context.obtainStyledAttributes(attrs, R.styleable.MissionControlWidget).use { typedArray ->
            typedArray.getColorAndUse(R.styleable.MissionControlWidget_enableColor) {
                enableColor = it
            }
            typedArray.getColorAndUse(R.styleable.MissionControlWidget_disableColor) {
                disableColor = it
            }
            typedArray.getDrawableAndUse(R.styleable.MissionControlWidget_playDrawable) {
                playDrawable = it
            }
            typedArray.getDrawableAndUse(R.styleable.MissionControlWidget_pauseDrawable) {
                pauseDrawable = it
            }
            typedArray.getDrawableAndUse(R.styleable.MissionControlWidget_resumeDrawable) {
                resumeDrawable = it
            }
        }
    }

    override fun onShowWarning(title: String, msg: String) {
        AlertDialog.Builder(context, R.style.DialogTheme)
            .setTitle(title)
            .setMessage(msg)
            .setPositiveButton(R.string.mavlink_widget_ok) { dialogInterface, _ -> dialogInterface.dismiss() }
            .create()
            .show()
    }

    override fun onShowProgressBar(title: String, msg: String, count: Int, total: Int) {
        if (alertDialog == null) {
            log.d("Start showing progress dialog")

            val progressBar = ProgressBar(context).apply {
                isIndeterminate = false
                max = total
                progress = count
            }

            val builder = AlertDialog.Builder(context, R.style.DialogTheme)
            builder.setTitle(title)
                .setMessage(msg)
                .setView(progressBar)
                .setCancelable(false)
                .setPositiveButton(R.string.mavlink_widget_ok) { dialog, _ -> dialog.dismiss() }

            alertDialog = builder.create()
            alertDialog?.show()
        } else {
            alertDialog?.findViewById<ProgressBar>(android.R.id.progress)?.progress = count
        }

        if (count >= total) {
            log.d("Close progress dialog")
            alertDialog?.dismiss()
            alertDialog = null
        }
    }

    private fun checkTriggerMovement() {
        // If one of the sticks is moved, pause mission and add safety
        if (remoteLeftVerticalStick > remoteSafetyTriggerValue
            || remoteLeftVerticalStick < -remoteSafetyTriggerValue
            || remoteLeftHorizontalStick > remoteSafetyTriggerValue
            || remoteLeftHorizontalStick < -remoteSafetyTriggerValue
            || remoteRightVerticalStick > remoteSafetyTriggerValue
            || remoteRightVerticalStick < -remoteSafetyTriggerValue
            || remoteRightHorizontalStick > remoteSafetyTriggerValue
            || remoteRightHorizontalStick < -remoteSafetyTriggerValue
        ) {
            if (!isCurrentlyPausingMission) {
                isCurrentlyPausingMission = true
                compositeDisposable?.add(djiCompletableFunction(WaypointMissionManager.getInstance()::pauseMission)
                    .subscribeOn(Schedulers.io())
                    .subscribe(
                        {
                            log.w("Mission paused because RC sticks were moved!")
                            showWarning(context.getString(R.string.mission_control_widget_trigger_title), context.getString(R.string.mission_control_widget_trigger_message))

                            isCurrentlyPausingMission = false
                        },
                        RXUtils::handleError
                    )
                )
            }
        }
    }

    override fun onStatusChanged(status: MavManager.MavlinkStatus) {
        missionState = status.missionState;
        reactToModelChanges()
    }

    override fun onError(error: RxException) {
        log.e("Cannot subscribe from MavlinkStatus: $error")
    }

    fun setPlayIcon(@DrawableRes resourceId: Int) {
        playDrawable = getDrawable(resourceId)
    }

    fun setPauseIcon(@DrawableRes resourceId: Int) {
        pauseDrawable = getDrawable(resourceId)
    }

    fun setResumeIcon(@DrawableRes resourceId: Int) {
        resumeDrawable = getDrawable(resourceId)
    }

    private fun showWarning(title: String, msg: String) {
        AlertDialog.Builder(context, R.style.DialogTheme)
            .setTitle(title)
            .setMessage(msg)
            .setPositiveButton(R.string.mavlink_widget_ok) { dialogInterface, _ -> dialogInterface.dismiss() }
            .create()
            .show()
    }

    companion object {
        private val log = Logger("MavMissionControlWidget")
    }
}