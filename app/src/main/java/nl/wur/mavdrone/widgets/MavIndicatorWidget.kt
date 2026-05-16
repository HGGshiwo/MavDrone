package nl.wur.mavdrone.widgets

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.drawable.Drawable
import android.util.AttributeSet
import android.view.View
import androidx.annotation.DrawableRes
import androidx.core.content.res.use

import dji.v5.ux.core.base.widget.IconButtonWidget
import dji.v5.ux.core.extension.*
import nl.wur.mavdrone.MavManager
import nl.wur.mavdrone.MavManager.MavlinkStatus;

import nl.wur.mavdrone.R
import nl.wur.mavdrone.utils.Logger
import nl.wur.mavdrone.utils.RxException

open class MavIndicatorWidget @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : IconButtonWidget<Any>(
    context,
    attrs,
    defStyleAttr
), View.OnClickListener, MavManager.MavlinkStatusListener {

    //region Fields
    private var stateChangeResourceId: Int = INVALID_RESOURCE
    private var lastMavlinkStatus: MavlinkStatus? = null

    var mavlinkConnectedIcon: Drawable? = getDrawable(R.drawable.ic_mavlink_connected)
        set(value) {
            field = value
            checkAndUpdateIcon()
        }

    var mavlinkServerRunningIcon: Drawable? = getDrawable(R.drawable.ic_mavlink_server_running)
        set(value) {
            field = value
            checkAndUpdateIcon()
        }

    var mavlinkInactiveIcon: Drawable? = getDrawable(R.drawable.ic_mavlink_inactive)
        set(value) {
            field = value
            checkAndUpdateIcon()
        }

    //endregion

    //region Lifecycle
    init {
        attrs?.let { initAttributes(context, it) }
        connectedStateIconColor = getColor(R.color.uxsdk_white)
    }

    override fun onAttachedToWindow() {
        super.onAttachedToWindow()
        if (!isInEditMode) {
            MavManager.getInstance().addMavlinkStatusListener(this)
        }
    }

    override fun onDetachedFromWindow() {
        if (!isInEditMode) {
            MavManager.getInstance().removeMavlinkStatusListener(this)
        }
        super.onDetachedFromWindow()
    }

    override fun getIdealDimensionRatioString(): String {
        return getString(R.string.uxsdk_widget_default_ratio)
    }

    override fun reactToModelChanges() {
    }

    override fun checkAndUpdateIconColor() {
        if (!isInEditMode) {
            lastMavlinkStatus?.let { updateUI(it.isServerRunning, it.gcsConnected) }
        }
    }

    override fun onStatusChanged(status: MavlinkStatus) {
        lastMavlinkStatus = status
        updateUI(status.isServerRunning, status.gcsConnected)
    }

    override fun onError(error: RxException) {
        log.e(error.toString())
    }

    //endregion

    //region private Methods

    private fun updateUI(isRunning: Boolean, isConnected: Boolean) {
        if (isRunning && isConnected) {
            foregroundImageView.imageDrawable = mavlinkConnectedIcon
        } else if (isRunning) {
            foregroundImageView.imageDrawable = mavlinkServerRunningIcon
        } else {
            foregroundImageView.imageDrawable = mavlinkInactiveIcon
        }
    }


    private fun checkAndUpdateIcon() {
        if (!isInEditMode) {
            lastMavlinkStatus?.let { updateUI(it.isServerRunning, it.gcsConnected) }
        }
    }

    @SuppressLint("Recycle")
    private fun initAttributes(context: Context, attrs: AttributeSet) {
        context.obtainStyledAttributes(attrs, R.styleable.MavlinkIndicatorWidget).use { typedArray ->
            typedArray.getResourceIdAndUse(R.styleable.MavlinkIndicatorWidget_onStateChange) {
                stateChangeResourceId = it
            }
            typedArray.getDrawableAndUse(R.styleable.MavlinkIndicatorWidget_mavlinkConnectedIcon) {
                mavlinkConnectedIcon = it
            }
            typedArray.getDrawableAndUse(R.styleable.MavlinkIndicatorWidget_mavlinkServerRunningDrawable) {
                mavlinkServerRunningIcon = it
            }
            typedArray.getDrawableAndUse(R.styleable.MavlinkIndicatorWidget_mavlinkInactiveDrawable) {
                mavlinkInactiveIcon = it
            }
        }
    }
    //endregion

    //region customizations

    fun setMavlinkConnectedIcon(@DrawableRes resourceId: Int) {
        mavlinkConnectedIcon = getDrawable(resourceId)
    }

    fun setMavlinkServerRunningIcon(@DrawableRes resourceId: Int) {
        mavlinkServerRunningIcon = getDrawable(resourceId)
    }

    fun setMavlinkInactiveIcon(@DrawableRes resourceId: Int) {
        mavlinkInactiveIcon = getDrawable(resourceId)
    }

    //endregion

    companion object {
        private val log = Logger("MavIndicatorWidget")
    }
}