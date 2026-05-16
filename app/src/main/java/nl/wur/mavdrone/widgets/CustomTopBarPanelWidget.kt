package nl.wur.mavdrone.widgets

import android.annotation.SuppressLint
import android.content.Context
import android.util.AttributeSet
import androidx.core.content.res.use
import dji.v5.ux.core.base.WidgetSizeDescription
import dji.v5.ux.core.base.panel.BarPanelWidget
import dji.v5.ux.core.base.panel.PanelItem
import dji.v5.ux.core.base.panel.PanelWidgetConfiguration
import dji.v5.ux.core.extension.getDimension
import dji.v5.ux.core.extension.getIntegerAndUse
import dji.v5.ux.core.widget.airsense.AirSenseWidget
import dji.v5.ux.core.widget.battery.BatteryWidget
import dji.v5.ux.core.widget.flightmode.FlightModeWidget
import dji.v5.ux.core.widget.gpssignal.GpsSignalWidget
import dji.v5.ux.core.widget.remotecontrollersignal.RemoteControllerSignalWidget
import dji.v5.ux.core.widget.simulator.SimulatorIndicatorWidget
import dji.v5.ux.core.widget.systemstatus.SystemStatusWidget
import dji.v5.ux.core.widget.videosignal.VideoSignalWidget
import dji.v5.ux.core.widget.perception.PerceptionStateWidget
import dji.v5.ux.warning.DeviceHealthAndStatusWidget
import dji.v5.ux.core.widget.setting.SettingWidget
import nl.wur.mavdrone.R
import nl.wur.mavdrone.utils.Logger

open class CustomTopBarPanelWidget @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0,
    barPanelWidgetOrientation: BarPanelWidgetOrientation = BarPanelWidgetOrientation.HORIZONTAL
) : BarPanelWidget<Any>(context, attrs, defStyleAttr, barPanelWidgetOrientation) {

    //region Widgets Properties

    //region Widgets Properties
    val deviceHealthAndStatusWidget: DeviceHealthAndStatusWidget?
    val systemStatusWidget: SystemStatusWidget?
    val flightModeWidget: FlightModeWidget?
    val simulatorIndicatorWidget: SimulatorIndicatorWidget?
    val mavlinkIndicatorWidget: MavIndicatorWidget?
    val airSenseWidget: AirSenseWidget?
    val gpsSignalWidget: GpsSignalWidget?
    val visionWidget: PerceptionStateWidget?
    val remoteControllerSignalWidget: RemoteControllerSignalWidget?
    val videoSignalWidget: VideoSignalWidget?
    val batteryWidget: BatteryWidget?
    val settingWidget: SettingWidget?
//    val connectionWidget: ConnectionWidget?
    //endregion

    //region Private properties
    private var excludedItemsValue = 0
    //endregion

    //region Lifecycle & Setup

    override fun initPanelWidget(context: Context, attrs: AttributeSet?, defStyleAttr: Int, widgetConfiguration: PanelWidgetConfiguration?) {
        // Nothing to do
    }

    init {
        val leftPanelItems = ArrayList<PanelItem>()
        if (!WidgetValues.SYSTEM_STATUS.isItemExcluded(excludedItemsValue)) {
            systemStatusWidget = SystemStatusWidget(context, attrs)
            leftPanelItems.add(PanelItem(systemStatusWidget, itemMarginTop = 0, itemMarginBottom = 0))
        } else {
            systemStatusWidget = null
        }
        addLeftWidgets(leftPanelItems.toTypedArray())

        val rightPanelItems = ArrayList<PanelItem>()
        if (!WidgetValues.DEVICE_HEALTH.isItemExcluded(excludedItemsValue)) {
            deviceHealthAndStatusWidget = DeviceHealthAndStatusWidget(context, attrs)
            rightPanelItems.add(PanelItem(deviceHealthAndStatusWidget))
        } else {
            deviceHealthAndStatusWidget = null
        }
        if (!WidgetValues.GPS_SIGNAL.isItemExcluded(excludedItemsValue)) {
            gpsSignalWidget = GpsSignalWidget(context, attrs)
            rightPanelItems.add(PanelItem(gpsSignalWidget))
        } else {
            gpsSignalWidget = null
        }
        if (!WidgetValues.FLIGHT_MODE.isItemExcluded(excludedItemsValue)) {
            flightModeWidget = FlightModeWidget(context, attrs)
            rightPanelItems.add(PanelItem(flightModeWidget))
        } else {
            flightModeWidget = null
        }
        if (!WidgetValues.SIMULATOR_INDICATOR.isItemExcluded(excludedItemsValue)) {
            simulatorIndicatorWidget = SimulatorIndicatorWidget(context, attrs)
            rightPanelItems.add(PanelItem(simulatorIndicatorWidget))
        } else {
            simulatorIndicatorWidget = null
        }
        if (!WidgetValues.MAVLINK_INDICATOR.isItemExcluded(excludedItemsValue)) {
            mavlinkIndicatorWidget = MavIndicatorWidget(context, attrs)
            rightPanelItems.add(PanelItem(mavlinkIndicatorWidget))
        } else {
            mavlinkIndicatorWidget = null
        }
        if (!WidgetValues.AIR_SENSE.isItemExcluded(excludedItemsValue)) {
            airSenseWidget = AirSenseWidget(context, attrs)
            rightPanelItems.add(PanelItem(airSenseWidget))
        } else {
            airSenseWidget = null
        }
        if (!WidgetValues.VISION.isItemExcluded(excludedItemsValue)) {
            visionWidget = PerceptionStateWidget(context, attrs)
            rightPanelItems.add(PanelItem(visionWidget))
        } else {
            visionWidget = null
        }
        if (!WidgetValues.RC_SIGNAL.isItemExcluded(excludedItemsValue)) {
            remoteControllerSignalWidget = RemoteControllerSignalWidget(context, attrs)
            rightPanelItems.add(PanelItem(remoteControllerSignalWidget))
        } else {
            remoteControllerSignalWidget = null
        }
        if (!WidgetValues.VIDEO_SIGNAL.isItemExcluded(excludedItemsValue)) {
            videoSignalWidget = VideoSignalWidget(context, attrs)
            rightPanelItems.add(PanelItem(videoSignalWidget))
        } else {
            videoSignalWidget = null
        }
        if (!WidgetValues.BATTERY.isItemExcluded(excludedItemsValue)) {
            batteryWidget = BatteryWidget(context, attrs)
            rightPanelItems.add(PanelItem(batteryWidget))
        } else {
            batteryWidget = null
        }

        if (!WidgetValues.SETTING.isItemExcluded(excludedItemsValue)) {
            settingWidget = SettingWidget(context, attrs)
            rightPanelItems.add(PanelItem(settingWidget))
        } else {
            settingWidget = null
        }
        addRightWidgets(rightPanelItems.toTypedArray())
    }

    @SuppressLint("Recycle")
    override fun initAttributes(attrs: AttributeSet) {
        guidelinePercent = 0.25f
        itemsMarginTop = getDimension(R.dimen.uxsdk_bar_panel_margin).toInt()
        itemsMarginBottom = getDimension(R.dimen.uxsdk_bar_panel_margin).toInt()

        context.obtainStyledAttributes(attrs, R.styleable.CustomTopBarPanelWidget).use { typedArray ->
            typedArray.getIntegerAndUse(R.styleable.CustomTopBarPanelWidget_excludeTopBarItem) {
                excludedItemsValue = it
            }
        }

        super.initAttributes(attrs)
    }

    override fun reactToModelChanges() {
        // Nothing to do
    }
    //endregion

    //region Customizations
    override fun getIdealDimensionRatioString(): String? = null

    override val widgetSizeDescription: WidgetSizeDescription =
        WidgetSizeDescription(
            WidgetSizeDescription.SizeType.OTHER,
            widthDimension = WidgetSizeDescription.Dimension.EXPAND,
            heightDimension = WidgetSizeDescription.Dimension.EXPAND
        )
    //endregion

    companion object {
        private enum class WidgetValues(val value: Int) {
            SYSTEM_STATUS(1),           //   0000 0000 0001
            DEVICE_HEALTH(2),           //   0000 0000 0010
            FLIGHT_MODE(4),             //   0000 0000 0100
            SIMULATOR_INDICATOR(8),     //   0000 0000 1000
            AIR_SENSE(16),              //   0000 0001 0000
            GPS_SIGNAL(32),             //   0000 0010 0000
            VISION(64),                 //   0000 0100 0000
            RC_SIGNAL(128),             //   0000 1000 0000
            VIDEO_SIGNAL(256),          //   0001 0000 0000
            BATTERY(512),               //   0010 0000 0000
            SETTING(1024),              //   0100 0000 0000
            CONNECTION(2048),           //   1000 0000 0000
            MAVLINK_INDICATOR(4096);    // 1 0000 0000 0000

            fun isItemExcluded(excludeItems: Int): Boolean {
                return excludeItems and this.value == this.value
            }
        }

        private val LOG = Logger("CustomPanelWidget")
    }
}