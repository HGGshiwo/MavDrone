package nl.wur.mavdrone

import android.content.res.Resources
import android.graphics.Color
import android.graphics.drawable.Drawable
import android.os.Bundle
import android.view.Gravity
import android.view.View
import android.view.ViewGroup
import android.view.ViewTreeObserver
import android.widget.FrameLayout
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.GravityCompat
import androidx.core.view.isVisible
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.RemoteControllerKey
import dji.sdk.keyvalue.value.common.CameraLensType
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.KeyManager
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager.AvailableCameraUpdatedListener
import dji.v5.network.DJINetworkManager
import dji.v5.network.IDJINetworkStatusListener
import dji.v5.utils.common.JsonUtil
import dji.v5.ux.accessory.RTKStartServiceHelper.startRtkService
import dji.v5.ux.core.base.SchedulerProvider.io
import dji.v5.ux.core.base.SchedulerProvider.ui
import dji.v5.ux.core.communication.BroadcastValues
import dji.v5.ux.core.communication.GlobalPreferenceKeys
import dji.v5.ux.core.communication.ObservableInMemoryKeyedStore
import dji.v5.ux.core.communication.UXKeys
import dji.v5.ux.core.extension.hide
import dji.v5.ux.core.extension.toggleVisibility
import dji.v5.ux.core.util.CameraUtil
import dji.v5.ux.core.util.DataProcessor
import dji.v5.ux.core.util.ViewUtil
import dji.v5.ux.core.widget.fpv.FPVStreamSourceListener
import dji.v5.ux.core.ui.component.SegmentedButtonGroup
// import dji.v5.ux.mapkit.core.maps.DJIMap  // 地图暂不支持
import dji.v5.ux.training.simulatorcontrol.SimulatorControlWidget
import io.reactivex.rxjava3.android.schedulers.AndroidSchedulers
import io.reactivex.rxjava3.disposables.CompositeDisposable
import nl.wur.mavdrone.databinding.ActivityMainBinding
import nl.wur.mavdrone.utils.Logger
import nl.wur.mavdrone.utils.RxException
import nl.wur.mavdrone.widgets.MavControlWidget
import java.util.concurrent.TimeUnit
import androidx.core.graphics.drawable.toDrawable

class MainActivity : AppCompatActivity() {
    //region Fields
    private val log = Logger("MavMainActivity")

    private lateinit var binding: ActivityMainBinding

    // 模式切换开关(GUIDED/POSHOLD)状态: 经 MavlinkStatusListener 同步,
    // 显示实际 guided 状态而非点击意图
    private var guidedActive = false
    private var guidedButtonConnected = false

    // SegmentedButtonGroup.check() 会触发点击回调且无法读取当前选中项,
    // 用上次已应用的选择做防重入: 仅在期望状态变化时才编程选中
    private var lastAppliedModeSelection: Int? = null

    private val mavlinkStatusListener = object : MavManager.MavlinkStatusListener {
        override fun onStatusChanged(status: MavManager.MavlinkStatus) {
            guidedActive = status.guidedActive
            guidedButtonConnected = status.gcsConnected
            updateModeToggle()
        }

        override fun onError(error: RxException) {}
    }

    // 起降/返航按钮强制器(取代 a2f9905 的纯可见性监控): uxsdk 按钮状态依赖
    // KeyConnection 推送, 该键在本机型上时有时无(README 已知问题 2), 推送缺失时
    // 按钮停留在初始 GONE。把 uxsdk 的"隐藏"重解释为"置灰": 想隐藏 -> 强制
    // VISIBLE 且不可点(alpha 0.4), 认为可用 -> 恢复。可用性判断完全沿用 uxsdk
    // 内部状态机, 不重复造轮子
    private val grayedFlightButtons = mutableSetOf<View>()
    private val flightButtonEnforcer = ViewTreeObserver.OnGlobalLayoutListener {
        // FPV 全屏模式统一隐藏所有悬浮控件, 此时跳过强制
        if (viewVisibilityMap.isNotEmpty()) {
            return@OnGlobalLayoutListener
        }
        enforceFlightButton(binding.widgetTakeOff)
        enforceFlightButton(binding.widgetReturnToHome)
    }

    private fun enforceFlightButton(widget: View) {
        val name = widget.resources.getResourceEntryName(widget.id)
        if (widget.visibility != View.VISIBLE) {
            if (grayedFlightButtons.add(widget)) {
                log.i("$name unavailable: graying out")
            }
            widget.visibility = View.VISIBLE
            widget.isEnabled = false
            widget.alpha = 0.4f
        } else if (grayedFlightButtons.remove(widget)) {
            // 只还原透明度, 不动 isEnabled —— 可用与否由 uxsdk 状态机决定,
            // 强制器不得把 uxsdk 有意禁用的按钮重新点亮
            log.i("$name available")
            widget.alpha = 1.0f
        }
    }

    private fun updateModeToggle() {
        binding.btnModeGuided.isEnabled = guidedButtonConnected
        binding.btnModePoshold.isEnabled = guidedButtonConnected
        binding.modeToggleGroup.alpha = if (guidedButtonConnected) 1.0f else 0.4f

        val desired = if (guidedActive) R.id.btn_mode_guided else R.id.btn_mode_poshold
        if (lastAppliedModeSelection != desired) {
            lastAppliedModeSelection = desired
            binding.modeToggleGroup.check(desired)
        }
    }

    private var lastDevicePosition = ComponentIndexType.UNKNOWN
    private var lastLensType = CameraLensType.UNKNOWN

    private val viewVisibilityMap = mutableMapOf<View, Int>()
    private val originalBackgrounds = mutableMapOf<Int, Drawable?>()
    private var originalFPVLayoutParams: ViewGroup.LayoutParams? = null

    private var compositeDisposable: CompositeDisposable? = null
    private val cameraSourceProcessor = DataProcessor.create(
        CameraSource(ComponentIndexType.UNKNOWN, CameraLensType.UNKNOWN)
    )
    private val networkStatusListener = IDJINetworkStatusListener { networkAvailable ->
        if (networkAvailable) {
            log.d("isNetworkAvailable=" + true)
            startRtkService(false)
        }
    }
    private val availableCameraUpdatedListener = object : AvailableCameraUpdatedListener {
        override fun onAvailableCameraUpdated(availableCameraList: List<ComponentIndexType>) {
            runOnUiThread { updateFPVWidgetSource(availableCameraList) }
        }

        override fun onCameraStreamEnableUpdate(cameraStreamEnableMap: Map<ComponentIndexType, Boolean>) {
            //
        }
    }

    //endregion
    //region Lifecycle
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        initClickListener()
        MediaDataCenter.getInstance().cameraStreamManager.addAvailableCameraUpdatedListener(availableCameraUpdatedListener)
        
        binding.widgetPrimaryFpv.setOnFPVStreamSourceListener(object : FPVStreamSourceListener {
            override fun onStreamSourceUpdated(devicePosition: ComponentIndexType, lensType: CameraLensType) {
                cameraSourceProcessor.onNext(CameraSource(devicePosition, lensType))
            }
        })
        binding.widgetSecondaryFpv.setSurfaceViewZOrderOnTop(true)
        binding.widgetSecondaryFpv.setSurfaceViewZOrderMediaOverlay(true)


        // 地图暂不支持(布局中的 MapWidget 已注释), 需要恢复时与布局一起还原
        // binding.widgetMap.initMapLibreMap(applicationContext) { map: DJIMap ->
        //     map.uiSettings.setZoomControlsEnabled(false)
        // }
        // binding.widgetMap.onCreate(savedInstanceState)
        window.setBackgroundDrawable(Color.BLACK.toDrawable())

        DJINetworkManager.getInstance().addNetworkStatusListener(networkStatusListener)

        MavManager.getInstance().addMavlinkStatusListener(mavlinkStatusListener)
        binding.widgetTakeOff.viewTreeObserver.addOnGlobalLayoutListener(flightButtonEnforcer)
    }

    private fun isGimbalAdjustClicked(broadcastValues: BroadcastValues) {
        if (binding.rootView.isDrawerOpen(GravityCompat.END)) {
            binding.rootView.closeDrawers()
        }
        binding.widgetHorizontalSituationIndicator.visibility = View.GONE
        binding.settingMenuGimbalFineTune.visibility = View.VISIBLE
    }

    private fun initClickListener() {
        binding.widgetSecondaryFpv.setOnClickListener { swapVideoSource() }
        binding.panelTopBar.settingWidget?.setOnClickListener { toggleRightDrawer() }

        // Setup top bar state callbacks
        binding.widgetPanelSystemStatusList.setOnClickListener { binding.widgetPanelSystemStatusList.toggleVisibility() }
        binding.panelTopBar.simulatorIndicatorWidget?.setOnClickListener { binding.widgetSimulatorControl.toggleVisibility() }
        binding.panelTopBar.mavlinkIndicatorWidget?.setOnClickListener { binding.widgetMavlinkControl.toggleVisibility() }

        // TODO: change to viewbinding
        findViewById<TextView>(R.id.fpv_gimbal_ok_btn).setOnClickListener {
            binding.widgetHorizontalSituationIndicator.visibility = View.VISIBLE
            binding.settingMenuGimbalFineTune.visibility = View.GONE
        }

        binding.btnModeGuided.setOnClickListener {
            log.i("Mode toggle clicked: GUIDED")
            if (guidedActive) {
                log.i("Already in guided mode, ignoring")
                return@setOnClickListener
            }
            MavManager.getInstance().setGuidedMode(true)
        }
        binding.btnModePoshold.setOnClickListener {
            log.i("Mode toggle clicked: POSHOLD")
            if (!guidedActive) {
                log.i("Already not in guided mode, ignoring")
                return@setOnClickListener
            }
            MavManager.getInstance().setGuidedMode(false)
        }
    }

    private fun toggleRightDrawer() {
        binding.rootView.openDrawer(GravityCompat.END)
    }


    override fun onDestroy() {
        super.onDestroy()
        val viewTreeObserver = binding.widgetTakeOff.viewTreeObserver
        if (viewTreeObserver.isAlive) {
            viewTreeObserver.removeOnGlobalLayoutListener(flightButtonEnforcer)
        }
        MavManager.getInstance().removeMavlinkStatusListener(mavlinkStatusListener)
        // binding.widgetMap.onDestroy()  // 地图暂不支持
        MediaDataCenter.getInstance().cameraStreamManager.removeAvailableCameraUpdatedListener(availableCameraUpdatedListener)
        DJINetworkManager.getInstance().removeNetworkStatusListener(networkStatusListener)
    }

    override fun onResume() {
        super.onResume()
        // binding.widgetMap.onResume()  // 地图暂不支持
        compositeDisposable = CompositeDisposable()
        compositeDisposable?.add(
            binding.widgetPanelSystemStatusList.closeButtonPressed()
                .observeOn(AndroidSchedulers.mainThread())
                .subscribe { pressed: Boolean ->
                    if (pressed) {
                        binding.widgetPanelSystemStatusList.hide()
                    }
                })
        compositeDisposable?.add(
            binding.widgetSimulatorControl.getUIStateUpdates()
                .observeOn(AndroidSchedulers.mainThread())
                .subscribe { simulatorControlWidgetState: SimulatorControlWidget.UIState ->
                    if (simulatorControlWidgetState is SimulatorControlWidget.UIState.VisibilityUpdated) {
                        if (simulatorControlWidgetState.isVisible) {
                            hideOtherPanels(binding.widgetSimulatorControl)
                        }
                    }
                })
        compositeDisposable?.add(
            binding.widgetMavlinkControl.getUIStateUpdates()
                .observeOn(AndroidSchedulers.mainThread())
                .subscribe { mavlinkControlWidgetState: MavControlWidget.UIState ->
                    if (mavlinkControlWidgetState is MavControlWidget.UIState.VisibilityUpdated) {
                        if (mavlinkControlWidgetState.isVisible) {
                            hideOtherPanels(binding.widgetMavlinkControl)
                        }
                    }
                })
        compositeDisposable?.add(cameraSourceProcessor.toFlowable()
            .observeOn(io())
            .throttleLast(500, TimeUnit.MILLISECONDS)
            .subscribeOn(io())
            .subscribe { result: CameraSource -> runOnUiThread { onCameraSourceUpdated(result.devicePosition, result.lensType) } }
        )
        compositeDisposable?.add(ObservableInMemoryKeyedStore.getInstance()
            .addObserver(UXKeys.create(GlobalPreferenceKeys.GIMBAL_ADJUST_CLICKED))
            .observeOn(ui())
            .subscribe { broadcastValues: BroadcastValues -> this.isGimbalAdjustClicked(broadcastValues) })

        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyCustomButton1Down), this) { oldValue, newValue ->
            if (oldValue == false && newValue == true) {
                toggleToFPVFullScreen()
            }
        }

        ViewUtil.setKeepScreen(this, true)
    }

    override fun onPause() {
        compositeDisposable?.dispose()
        compositeDisposable = null

        KeyManager.getInstance().cancelListen(KeyTools.createKey(RemoteControllerKey.KeyCustomButton1Down), this)

        // binding.widgetMap.onPause()  // 地图暂不支持
        super.onPause()
        ViewUtil.setKeepScreen(this, false)
    }

    //endregion
    private fun toggleToFPVFullScreen() {
        if (viewVisibilityMap.isEmpty()) {
            log.i("Clear all overlay widgets")
            for (view in listOf(
                binding.widgetFpvInteraction, binding.widgetSecondaryFpv, binding.panelTopBar, binding.widgetRemainingFlightTime,
                binding.widgetHorizontalSituationIndicator, binding.settingMenuGimbalFineTune, binding.panelNdviCamera, binding.panelVisualCamera,
                binding.widgetAutoExposureLock, binding.widgetFocusMode, binding.widgetFocusExposureSwitch, binding.widgetCameraControls,
                binding.widgetFocalZoom, binding.widgetTakeOff, binding.widgetLensControl,
                binding.widgetReturnToHome, binding.widgetSimulatorControl, binding.widgetMavlinkControl, binding.widgetFpvFlightDisplayWidget,
                binding.widgetPanelSystemStatusList, binding.modeToggleGroup, binding.manualRightNavSetting
            )) {
                binding.widgetPrimaryFpv.cameraNameTextColor = Color.TRANSPARENT
                binding.widgetPrimaryFpv.cameraSideTextColor = Color.TRANSPARENT

                viewVisibilityMap[view] = view.visibility
                view.visibility = View.INVISIBLE
            }

            originalBackgrounds[0] = binding.widgetPrimaryFpv.cameraNameTextBackground?.constantState?.newDrawable()?.mutate()
            originalBackgrounds[1] = binding.widgetPrimaryFpv.cameraSideTextBackground?.constantState?.newDrawable()?.mutate()

            binding.widgetPrimaryFpv.cameraNameTextBackground = null
            binding.widgetPrimaryFpv.cameraSideTextBackground = null

            // Increase size of FPV
            originalFPVLayoutParams = binding.widgetPrimaryFpv.layoutParams
            binding.fpvHolder.removeView(binding.widgetPrimaryFpv)
            binding.rootView.addView(binding.widgetPrimaryFpv)

            val displayMetrics = Resources.getSystem().displayMetrics
            val aspectRatio = binding.widgetPrimaryFpv.width / binding.widgetPrimaryFpv.height
            var newWidth = displayMetrics.widthPixels
            var newHeight = (displayMetrics.widthPixels / aspectRatio)
            if (newHeight > displayMetrics.heightPixels) {
                newHeight = displayMetrics.heightPixels
                newWidth = (displayMetrics.heightPixels * aspectRatio)
            }

            binding.widgetPrimaryFpv.layoutParams.width = newWidth
            binding.widgetPrimaryFpv.layoutParams.height = newHeight
        } else {
            log.i("Restore all overlay widgets")

            binding.widgetPrimaryFpv.cameraNameTextBackground = originalBackgrounds[0]
            binding.widgetPrimaryFpv.cameraSideTextBackground = originalBackgrounds[1]
            originalBackgrounds.clear()

            binding.widgetPrimaryFpv.cameraNameTextColor = Color.WHITE
            binding.widgetPrimaryFpv.cameraSideTextColor = Color.WHITE

            binding.rootView.removeView(binding.widgetPrimaryFpv)
            binding.fpvHolder.addView(binding.widgetPrimaryFpv)

            binding.widgetPrimaryFpv.layoutParams = originalFPVLayoutParams
            originalFPVLayoutParams = null

            for ((view, visibility) in viewVisibilityMap) {
                view.visibility = visibility
            }
            viewVisibilityMap.clear()
        }
    }

    private fun hideOtherPanels(widget: View) {
        arrayOf(
            binding.widgetSimulatorControl,
            binding.widgetMavlinkControl
        ).forEach { panel ->
            if (widget !== panel) {
                panel.visibility = View.GONE
            }
        }
    }

    private fun updateFPVWidgetSource(availableCameraList: List<ComponentIndexType>?) {
        log.d(JsonUtil.toJson(availableCameraList))
        if (availableCameraList == null) {
            return
        }

        val cameraList = ArrayList(availableCameraList)

        if (cameraList.isEmpty()) {
            binding.widgetSecondaryFpv.visibility = View.GONE
            return
        }

        if (cameraList.size == 1) {
            binding.widgetPrimaryFpv.updateVideoSource(availableCameraList[0])
            binding.widgetSecondaryFpv.visibility = View.GONE
            return
        }

        val primarySource = getSuitableSource(cameraList, ComponentIndexType.LEFT_OR_MAIN)
        binding.widgetPrimaryFpv.updateVideoSource(primarySource)
        cameraList.remove(primarySource)

        val secondarySource = getSuitableSource(cameraList, ComponentIndexType.FPV)
        binding.widgetSecondaryFpv.updateVideoSource(secondarySource)

        binding.widgetSecondaryFpv.visibility = View.VISIBLE
    }

    private fun getSuitableSource(cameraList: List<ComponentIndexType>, defaultSource: ComponentIndexType): ComponentIndexType {
        return when {
            cameraList.contains(ComponentIndexType.LEFT_OR_MAIN) -> ComponentIndexType.LEFT_OR_MAIN
            cameraList.contains(ComponentIndexType.RIGHT) -> ComponentIndexType.RIGHT
            cameraList.contains(ComponentIndexType.UP) -> ComponentIndexType.UP
            cameraList.contains(ComponentIndexType.PORT_1) -> ComponentIndexType.PORT_1
            cameraList.contains(ComponentIndexType.PORT_2) -> ComponentIndexType.PORT_2
            cameraList.contains(ComponentIndexType.PORT_3) -> ComponentIndexType.PORT_3
            cameraList.contains(ComponentIndexType.PORT_4) -> ComponentIndexType.PORT_4
            cameraList.contains(ComponentIndexType.VISION_ASSIST) -> ComponentIndexType.VISION_ASSIST
            else -> defaultSource
        }
    }

    private fun onCameraSourceUpdated(devicePosition: ComponentIndexType, lensType: CameraLensType) {
        log.d("onCameraSourceUpdated", devicePosition, lensType)
        if (devicePosition == lastDevicePosition && lensType == lastLensType) {
            return
        }
        lastDevicePosition = devicePosition
        lastLensType = lensType
        updateViewVisibility(devicePosition, lensType)
        updateInteractionEnabled()

        if (binding.widgetFpvInteraction.isInteractionEnabled) {
            binding.widgetFpvInteraction.updateCameraSource(devicePosition, lensType)
        }
        if (binding.widgetLensControl.isVisible) {
            binding.widgetLensControl.updateCameraSource(devicePosition, lensType)
        }
        if (binding.panelNdviCamera.isVisible) {
            binding.panelNdviCamera.updateCameraSource(devicePosition, lensType)
        }
        if (binding.panelVisualCamera.isVisible) {
            binding.panelVisualCamera.updateCameraSource(devicePosition, lensType)
        }
        if (binding.widgetAutoExposureLock.isVisible) {
            binding.widgetAutoExposureLock.updateCameraSource(devicePosition, lensType)
        }
        if (binding.widgetFocusMode.isVisible) {
            binding.widgetFocusMode.updateCameraSource(devicePosition, lensType)
        }
        if (binding.widgetFocusExposureSwitch.isVisible) {
            binding.widgetFocusExposureSwitch.updateCameraSource(devicePosition, lensType)
        }
        if (binding.widgetCameraControls.isVisible) {
            binding.widgetCameraControls.updateCameraSource(devicePosition, lensType)
        }
        if (binding.widgetFocalZoom.isVisible) {
            binding.widgetFocalZoom.updateCameraSource(devicePosition, lensType)
        }
        if (binding.widgetHorizontalSituationIndicator.isVisible) {
            binding.widgetHorizontalSituationIndicator.updateCameraSource(devicePosition, lensType)
        }
    }

    private fun updateViewVisibility(devicePosition: ComponentIndexType, lensType: CameraLensType) {
        binding.widgetFpvFlightDisplayWidget.visibility = if (CameraUtil.isFPVTypeView(devicePosition)) View.VISIBLE else View.INVISIBLE

        binding.widgetLensControl.visibility = if (CameraUtil.isFPVTypeView(devicePosition)) View.INVISIBLE else View.VISIBLE
        binding.panelNdviCamera.visibility = if (CameraUtil.isFPVTypeView(devicePosition)) View.INVISIBLE else View.VISIBLE
        binding.panelVisualCamera.visibility = if (CameraUtil.isFPVTypeView(devicePosition)) View.INVISIBLE else View.VISIBLE
        binding.widgetAutoExposureLock.visibility = if (CameraUtil.isFPVTypeView(devicePosition)) View.INVISIBLE else View.VISIBLE
        binding.widgetFocusMode.visibility = if (CameraUtil.isFPVTypeView(devicePosition)) View.INVISIBLE else View.VISIBLE
        binding.widgetFocusExposureSwitch.visibility = if (CameraUtil.isFPVTypeView(devicePosition)) View.INVISIBLE else View.VISIBLE
        binding.widgetCameraControls.visibility = if (CameraUtil.isFPVTypeView(devicePosition)) View.INVISIBLE else View.VISIBLE
        binding.widgetFocalZoom.visibility = if (CameraUtil.isFPVTypeView(devicePosition)) View.INVISIBLE else View.VISIBLE
        binding.widgetHorizontalSituationIndicator.setSimpleModeEnable(devicePosition != ComponentIndexType.FPV)

        binding.panelNdviCamera.visibility = if (CameraUtil.isSupportForNDVI(lensType)) View.VISIBLE else View.INVISIBLE
    }

    /**
     * Swap the video sources of the FPV and secondary FPV widgets.
     */
    private fun swapVideoSource() {
        val primarySource = binding.widgetPrimaryFpv.widgetModel.getCameraIndex()
        val secondarySource = binding.widgetSecondaryFpv.widgetModel.getCameraIndex()
        if (primarySource != ComponentIndexType.UNKNOWN && secondarySource != ComponentIndexType.UNKNOWN) {
            binding.widgetPrimaryFpv.updateVideoSource(secondarySource)
            binding.widgetSecondaryFpv.updateVideoSource(primarySource)
        }
    }

    private fun updateInteractionEnabled() {
        binding.widgetFpvInteraction.isInteractionEnabled = !CameraUtil.isFPVTypeView(binding.widgetPrimaryFpv.widgetModel.getCameraIndex())
    }

    private class CameraSource(var devicePosition: ComponentIndexType, var lensType: CameraLensType)

    @Deprecated("Deprecated in Java")
    override fun onBackPressed() {
        if (binding.rootView.isDrawerOpen(GravityCompat.END)) {
            binding.rootView.closeDrawers()
        } else {
            super.onBackPressed()
        }
    }
}