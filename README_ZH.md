# MavDrone 中文说明

本文档面向本仓库的使用与维护者，记录项目结构、在原开源项目基础上做出的全部修改，以及使用中需要特别注意的事项。

## 一、项目简介

MavDrone 是一个运行在安卓设备上的应用程序。它通过大疆移动 SDK（DJI Mobile SDK V5）获取无人机的飞行状态，并把整个无人机包装为一个 MAVLink 飞控端（系统编号下的自动驾驶仪组件）。任意支持 MAVLink 协议的地面站（例如 QGroundControl、Mission Planner、基于 MAVSDK 的自研程序）都可以通过 TCP 连接到本应用，实现遥测查看、参数读写、航线任务上传与执行、解锁与起飞等完整交互。

简而言之：本应用在大疆无人机与 MAVLink 地面站之间承担协议转换的角色。

## 二、项目结构

```
MavDrone/
├── app/                                 应用主模块
│   └── src/main/java/nl/wur/mavdrone/
│       ├── MainActivity.kt              主界面
│       ├── MavDroneApplication.kt       应用入口：大疆 SDK 初始化与注册
│       ├── MavService.kt                前台服务：维持 MAVLink 链路
│       ├── MavManager.kt                应用层管理器（连接信息、组件信息）
│       ├── communication/
│       │   ├── MavConnection.kt         TCP 服务端：解析收到的消息、分发发送（默认端口 5760）
│       │   ├── MavComponents.kt         组件装配：飞控、三路相机、三路云台、两块电池
│       │   ├── components/
│       │   │   ├── MavBaseComponent.kt  组件基类：连接管理、消息收发、参数协议、心跳
│       │   │   ├── Drone.kt             飞控组件（MAV_COMP_ID_AUTOPILOT1）：命令、任务入口、解锁检测
│       │   │   ├── Camera.kt            相机组件（拍照、录像、变焦、图传状态）
│       │   │   ├── Gimbal.kt            云台组件（云台管理器协议）
│       │   │   ├── Battery.kt           电池组件（BATTERY_STATUS、SMART_BATTERY_INFO）
│       │   │   └── impl/                飞控子模块：任务管理、航线、围栏、日志、文件传输、
│       │   │                             雷达、健康、遥控器、板载相机、RTK 等
│       │   └── utils/                   参数协议、枚举工具、MAVLink 转换、标准应答等
│       ├── widgets/                     自定义界面控件
│       └── utils/                       通用工具（日志、网络、坐标换算等）
├── uxsdk/                               大疆官方界面控件库（本项目有修改，见下文）
├── mavlink/                             MAVLink 协议库（dronefleet/mavlink-kotlin 及其生成代码）
├── mavdrone_python/                     基于 MAVSDK 的 Python 地面站示例
├── mavdrone_ros2/                       ROS2 接口示例
├── doc/、misc/                          文档与杂项资源
├── README.md                            上游英文说明
└── README_ZH.md                         本文件
```

## 三、相对原项目做出的修改

以下修改均已在本工作区完成，尚未提交。按类别列出，每项注明涉及文件。

### 3.1 功能增强

| 修改内容 | 涉及文件 |
|---|---|
| 支持地面站命令 `MAV_CMD_RUN_PREARM_CHECKS`：执行解锁前检查（GPS 定位、RTK 健康、Home 点、传感器健康、电池电量），逐条以 `PreArm: ` 开头的 `STATUSTEXT` 上报失败项，并返回接受或拒绝 | `communication/components/Drone.kt`、`communication/components/impl/HealthManager.kt` |
| 支持地面站以 ArduPilot 风格切换 GUIDED 模式：`MAV_CMD_DO_SET_MODE` 携带主模式 4 且无 PX4 自动子模式时，与 PX4 OFFBOARD 一并映射为地面站受控状态（置 `MAV_MODE_FLAG_GUIDED_ENABLED`，回读为 OFFBOARD）；`MAV_CMD_NAV_TAKEOFF` 在 GUIDED 标志置位时同样放行 | `communication/components/Drone.kt` |
| `MAV_CMD_NAV_TAKEOFF` 支持指定起飞高度：param7 高于 1.2 米时，DJI 自动起飞（固定约 1.2 米，固件最低值）完成后自动启用虚拟摇杆、垂直位置模式爬升到目标高度（相对起飞点），到位判定为容差 0.2 米且连续 3 个周期，带总超时保护，`COMMAND_CANCEL` 可中止爬升；param7 低于 1.2 米或未指定时按 1.2 米处理，即普通自动起飞。虚拟摇杆使用大疆官方 `dji.v5.manager.aircraft.virtualstick.VirtualStickManager` 封装（含控制权接管）与高级模式定高，直接用裸键不接管控制权时飞控会忽略摇杆数据 | `communication/components/Drone.kt`、新增 `communication/components/impl/VirtualStickManager.kt` |
| 新增参数 `ARMING_CHECK`：解锁前检测的总开关（0 为关闭，1 为开启），数值持久化保存，并作为上述检测的生效条件 | `communication/components/impl/DroneParameters.kt`、`Drone.kt` |
| 解锁命令 `MAV_CMD_COMPONENT_ARM_DISARM` 在执行前先运行解锁前检测：存在失败项时发送 `PreArm: ` 文本并拒绝解锁 | `communication/components/Drone.kt` |
| 主界面固定为横屏，消除旋转屏幕导致的界面重建崩溃 | `app/src/main/AndroidManifest.xml` |
| 模拟器默认配置写死为：纬度 30.11269、经度 120.1429、卫星数 20（面板仍可临时调整） | `uxsdk/.../simulatorcontrol/SimulatorControlWidget.kt` |

### 3.2 模拟器迁移到官方接口

原实现通过 uxsdk 示例控件的 ActionKey（`KeyStartSimulator`/`KeyStopSimulator`）启停模拟器，并依赖 `KeySimulatorState` 键推送获取遥测。实测在本项目使用的机型上该键推送缺失，导致模拟遥测为空。

现迁移到大疆官方教程推荐的 `SimulatorManager` 接口：

| 修改内容 | 涉及文件 |
|---|---|
| 启动改调 `SimulatorManager.enableSimulator(InitializationSettings, 回调)`，停止改调 `disableSimulator(回调)`；启动与失败结果通过回调显式暴露，不再被静默吞掉 | `uxsdk/.../simulatorcontrol/SimulatorControlWidgetModel.kt` |
| 注册 `addSimulatorStateListener` 状态回调，把回调中的位置、高度、姿态直接桥接到飞控组件的遥测字段；Home 点首次置为模拟器起始位置，模拟器停止后自动复位 | `communication/components/Drone.kt` |
| 模拟器控制面板不再依赖"产品连接"推送键控制可用状态（该键在部分机型上不推送） | `uxsdk/.../simulatorcontrol/SimulatorControlWidget.kt` |

### 3.3 问题修复

| 问题与修复 | 涉及文件 |
|---|---|
| 大疆 SDK 注册回调在部分场景下不触发，导致 SDK 长期未注册（表现为产品不连接、组件不上线）。修复：初始化后 15 秒仍未注册成功时，直接调用 `registerApp()` 兜底 | `MavDroneApplication.kt` |
| 组件连接状态推送键（`KeyConnection`）时有时无，导致飞控等组件不上线、解锁命令无响应。修复：所有组件每 2 秒轮询一次连接键当前值，为真且未上线时强制走上线流程；上线流程增加幂等保护 | `communication/components/MavBaseComponent.kt` |
| 相机视场角计算在焦距或传感器宽度为零时崩溃（`NaN` 取整异常），且存在空指针风险。修复：无效时跳过计算 | `communication/components/Camera.kt` |
| 障碍物距离消息（`ObstacleDistance`）在障碍列表为空时除以零，导致该消息从未发送成功。修复：空列表时跳过发送 | `communication/components/impl/Radar.kt` |
| 参数值读取依赖大疆键异步获取，部分键会挂起或失败，导致参数永远不应答，地面站（尤其是 MAVSDK）无限重发参数列表请求，进而阻塞解锁与起飞。修复：读取加 300 毫秒超时，超时或失败时回传参数默认值，保证每个参数必定应答 | `communication/utils/MavParameter.kt` |
| 参数写入被真机拒绝（例如超出允许范围）时不返回任何应答，地面站只能反复重试。修复：写入失败时回传当前值 | `communication/utils/MavParameter.kt` |
| 电池百分比键在部分机型上不推送，导致电池消息从未上报。修复：新增聚合键 `KeyBatteryOverviews` 作为备用数据源；百分比无效时按充电余量与满充容量换算；电池剩余量未知时显式发送 -1（协议未知标记）而非溢出值，并记录一次性警告日志 | `communication/components/Battery.kt` |
| 多个遥测消息构建函数使用非空断言，数据尚未到达时会抛出异常且被静默吞掉。修复：数据缺失时返回空并跳过发送；周期发送器的异常改为记录日志，不再静默 | `communication/components/Drone.kt`、`MavBaseComponent.kt` |
| 健康管理器新增未健康传感器查询接口，供解锁前检测使用 | `communication/components/impl/HealthManager.kt` |
| `SYS_STATUS` 的 `onboard_control_sensors_health` 从未包含 `MAV_SYS_STATUS_PREARM_CHECK`（bit 28，0x10000000），依赖该位判断解锁前检查结果的地面站（等待该位置位）会一直阻塞到超时。修复：把该位加入 present/enabled 掩码，并按实时解锁前检测结果在 health 掩码中置位或清零（`ARMING_CHECK=0` 时视为通过） | `communication/components/Drone.kt`、`communication/components/impl/HealthManager.kt` |
| 从未发送 `LOCAL_POSITION_NED`，MAVSDK 的 `position_velocity_ned` 订阅无数据，地面站的 ENU 位置（`pos_enu`）无法得到。修复：按 GPS 相对 Home 点偏移派生北/东坐标、高度取反为 NED z、速度取大疆速度键（无数据时为 0），以 10 Hz 发送。随后增加逐轴 2 状态卡尔曼滤波（协方差在线传播）：用大疆融合速度做预测、GPS 派生位置做观测（9σ 野点门控、连续拒识后强制重锁定），输出平滑的 `x/y`（z 不滤波）；速度键无推送时自动退化为 GPS 直通；Home 点重设时滤波器重锁定。注意其精度仍受 GPS 限制，不如真实 EKF 的本地定位 | `communication/utils/LocalPositionFilter.kt`（新增）、`communication/components/Drone.kt` |
| 新增 offboard 速度控制：处理 `SET_POSITION_TARGET_LOCAL_NED`（MAVSDK Offboard 的 `set_velocity_body`/`set_velocity_ned`/`set_position_velocity_ned` 都发这条消息，此前完全不生效），通过虚拟摇杆（速度模式）下发；首条设定点自动接管控制权，设定点断流 2 秒或落地自动归还。坐标系注意：实测 Mini 4 Pro 固件忽略 BODY 坐标请求、始终按 DJI 地面系（X=东、Y=北）解释水平速度，因此应用侧把机体系设定点自行旋转到地面系并显式请求 GROUND（NED 输入经两次旋转后恒等映射，与航向无关）；只消费速度分量，水平位置由地面站闭环控制（DJI 无水平位置模式） | `communication/components/Drone.kt`、`communication/components/impl/VirtualStickManager.kt` |
| 长时间仿真飞行会被固件低电量智能返航打断（模拟器电池随仿真时间放电，剩余电量到达固件估算的"够回家"水平即自动返航，表现为飞一段时间后突然进入 GO_HOME）。修复：仿真开启时自动关闭固件低电量返航（`KeyLowBatteryRTHEnabled`），仿真停止后恢复默认开启；同时把固件的低电量警告、严重低电量、低电量返航触发状态以 `STATUSTEXT` 转发给地面站 | `communication/components/Drone.kt` |
| 自动降落会在确认高度暂停、等待 DJI App 的"滑动确认降落"，地面站的降落流程被卡住；重复下发降落命令还会被固件以错误码 -7（`FLIGHTCONTROLLER.StartAutoLanding`，已在降落中）拒绝。修复：启动自动降落成功后监视 `KeyIsLandingConfirmationNeeded`（30 秒窗口），固件请求确认时自动调用 `KeyConfirmLanding`（等价于 App 里的滑动），降落全程无需人工介入 | `communication/components/Drone.kt` |
| `STATUSTEXT` 按字符数截断导致中文被从 UTF-8 序列中间切断（MAVLink 字段上限是 50 字节），非法字节进入地面站的 nlohmann::json 序列化（`dump()` 严格校验）会直接把节点 abort 掉（`json.exception.type_error.316`）。修复：新增 `MavlinkTexts.truncateStatustext()`，按 50 字节边界截断且绝不切开 UTF-8 序列，所有含动态内容的 STATUSTEXT（PreArm 失败项、RTL 原因、虚拟摇杆状态，含 DJI 中文错误描述）统一走该函数 | `utils/MavlinkTexts.kt`（新增）、`communication/components/Drone.kt`、`communication/components/impl/VirtualStickManager.kt` |
| App 重启或链路重建后立刻下发起飞命令，会撞上 `REQUEST_HANDLER_NOT_FOUND`（`FLIGHTCONTROLLER.StartTakeoff:-1`，FC 组件尚未重新上线）。修复：`KeyStartTakeoff` 增加短重试（3 次、间隔 500 毫秒），连接恢复瞬间到达的命令在连接就绪后自动成功；重试耗尽仍失败才返回命令失败 | `communication/components/Drone.kt` |
| 新增下视/主相机图像 MJPEG 输出：服务启动时在 8090 端口起一个 HTTP 服务，`/` 返回 MJPEG 流（multipart/x-mixed-replace，15fps、JPEG 质量 60、缩放到 480×320 以内），`/snapshot` 返回单帧 JPEG（无帧时 503，便于区分 HTTP 问题与帧管线问题），供地面站拉流发布为 ROS topic；通过共享的 `ICameraStreamManager` 解码取帧（与 FPV 控件同源），注册不被不可靠的 `KeyConnection` 门控、失败自动重试，"listener already added" 视为复用残留监听器，并挂载离屏 Surface 保证无 App 界面时也持续解码。注意：Mini 4 Pro 的下视视觉传感器无 SDK 图像接口，此流来自主云台相机，需要下视画面时由地面站下发云台 pitch=-90° | `video/DownCamStreamer.kt`（新增）、`MavService.kt` |
| `CameraFrameListener` 回调的五个整数参数实测顺序为 `(offset, size, width, height)`，与文档记载的 `(width, height, offset, stride)` 不符，按文档解释会把 1080p 标准帧误判为非法布局而全部跳帧。修复：按实测顺序取值并对照缓冲区大小自校验，不符时回退文档顺序；另移除离屏 `SurfaceTexture` 上非法的 `detachFromGLContext()` 调用（该对象不附着 GL 上下文，调用必然抛错导致监听注册整体失败） | `video/DownCamStreamer.kt` |
| 从未发送 `VFR_HUD`，地面站读不到油门值（初始 -1，无法做落地/悬停判定）。DJI 不暴露电机输出遥测，油门按飞行状态合成：电机停转 0、已解锁未起飞 5、飞行中 50（悬停水平），另外顺带填充地速、航向、相对高度和爬升率，以 5 Hz 发送 | `communication/components/Drone.kt` |
| 心跳 `custom_mode` 未按 PX4 打包格式编码（主模式在 bit16-23、自动子模式在 bit24-31），MAVSDK 按 PX4 解码导致回读的飞行模式永远是 Unknown。修复：心跳按 PX4 打包发送，并补全大疆飞行模式的标准主模式映射（P 模式/起飞就绪→POSCTL、ATTI→ALTCTL、自动起飞/降落/返航/跟随→AUTO+对应子模式）；子模式状态移入组件基类 | `communication/components/MavBaseComponent.kt`、`communication/components/Drone.kt` |
| `MAV_CMD_DO_SET_MODE` 携带未知的自定义模式时原实现抛出异常且被静默吞掉，地面站收不到任何应答。修复：解析失败时返回命令失败 | `communication/components/Drone.kt` |
| 仿真遥测两处修复：一、`SimulatorState` 的 positionX/Y/Z 是以起飞点为原点的北东地（NED）坐标系，Z 轴朝下，原实现直接把 `positionZ` 当高度使用，爬升时高度显示为负值，现取反后桥接；二、`GLOBAL_POSITION_INT` 与 `GPS_RAW_INT` 原先要求速度键有推送才发送，部分机型/仿真下速度键不推送导致这两条消息从未发出，地面站看不到位置与高度，现速度未知时回退为 0，保证消息始终发送 | `communication/components/Drone.kt` |
| 电池组件的组件序号与低电量阈值改为外部可读，供解锁前检测使用 | `communication/components/Battery.kt` |

## 四、已知注意事项

1. **大疆 SDK 对应用进程状态非常敏感。** 重新安装、热替换代码之后，SDK 的初始化回调可能不再触发，进而引发注册失败、组件不上线、启动错误等一系列现象。规范操作：每次重新构建安装后，先彻底结束应用进程再启动；出现异常行为时，先彻底重启应用，必要时重启手机与飞机。

2. **部分机型上部分键值不推送。** 本项目使用的机型（DJI Mini 4 Pro）上，连接状态键、模拟器状态键、电池数据键等均出现过不推送的情况。目前已通过轮询兜底、聚合键回退等手段覆盖；若日志出现 `Battery data unavailable`，表示两个电池数据源均无数据，属于机型能力边界。

3. **模拟器遥测在消费级机型上不完整。** 大疆官方模拟器主要面向行业机型（M300/M350 RTK、M30 系列、Mavic 3 行业系列）。本应用已迁移到官方 `SimulatorManager` 接口，但若状态回调仍不产出数据，说明该机型不支持模拟遥测。模拟器面板内 uxsdk 自带的 World-X 等遥测显示可能始终为空，请以地面站实际收到的数据为准。

4. **`LOCAL_POSITION_NED` 由 GPS 相对 Home 点的偏移派生。** DJI 不提供本地位置，本应用用飞机位置相对 Home 点的经纬度差换算北/东坐标（高度取反作为 NED 的 z，速度取自大疆速度键、无数据时为 0），以 10 Hz 发送 `LOCAL_POSITION_NED`，MAVSDK 的 `position_velocity_ned` 订阅因此有数据；其精度受 GPS 限制，不如真实 EKF 的本地定位平滑。

5. **解锁前检测的电量门槛。** 拒绝解锁的条件是真实电量低于低电量报警阈值（大疆键 `KeyLowBatteryWarningThreshold`，应用兜底默认 20%），可通过参数 `BAT_LVL_WARN`（范围 15 至 50）调整；写入超出真机允许范围的值会被拒绝并回传当前值。解锁前检测只作用于 MAVLink 解锁路径，遥控器手动操作走大疆自身逻辑。

6. **无线链路存在间歇性丢包。** 地面站与手机之间的 WiFi 偶发丢包会表现为地面站心跳丢失（连续 4 次未收到即判离线，可通过参数 `SYS_GCS_HB_MISS` 调整）与偶发指令超时，属于环境因素。

7. **RTK 组件未实现任何 MAVLink 消息。** 组件存在但仅保留扩展位。

8. **调试日志关键字速查。** 以下日志可快速定位对应环节：
   - `Successfully registered application`：大疆 SDK 注册成功；
   - `Start sending Mavlink messages for MAV_COMP_ID_AUTOPILOT1`：飞控组件上线，之后地面站命令才会被处理；
   - `Simulator home set to ...`：模拟器状态回调有数据，桥接生效；
   - `Battery data unavailable`：两个电池数据源均无数据；
   - `Received battery overview`：聚合电池键有数据；
   - `Prearm checks passed` / `Cannot arm drone, prearm checks failed`：解锁前检测结果；
   - `Start virtual stick climb to ...` / `Climb height of ... reached` / `Climb to ... failed`：虚拟摇杆爬升过程；
   - `Cannot send message ...`：某条消息构建或发送异常。

9. **工作区中的其他既有调整**（目标系统版本、MultiDex、地图组件地图样式等）不属于本文档记录范围。
