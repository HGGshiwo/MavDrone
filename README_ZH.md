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
| 电池组件的组件序号与低电量阈值改为外部可读，供解锁前检测使用 | `communication/components/Battery.kt` |

## 四、已知注意事项

1. **大疆 SDK 对应用进程状态非常敏感。** 重新安装、热替换代码之后，SDK 的初始化回调可能不再触发，进而引发注册失败、组件不上线、启动错误等一系列现象。规范操作：每次重新构建安装后，先彻底结束应用进程再启动；出现异常行为时，先彻底重启应用，必要时重启手机与飞机。

2. **部分机型上部分键值不推送。** 本项目使用的机型（DJI Mini 4 Pro）上，连接状态键、模拟器状态键、电池数据键等均出现过不推送的情况。目前已通过轮询兜底、聚合键回退等手段覆盖；若日志出现 `Battery data unavailable`，表示两个电池数据源均无数据，属于机型能力边界。

3. **模拟器遥测在消费级机型上不完整。** 大疆官方模拟器主要面向行业机型（M300/M350 RTK、M30 系列、Mavic 3 行业系列）。本应用已迁移到官方 `SimulatorManager` 接口，但若状态回调仍不产出数据，说明该机型不支持模拟遥测。模拟器面板内 uxsdk 自带的 World-X 等遥测显示可能始终为空，请以地面站实际收到的数据为准。

4. **本项目不发送 `LOCAL_POSITION_NED` 消息。** 仅发送 `GLOBAL_POSITION_INT`，地面站会自行派生本地位置；MAVSDK 的 `position_velocity_ned` 订阅因此不会有数据。

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
   - `Cannot send message ...`：某条消息构建或发送异常。

9. **工作区中的其他既有调整**（目标系统版本、MultiDex、地图组件地图样式等）不属于本文档记录范围。
