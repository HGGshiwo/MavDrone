# dankong 地面站接入说明（WSL）

[dankong](https://github.com/HGGshiwo) 是本项目的地面站/机载计算机侧工程
（ROS1 Noetic，运行于 WSL Ubuntu-20.04），负责：MAVSDK/MAVLink 的飞行控制、
精准降落（pland）、下视相机拉流（`down_cam_node.py`）、以及把 rsos 云台服务
桥接到本 App 的 MAVLink 云台接口（`GimbalBridge`）。

## 1. 访问 dankong

| 项 | 值 |
|---|---|
| 位置 | WSL Ubuntu-20.04：`/home/hggshiwo/catkin_ws/src/dankong` |
| Windows 侧访问 | `\\wsl.localhost\Ubuntu-20.04\home\hggshiwo\catkin_ws\src\dankong` |
| 编译 | `wsl.exe -d Ubuntu-20.04 -- bash -lc "source /opt/ros/noetic/setup.bash && cd ~/catkin_ws && catkin build dankong --no-status --no-deps"` |
| 运行 | `roslaunch dankong dji.launch`（配置：`config/dji-config.yaml`） |

注意：

- 仓库有 pre-commit 钩子（black / isort / clang-format），格式不符会自动改写
  并中止提交，重新 `git add` 再提交；钩子会重置 python 脚本的可执行位，提交后
  检查 `chmod +x scripts/*.py`。
- dankong 启动时若连不上本 App（`connect to fcu failed`）会直接 abort，先确认
  App 的 MAVLink server 已启动。
- 若 App 显示的 Server address 是 172.x/10.x 网段且连不上：历史版本取的是系统
  默认路由网络的首个 IPv4，遥控器开了蜂窝数据（10.x）、USB 共享或 VPN
  （172.16-31.x）时显示的就是不可达地址；现版本已改为优先取 `wlan*`/`eth*`
  接口并在每轮等待连接时刷新。服务端实际绑定 `0.0.0.0:5760`，用
  `adb shell ip addr show wlan0` 的地址填 `mavsdk_url` 即可连接。
- **offboard/虚拟摇杆有 guided 准入门控**：App 界面左侧新增 GUIDED 按钮，只有
  guided 状态下才响应速度流与起飞爬升。进入 guided 的途径：MAVSDK
  `Offboard::start()`（协议会先发 `DO_SET_MODE`，正常路径不受影响）、
  `NAV_TAKEOFF`（带爬升目标时自动进入）、或直接对 App 发 `DO_SET_MODE`
  offboard/guided。若某路径不设模式直接流指令，会被拒（STATUSTEXT
  `Virtual stick denied: not in guided mode`，限频 2 s）——联调时看到该文本
  即说明流指令前缺一次模式设置。
- **应急接管**：遥控器切档/急停短按/长按返航夺走控制权时，App 自动退出 guided
  并锁存（发 STATUSTEXT `Guided interrupted: RC took control`，GUIDED 按钮变
  灰）；此后 MAVSDK 需重新 `Offboard::start()` 才能拿回控制。起飞爬升结束
  （飞行模式回到位置模式）与落地也会自动退出 guided。

## 2. 链路总览

| 链路 | 地址 | 方向 |
|---|---|---|
| MAVLink 控制 | `tcp://<controller-ip>:5760`（dankong `mavsdk_url` 配置） | dankong MAVSDK/MAVLink → 本 App |
| 下视相机流 | `http://<controller-ip>:8090/stream`，MJPEG 640×480 @ ~15 fps | 本 App → dankong `down_cam_node.py` |
| 云台 | rsos service `/UAV0/sensor/serial_gimbal/set_gimbal_angle` | dankong `GimbalBridge` → 本 App 云台组件（compid 154） |
| 透传指令 | `/mavros/cmd/command`（`mavros_msgs/CommandLong` 原样转发） | dankong `MavrosBridge` → 本 App |

## 3. 云台（语义在本 App 侧实现）

`DO_MOUNT_CONFIGURE` 任意 mount mode 一律映射 DJI `YAW_FOLLOW`：

| 轴 | 参考系 | 行为 |
|---|---|---|
| yaw | 机体 | 跟随机头 |
| pitch | **世界系** | 一次性指令（按指令时刻飞机姿态换算 + 机体限位钳制，Mini 4 Pro 约 -90°~+35°），固件稳定器保持 |
| roll | 世界系 | 保持水平 |

- `body` / `abs` 均接受且**行为一致**（接口兼容保留）；机体相对跟随不做模拟
  （DJI 稳定器按世界系保持，软件模拟无法做到确定性）。
- 转速为硬件上限（约 2-3 s / 90°），调用方需等云台到位。
- 本产品不提供 `KeyGimbalAttitude`，因此**没有云台姿态回读**
  （不广播 `GIMBAL_DEVICE_ATTITUDE_STATUS`）；pland 需要云台角时使用
  "已知指令角"模型（`pland_gimbal_abs=true`，世界系语义与本行为一致）。

## 4. 解锁/上锁（ARM_DISARM）

`MAV_CMD_COMPONENT_ARM_DISARM`（400）：

- `param1=1`：武装（仅置 MAVLink 状态标志，电机实际启动由 takeoff/虚拟摇杆完成）；
- `param1=0`：disarm——**飞行中一律拒绝**，地面正常执行（清 ARMED 标志）；
- `param2=21196`（MAVLink 强制停桨魔数，QGC Kill 按钮同款）**不做任何特殊
  处理**，与普通 disarm 规则相同。

原因：DJI 没有空中停桨接口。`KeyLockMotors` 是**未起飞状态下的电机锁**
（锁定后掰杆无法启动电机，防盗/运输用途），空中调用它不会停桨，只会触发
"锁定飞机、要求更新固件"提示（2026-10-10 真机实测：约 0.1-0.3 m 高度发送
后电机未停，App 弹出锁机/固件更新提示）。此前的 `param2=21196 →
KeyLockMotors` 映射已删除。降落到移动平台的接触断电：触地后的最终停桨由
固件落地自动停桨兜底，接触瞬间切断需另行评估（当前无可用接口）。

对 dankong 的兼容性：`pland_controller` 的 `trigger_disarm()`（LANDED 态、
触地后发送 400/21196）不受影响——App 忽略 param2，触地后按普通 disarm
正常执行；若在空中发送则会被拒绝（此前也从未生效过）。

## 5. 降落与落地保护

- 普通降落（dankong `robot->land()` = DJI auto-land）：固件自动下压云台。
- pland 精准降落（Offboard）：固件不管云台，由 `SetGimbalEvent(90)` 下压。
- **落地保护（2026-10-10 bag 实测 + DJI 客服确认根因）**：冻结由**下视避障 +
  降落保护两层门控**造成——行业机只需关下避障；消费机型（Mini 4 Pro/Mini 3/
  Mini 3 Pro）需同时关闭两者，虚拟摇杆才能降到底。App 现已在**虚拟摇杆接管
  期间自动关闭两者、归还时自动恢复**（参数 `VS_LAND_PROT`，默认开，地面可
  读写）。此前 bag 实测的现象与结论仍然有效：
  - *auto-land*：地面判定无需确认时直接落底；需要确认时置位
    `KeyIsLandingConfirmationNeeded`（官方文档的"0.7 m 悬停等确认"），本 App
    常驻监视并自动 `KeyConfirmLanding`。App logcat 关键字：
    `Landing confirmation needed`。
  - *虚拟摇杆/offboard 速度下降（保护未关闭时）*：气压相对高度约 0.6-0.8 m
    （真实离地约 0.3 m）处冻结一切下降指令且 confirmation 键从不置位，气压
    读数同时死钉不变。**悬停没有超时兜底，不会自行恢复**——实测两次均是人工
    在 App 端手动点降落才恢复（日志仅见 `AUTO_LANDING` 模式切换，此前无任何
    MAVLink 命令）。App 检测到"持续下令下降 + 垂直速度为零 + 高度 ≤2 m"达
    3 秒会发 STATUSTEXT：`Landing protection: descent blocked at ...`，作为
    保护关闭失效时的保险告警。
  - logcat/App 日志关键字：`Virtual stick: downward avoidance + landing
    protection disabled/restored`（App 会话日志可确认开关动作已执行）。
- **真实离地高（AGL）**：本 App 现已发送 `DISTANCE_SENSOR`（下视超声/TOF，
  5 Hz）。dankong 既有链路（MAVSDK `subscribe_distance_sensor` →
  `ctx.rangefinder_alt` → `/mavros/distance_sensor/rangefinder_pub`）无需改
  代码即激活；pland `get_current_z` 在 3 m 以下优先采用测距值（`frame_height`
  补偿起落架高度），触地/盲降判决不再依赖漂移且会被冻结的气压高度。
  保护关闭后虚拟摇杆可正常降到底，pland 现有配置（`exit_alt` 等）预期可
  直接走通，无需再依赖"低空切 LAND"兜底。
- **低空模式切换（实测确认）**：pland 流程在低空会主动切
  `GUIDED → LAND`（`pland_controller` 的 BLIND_DROP 或 `LandState` 判定），
  之后固件接管下降、offboard 速度指令不再生效——这是固件模式语义，不是
  故障；触地后的电机停止由固件落地自动停桨完成（DJI 无空中停桨接口，
  见第 4 节）。

## 6. 调试与日志

| 日志 | 位置 | 读取方式 |
|---|---|---|
| App 会话日志（全部 `Mav` tag） | `/sdcard/Android/data/nl.wur.mavdrone/files/MavlinkLogs/<启动时间>.txt` | `adb shell cat <文件>`（files 目录 adb 可直读） |
| logcat 环形缓冲 | 设备内存 | `adb logcat -d -v time`（App 重启不清，**装 APK 会重置**——先抓 logcat 再装） |
| DJI SDK 内部日志 | `files/LOG/...` | **加密**，不可读 |

版本确认：`adb shell dumpsys package nl.wur.mavdrone | grep versionName`，
格式为 `1.0-<MMdd_HHMM>`（构建时间戳，分钟级），一眼可辨新旧。

排查顺序建议：复现问题 → `adb logcat -d` 抓现场 → 测试结束后
`adb shell cat` 拉会话文件。不要在抓取 logcat 之前安装 APK。

## 7. 注入点同步（重要）

`camera_inner_matrix` 与图像话题有**两处注入点**，修改标定/话题时必须同步：

| 注入点 | 消费方 |
|---|---|
| `config/dji-config.yaml`（`camera_inner_matrix`、`pland_image_topic`） | dankong 内部 `LandingDetector`（Web 接口 `/start_pland_detect`、`/offset_estimate/start` 触发） |
| `launch/dankong.launch` → `pland_detector.launch`（`camera_inner_matrix`、`image_topic`） | 外部 pland_detector 节点（精准降落主流程） |
