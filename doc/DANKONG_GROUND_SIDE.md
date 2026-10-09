# dankong 地面站接入说明（WSL）

[dankong](https://github.com/HGGshiwo) 是本项目的地面站/机载计算机侧工程
（ROS1 Noetic，运行于 WSL Ubuntu-20.04），负责：MAVSDK→MAVLink 的飞行控制、
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

## 2. 链路总览

| 链路 | 地址 | 方向 |
|---|---|---|
| MAVLink 控制 | `tcp://<controller-ip>:5760`（dankong `mavsdk_url` 配置） | dankong MAVSDK → 本 App |
| 下视相机流 | `http://<controller-ip>:8090/stream`，MJPEG 640×480 @ ~15 fps | 本 App → dankong `down_cam_node.py` |
| 云台 | rsos service `/UAV0/sensor/serial_gimbal/set_gimbal_angle` | dankong `GimbalBridge` → 本 App 云台组件（compid 154） |

## 3. 云台

云台语义全部在本 App 侧实现（见 [MAVLINK_COMPATIBILITY.md](MAVLINK_COMPATIBILITY.md)）：
`DO_MOUNT_CONFIGURE` 任意模式映射 `YAW_FOLLOW`（yaw 随机体，pitch/roll 世界系），
`DO_GIMBAL_MANAGER_PITCHYAW` 为一次性世界系 pitch（按指令时刻飞机姿态换算，
按机体可控范围 clamp，Mini 4 Pro 约 -90°~+35°；转速为硬件上限，约 2-3 s/90°）。

`GimbalBridge` 侧约定：

- rsos `angle` 正值向下（90 = 垂直向下），桥接层转换为 DJI 负值向下符号；
- `body` / `abs` 均接受且行为一致（接口兼容保留，机体相对跟随不做模拟）；
- 开关 `gimbal_bridge_enable: true`（`dji-config.yaml`）；串口云台机型保持
  false，service 由各自驱动节点提供；
- 本产品不提供 `KeyGimbalAttitude`，因此**没有云台姿态回读**
  （不广播 `GIMBAL_DEVICE_ATTITUDE_STATUS`）；pland 需要云台角时使用
  "已知指令角"模型（`pland_gimbal_abs=true`，世界系语义与本行为一致）。

## 4. 下视相机

- 源流 16:9（853×480），App **居中裁切**为 640×480（左右各裁 ~107 px，1:1
  像素无缩放无变形）。
- `camera_inner_matrix` 按裁切后图像标定：居中裁切只平移 `cx`
  （853 宽几何中心 426.5 → 320.5），`fx/fy` 沿用源相机。
- dankong 侧 `down_cam_node.py` 已内置两个规避：清除代理环境变量（FFmpeg
  会读 `http_proxy`，代理不在机载网段会导致打不开流）；手工填充
  `sensor_msgs/Image`（Noetic cv_bridge 与新版 numpy/OpenCV 组合存在
  `KeyError: 16` 兼容性问题）。

## 5. 注入点同步（重要）

`camera_inner_matrix` 与图像话题有**两处注入点**，修改标定/话题时必须同步：

| 注入点 | 消费方 |
|---|---|
| `config/dji-config.yaml`（`camera_inner_matrix`、`pland_image_topic`） | dankong 内部 `LandingDetector`（Web 接口 `/start_pland_detect`、`/offset_estimate/start` 触发） |
| `launch/dankong.launch` → `pland_detector.launch`（`camera_inner_matrix`、`image_topic`） | 外部 pland_detector 节点（精准降落主流程） |

## 6. 降落与落地保护

- 普通降落（dankong `robot->land()` = DJI auto-land）：固件自动下压云台。
- pland 精准降落（Offboard）：固件不管云台，由 `SetGimbalEvent(90)` 下压。
- **落地保护**：下降到离地约 0.5-1 m 时固件暂停下降并请求"滑动确认降落"，
  此高度以下所有下降指令（含虚拟摇杆）被忽略。本 App 常驻监视
  `KeyIsLandingConfirmationNeeded` 并自动 `KeyConfirmLanding`，确认后下降
  继续。App logcat 关键字：`Landing confirmation needed`。
