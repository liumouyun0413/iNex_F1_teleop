# f1_rokae_teleop —— F1（珞石 Rokae 双臂 + iNex ROS2）PICO 遥操作

按 `f1_tianji_teleop` 已验证的架构（XRoboToolkit **Galaxea R1 Lite 真机示例**范式）重写的
F1-珞石版遥操作程序，功能对齐 `lxl_origin_rokae_v1.1`（`start_all_pico2_log.sh` 启动的三进程方案）：
双臂 IK 遥操（iNex ServoJ 流）、夹爪、腰部升降、底盘摇杆、复位、数采、外部位姿回放，
并保留 `f1_tianji_teleop` 的头部摇杆模式（可选）与质心/碰撞安全门（可选）。

与 `f1_tianji_teleop` 的差异只在**硬件层**：天机 `/motion_ctl/joint_ctl` 话题 → iNex `/inex/servo_cmd` ServoJ 流，
`/hal/joint_states`（度）→ `/inex/joint_states`（弧度，按前缀解析），以及 iNex 的上电/模式/ServoJ 准备时序。
底盘、腰部节点、数采、手柄逻辑与 IK 核心完全复用。

## 目录结构

```
f1_rokae_teleop/
├── start_teleop.sh / stop_teleop.sh      一键启停（腰部节点 + RoboticsService + 遥操主程序）
├── scripts/
│   ├── teleop_f1_hardware.py             入口（≈ 官方 teleop_r1lite_hardware.py）
│   └── check_joint_map.py                ★ 首次上机必跑：JointMap 方向/零位核对（passive / jog）
├── f1_teleop/                            Python 包
│   ├── config/f1_teleop.yaml             ★ 全部参数（话题、关节映射、位姿、iNex 时序、规划器、安全、数采）
│   ├── common/
│   │   ├── xr_client.py                  PICO SDK 封装（官方同款）
│   │   ├── base_teleop_controller.py     Placo IK 核心：三模式、正则化、构型同步、摆率限制、
│   │   │                                 位置限位 qp/clamp 两种模式（本 URDF 默认 clamp，见下）
│   │   └── hardware_teleop_controller.py 多线程骨架：ik / control / aux 线程 + run()
│   ├── hardware/
│   │   ├── f1_teleop_controller.py       ★ F1RokaeTeleopController（≈ 官方 galaxea_r1_lite_teleop_controller.py）
│   │   └── interface/                    硬件接口（≈ 官方 hardware/interface/galaxea.py）
│   │       ├── ros_context.py            进程内唯一 rclpy 节点 + spin 线程
│   │       ├── inex_client.py            ★ iNex 服务封装：心跳/上电/模式/profile/ServoJ/软限位/夹爪初始化/Halt
│   │       ├── inex_arm.py               ★ 双臂：/inex/servo_cmd 发布、/inex/joint_states 解析、S 曲线运动、
│   │       │                               ServoJPlanner 速度受限规划器（lxl 桥接 send_loop 的等价实现）
│   │       ├── inex_head.py              ★ 头部（ServoJ part=3，默认关闭）
│   │       ├── f1_waist.py               腰部升降（→ EC_Line_Control 节点 → /inex/move_to）
│   │       ├── f1_chassis.py             底盘速度 + SetControlMode（未变）
│   │       ├── safety_guard.py          碰撞/质心安全门（订阅外部检测节点 /f1_safety/is_safe）
│   │       ├── recording.py              ros2 bag 录制 + 外部相机录制器信号
│   │       ├── lead_publisher.py         主端数据镜像 /lead/*
│   │       └── external_pose_input.py    外部位姿回放输入
│   └── utils/                            geometry / control_utils(JointMap, 边沿, 限频) / profiler / path_utils
├── assets/huaqin_20260924/               当前使用的 URDF + meshes（iNex 原生 22 轴关节名，关节坐标=驱动坐标）
├── assets/f1l01/                         旧 珞石 f1l01 URDF（lxl 珞石版；零位差 180°，仅参考）
├── EC_Line_Control/                      （已有）腰部升降 ROS2 节点（iNex 版）
├── interface_20260623/                   （已有）底盘 ROS2 接口定义（move/srv/SetControlMode 等）
├── rokae-v1.1/                           （已有，只读）iNex SDK / ROS2 驱动与接口定义资料
└── XRoboToolkit-Teleop-Sample-Python-1.0.3/  （已有，只读）官方参考包
```

## 运行

```bash
# 机器人上：ROS2 jazzy + iNex 工作区（~/workspace/inex_ros，接口包 v1.0）+ conda pico4ultra_py312
./start_teleop.sh                          # TELEOP_ARGS="--visualize-placo" ./start_teleop.sh
tail -f log/teleop_*.log
./stop_teleop.sh                           # 先 SIGINT（程序内 /inex/halt 平滑制动）再清理

# 手动运行
source /opt/ros/jazzy/setup.bash && source ~/workspace/inex_ros/install/setup.bash
conda activate pico4ultra_py312            # 注意 start_teleop.sh 里有 LD_LIBRARY_PATH 清理，手动跑请照抄
python scripts/teleop_f1_hardware.py --help
```

启动流程：等待 `/inex/joint_states` → **iNex 准备**（关单点心跳 → 按 powered_mask 逐臂上电并复核 →
机器人模式 DualArmOnly/WholeBody → 双臂 JOINT_POSITION profile → ServoJ）→ 等驱动订阅 `/inex/servo_cmd` →
用 `/inex/get_soft_limits` 收紧 IK 限位 → 双臂 S 曲线回 `home_pose`（5 s）→ Placo / 规划器同步 → 自检反馈偏差 →
夹爪初始化（`/inex/init_gripper`，状态 30 Hz）→ 安全门（可选）→ 启动 ik / control / aux 三线程。
退出时 `/inex/halt(0xFF)` 平滑制动（不下电）。

## 手柄操作

| 输入 | 功能 |
|---|---|
| 握把 > 0.9 | 激活该臂，手柄位姿增量驱动末端 |
| 扳机 | 夹爪（0→张开指令 100，1→闭合指令 0；`/motion_ctl/gripper/*` 由 iNex 驱动内置夹爪组件消费） |
| 左摇杆按压 / 右摇杆按压 / 同时 | 末端模式 position / orientation / pose |
| X + Y | 双臂复位到 `reset_pose`（2 s S 曲线，Placo 同步） |
| Y 长按 0.3s / X 长按 0.3s | 腰部上升 / 下降一步（`/control/waist/updown` → EC_Line_Control → `/inex/move_to`） |
| A / B | 开始 / 停止数采（ros2 bag + 外部录制器信号） |
| 右扳机按住且双握把松开 | 头部模式（`head.enable=true` 时下发 ServoJ part=3） |
| 右摇杆上拨 / 下拨 | 底盘使能 / 停止；使能后左摇杆平移、右摇杆 X 旋转 |

## 关键设计

* **单进程**：lxl 珞石版 teleop → UDP 8888 → all_remote_lxl（规划器 + ServoJ）双进程，改为单进程直接发布
  `/inex/servo_cmd`，去掉一跳 UDP 与"先发送上一帧解再 solve"的结构性 +1 帧延迟。
* **ServoJPlanner**（`inex.servo_planner`）：保留 lxl 桥接验证过的速度受限规划器（vmax 90 °/s、amax 300 °/s²、
  τ 0.04/0.25 s、制动前瞻），在控制线程 100 Hz 把 IK 目标摆成速度连续的 ServoJ 流；目标停更自动刹停保持。
  首次有臂激活后持续下发；复位期间由复位线程独占通道，结束后规划器重新锚定。
* **开环虚拟机器人**：Placo 只在启动/复位时与真机同步一次，之后仅由手柄驱动。
* **统一关节映射** `JointMap`：`deg = sign·rad2deg(rad) + offset`。参数由 lxl 珞石版 `angle_map` +
  `sync_placo_to_real` + 启动偏差补偿在 home 位姿处合并推导（见 yaml 注释）。
  ⚠️ lxl 原码两套映射在 L1/L3/L4/R3/R4/R5 等关节符号并不自洽，仅在 home 处数值一致——
  **首次上机请先跑 `scripts/check_joint_map.py`**：`--mode passive` 核对零位，`--mode jog --arm left --joint 7 --deg 5`
  逐关节点动，用驱动自己的 `/inex/tcp_pose` 作独立真值核对方向；方向反了就把该关节 `sign` 取反、
  `offset = home_deg − sign·rad2deg(q_home)` 重算。
* **位置限位 clamp 模式**：lxl 在本 URDF 上实测 Placo 内置位置限位错位（J7 套用了夹爪 prismatic 关节的 [0, 0.04]），
  故默认 `teleop.ik.joint_limit_mode: clamp`（求解后按 q 索引钳位，限位 = URDF ∩ 驱动软限位 − 3°）；
  速度限位不受影响仍交给 QP。换 URDF 后可改回 `qp`。
* **安全门**：碰撞/质心检测由独立节点 `realtime_joint_safety_monitor.py`（cyb `SafetyProtector`，读 `/inex/joint_states`）完成，
  发布 `std_msgs/Bool` 到 `/f1_safety/is_safe`；遥操只订阅该结果，False 时双臂冻结在最近安全指令。默认 `safety.enable: false`。
* **所有魔法数字进 yaml**：话题名、关节名、位姿、iNex 时序参数、规划器参数、阈值、权重、限位。

## 与 lxl_origin_rokae_v1.1 的对应关系

| lxl 珞石版 | 本项目 |
|---|---|
| `remote_sth/init_f1.py` `_prepare_system` / `init_grippers` / `smooth_halt_all` | `interface/inex_client.py` |
| `remote_sth/init_f1.py` `plan_and_move_both_arms`（S 曲线）/ 反馈解析 | `interface/inex_arm.py` `move_to` / `resolve_joint_layout` |
| `remote_sth/send2real_robot_f1_udp2topic.py` `send_loop` 规划器 | `interface/inex_arm.py` `ServoJPlanner` |
| `xrobotoolkit_teleop/common/send2real_robot_f1_udp.py` `angle_map` + `sync_placo_to_real` + 偏差补偿 | `JointMap`（yaml `robot.joint_map`） |
| `base_teleop_controller_ik_mixin._clamp_arm_joint_limits` | `base_teleop_controller` `joint_limit_mode: clamp` |
| `base_teleop_controller_ik_mixin._limit_send_delta` | 规划器 vmax/amax（或 `max_joint_speed_deg_s`） |
| `collision_check_worker.py` | 独立节点 `realtime_joint_safety_monitor.py` + `interface/safety_guard.py`（订阅 `/f1_safety/is_safe`） |
| `ip_config.py`（IP/端口/位姿） | yaml `robot.home_pose/reset_pose`（UDP 已取消） |
| `start_all_pico2_log.sh` / `stop_all_pico.sh` | `start_teleop.sh` / `stop_teleop.sh` |

## 依赖

ROS2 jazzy（rclpy, sensor_msgs, control_msgs, geometry_msgs, std_msgs），`inex_ros2_interface` **v1.0**
（含 `ServoCommand` / `RobotSystemStatus` / `TcpPosition`，由 `~/workspace/inex_ros` 工作区提供），
`move` 接口包（interface_20260623），`xrobotoolkit_sdk`，`placo==0.9.4`，`meshcat`，`numpy`，`pyyaml`，`tyro`；
安全门需先启动 `realtime_joint_safety_monitor.py`（依赖 `cyb_safe_control_real`）发布 `/f1_safety/is_safe`。
