"""
F1RokaeTeleopController —— F1（珞石 Rokae 双臂 + iNex ROS2 驱动）真机遥操作控制器。

对应官方示例 GalaxeaR1LiteTeleopController 的角色：把通用 IK 骨架
（HardwareTeleopController）与 F1-Rokae 的硬件接口（iNex 双臂 / 夹爪 / 头 / 腰 / 底盘 /
安全门 / 数采）组装起来。结构与 f1_tianji_teleop 的 F1TeleopController 完全一致，
差异只在双臂接口（ServoJ 流 + 速度受限规划器）与启动时序（iNex 上电/模式/ServoJ）。

手柄按键一览
------------
  握把(grip) > 0.9          激活对应臂，手柄位姿增量 → 末端位姿
  扳机(trigger)             夹爪开合（0 → 张开指令，1 → 闭合指令）
  left_axis_click           末端模式 = position（纯位置）
  right_axis_click          末端模式 = orientation（纯姿态）
  两摇杆同时按压            末端模式 = pose（位置+姿态，默认）
  X + Y                     双臂复位到 reset_pose（S 曲线），Placo 同步
  Y 长按 0.3s               腰部上升一步        X 长按 0.3s   腰部下降一步
  A                         开始数采            B             停止数采
  右扳机按住 + 双握把松开   头部模式（head.enable=true 时下发 ServoJ part=3）
  右摇杆上拨                底盘使能            右摇杆下拨    底盘停止
  底盘使能后：左摇杆 → 前后/横移，右摇杆 X → 旋转（头部模式期间挂起）

设计说明
--------
* Placo 是开环"虚拟机器人"：启动 / 复位时用真机位姿同步一次，之后只由手柄驱动。
* Placo 弧度 ↔ 驱动度 的映射集中在 JointMap，正反两向共用同一组参数。
* ServoJ 流在首次有臂激活后持续 100Hz 下发（规划器保持目标），复位期间由复位线程独占通道。
"""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace
from typing import Dict, Optional, Tuple

import numpy as np

from f1_teleop.common.hardware_teleop_controller import HardwareTeleopController
from f1_teleop.hardware.interface import (
    BagRecorder,
    ComGuard,
    ExternalPoseInput,
    F1ChassisController,
    F1WaistController,
    InexArmController,
    InexClient,
    InexHeadController,
    LeadDataPublisher,
    RecordingManager,
    RosContext,
    SafetyGuard,
    ServoJPlanner,
    TianjiRecorderClient,
)
from f1_teleop.hardware.interface.inex_client import PART_LEFT_ARM, PART_RIGHT_ARM
from f1_teleop.utils.control_utils import HoldTrigger, JointMap, RateLimiter, RisingEdge

ARM_ORDER = ("left_arm", "right_arm")
_SIDE = {"left_arm": "left", "right_arm": "right"}
_PART = {"left_arm": PART_LEFT_ARM, "right_arm": PART_RIGHT_ARM}


def build_manipulator_config(cfg: SimpleNamespace) -> Dict[str, dict]:
    r, g = cfg.robot, cfg.teleop.gripper
    return {
        "left_arm": {
            "link_name": r.left_ee_link,
            "joint_names": list(r.urdf_left_joint_names),
            "shoulder_link": getattr(r, "left_shoulder_link", None),
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "gripper_config": {"gripper_trigger": "left_trigger", "open_pos": g.open_cmd, "close_pos": g.close_cmd},
        },
        "right_arm": {
            "link_name": r.right_ee_link,
            "joint_names": list(r.urdf_right_joint_names),
            "shoulder_link": getattr(r, "right_shoulder_link", None),
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "gripper_config": {"gripper_trigger": "right_trigger", "open_pos": g.open_cmd, "close_pos": g.close_cmd},
        },
    }


class F1RokaeTeleopController(HardwareTeleopController):
    WARN_HAND_EPS_M = 3e-4        # 手柄每帧位移 ±0.3 mm（3 cm/s）以内视为中性（静止/横移/抖动）

    def __init__(self, cfg: SimpleNamespace, visualize_placo: bool = False) -> None:
        self.cfg = cfg
        r, t = cfg.robot, cfg.teleop

        self.joint_map: Dict[str, JointMap] = {
            "left_arm": JointMap.from_lists(r.joint_map.left.sign, r.joint_map.left.offset_deg),
            "right_arm": JointMap.from_lists(r.joint_map.right.sign, r.joint_map.right.offset_deg),
        }
        self.home_pose_deg = np.array([*r.home_pose.left, *r.home_pose.right], dtype=float)
        self.reset_pose_deg = np.array([*r.reset_pose.left, *r.reset_pose.right], dtype=float)

        super().__init__(
            robot_urdf_path=r.urdf_path,
            manipulator_config=build_manipulator_config(cfg),
            scale_factor=t.scale_factor,
            visualize_placo=visualize_placo,
            control_rate_hz=int(t.control_rate_hz),
            aux_rate_hz=float(cfg.ros.lead_publish_rate_hz),
            grip_activate_threshold=t.grip_activate_threshold,
            frame_task_weight=t.ik.frame_task_weight,
            manipulability_weight=t.ik.manipulability_weight,
            joint_regularization_weight=t.ik.joint_regularization_weight,
            workspace_radius=t.workspace_radius,
            floor_z_min=getattr(t, "floor_z_min_m", None),
            tool_length=float(getattr(t, "tool_length_m", 0.0) or 0.0),
            max_ee_speed=getattr(t, "max_ee_speed_m_s", None),
            max_ee_ang_speed=getattr(t, "max_ee_ang_speed_rad_s", None),
            enable_joint_velocity_limits=bool(getattr(t.ik, "enable_joint_velocity_limits", False)),
            joint_limit_mode=getattr(t.ik, "joint_limit_mode", "clamp"),
        )
        # 先用配置表收紧限位；连上驱动后再用 /inex/get_soft_limits 的实测值覆盖
        self._apply_driver_joint_limits(self._limits_cfg_to_dict(getattr(r, "arm_joint_limits_deg", None)))
        self.sync_arm_joints(self._deg14_to_placo(self.home_pose_deg))

        # 运行时状态
        self._resetting = threading.Event()
        self._streaming = False            # 首次有臂激活后开始持续 ServoJ 流
        self._head_active = False
        self._xy_edge = RisingEdge()
        self._y_hold = HoldTrigger(t.waist_button_hold_sec)
        self._x_hold = HoldTrigger(t.waist_button_hold_sec)
        self._a_edge = RisingEdge()
        self._b_edge = RisingEdge()
        self._base_up_edge = RisingEdge()
        self._base_down_edge = RisingEdge()
        self._gripper_limiter = RateLimiter(cfg.teleop.gripper.publish_rate_hz)
        self._max_step_deg = float(getattr(t, "max_joint_speed_deg_s", 180.0)) / self.control_rate_hz

        # 硬件接口在 _robot_setup 中创建
        self.ros: Optional[RosContext] = None
        self.inex: Optional[InexClient] = None
        self.arm: Optional[InexArmController] = None
        self.head: Optional[InexHeadController] = None
        self.chassis: Optional[F1ChassisController] = None
        self.waist: Optional[F1WaistController] = None
        self.lead_pub: Optional[LeadDataPublisher] = None
        self.external_input: Optional[ExternalPoseInput] = None
        self.recorder: Optional[RecordingManager] = None
        self.safety: Optional[SafetyGuard] = None
        self.com_guard: Optional[ComGuard] = None
        self._com_state = "OK"
        self._com_last_print = 0.0
        self._warn_ref: Optional[np.ndarray] = None   # WARN：驱动质心 - 模型质心 的 xy 偏置（m），冻结时标定
        self._warn_lock_m = 0.002                     # WARN 中允许预测质心越出 warn 盒的深度（配置 com_guard.lock_margin_m）

    # ------------------------------------------------------------------ #
    # 关节映射 / 限位辅助
    # ------------------------------------------------------------------ #
    @staticmethod
    def _limits_cfg_to_dict(limits_cfg) -> Optional[Dict[str, Tuple[float, float]]]:
        if not limits_cfg:
            return None
        return {k: (v[0], v[1]) for k, v in vars(limits_cfg).items()}

    def _apply_driver_joint_limits(self, limits: Optional[Dict[str, Tuple[Optional[float], Optional[float]]]]) -> None:
        """把驱动软限位（度，驱动坐标）换算到 Placo 坐标后写入模型。

        驱动限位为权威来源：该 URDF 的关节零位与真机差 180°（见 joint_map.offset_deg），
        其声明的 <limit> 区间与物理行程并不对应，因此不与 URDF 限位取交集；
        仅当某侧驱动限位缺失（None）时才回退到 URDF 声明值。
        """
        if not limits:
            return
        r = self.cfg.robot
        name_to_arm = {}
        for i, n in enumerate(r.left_joint_names):
            name_to_arm[n] = ("left_arm", i)
        for i, n in enumerate(r.right_joint_names):
            name_to_arm[n] = ("right_arm", i)

        new_limits = {arm: [lo.copy(), hi.copy()] for arm, (lo, hi) in self.arm_q_limits.items()}
        for real_name, (lo_deg, hi_deg) in limits.items():
            if real_name not in name_to_arm:
                print(f"[Placo] ⚠️ arm_joint_limits_deg 未知关节 {real_name}")
                continue
            arm, idx = name_to_arm[real_name]
            jm = self.joint_map[arm]
            urdf_lo, urdf_hi = float(new_limits[arm][0][idx]), float(new_limits[arm][1][idx])

            def to_deg(rad):
                full = np.zeros(7); full[idx] = rad
                return float(jm.rad_to_deg(full)[idx])

            def to_rad(deg):
                full = np.zeros(7); full[idx] = deg
                return float(jm.deg_to_rad(full)[idx])

            urdf_deg = sorted((to_deg(urdf_lo), to_deg(urdf_hi)))
            d_lo = urdf_deg[0] if lo_deg is None else float(lo_deg)
            d_hi = urdf_deg[1] if hi_deg is None else float(hi_deg)
            lo, hi = sorted((to_rad(d_lo), to_rad(d_hi)))
            new_limits[arm][0][idx], new_limits[arm][1][idx] = lo, hi
            print(f"[Placo] {real_name} 限位 驱动[{d_lo:.1f}, {d_hi:.1f}]° = URDF[{np.rad2deg(lo):.1f}, {np.rad2deg(hi):.1f}]°")
        for arm, (lo, hi) in new_limits.items():
            self.set_arm_joint_limits(arm, lo, hi)
        self._apply_planner_position_limits()

    def _apply_planner_position_limits(self) -> None:
        """把当前臂限位（Placo 弧度）换成驱动度写入规划器：接近限位时限速 + 硬钳位。"""
        if getattr(self, "arm", None) is None or self.arm.planner is None:
            return
        lo14, hi14 = [], []
        for arm in ARM_ORDER:
            lo, hi = self.arm_q_limits[arm]
            a = self.joint_map[arm].rad_to_deg(lo)
            b = self.joint_map[arm].rad_to_deg(hi)
            lo14.append(np.minimum(a, b)); hi14.append(np.maximum(a, b))
        lead = float(getattr(self.cfg.inex.servo_planner, "limit_lead_sec", 0.0))
        buf = float(getattr(self.cfg.robot, "soft_limit_margin_deg", 0.0))
        self.arm.planner.set_position_limits(np.concatenate(lo14), np.concatenate(hi14), lead, buf)
        print(f"[Setup] 规划器位置限位已设（驱动度，前瞻 {lead*1000:.0f} ms，软限位内缩 {buf:.0f}°）")

    def _apply_runtime_soft_limits(self) -> None:
        """用 /inex/get_soft_limits 的实测软限位覆盖配置表（查询失败则保留配置表）。"""
        self._apply_ik_velocity_limits()
        r = self.cfg.robot
        margin = float(getattr(r, "soft_limit_margin_deg", 0.0))
        limits = {}
        for arm, part, names in (("left_arm", PART_LEFT_ARM, r.left_joint_names),
                                 ("right_arm", PART_RIGHT_ARM, r.right_joint_names)):
            soft = self.inex.get_soft_limits_deg(part)
            if soft is None:
                print(f"[Setup] {arm} 软限位查询失败，沿用配置表")
                continue
            print(f"[Setup] {arm} 驱动软限位(deg): {[(round(a, 1), round(b, 1)) for a, b in soft]}")
            for n, (lo, hi) in zip(names, soft):
                limits[n] = (lo + margin, hi - margin)
        if limits:
            self._apply_driver_joint_limits(limits)

    def _apply_ik_velocity_limits(self) -> None:
        """ik.joint_velocity_limits_deg_s（7 轴，左右共用）覆盖 URDF 的 QP 关节速度上限；null 沿用 URDF。"""
        lim = getattr(self.cfg.teleop.ik, "joint_velocity_limits_deg_s", None)
        if not lim or not self.enable_joint_velocity_limits:
            return
        v = np.deg2rad(np.asarray(lim, dtype=float))
        if v.size != 7:
            print(f"[Placo] ⚠️ ik.joint_velocity_limits_deg_s 需 7 个值，得到 {v.size}，忽略")
            return
        for arm in ARM_ORDER:
            self.set_arm_joint_velocity_limits(arm, v)
        print(f"[Placo] QP 关节速度上限(°/s) = {list(map(float, lim))}")

    def _deg14_to_placo(self, arm_deg: np.ndarray) -> Dict[str, np.ndarray]:
        return {
            "left_arm": self.joint_map["left_arm"].deg_to_rad(arm_deg[:7]),
            "right_arm": self.joint_map["right_arm"].deg_to_rad(arm_deg[7:14]),
        }

    def _placo_to_deg14(self) -> np.ndarray:
        return np.concatenate([self.joint_map[n].rad_to_deg(self.get_arm_q(n)) for n in ARM_ORDER])

    def _sync_to_robot_pose(self, arm_deg: np.ndarray) -> None:
        """Placo、安全门、ServoJ 规划器同时对齐到给定真机位姿（度）。"""
        self.sync_arm_joints(self._deg14_to_placo(arm_deg))
        if self.safety:
            self.safety.reset_last_safe(arm_deg)
        if self.arm:
            self.arm.anchor_planner(arm_deg)

    def _body_state_deg(self) -> np.ndarray:
        """安全门用的 8 维 [head j1..3, body j1..5]（度）；body 可按配置反序。"""
        head = self.arm.get_part_deg("head", 3)
        body = self.arm.get_part_deg("body", 5)
        if getattr(self.cfg.safety, "body_joint_order_reversed", False):
            body = body[::-1]
        return np.concatenate([head, body])

    # ------------------------------------------------------------------ #
    # 初始化 / 停机
    # ------------------------------------------------------------------ #
    def _robot_setup(self) -> None:
        cfg, r, ros_cfg, inex_cfg = self.cfg, self.cfg.robot, self.cfg.ros, self.cfg.inex
        self.ros = RosContext(ros_cfg.node_name)
        self.inex = InexClient(self.ros, service_timeout=inex_cfg.service_timeout_sec)

        pl = inex_cfg.servo_planner
        planner = None
        if pl.enable:
            planner = ServoJPlanner(14, pl.vmax_deg_s, pl.amax_deg_s2, pl.tau_fast_sec, pl.tau_slow_sec, pl.gate_deg,
                                    mode=getattr(pl, "mode", ServoJPlanner.MODE_CLAMP))
            print(f"[Setup] ServoJ 规划器 mode={planner.mode} vmax={pl.vmax_deg_s}°/s amax={pl.amax_deg_s2}°/s²")
        self.arm = InexArmController(
            self.ros,
            servo_cmd_topic=ros_cfg.servo_cmd_topic,
            joint_state_topic=ros_cfg.joint_state_topic,
            gripper_left_cmd_topic=ros_cfg.gripper_left_cmd_topic,
            gripper_right_cmd_topic=ros_cfg.gripper_right_cmd_topic,
            gripper_max_effort=cfg.teleop.gripper.max_effort,
            planner=planner,
        )
        self.chassis = F1ChassisController(
            self.ros,
            move_topic=cfg.chassis.move_topic,
            control_mode_service=cfg.chassis.control_mode_service,
            control_mode_value=cfg.chassis.control_mode_value,
            frame_id=cfg.chassis.frame_id,
            publish_rate_hz=cfg.chassis.publish_rate_hz,
        )
        self.waist = F1WaistController(self.ros, ros_cfg.waist_cmd_topic)
        # 调试：规划器之前的 IK 目标（度，14 轴），与 /inex/servo_cmd 对比可量化规划器引入的滞后
        from sensor_msgs.msg import JointState
        self._ik_target_pub = self.ros.node.create_publisher(JointState, "/f1_teleop/ik_target_deg", 10)
        self.lead_pub = LeadDataPublisher(
            self.ros,
            joint_state_topic=ros_cfg.lead_joint_state_topic,
            left_tcp_topic=ros_cfg.lead_left_tcp_topic,
            right_tcp_topic=ros_cfg.lead_right_tcp_topic,
            lead_joint_names=[*r.urdf_left_joint_names, *r.urdf_right_joint_names, *r.head_joint_names],
        )
        self.external_input = ExternalPoseInput(
            self.ros, ros_cfg.replay_mode_topic, ros_cfg.replay_left_tcp_topic, ros_cfg.replay_right_tcp_topic
        )
        rec = cfg.recording
        self.recorder = RecordingManager(
            bag=BagRecorder(rec.bag_dir, rec.topics, rec.storage, rec.validate) if rec.enable else None,
            tianji=TianjiRecorderClient(self.ros, ros_cfg.tianji_start_topic, ros_cfg.tianji_stop_topic),
        )

        # 1. 等真机状态
        print(f"[Setup] 等待 {ros_cfg.joint_state_topic} ...")
        if not self.arm.wait_for_state(timeout=inex_cfg.state_timeout_sec):
            raise RuntimeError(f"未收到 {ros_cfg.joint_state_topic}，iNex 驱动未启动？")

        # 2. iNex 准备时序：心跳 → 上电 → 模式 → 运动档（位置 / 关节阻抗）→ ServoJ
        imp = getattr(inex_cfg, "impedance", None)
        imp_parts: Tuple[int, ...] = ()
        if imp is not None and imp.enable:
            imp_parts = tuple({"left": PART_LEFT_ARM, "right": PART_RIGHT_ARM}[s] for s in imp.arms)
            print(f"[Setup] ⚠️ 关节阻抗模式: {list(imp.arms)} 刚度={imp.joint_stiffness or '驱动默认'}（臂柔顺，跟踪误差大，需有人现场看护）")
        if not self.inex.prepare_arms(impedance_parts=imp_parts,
                                      joint_stiffness=(imp.joint_stiffness if imp_parts else None),
                                      payload_cfg=getattr(inex_cfg, "payload", None)):
            if inex_cfg.require_powered:
                raise RuntimeError("双臂上电复核失败（inex.require_powered=true），终止启动")
        if self.inex.state == 3:
            raise RuntimeError("驱动仍处于 ERROR，回零/ServoJ 会被拒绝，终止启动（请在 UI 做报警复位后重试）")
        self.arm.wait_servo_subscriber(timeout=inex_cfg.servo_subscriber_timeout_sec)
        self._apply_runtime_soft_limits()

        # 3. 头部：目标从真实角度起步（可选 ServoJ 下发）
        self.head = InexHeadController(
            self.ros,
            limits_deg=cfg.head.limits_deg,
            step_deg_per_frame=cfg.head.step_deg_per_frame,
            servo_cmd_topic=ros_cfg.servo_cmd_topic,
            enable=bool(cfg.head.enable),
            inex=self.inex,
            servo_axes=int(getattr(cfg.head, "servo_axes", 6)),
        )
        self.head.set_target(self.arm.get_part_deg("head", len(r.head_joint_names)))

        # 4. 双臂回零位（S 曲线），Placo / 规划器同步，夹爪初始化
        self._check_feedback_inside_soft_limits()
        self.arm.move_to(self.home_pose_deg, r.home_duration_sec, r.move_rate_hz)
        self._sync_to_robot_pose(self.home_pose_deg)
        self._self_check(self.home_pose_deg)
        if inex_cfg.init_grippers:
            fs = getattr(inex_cfg, "gripper_force_speed", None)
            self.inex.init_grippers(publish_hz=int(inex_cfg.gripper_state_hz),
                                    force_speed=None if fs is None else (int(fs[0]), int(fs[1])))

        # 5. 安全门（质心 + 碰撞）
        cg = getattr(cfg, "com_guard", None)
        if cg is not None and cg.enable:
            self.com_guard = ComGuard(
                self.ros, cg.topic, cg.stop_min, cg.stop_max, cg.warn_min, cg.warn_max,
                timeout_ms=cg.timeout_ms, hysteresis_m=cg.hysteresis_m, hold_warn_sec=cg.hold_warn_sec,
            )
            self._warn_lock_m = float(getattr(cg, "lock_margin_m", self._warn_lock_m))
            gap = float(np.min(np.minimum(self.com_guard.warn_min - self.com_guard.stop_min,
                                          self.com_guard.stop_max - self.com_guard.warn_max)[:2]))
            if not 0.0 <= self._warn_lock_m < gap:
                raise RuntimeError(f"[CoM] com_guard.lock_margin_m={self._warn_lock_m} 必须在 [0, warn→stop 间距 {gap:.3f}) 内")
            if self.com_guard.wait_for_data(5.0):
                p = self.com_guard.position; mw, ms = self.com_guard.margin()
                print(f"[CoM] 质心保护已启用：当前 {np.round(p, 3)} m (M={self.com_guard.mass:.1f} kg) | 距 warn/stop 盒 {mw*100:.1f}/{ms*100:.1f} cm")
                if ms < 0:
                    raise RuntimeError("[CoM] 启动时质心已在 stop 盒外，请检查姿态/负载或放宽 com_guard 参数")
                self.com_guard.arm()
            else:
                raise RuntimeError(f"[CoM] 未收到 {cg.topic}，com_guard.enable=true 时不允许无保护运行")
        else:
            print("[CoM] 质心保护未启用")
        if cfg.safety.enable:
            init_cmd = np.zeros(24)
            init_cmd[:14] = self.home_pose_deg
            init_cmd[14:22] = self._body_state_deg()
            self.safety = SafetyGuard(
                module_dir=cfg.safety.module_dir,
                body_state_deg_getter=self._body_state_deg,
                initial_cmd_deg=init_cmd,
                enable_com_guard=cfg.safety.enable_com_guard,
                enable_collision_guard=cfg.safety.enable_collision_guard,
                check_rate_hz=cfg.safety.check_rate_hz,
                warn_interval_sec=cfg.safety.warn_interval_sec,
            )
        else:
            print("[Setup] ⚠️ 安全门已禁用（无质心/碰撞保护）")

    def _check_feedback_inside_soft_limits(self) -> None:
        """反馈在驱动软限位外时任何 ServoJ/PTP 都会被拒并使整机掉电（2026-11-13 R-J4 141° 实例），回零前必须先用 jog 拉回。"""
        fb = self.arm.get_arm_state_deg()
        bad = []
        for part, arm, sl in ((PART_LEFT_ARM, "left_arm", slice(0, 7)), (PART_RIGHT_ARM, "right_arm", slice(7, 14))):
            soft = self.inex.get_soft_limits_deg(part)
            if soft is None:
                continue
            for j, (q, (lo, hi)) in enumerate(zip(fb[sl], soft)):
                if q < lo or q > hi:
                    bad.append((part, arm, j, q, lo, hi))
        if not bad:
            return
        lines = [f"  {arm} j{j+1} = {q:.1f}° 不在软限位 [{lo:.0f}, {hi:.0f}]，先用驱动 jog 拉回："
                 f"ros2 service call /inex/start_jog inex_ros2_interface/srv/StartJog "
                 f"'{{space: 0, part: {part}, axis: {j}, direction: {-1 if q > hi else 1}, speed_scale: 0.1}}'，"
                 f"约 0.5 s 后 ros2 service call /inex/stop_jog inex_ros2_interface/srv/StopJog '{{}}'"
                 for part, arm, j, q, lo, hi in bad]
        raise RuntimeError("关节反馈在驱动软限位外，回零会被驱动拒绝并掉电，终止启动：\n" + "\n".join(lines))

    def _self_check(self, expected_deg: np.ndarray) -> None:
        fb = self.arm.get_arm_state_deg()
        diff = np.max(np.abs(fb - expected_deg))
        if diff > 10.0:
            print(f"[Setup] ⚠️ 真机反馈与目标位姿最大偏差 {diff:.1f}°（>10°），请检查零位/方向标定\n"
                  f"        反馈 L={np.round(fb[:7], 1)} R={np.round(fb[7:], 1)}")
        else:
            print(f"[Setup] 真机反馈与目标位姿最大偏差 {diff:.1f}°，一致")

    def _should_keep_running(self) -> bool:
        return super()._should_keep_running() and (self.ros is None or self.ros.ok())

    def _shutdown_robot(self) -> None:
        if self.recorder:
            self.recorder.stop()
        if self.chassis:
            self.chassis.stop()
        if self.safety:
            self.safety.stop()
        if self.inex and self.cfg.inex.halt_on_exit:
            self.inex.halt()
        if self.inex:
            self.inex.stop_impedance_sessions()
        if self.ros:
            self.ros.shutdown()

    # ------------------------------------------------------------------ #
    # 输入
    # ------------------------------------------------------------------ #
    def _update_robot_state(self) -> None:
        """开环虚拟机器人：不回读真机反馈。"""

    def _read_arm_input(self, name: str) -> Tuple[bool, Optional[np.ndarray]]:
        if self._resetting.is_set():
            return False, None
        side = _SIDE[name]
        if self.external_input and self.external_input.enabled:
            return self.external_input.read(side)
        active, pose = super()._read_arm_input(name)
        if self.lead_pub:
            self.lead_pub.update_controller_pose(side, pose, active)
        return active, pose

    def _mode_switch_allowed(self) -> bool:
        return not self._head_active

    def _pre_ik_update(self) -> None:
        if self._resetting.is_set():
            return
        self._handle_buttons()
        self._update_head()
        self._update_chassis()

    # ------------------------------------------------------------------ #
    # 按键：复位 / 腰
    # ------------------------------------------------------------------ #
    def _handle_buttons(self) -> None:
        x = bool(self.xr_client.get_button_state_by_name("X"))
        y = bool(self.xr_client.get_button_state_by_name("Y"))
        if self._xy_edge(x and y):
            self._start_reset()
            return
        if self._com_state != "OK":
            return  # 质心预警/停机期间禁止腰部升降
        if self._y_hold(y and not x):
            self.waist.step(True)
        if self._x_hold(x and not y):
            self.waist.step(False)

    def _start_reset(self) -> None:
        if self._resetting.is_set():
            return
        self._resetting.set()
        threading.Thread(target=self._do_reset, name="reset", daemon=True).start()

    def _do_reset(self) -> None:
        try:
            print("[Reset] X+Y → 双臂复位")
            r = self.cfg.robot
            self._streaming = False
            if self.com_guard is not None and self.com_guard.latched_stop:
                # STOP 后已 halt：重新进 ServoJ 再回复位位姿（复位位姿是质心安全的）
                for part in (PART_LEFT_ARM, PART_RIGHT_ARM):
                    self.inex.set_servo_mode(part)
            self.arm.move_to(self.reset_pose_deg, r.reset_duration_sec, r.move_rate_hz)
            self._sync_to_robot_pose(self.reset_pose_deg)
            self._self_check(self.reset_pose_deg)
            if self.com_guard is not None and self.com_guard.latched_stop:
                if self.com_guard.clear_latch():
                    self._com_state = "OK"
                    print("[CoM] 质心已回到安全区，停机闩锁解除")
                else:
                    print(f"[CoM] ❌ 质心仍在 warn 盒外 {np.round(self.com_guard.position, 3)}，保持停机；请检查腰部/负载后再 X+Y")
            print("[Reset] 完成")
        except Exception as e:
            print(f"[Reset] 异常: {e!r}")
        finally:
            self._resetting.clear()

    # ------------------------------------------------------------------ #
    # 头部
    # ------------------------------------------------------------------ #
    @staticmethod
    def _axis(raw: float, deadzone: float, invert: bool) -> float:
        if abs(raw) < deadzone:
            return 0.0
        return -raw if invert else raw

    def _update_head(self) -> None:
        h = self.cfg.head
        rt = self.xr_client.get_key_value_by_name("right_trigger")
        lg = self.xr_client.get_key_value_by_name("left_grip")
        rg = self.xr_client.get_key_value_by_name("right_grip")
        self._head_active = rt >= h.trigger_threshold and lg < h.grip_release_threshold and rg < h.grip_release_threshold
        if not self._head_active:
            return
        lx, ly = self.xr_client.get_joystick_state("left")
        rx, _ = self.xr_client.get_joystick_state("right")
        axes = [
            self._axis(rx, h.deadzone, h.invert_yaw),     # head_j1 yaw
            self._axis(ly, h.deadzone, h.invert_pitch),   # head_j2 pitch
            self._axis(lx, h.deadzone, h.invert_roll),    # head_j3 roll
        ]
        if self.head.step_by(axes):
            self.head.publish()

    # ------------------------------------------------------------------ #
    # 底盘
    # ------------------------------------------------------------------ #
    def _update_chassis(self) -> None:
        c = self.cfg.chassis
        lx, ly = self.xr_client.get_joystick_state("left")
        rx, ry = self.xr_client.get_joystick_state("right")

        if self._head_active:
            self._base_up_edge(ry >= c.trigger_up)
            self._base_down_edge(ry <= -c.trigger_down)
            self.chassis.set_velocity_command(0.0, 0.0, 0.0)
            return

        if self._base_up_edge(ry >= c.trigger_up):
            self.chassis.enable()
        if self._base_down_edge(ry <= -c.trigger_down):
            self.chassis.disable()

        if self.chassis.enabled:
            if self._com_state != "OK":
                self.chassis.set_velocity_command(0.0, 0.0, 0.0)   # 质心预警/停机：底盘只允许停
                return
            vx = self._axis(ly, c.deadzone, c.invert_linear_x) * c.max_linear
            vy = self._axis(lx, c.deadzone, c.invert_linear_y) * c.max_linear
            wz = self._axis(-rx, c.deadzone, c.invert_angular_z) * c.max_angular   # 左拨 = 逆时针为正
            self.chassis.set_velocity_command(vx, vy, wz)

    # ------------------------------------------------------------------ #
    # 下发
    # ------------------------------------------------------------------ #
    def _limit_step(self, arm_deg: np.ndarray) -> np.ndarray:
        prev = self.arm.last_cmd_deg
        if prev is None:
            return arm_deg
        delta = arm_deg - prev
        clipped = np.clip(delta, -self._max_step_deg, self._max_step_deg)
        if self.prof_ctrl.enabled:
            self.prof_ctrl.note("max_step_deg", f"{np.max(np.abs(delta)):.3f}")
            self.prof_ctrl.note("clipped", int(np.any(clipped != delta)))
        return prev + clipped

    def _publish_ik_target(self, arm_deg: np.ndarray) -> None:
        if self._ik_target_pub.get_subscription_count() == 0:
            return
        from sensor_msgs.msg import JointState
        msg = JointState()
        msg.header.stamp = self.ros.now_msg()
        msg.position = [float(v) for v in arm_deg]
        self._ik_target_pub.publish(msg)

    def _update_com_guard(self) -> str:
        """每拍评估质心状态；状态变化或 WARN/STOP 持续时限速打印。进入 STOP 时平滑制动。"""
        if self.com_guard is None:
            return "OK"
        new = self.com_guard.evaluate()
        now = time.monotonic()
        if new != self._com_state or (new != "OK" and now - self._com_last_print > 2.0):
            p = self.com_guard.position; mw, ms = self.com_guard.margin()
            tag = {"OK": "✅", "WARN": "⚠️", "STOP": "❌"}[new]
            print(f"[CoM] {tag} {new} 质心={None if p is None else np.round(p, 3)} 距 warn/stop 边界 {mw*100:.1f}/{ms*100:.1f} cm"
                  + ("（双臂冻结、禁腰/底盘；请 X+Y 复位，或等质心自行回到安全区）" if new == "WARN" else
                     "（已平滑制动，X+Y 复位后解除）" if new == "STOP" else ""))
            self._com_last_print = now
        if new == "STOP" and self._com_state != "STOP":
            self._streaming = False
            self.inex.halt()
        if self._com_state == "WARN" and new == "OK" and self.arm.last_cmd_deg is not None:
            # 解除冻结：以当前下发构型重新锚定 Placo 与手柄参考，避免追向冻结前的旧目标而立刻再次越界
            self._sync_to_robot_pose(self.arm.last_cmd_deg.copy())
        if new != "WARN":
            self._warn_ref = None
        self._com_state = new
        return new

    def _warn_step(self, ik_deg: np.ndarray) -> None:
        """WARN 期间：冻结并重新锚定后，按“从头开始的增量遥操”继续，但只要目标构型的预测质心越出锁定盒就锁住。

        冻结 = planner.hold()（刹停到指令快照）+ 把 Placo 双臂回写为实际下发值 + 手柄参考重新锚定到该构型（离合打滑）。
        放行判据：本拍 IK 候选构型的模型质心 + 偏置（驱动实测 - 模型，在指令构型上标定，每次冻结时刷新）落在
        锁定盒（warn 盒向外扩 com_guard.lock_margin_m）内 → 正常遥操（回收/横移/小幅前推都连续跟随）；
        越出 → 拒绝：刹停 + 回写 + 重锚，前推量被吃掉。锁定盒到 stop 盒之间是模型误差/滞后的余量，stop 盒是硬底线。
        逃生口：指令构型本身已在锁定盒外（偏置刷新/动态超调）时，所有候选都在盒外会死锁，此时操作者明确回收（所有激活手柄不向外、
        至少一只明显向内）也放行。

        弃用的逐拍方向判据（2026-11-14 实测）：模型质心符号（回收时肘部抬起，质心每步反而向外 0.04~0.34 mm → 锁死）、
        IK 候选末端方向（另一臂 IK 抖动 ±0.2 mm 误拒 → 蠕动）、手柄每帧位移（另一只手的抖动 ±1~2 mm/帧，25% 误拒 → 蠕动）。
        共同教训：逐拍拒绝会把 IK 一阶响应反复打断，判据必须是“构型落在哪里”而不是“这一步朝哪走”。
        为什么被拒时必须重新锚定手柄参考：目标 = 锚定末端 + 手柄增量，否则臂冻结后目标跑到臂前方，要回收到越过冻结点臂才动。
        历史教训：被拒时绝不能把上一帧规划器输出当目标喂回去（clamp 模式 err=0 时 v=v_target → 自锁匀速漂走），必须用 hold()。"""
        if self.arm.last_cmd_deg is None:
            self.arm.publish_hold_cmd(self.dt)
            return
        if self._warn_ref is None:
            self._hold_and_revert()
            self._warn_ref = self._com_bias_xy()
            return
        pred = self._placo_com_xy() + self._warn_ref                 # Placo 当前 = 本拍 IK 候选
        m_pred = self.com_guard.margin(np.append(pred, 0.0))[0]
        d = self.com_guard.inward_dir()
        steps = [float(np.dot(self.controller_step_xyz[n][:2], d)) for n in ARM_ORDER if self.active[n]]
        eps = self.WARN_HAND_EPS_M
        retracting = bool(steps) and np.any(d) and min(steps) >= -eps and max(steps) > 3 * eps
        accept = m_pred >= -self._warn_lock_m or retracting
        self._warn_dbg_update(accept, m_pred, self.com_guard.margin()[0])
        if accept:
            self.arm.publish_planned_cmd(ik_deg, self.dt)
            self._publish_ik_target(ik_deg)
            return
        self._hold_and_revert()
        self._warn_ref = self._com_bias_xy()

    def _placo_com_xy(self) -> np.ndarray:
        with self._placo_lock:
            self.placo_robot.update_kinematics()      # solver.solve(True) 只写 q，不刷新运动学
            return np.asarray(self.placo_robot.com_world())[:2].copy()

    def _com_bias_xy(self) -> np.ndarray:
        """驱动实测质心 - 模型质心（xy）；调用时 Placo 应已回写为指令构型。"""
        p = self.com_guard.position
        return (p[:2] if p is not None else np.zeros(2)) - self._placo_com_xy()

    def _warn_dbg_update(self, accept: bool, m_pred: float, m_drv: float) -> None:
        """WARN 放行/拒绝的 1 Hz 诊断统计。"""
        st = self.__dict__.setdefault("_warn_dbg", {"t": time.monotonic(), "acc": 0, "rej": 0, "dp_min": 1e9, "dp_max": -1e9})
        st["acc" if accept else "rej"] += 1
        st["dp_min"] = min(st["dp_min"], m_pred); st["dp_max"] = max(st["dp_max"], m_pred)
        now = time.monotonic()
        if now - st["t"] >= 1.0:
            print(f"[CoM] WARN 诊断: 放行 {st['acc']} / 拒绝 {st['rej']} 帧 | 候选构型预测质心距 warn 盒 "
                  f"{st['dp_min']*100:+.1f}~{st['dp_max']*100:+.1f} cm (锁定线 {-self._warn_lock_m*100:+.1f}) | 驱动实测 {m_drv*100:+.1f} cm")
            st.update(t=now, acc=0, rej=0, dp_min=1e9, dp_max=-1e9)

    def _hold_and_revert(self) -> None:
        self.arm.publish_hold_cmd(self.dt)
        if self.arm.last_cmd_deg is not None:
            self._revert_placo_arms(self.arm.last_cmd_deg)
            self.rebase_arm_references()
            self._publish_ik_target(self.arm.last_cmd_deg)

    def _revert_placo_arms(self, arm_deg: np.ndarray) -> None:
        """把 Placo 双臂 q 回写为给定驱动角。"""
        q = self._deg14_to_placo(arm_deg)
        with self._placo_lock:
            for name, qi in q.items():
                self.placo_robot.state.q[self.arm_q_slice[name]] = qi
            self.placo_robot.update_kinematics()

    def _send_command(self) -> None:
        if not self._resetting.is_set():
            com = self._update_com_guard()
            if any(self.active.values()):
                self._streaming = True
            if self._streaming and com == "WARN":
                self._warn_step(self._placo_to_deg14())
            elif self._streaming and com != "STOP":
                # 有臂激活：目标 = IK 解；全部松开：目标保持上次 IK 解（规划器刹停保持）
                arm_deg = self._placo_to_deg14()
                if self.safety:
                    arm_deg = self.safety.filter(arm_deg)
                if self.arm.planner is None:
                    arm_deg = self._limit_step(arm_deg)
                self.arm.publish_planned_cmd(arm_deg, self.dt)
                self._publish_ik_target(arm_deg)
                self.prof_ctrl.note("pub", 1)
                if self.arm.planner is not None and self.prof_ctrl.enabled:
                    self.prof_ctrl.note("planner_limited", self.arm.planner.limited_frames)
                    self.prof_ctrl.note("limit_clip_L", self.arm.planner.limit_clip_frames[:7].tolist())
                    self.prof_ctrl.note("limit_clip_R", self.arm.planner.limit_clip_frames[7:].tolist())
            if self._gripper_limiter.ready():
                self.arm.publish_gripper_cmd(
                    self.gripper_pos_target["left_arm"], self.gripper_pos_target["right_arm"]
                )
        self.chassis.publish()

    # ------------------------------------------------------------------ #
    # 辅助线程（30Hz）：数采按键 + 主端数据镜像
    # ------------------------------------------------------------------ #
    def _aux_update(self) -> None:
        if self._a_edge(bool(self.xr_client.get_button_state_by_name("A"))):
            self.recorder.start()
        if self._b_edge(bool(self.xr_client.get_button_state_by_name("B"))):
            self.recorder.stop()
        self.lead_pub.publish(self.arm.last_cmd_deg, self.head.get_target())
