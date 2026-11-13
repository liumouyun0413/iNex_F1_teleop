"""
InexArmController —— 珞石 (Rokae) 双臂 + 夹爪的 iNex ROS2 接口（对应官方示例的 A1XController，
与 f1_tianji_teleop 的 F1ArmController 同角色）。

职责：
  * 订阅 /inex/joint_states（弧度，驱动按 body→head→left→right 排列，
    v1.0 回退名 body_N/head_N/left_N/right_N；按关节名前缀解析，兼容旧 yaml 名），缓存各分区角度（度）；
  * 发布 /inex/servo_cmd（ServoCommand, cmd_type=ServoJ, q_deg 度, SensorDataQoS），左右臂各一帧；
  * 发布 /motion_ctl/gripper/{left,right}（GripperCommand，驱动内置夹爪组件消费）；
  * move_to(目标, 时长)：S 曲线插值运动（启动回零位 / X+Y 复位），逐帧定时发送；
  * ServoJPlanner：速度/加速度受限的输出规划器（lxl 桥接 send_loop 的等价实现），
    把 100Hz IK 目标摆成速度连续、永不超 vmax/amax 的 ServoJ 流，目标停更时自动刹停保持；
  * 记录最近一次下发的双臂指令 last_cmd_deg（供数采镜像 / 安全门回退）。
"""

from __future__ import annotations

import math
import threading
import time
from typing import Dict, List, Optional, Sequence

import numpy as np
from control_msgs.msg import GripperCommand
from inex_ros2_interface.msg import ServoCommand
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState

from f1_teleop.hardware.interface.inex_client import PART_LEFT_ARM, PART_RIGHT_ARM
from f1_teleop.hardware.interface.ros_context import RosContext

RAD2DEG = 180.0 / math.pi
ARM_AXES = 7


def joint_part_of_name(name: str) -> Optional[str]:
    """/inex/joint_states 关节名 → body / head / left / right / None。"""
    n = name.lower()
    if n.startswith("body"):
        return "body"
    if n.startswith("head"):
        return "head"
    if "left" in n:
        return "left"
    if "right" in n:
        return "right"
    base = name.rsplit("_joint", 1)[0]          # 旧 yaml 名 AR10-10_08L-xxx_joint_1
    if "L-" in base and "R-" not in base:
        return "left"
    if "R-" in base and "L-" not in base:
        return "right"
    return None


def resolve_joint_layout(names: Sequence[str], n_pos: int) -> Dict[str, List[int]]:
    """各分区在 position 数组中的索引；名字不可识别时回退"最后 14 轴 = 左7 + 右7"。"""
    layout: Dict[str, List[int]] = {"body": [], "head": [], "left": [], "right": []}
    if names and len(names) == n_pos:
        for i, nm in enumerate(names):
            part = joint_part_of_name(nm)
            if part is not None:
                layout[part].append(i)
    if len(layout["left"]) == ARM_AXES and len(layout["right"]) == ARM_AXES:
        return layout
    rest = n_pos - 2 * ARM_AXES
    nb = 5 if rest == 8 else rest // 2        # 22 轴 body5+head3 / 26 轴 body6+head6
    print(f"[Arm] ⚠️ /inex/joint_states 关节名不可识别，按位置回退解析（{n_pos} 轴）")
    return {
        "body": list(range(0, nb)),
        "head": list(range(nb, rest)),
        "left": list(range(n_pos - 14, n_pos - 7)),
        "right": list(range(n_pos - 7, n_pos)),
    }


class ServoJPlanner:
    """ServoJ 输出规划器，两种模式：
    * clamp（默认）：v = (target−pos)/dt，只在超出 ±vmax / ±amax 时夹住；并以目标自身速度为基准做
      制动前瞻 |v − v_target| ≤ √(2·amax·|err|)：正常跟随（err≈0）时输出恒等于目标、零附加延迟，
      只有目标跳变导致饱和追赶时才提前减速，避免越过停下来的目标。
    * tracking（lxl 原版）：v_des = err/τ（τ 随 |err| 在 τ_slow→τ_fast 间连续过渡），
      夹 ±vmax 与制动前瞻 √(2·amax·|err|)，再按 amax 限加速度。自带 ≈τ 的一阶滞后。
    两种模式下目标停更都会 v→0、位置停在目标上（断流保护）。"""

    MODE_CLAMP = "clamp"
    MODE_TRACKING = "tracking"
    _VT_ALPHA = 0.5          # 目标速度 EMA（PICO 87Hz vs 控制 100Hz 的重复/跳帧抖动）

    def __init__(self, n: int, vmax_deg_s, amax_deg_s2,
                 tau_fast: float, tau_slow: float, gate_deg: float, mode: str = MODE_CLAMP) -> None:
        if mode not in (self.MODE_CLAMP, self.MODE_TRACKING):
            raise ValueError(f"ServoJPlanner mode 应为 {self.MODE_CLAMP!r} 或 {self.MODE_TRACKING!r}")
        self.n = n
        self.mode = mode
        # 标量 = 所有轴同限；长度 n 或 n/2（单臂 7 轴，左右复用）= 逐轴限
        self.vmax = self._per_joint(vmax_deg_s)
        self.amax = self._per_joint(amax_deg_s2)
        self.tau_fast = float(tau_fast)
        self.tau_slow = float(tau_slow)
        self.gate = max(1e-6, float(gate_deg))
        self.pos: Optional[np.ndarray] = None
        self.vel = np.zeros(n)
        self._prev_target: Optional[np.ndarray] = None
        self._v_target = np.zeros(n)
        self.limited_frames = 0          # 被夹住的帧计数（诊断用）        # 位置限位（驱动度）与接近限位时的速度前瞻时间：驱动 ServoJ 内部滤波有 ≈80 ms 滞后且会超调，
        # 快速反向时反馈可超出指令 ≈ v·τ（2026-11-13 实测 255°/s → +15°，撞软限位掉电）。
        # 因此要求 pos + v·(dt+lead) 不越限：距限位越近允许速度越小。
        self._lo: Optional[np.ndarray] = None
        self._hi: Optional[np.ndarray] = None
        self.limit_lead = 0.0
    @property
    def anchored(self) -> bool:
        return self.pos is not None

    def set_position_limits(self, lo_deg: Sequence[float], hi_deg: Sequence[float], lead_sec: float) -> None:
        self._lo = np.asarray(lo_deg, dtype=float).copy()
        self._hi = np.asarray(hi_deg, dtype=float).copy()
        self.limit_lead = max(0.0, float(lead_sec))

    def _per_joint(self, value) -> np.ndarray:
        arr = np.atleast_1d(np.asarray(value, dtype=float))
        if arr.size == 1:
            return np.full(self.n, float(arr[0]))
        if arr.size * 2 == self.n:
            return np.tile(arr, 2)
        if arr.size != self.n:
            raise ValueError(f"ServoJPlanner 限幅长度应为 1、{self.n // 2} 或 {self.n}，得到 {arr.size}")
        return arr.copy()

    def anchor(self, pos_deg: Sequence[float]) -> None:
        self.pos = np.asarray(pos_deg, dtype=float).copy()
        self.vel = np.zeros(self.n)
        self._prev_target = None
        self._v_target = np.zeros(self.n)

    def reset(self) -> None:
        self.pos = None
        self.vel = np.zeros(self.n)
        self._prev_target = None
        self._v_target = np.zeros(self.n)

    def step(self, target_deg: Sequence[float], dt: float) -> np.ndarray:
        target = np.asarray(target_deg, dtype=float)
        if self.pos is None:
            self.anchor(target)
            self._prev_target = target.copy()
            return self.pos.copy()
        if self._prev_target is None:
            self._prev_target = target.copy()
        self._v_target = self._VT_ALPHA * (target - self._prev_target) / dt + (1 - self._VT_ALPHA) * self._v_target
        self._prev_target = target.copy()

        err = target - self.pos
        v_brake = np.sqrt(2.0 * self.amax * np.abs(err))
        if self.mode == self.MODE_CLAMP:
            v_des = np.clip(err / dt, self._v_target - v_brake, self._v_target + v_brake)
        else:
            a_err = np.abs(err)
            tau = self.tau_slow + (self.tau_fast - self.tau_slow) * np.minimum(a_err / self.gate, 1.0)
            v_des = np.clip(err / tau, -v_brake, v_brake)
        v = np.clip(v_des, -self.vmax, self.vmax)
        v = np.clip(v, self.vel - self.amax * dt, self.vel + self.amax * dt)
        if self._lo is not None:
            horizon = dt + self.limit_lead
            v = np.clip(v, (self._lo - self.pos) / horizon, (self._hi - self.pos) / horizon)
        if np.any(np.abs(v - err / dt) > 1e-9):
            self.limited_frames += 1
        self.vel = v
        self.pos = self.pos + v * dt
        if self._lo is not None:
            self.pos = np.clip(self.pos, self._lo, self._hi)
        return self.pos.copy()


class InexArmController:
    def __init__(
        self,
        ros: RosContext,
        servo_cmd_topic: str,
        joint_state_topic: str,
        gripper_left_cmd_topic: str,
        gripper_right_cmd_topic: str,
        gripper_max_effort: float = 15.0,
        planner: Optional[ServoJPlanner] = None,
    ) -> None:
        self.ros = ros
        self.servo_cmd_topic = servo_cmd_topic
        self.gripper_max_effort = float(gripper_max_effort)
        self.planner = planner

        node = ros.node
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        # 驱动端订阅 servo_cmd 为 SensorDataQoS（BEST_EFFORT），发布端必须兼容
        self._servo_pub = node.create_publisher(ServoCommand, servo_cmd_topic, qos_profile_sensor_data)
        self._grip_left_pub = node.create_publisher(GripperCommand, gripper_left_cmd_topic, reliable)
        self._grip_right_pub = node.create_publisher(GripperCommand, gripper_right_cmd_topic, reliable)
        node.create_subscription(JointState, joint_state_topic, self._on_joint_state, qos_profile_sensor_data)

        self._msg = {}
        for part in (PART_LEFT_ARM, PART_RIGHT_ARM):
            m = ServoCommand()
            m.part = part
            m.cmd_type = ServoCommand.CMD_TYPE_J
            self._msg[part] = m

        self._lock = threading.Lock()
        self._layout: Optional[Dict[str, List[int]]] = None
        self._n_axes = 0
        self._state: Dict[str, np.ndarray] = {k: np.zeros(0) for k in ("body", "head", "left", "right")}
        self._state_stamp = -1.0
        self._sub_warn_at = 0.0

        self.last_cmd_deg: Optional[np.ndarray] = None
        self.last_gripper_cmd: List[float] = [0.0, 0.0]

    # ------------------------------------------------------------------ #
    # 状态
    # ------------------------------------------------------------------ #
    def _on_joint_state(self, msg: JointState) -> None:
        pos = list(msg.position)
        if len(pos) < 2 * ARM_AXES:
            return
        if self._layout is None or self._n_axes != len(pos):
            self._layout = resolve_joint_layout(list(msg.name), len(pos))
            self._n_axes = len(pos)
            lay = self._layout
            print(f"[Arm] /inex/joint_states {len(pos)} 轴 | body{len(lay['body'])} head{len(lay['head'])} "
                  f"left{len(lay['left'])} right{len(lay['right'])}")
        lay = self._layout
        with self._lock:
            for part in ("body", "head", "left", "right"):
                self._state[part] = np.array([pos[i] * RAD2DEG for i in lay[part]], dtype=float)
            self._state_stamp = time.time()

    @property
    def has_state(self) -> bool:
        return self._state_stamp > 0

    def wait_for_state(self, timeout: float = 5.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.has_state:
                return True
            time.sleep(0.05)
        return False

    def get_arm_state_deg(self) -> np.ndarray:
        """14 维 [左7, 右7]（度）。"""
        with self._lock:
            return np.concatenate([self._state["left"], self._state["right"]])

    def get_part_deg(self, part: str, n: Optional[int] = None) -> np.ndarray:
        """body / head 反馈（度）；n 指定截断/补零长度（v1.0 缓冲 6 轴含补零轴）。"""
        with self._lock:
            v = self._state[part].copy()
        if n is None:
            return v
        out = np.zeros(n)
        out[: min(n, len(v))] = v[:n]
        return out

    # ------------------------------------------------------------------ #
    # 指令
    # ------------------------------------------------------------------ #
    def servo_subscribed(self) -> bool:
        return self._servo_pub.get_subscription_count() > 0

    def wait_servo_subscriber(self, timeout: float = 3.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.servo_subscribed():
                print(f"[Arm] 驱动已订阅 {self.servo_cmd_topic}，ServoJ 通道就绪")
                return True
            time.sleep(0.05)
        print(f"[Arm] ❌ {timeout:.0f}s 内无 {self.servo_cmd_topic} 订阅者：驱动未启动 / 接口包非 v1.0 / RMW 不一致")
        return False

    def publish_arm_cmd(self, arm_deg: Sequence[float]) -> bool:
        """直接下发 14 维双臂目标（度）→ 左右臂各一帧 ServoJ。无订阅者时丢帧并节流告警。"""
        arm = np.asarray(arm_deg, dtype=float)
        if arm.shape != (14,):
            raise ValueError(f"双臂指令应为 14 维，实际 {arm.shape}")
        if not self.servo_subscribed():
            now = time.monotonic()
            if now - self._sub_warn_at > 2.0:
                self._sub_warn_at = now
                print(f"[Arm] ⚠️ {self.servo_cmd_topic} 无订阅者，关节指令被丢弃")
            return False
        for part, sl in ((PART_LEFT_ARM, slice(0, 7)), (PART_RIGHT_ARM, slice(7, 14))):
            m = self._msg[part]
            m.q_deg = arm[sl].tolist()
            self._servo_pub.publish(m)
        self.last_cmd_deg = arm
        return True

    def publish_planned_cmd(self, target_deg: Sequence[float], dt: float) -> bool:
        """经 ServoJPlanner 平滑后下发；planner 为 None 时等同 publish_arm_cmd。"""
        if self.planner is None:
            return self.publish_arm_cmd(target_deg)
        return self.publish_arm_cmd(self.planner.step(target_deg, dt))

    def anchor_planner(self, pos_deg: Optional[Sequence[float]] = None) -> None:
        """把规划器锚定到给定位置（默认真机反馈），速度清零。"""
        if self.planner is None:
            return
        if pos_deg is None:
            pos_deg = self.get_arm_state_deg() if self.has_state else self.last_cmd_deg
        if pos_deg is None:
            self.planner.reset()
        else:
            self.planner.anchor(pos_deg)

    def publish_gripper_cmd(self, left: float, right: float) -> None:
        for pub, value in ((self._grip_left_pub, left), (self._grip_right_pub, right)):
            msg = GripperCommand()
            msg.position = float(value)
            msg.max_effort = self.gripper_max_effort
            pub.publish(msg)
        self.last_gripper_cmd = [float(left), float(right)]

    def move_to(self, target_deg: Sequence[float], duration_sec: float, rate_hz: float = 50.0) -> bool:
        """从真机当前反馈 S 曲线插值到目标（阻塞，逐帧定时，绝不连发追赶）。"""
        target = np.asarray(target_deg, dtype=float)
        if target.shape != (14,):
            raise ValueError("move_to 目标应为 14 维")
        if self.has_state:
            start = self.get_arm_state_deg()
        elif self.last_cmd_deg is not None:
            start = self.last_cmd_deg.copy()
        else:
            print("[Arm] 无真机反馈且无历史指令，无法安全插值，跳过运动")
            return False
        if not self.servo_subscribed() and not self.wait_servo_subscriber(3.0):
            return False
        if np.max(np.abs(target - start)) < 1e-3:
            self.last_cmd_deg = start
            return True

        steps = max(1, int(duration_sec * rate_hz))
        dt = 1.0 / rate_hz
        print(f"[Arm] S 曲线运动 {duration_sec:.1f}s: {np.round(start, 1)} → {np.round(target, 1)}")
        next_t = time.perf_counter()
        for i in range(steps + 1):
            t = i / steps
            s = 3.0 * t * t - 2.0 * t * t * t
            now = time.perf_counter()
            if now < next_t:
                time.sleep(next_t - now)
            self.publish_arm_cmd(start + (target - start) * s)
            next_t = time.perf_counter() + dt
        return True
