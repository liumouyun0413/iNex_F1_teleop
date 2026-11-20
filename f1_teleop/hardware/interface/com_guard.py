"""
ComGuard —— 基于 iNex 原生整机质心 /inex/com_state 的倾倒保护。

原理（静态稳定判据 + 动态裕度）：质心在 chassis_link（≡ URDF base_link，原点 body_joint_1 轴，
+X 前 +Z 上）的地面投影必须落在四轮支撑矩形内；把矩形按底盘加减速 a·h/g、坡度 h·tanθ、
臂反力、模型误差向内收缩，得到两层轴对齐盒：
  * warn 盒（内层）：进入 → 双臂目标冻结、禁止腰/底盘，退回后自动恢复
  * stop 盒（外层）：越出 → 平滑制动，需人工复位
数据无效或超时（> timeout_ms 无 valid 帧）按 STOP 处理（fail-safe，与厂商 servoj_mass_guard 一致）。
只用 position；velocity 字段实测为数值噪声（静止时 σ≈20 m/s），不参与判断。
可选末端高度门（tcp_z_gate_m）：只有双臂 TCP（/inex/tcp_pose，chassis_link 系）z 都高于阈值才执行 WARN/STOP 判定；
任一臂低于阈值（低位搬运、质心本就靠下）时不干预。已闩锁的 STOP 不受门影响。tcp_pose 超时按门开（保护生效）处理。
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Sequence, Tuple

import numpy as np
from rclpy.qos import qos_profile_sensor_data

from f1_teleop.hardware.interface.ros_context import RosContext

try:
    from inex_ros2_interface.msg import ComState, TcpPosition
except ImportError as e:  # pragma: no cover
    raise ImportError("无法导入 inex_ros2_interface.msg.ComState/TcpPosition：请先 source iNex 工作区") from e

STATE_OK = "OK"
STATE_WARN = "WARN"
STATE_STOP = "STOP"


class ComGuard:
    def __init__(
        self,
        ros: RosContext,
        topic: str,
        stop_min: Sequence[float],
        stop_max: Sequence[float],
        warn_min: Sequence[float],
        warn_max: Sequence[float],
        timeout_ms: float = 200.0,
        hysteresis_m: float = 0.01,
        hold_warn_sec: float = 0.3,
        stop_confirm_frames: int = 3,
        tcp_z_gate_m: Optional[float] = None,
        tcp_z_gate_hyst_m: float = 0.02,
        tcp_topic: str = "/inex/tcp_pose",
        low_boxes: Optional[Tuple[Sequence[float], Sequence[float], Sequence[float], Sequence[float]]] = None,
    ) -> None:
        self._boxes_high = self._make_boxes(stop_min, stop_max, warn_min, warn_max, "高位")
        self._boxes_low = None if low_boxes is None else self._make_boxes(*low_boxes, "低位")
        self.stop_min, self.stop_max, self.warn_min, self.warn_max = self._boxes_high
        self.timeout = float(timeout_ms) / 1000.0
        self.hyst = float(hysteresis_m)
        self.hold_warn = float(hold_warn_sec)
        self.stop_confirm = max(1, int(stop_confirm_frames))   # 连续多少次 evaluate 越出 stop 盒才闩锁（滤单帧野值）
        self._stop_viol = 0
        self.last_stop_reason = ""

        self._lock = threading.Lock()
        self._pos: Optional[np.ndarray] = None
        self._mass = 0.0
        self._t_last_valid = 0.0
        self._n_msgs = 0
        self._armed = False
        self._state = STATE_OK
        self._t_warn_enter = 0.0
        self._latched_stop = False
        ros.node.create_subscription(ComState, topic, self._on_msg, qos_profile_sensor_data)

        self.tcp_gate = None if tcp_z_gate_m is None else float(tcp_z_gate_m)
        self.tcp_gate_hyst = max(0.0, float(tcp_z_gate_hyst_m))
        self._tcp_z: Optional[np.ndarray] = None          # [左 z, 右 z]
        self._t_tcp = 0.0
        self._gate_open = True                              # 门开 = 保护生效
        if self.tcp_gate is not None:
            ros.node.create_subscription(TcpPosition, tcp_topic, self._on_tcp, qos_profile_sensor_data)

    # ------------------------------------------------------------------ #
    @staticmethod
    def _make_boxes(stop_min, stop_max, warn_min, warn_max, tag: str):
        b = tuple(np.asarray(v, dtype=float) for v in (stop_min, stop_max, warn_min, warn_max))
        if not (np.all(b[0] <= b[2]) and np.all(b[3] <= b[1])):
            raise ValueError(f"ComGuard: {tag} warn 盒必须包含在 stop 盒内")
        return b

    @property
    def zone(self) -> str:
        """当前生效的盒：'high'（门开）/ 'low'（门关且配了低位盒）/ 'off'（门关、无低位盒，不干预）。"""
        if self._gate_open:
            return "high"
        return "low" if self._boxes_low is not None else "off"

    def _on_msg(self, msg: ComState) -> None:
        with self._lock:
            self._n_msgs += 1
            if msg.valid:
                self._pos = np.array([msg.position.x, msg.position.y, msg.position.z])
                self._mass = float(msg.mass)
                self._t_last_valid = time.monotonic()

    def _on_tcp(self, msg: TcpPosition) -> None:
        with self._lock:
            self._tcp_z = np.array([msg.left_arm.position.z, msg.right_arm.position.z])
            self._t_tcp = time.monotonic()

    def wait_for_data(self, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._pos is not None and (self.tcp_gate is None or self._tcp_z is not None):
                    return True
            time.sleep(0.05)
        return False

    @property
    def tcp_z(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._tcp_z is None else self._tcp_z.copy()

    @property
    def gate_open(self) -> bool:
        """末端高度门是否打开（保护生效）。未配置门时恒为 True。"""
        return self._gate_open

    def _update_gate(self, now: float) -> bool:
        if self.tcp_gate is None:
            return True
        with self._lock:
            z = self._tcp_z
            age = now - self._t_tcp
        if z is None or age > self.timeout:
            self._gate_open = True                          # 无数据 → fail-safe 保护生效
            return True
        if self._gate_open:
            if np.all(z < self.tcp_gate - self.tcp_gate_hyst):
                self._gate_open = False
        elif np.any(z > self.tcp_gate):
            self._gate_open = True
        boxes = self._boxes_high if (self._gate_open or self._boxes_low is None) else self._boxes_low
        self.stop_min, self.stop_max, self.warn_min, self.warn_max = boxes
        return self._gate_open

    def arm(self) -> None:
        """启用超时判据（在确认已收到首帧之后调用，避免启动阶段误报）。"""
        self._armed = True

    @property
    def position(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._pos is None else self._pos.copy()

    @property
    def mass(self) -> float:
        return self._mass

    @property
    def state(self) -> str:
        return self._state

    @property
    def latched_stop(self) -> bool:
        return self._latched_stop

    def clear_latch(self) -> bool:
        """人工复位：仅当当前质心已回到 warn 盒内才允许解除 STOP 闩锁。"""
        p = self.position
        if p is None or not self._inside(p, self.warn_min, self.warn_max, 0.0):
            return False
        self._latched_stop = False
        self._state = STATE_OK
        return True

    # ------------------------------------------------------------------ #
    @staticmethod
    def _inside(p: np.ndarray, lo: np.ndarray, hi: np.ndarray, shrink: float) -> bool:
        return bool(np.all(p >= lo + shrink) and np.all(p <= hi - shrink))

    def margin(self, p: Optional[np.ndarray] = None) -> Tuple[float, float]:
        """返回 (到 warn 盒边界的最小距离, 到 stop 盒边界的最小距离)，负值=已越界；仅 x/y。"""
        p = self.position if p is None else p
        if p is None:
            return float("nan"), float("nan")
        mw = min(np.min(p[:2] - self.warn_min[:2]), np.min(self.warn_max[:2] - p[:2]))
        ms = min(np.min(p[:2] - self.stop_min[:2]), np.min(self.stop_max[:2] - p[:2]))
        return float(mw), float(ms)

    def inward_dir(self, p: Optional[np.ndarray] = None) -> np.ndarray:
        """越出 warn 盒时，指向盒内的单位方向（仅 x/y）；在盒内返回 0 向量。"""
        p = self.position if p is None else p
        d = np.zeros(2)
        if p is None:
            return d
        for i in range(2):
            if p[i] < self.warn_min[i]:
                d[i] = self.warn_min[i] - p[i]
            elif p[i] > self.warn_max[i]:
                d[i] = self.warn_max[i] - p[i]
        n = np.linalg.norm(d)
        return d / n if n > 1e-9 else d

    def evaluate(self) -> str:
        """每控制拍调用；返回 OK / WARN / STOP。STOP 带闩锁，需 clear_latch()。"""
        now = time.monotonic()
        with self._lock:
            p = self._pos
            n_msgs = self._n_msgs
            age = now - self._t_last_valid
            stale = self._armed and (p is None or age > self.timeout)
        if self._latched_stop:
            return STATE_STOP
        if not self._update_gate(now) and self._boxes_low is None:
            self._state = STATE_OK                          # 门关且无低位盒：不判 WARN/STOP，也不计超时
            self._stop_viol = 0
            return STATE_OK
        if stale:
            self.last_stop_reason = f"/inex/com_state 超时：{'未收到有效帧' if p is None else f'{age*1000:.0f} ms 无有效帧'}（累计收到 {n_msgs} 帧）"
            self._latched_stop = True
            self._state = STATE_STOP
            return STATE_STOP
        if p is None:
            return self._state
        if not self._inside(p, self.stop_min, self.stop_max, 0.0):
            self._stop_viol += 1
            if self._stop_viol >= self.stop_confirm:
                self.last_stop_reason = f"质心越出 stop 盒：p={np.round(p, 3)} 连续 {self._stop_viol} 拍"
                self._latched_stop = True
                self._state = STATE_STOP
                return STATE_STOP
        else:
            self._stop_viol = 0
        if self._state == STATE_WARN:
            # 退出 warn 需回到内缩 hyst 的区域且停留 hold_warn 秒
            if self._inside(p, self.warn_min, self.warn_max, self.hyst):
                if now - self._t_warn_enter > self.hold_warn:
                    self._state = STATE_OK
            else:
                self._t_warn_enter = now
            return self._state
        if not self._inside(p, self.warn_min, self.warn_max, 0.0):
            self._state = STATE_WARN
            self._t_warn_enter = now
        return self._state
