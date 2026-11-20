"""
SafetyGuard —— 碰撞 / 质心安全门（订阅外部检测节点的结果）。

检测本身由独立节点（realtime_joint_safety_monitor.py，持有 cyb SafetyProtector，读 /inex/joint_states）完成，
并以 std_msgs/Bool 发布到 /f1_safety/is_safe；本类只订阅该结果，不再在遥操进程内/子进程中跑 SafetyProtector。

工作方式（与旧版一致）：
  * 控制线程每拍调用 filter(arm_cmd_deg)：
      - is_safe=True  → 原样放行，并把该指令记为"最近安全指令"；
      - is_safe=False → 冻结：用最近安全指令替代目标（双臂一起冻结，碰撞/质心无法归因到单臂）。
  * 结果超时（> stale_timeout_sec 未更新）视为检测节点异常：打印告警并放行（与旧版"检查异常，暂时放行"一致）。
  * 启动时等待首帧，等不到则抛错（safety.enable=true 时不允许无保护运行）。
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Sequence

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

from f1_teleop.hardware.interface.ros_context import RosContext


class SafetyGuard:
    def __init__(
        self,
        ros: RosContext,
        topic: str,
        initial_cmd_deg: Sequence[float],
        warn_interval_sec: float = 3.0,
        stale_timeout_sec: float = 0.5,
        wait_timeout_sec: float = 5.0,
    ) -> None:
        """
        topic           : 检测节点发布的 std_msgs/Bool（True=安全）
        initial_cmd_deg : 14 维双臂初始指令（度），作为首个安全回退点
        """
        self.topic = topic
        self.warn_interval_sec = float(warn_interval_sec)
        self.stale_timeout = float(stale_timeout_sec)

        self._lock = threading.Lock()
        self._is_safe = True
        self._t_msg = 0.0
        self._n_msgs = 0
        self._last_safe: Optional[np.ndarray] = np.asarray(initial_cmd_deg, dtype=float)[:14].copy()
        self.frozen = False
        self.reject_count = 0
        self._last_warn = 0.0
        self._last_stale_warn = 0.0

        ros.node.create_subscription(Bool, topic, self._on_msg, qos_profile_sensor_data)
        deadline = time.monotonic() + float(wait_timeout_sec)
        while time.monotonic() < deadline:
            with self._lock:
                if self._n_msgs > 0:
                    break
            time.sleep(0.05)
        else:
            raise RuntimeError(f"[Safety] {wait_timeout_sec:.0f}s 内未收到 {topic}，碰撞检测节点未启动？（safety.enable=true 时不允许无保护运行）")
        print(f"[Safety] 已订阅 {topic}（碰撞/质心检测由外部节点完成）| 当前 {'安全' if self._is_safe else '不安全'}")

    # ------------------------------------------------------------------ #
    def _on_msg(self, msg: Bool) -> None:
        with self._lock:
            self._is_safe = bool(msg.data)
            self._t_msg = time.monotonic()
            self._n_msgs += 1

    # ------------------------------------------------------------------ #
    def filter(self, arm_cmd_deg: Sequence[float]) -> np.ndarray:
        """返回实际应下发的双臂指令：不安全时为最近安全指令，否则原样放行。"""
        raw = np.asarray(arm_cmd_deg, dtype=float)
        now = time.monotonic()
        with self._lock:
            is_safe, age = self._is_safe, now - self._t_msg
            if age > self.stale_timeout:
                is_safe = True                                  # 检测节点断流：放行并告警
                if now - self._last_stale_warn >= self.warn_interval_sec:
                    self._last_stale_warn = now
                    print(f"[Safety] ⚠️ {self.topic} 已 {age:.1f}s 未更新（检测节点异常？），暂时放行")
            self.frozen = not is_safe
            if is_safe:
                self._last_safe = raw.copy()
                return raw
            out = self._last_safe.copy() if self._last_safe is not None else raw
        self.reject_count += 1
        if now - self._last_warn >= self.warn_interval_sec:
            self._last_warn = now
            print("[Safety] 危险（碰撞或质心越界），双臂冻结在最近安全指令")
        return out

    def reset_last_safe(self, arm_cmd_deg: Sequence[float]) -> None:
        """复位/同步后重置安全回退点，避免解冻时跳回旧位置。"""
        with self._lock:
            self._last_safe = np.asarray(arm_cmd_deg, dtype=float)[:14].copy()
            self.frozen = False

    def stop(self) -> None:
        pass
