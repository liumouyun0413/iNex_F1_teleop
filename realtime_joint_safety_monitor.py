#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实时碰撞/质心检测节点，发布 /f1_safety/is_safe。

被检查的构型：
  * 双臂 —— 优先用遥操下发的 ServoJ 指令 /inex/servo_cmd（inex_ros2_interface/ServoCommand，q_deg 为度），
            它比反馈领先 ≈ 驱动跟踪滞后（80~150 ms），故能提前报警；且臂停下后指令≈反馈，不会像反馈判定那样卡在边界内持续 False。
            指令流停止 > CMD_STALE_SEC（非遥操、复位中、halt 后）则回退用反馈。
  * 腰/头 —— 始终用 /inex/joint_states 反馈。
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

try:
    from inex_ros2_interface.msg import ServoCommand
except ImportError:  # 未 source iNex 工作区：只能按反馈检查
    ServoCommand = None


# ── 文件顶部集中配置：运行脚本可从任意工作目录启动 ──────────────
THIS_FILE = Path(__file__).resolve()
THIS_DIR = THIS_FILE.parent
SAFETY_MODULE_DIR = THIS_DIR

LEFT_SIGN = np.ones(7, dtype=float)
LEFT_OFFSET_DEG = np.zeros(7, dtype=float)
RIGHT_SIGN = np.ones(7, dtype=float)
RIGHT_OFFSET_DEG = np.zeros(7, dtype=float)

JOINT_STATE_TOPIC = "/inex/joint_states"
SERVO_CMD_TOPIC = "/inex/servo_cmd"
IS_SAFE_TOPIC = "/f1_safety/is_safe"
CLEARANCE_TOPIC = "/f1_safety/collision_clearance"   # 最小带符号间隙 (m)，负 = 穿透；调 margin 用
CMD_STALE_SEC = 0.2
PART_LEFT_ARM, PART_RIGHT_ARM = 1, 2

# 安全模块不依赖脚本的启动目录，显式加入绝对模块路径。
sys.path.insert(0, str(SAFETY_MODULE_DIR))
from com_and_collision_protect_modules import SafetyProtector  # noqa: E402


def resolve_joint_layout(names: List[str], n_pos: int) -> Dict[str, List[int]]:
    """按关节名把 /inex/joint_states 的 position 数组解析为四个分区。"""
    layout: Dict[str, List[int]] = {"body": [], "head": [], "left": [], "right": []}
    if names and len(names) == n_pos:
        for i, name in enumerate(names):
            low = name.lower()
            if low.startswith("body"):
                layout["body"].append(i)
            elif low.startswith("head"):
                layout["head"].append(i)
            elif "08l" in low or "left" in low or ("l-" in low and "r-" not in low):
                layout["left"].append(i)
            elif "08r" in low or "right" in low or ("r-" in low and "l-" not in low):
                layout["right"].append(i)

    if len(layout["left"]) == 7 and len(layout["right"]) == 7:
        return layout

    rest = n_pos - 2 * 7
    body_n = 5 if rest == 8 else rest // 2
    print(f"[Safety] 关节名不可识别，按位置回退解析：body={body_n} head={rest-body_n}", flush=True)
    return {
        "body": list(range(0, body_n)),
        "head": list(range(body_n, rest)),
        "left": list(range(n_pos - 14, n_pos - 7)),
        "right": list(range(n_pos - 7, n_pos)),
    }


class JointSafetyMonitor(Node):
    def __init__(
        self,
        protector: SafetyProtector,
        safe_topic: str,
        check_hz: float,
        report_hz: float,
    ):
        super().__init__("inex_joint_safety_monitor")
        self.layout: Optional[Dict[str, List[int]]] = None
        self.n_axes = -1
        self.initialized = False
        self.protector = protector
        self.check_hz = float(check_hz)
        self.report_hz = float(report_hz)
        self.latest_cmd: Optional[np.ndarray] = None
        self.latest_feedback: Optional[Dict[str, float]] = None
        self.feedback_count = 0
        self.compute_count = 0
        self.compute_times_ms: List[float] = []
        self.last_report_time = time.monotonic()
        # ServoJ 指令（弧度，URDF 坐标）及最后更新时刻
        self.servo_cmd_rad: Dict[int, Optional[np.ndarray]] = {PART_LEFT_ARM: None, PART_RIGHT_ARM: None}
        self.servo_cmd_time = 0.0
        self.servo_cmd_count = 0
        self.using_cmd = False

        self.safe_pub = self.create_publisher(Bool, safe_topic, qos_profile_sensor_data)
        self.clearance_pub = self.create_publisher(Float64, CLEARANCE_TOPIC, qos_profile_sensor_data)
        # 后续需要时再启用单项标志发布：
        # self.com_safe_pub = self.create_publisher(Bool, "/f1_safety/com_safe", qos_profile_sensor_data)
        # self.collision_safe_pub = self.create_publisher(Bool, "/f1_safety/collision_safe", qos_profile_sensor_data)
        self.create_subscription(JointState, JOINT_STATE_TOPIC, self.on_joint_state, qos_profile_sensor_data)
        if ServoCommand is not None:
            self.create_subscription(ServoCommand, SERVO_CMD_TOPIC, self.on_servo_cmd, qos_profile_sensor_data)
        else:
            print("[Safety] ⚠️ 未能导入 inex_ros2_interface.msg.ServoCommand（未 source iNex 工作区），只按反馈检查", flush=True)

    def on_servo_cmd(self, msg) -> None:
        if msg.cmd_type != 0 or msg.part not in self.servo_cmd_rad or len(msg.q_deg) < 7:
            return
        sign, off = (LEFT_SIGN, LEFT_OFFSET_DEG) if msg.part == PART_LEFT_ARM else (RIGHT_SIGN, RIGHT_OFFSET_DEG)
        q_deg = np.asarray(msg.q_deg[:7], dtype=float)
        self.servo_cmd_rad[msg.part] = np.deg2rad((q_deg - off) / sign)
        self.servo_cmd_time = time.monotonic()
        self.servo_cmd_count += 1

    def on_joint_state(self, msg: JointState) -> None:
        positions = list(msg.position)
        names = list(msg.name)
        if len(positions) < 14:
            return

        if self.layout is None or self.n_axes != len(positions):
            self.layout = resolve_joint_layout(names, len(positions))
            self.n_axes = len(positions)

        cmd = np.zeros(24, dtype=np.float64)
        feedback: Dict[str, float] = {}

        for side in ("left", "right"):
            indices = self.layout[side]
            if len(indices) != 7:
                continue
            driver_rad = np.asarray([positions[i] for i in indices], dtype=float)
            driver_deg = np.degrees(driver_rad)
            urdf_rad = np.deg2rad(
                (driver_deg - (LEFT_OFFSET_DEG if side == "left" else RIGHT_OFFSET_DEG))
                / (LEFT_SIGN if side == "left" else RIGHT_SIGN)
            )
            start = 0 if side == "left" else 7
            prefix = "l" if side == "left" else "r"
            cmd[start:start + 7] = urdf_rad
            for i, value in enumerate(urdf_rad):
                feedback[f"arm_{prefix}_j{i+1}"] = float(value)

        for i, idx in enumerate(self.layout["head"][:3]):
            cmd[14 + i] = float(positions[idx])
            feedback[f"head_j{i+1}"] = float(positions[idx])
        for i, idx in enumerate(self.layout["body"][:5][::-1]):
            cmd[17 + i] = float(positions[idx])
            feedback[f"body_j{i+1}"] = float(positions[idx])

        self.feedback_count += 1
        self.latest_cmd = cmd
        self.latest_feedback = feedback

    def _compute_safety(self) -> None:
        if self.latest_cmd is None or self.latest_feedback is None:
            return

        cmd = self.latest_cmd.copy()
        feedback = dict(self.latest_feedback)
        # 双臂：ServoJ 指令新鲜时用指令替换反馈（腰/头仍为反馈）
        now_m = time.monotonic()
        ql, qr = self.servo_cmd_rad[PART_LEFT_ARM], self.servo_cmd_rad[PART_RIGHT_ARM]
        use_cmd = ql is not None and qr is not None and now_m - self.servo_cmd_time <= CMD_STALE_SEC
        if use_cmd:
            cmd[0:7] = ql
            cmd[7:14] = qr
        if use_cmd != self.using_cmd:
            self.using_cmd = use_cmd
            print(f"[Safety] 双臂检查对象 → {'ServoJ 指令 ' + SERVO_CMD_TOPIC if use_cmd else '反馈 ' + JOINT_STATE_TOPIC + '（指令流停止）'}", flush=True)
        self.protector.set_feedback(feedback)
        if not self.initialized:
            self.protector.init_safe_cmd(cmd)
            self.initialized = True

        t0 = time.perf_counter()
        is_safe, _, _, _, _, _ = self.protector.run(cmd)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        safe_msg = Bool()
        safe_msg.data = bool(is_safe)
        self.safe_pub.publish(safe_msg)
        col = self.protector.last_collision_result
        if col is not None and hasattr(col, "signed_clearance"):
            cl = Float64()
            cl.data = float(col.signed_clearance)
            self.clearance_pub.publish(cl)

        self.compute_count += 1
        self.compute_times_ms.append(elapsed_ms)
        if len(self.compute_times_ms) > 100:
            self.compute_times_ms.pop(0)

        now = time.monotonic()
        elapsed_report = now - self.last_report_time
        if elapsed_report >= 1.0 / max(self.report_hz, 0.01):
            actual_hz = self.compute_count / elapsed_report
            arr = np.asarray(self.compute_times_ms, dtype=float)
            print(
                f"[Safety] feedback_rx={self.feedback_count / elapsed_report:.1f}Hz "
                f"servo_cmd_rx={self.servo_cmd_count / elapsed_report:.1f}Hz src={'cmd' if self.using_cmd else 'fb'} "
                f"compute={actual_hz:.1f}Hz mean={arr.mean():.2f}ms "
                f"p95={np.percentile(arr, 95):.2f}ms status={'SAFE' if is_safe else 'UNSAFE'}",
                flush=True,
            )
            self.compute_count = 0
            self.feedback_count = 0
            self.servo_cmd_count = 0
            self.last_report_time = now


def main() -> int:
    parser = argparse.ArgumentParser(description="基于 /inex/servo_cmd（双臂指令）+ /inex/joint_states（腰/头反馈）实时检测碰撞与质心")
    parser.add_argument("--safe-topic", default=IS_SAFE_TOPIC)
    parser.add_argument("--check-hz", type=float, default=200.0, help="SafetyProtector 计算频率（实际受单次耗时限制）")
    parser.add_argument("--report-hz", type=float, default=1.0, help="性能与反馈频率报告频率")
    parser.add_argument("--com-guard", action="store_true", help="同时启用 cyb 质心门（默认只做碰撞，质心由遥操用 iNex com_state）")
    args = parser.parse_args()

    protector = SafetyProtector(
        config_path=None,
        use_com_guard=bool(args.com_guard),
        use_collision_guard=True,
        cmd_dim=24,
        verbose=False,
    )
    if not protector.has_guard:
        print("[Safety] 安全模块初始化失败", file=sys.stderr, flush=True)
        return 2

    rclpy.init()
    node = JointSafetyMonitor(protector, args.safe_topic, args.check_hz, args.report_hz)
    node.create_timer(1.0 / max(args.check_hz, 0.01), node._compute_safety)

    def stop_handler(signum, frame):
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    print(f"[Safety] 等待 {JOINT_STATE_TOPIC}；双臂优先按 {SERVO_CMD_TOPIC} 指令检查（停流 {CMD_STALE_SEC:.1f}s 后回退反馈），不下发控制指令", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
