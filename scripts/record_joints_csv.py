#!/usr/bin/env python3
"""
record_joints_csv.py —— 把遥操下发的关节指令（/inex/servo_cmd，ServoJ，度）与真机反馈（/inex/joint_states，转成度）
按固定采样率写成一张宽表 CSV，供测试/离线分析（Excel / pandas 直接打开）。只读，不影响遥操。

列：
  t_s            相对开始的秒数
  wall_time      本机时间 HH:MM:SS.mmm
  cmd_age_ms     最新一帧指令距采样时刻的毫秒数（无指令流时为空；>500 ms 视为无指令，cmd 列留空）
  cmd_L1..L7, cmd_R1..R7   最新 ServoJ 指令（度）
  fb_L1..L7,  fb_R1..R7    最新关节反馈（度）
  err_L1..L7, err_R1..R7   cmd - fb（度；无指令时留空）
  [ik_L1..R7]    --ik 时加入 IK 目标 /f1_teleop/ik_target_deg（规划器之前）

用法（机器人上，先 source ROS + iNex 工作区，或直接用 ./record_joints_csv.sh）：
  python scripts/record_joints_csv.py                      # 默认 100 Hz，Ctrl+C 结束，写到 data/joint_csv/joints_<时间>.csv
  python scripts/record_joints_csv.py --duration 60 --rate 50 --out /tmp/test.csv
  python scripts/record_joints_csv.py --ik
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import sys
import threading
import time
from datetime import datetime
from typing import List, Optional

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
# 加密发布版 f1_teleop_dist/（pyarmor）存在时优先用它，否则用源码包 f1_teleop/
sys.path.insert(0, os.path.join(_ROOT, "f1_teleop_dist") if os.path.isdir(os.path.join(_ROOT, "f1_teleop_dist")) else _ROOT)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from inex_ros2_interface.msg import ServoCommand  # noqa: E402

from f1_teleop.utils.path_utils import PROJECT_ROOT  # noqa: E402

PART_LEFT, PART_RIGHT = 1, 2
CMD_STALE_S = 0.5


class JointCsvRecorder(Node):
    def __init__(self, servo_topic: str, state_topic: str, ik_topic: Optional[str]) -> None:
        super().__init__("record_joints_csv")
        self.lock = threading.Lock()
        self.cmd = {PART_LEFT: None, PART_RIGHT: None}      # 7 维（度）
        self.t_cmd = 0.0
        self.fb: Optional[List[float]] = None               # 14 维（度）
        self.ik: Optional[List[float]] = None               # 14 维（度）
        self.n_cmd = self.n_fb = 0
        self._layout = None
        self.create_subscription(ServoCommand, servo_topic, self._on_cmd, qos_profile_sensor_data)
        self.create_subscription(JointState, state_topic, self._on_fb, qos_profile_sensor_data)
        if ik_topic:
            self.create_subscription(JointState, ik_topic, self._on_ik, 50)

    def _on_cmd(self, m: ServoCommand) -> None:
        if m.cmd_type != 0 or m.part not in self.cmd:
            return
        with self.lock:
            self.cmd[m.part] = [float(v) for v in m.q_deg[:7]]
            self.t_cmd = time.monotonic()
            self.n_cmd += 1

    def _on_fb(self, m: JointState) -> None:
        if self._layout is None:
            names = list(m.name)
            L = [i for i, x in enumerate(names) if "08L" in x or x.startswith("left")][:7]
            R = [i for i, x in enumerate(names) if "08R" in x or x.startswith("right")][:7]
            if len(L) != 7 or len(R) != 7:
                return
            self._layout = (L, R)
        L, R = self._layout
        p = m.position
        with self.lock:
            self.fb = [math.degrees(p[i]) for i in L] + [math.degrees(p[i]) for i in R]
            self.n_fb += 1

    def _on_ik(self, m: JointState) -> None:
        if len(m.position) >= 14:
            with self.lock:
                self.ik = [float(v) for v in m.position[:14]]

    def snapshot(self, now: float):
        with self.lock:
            cmd = None
            if self.cmd[PART_LEFT] is not None and self.cmd[PART_RIGHT] is not None and now - self.t_cmd <= CMD_STALE_S:
                cmd = self.cmd[PART_LEFT] + self.cmd[PART_RIGHT]
            age = None if self.t_cmd == 0.0 else (now - self.t_cmd) * 1000.0
            return cmd, age, (None if self.fb is None else list(self.fb)), (None if self.ik is None else list(self.ik))


def _cols(prefix: str) -> List[str]:
    return [f"{prefix}_L{i}" for i in range(1, 8)] + [f"{prefix}_R{i}" for i in range(1, 8)]


def main() -> None:
    ap = argparse.ArgumentParser(description="记录 ServoJ 关节指令与反馈到 CSV（只读）")
    ap.add_argument("--duration", type=float, default=0.0, help="录制秒数；0 = 直到 Ctrl+C")
    ap.add_argument("--rate", type=float, default=100.0, help="采样率 Hz（默认 100，与遥操控制周期一致）")
    ap.add_argument("--out", type=str, default=None, help="输出 CSV 路径（默认 data/joint_csv/joints_<时间>.csv）")
    ap.add_argument("--ik", action="store_true", help="同时记录 IK 目标 /f1_teleop/ik_target_deg")
    ap.add_argument("--servo-topic", default="/inex/servo_cmd")
    ap.add_argument("--state-topic", default="/inex/joint_states")
    args = ap.parse_args()

    out = args.out or os.path.join(PROJECT_ROOT, "data", "joint_csv", f"joints_{datetime.now():%Y%m%d_%H%M%S}.csv")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)

    rclpy.init()
    node = JointCsvRecorder(args.servo_topic, args.state_topic, "/f1_teleop/ik_target_deg" if args.ik else None)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    th = threading.Thread(target=executor.spin, daemon=True)
    th.start()

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    header = ["t_s", "wall_time", "cmd_age_ms"] + _cols("cmd") + _cols("fb") + _cols("err") + (_cols("ik") if args.ik else [])
    period = 1.0 / max(1.0, args.rate)
    n_rows = 0
    t0 = time.monotonic()
    next_t = t0
    last_report = t0
    print(f"[csv] 录制 → {out}  采样 {args.rate:.0f} Hz  {'时长 %.0f s' % args.duration if args.duration > 0 else 'Ctrl+C 结束'}", flush=True)
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        while not stop.is_set():
            now = time.monotonic()
            if args.duration > 0 and now - t0 >= args.duration:
                break
            if now < next_t:
                time.sleep(min(next_t - now, 0.05))
                continue
            next_t += period
            cmd, age, fb, ik = node.snapshot(now)
            blank14 = [""] * 14
            err = blank14 if (cmd is None or fb is None) else [f"{c - b:.3f}" for c, b in zip(cmd, fb)]
            row = [f"{now - t0:.3f}", datetime.now().strftime("%H:%M:%S.%f")[:-3], "" if age is None else f"{age:.0f}"]
            row += blank14 if cmd is None else [f"{v:.3f}" for v in cmd]
            row += blank14 if fb is None else [f"{v:.3f}" for v in fb]
            row += err
            if args.ik:
                row += blank14 if ik is None else [f"{v:.3f}" for v in ik]
            w.writerow(row)
            n_rows += 1
            if now - last_report >= 5.0:
                last_report = now
                f.flush()
                print(f"[csv] {now - t0:6.1f}s  行={n_rows}  收到 指令帧={node.n_cmd} 反馈帧={node.n_fb}"
                      f"{'' if cmd is not None else '  （当前无 ServoJ 指令流）'}", flush=True)
    print(f"[csv] 完成：{n_rows} 行 → {out}", flush=True)
    executor.shutdown(timeout_sec=1.0)
    th.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
