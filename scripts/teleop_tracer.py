#!/usr/bin/env python3
"""只读记录器：手柄位姿 / IK 目标 / 下发 ServoJ / 关节反馈 / 驱动 TCP / 整机质心，存 npz 供离线分析。"""
import sys, time, threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from inex_ros2_interface.msg import ServoCommand, TcpPosition, ComState

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/teleop_trace.npz"


class Rec(Node):
    def __init__(self):
        super().__init__("teleop_tracer")
        self.lock = threading.Lock()
        self.d = {k: [] for k in ("lead_L", "lead_R", "cmd_L", "cmd_R", "fb", "tcp", "ik", "com")}
        self.create_subscription(JointState, "/f1_teleop/ik_target_deg", self._ik, 50)
        self.create_subscription(JointState, "/lead/left_tcp", lambda m: self._lead("lead_L", m), 50)
        self.create_subscription(JointState, "/lead/right_tcp", lambda m: self._lead("lead_R", m), 50)
        self.create_subscription(ServoCommand, "/inex/servo_cmd", self._cmd, qos_profile_sensor_data)
        self.create_subscription(JointState, "/inex/joint_states", self._fb, qos_profile_sensor_data)
        self.create_subscription(TcpPosition, "/inex/tcp_pose", self._tcp, qos_profile_sensor_data)
        self.create_subscription(ComState, "/inex/com_state", self._com, qos_profile_sensor_data)
        self.layout = None
        self._com_n = 0

    def _lead(self, k, m):
        with self.lock:
            self.d[k].append([time.monotonic(), 1.0 if m.header.frame_id == "press_pico" else 0.0, *m.position[:6]])

    def _ik(self, m):
        with self.lock:
            self.d["ik"].append([time.monotonic(), *m.position[:14]])

    def _cmd(self, m):
        if m.cmd_type != 0 or m.part not in (1, 2):
            return
        with self.lock:
            self.d["cmd_L" if m.part == 1 else "cmd_R"].append([time.monotonic(), *m.q_deg[:7]])

    def _fb(self, m):
        if self.layout is None:
            n = list(m.name)
            self.layout = ([i for i, x in enumerate(n) if "08L" in x or x.startswith("left")][:7],
                           [i for i, x in enumerate(n) if "08R" in x or x.startswith("right")][:7])
        L, R = self.layout
        p = m.position
        with self.lock:
            self.d["fb"].append([time.monotonic(), *[np.degrees(p[i]) for i in L], *[np.degrees(p[i]) for i in R]])

    def _tcp(self, m):
        l, r = m.left_arm.position, m.right_arm.position
        with self.lock:
            self.d["tcp"].append([time.monotonic(), l.x, l.y, l.z, r.x, r.y, r.z])

    def _com(self, m):
        self._com_n += 1
        if self._com_n % 10:   # 1 kHz → 100 Hz
            return
        with self.lock:
            self.d["com"].append([time.monotonic(), 1.0 if m.valid else 0.0, m.position.x, m.position.y, m.position.z])


def main():
    rclpy.init()
    n = Rec()
    th = threading.Thread(target=rclpy.spin, args=(n,), daemon=True); th.start()
    t0 = time.monotonic()
    print(f"[tracer] recording {DUR:.0f}s → {OUT}", flush=True)
    while time.monotonic() - t0 < DUR:
        time.sleep(5)
        with n.lock:
            print("[tracer] " + " ".join(f"{k}={len(v)}" for k, v in n.d.items()), flush=True)
    with n.lock:
        np.savez(OUT, **{k: np.array(v) for k, v in n.d.items()})
    print("[tracer] saved", flush=True)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
