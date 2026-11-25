#!/usr/bin/env python3
"""
check_joint_map.py —— 珞石双臂 JointMap（Placo 弧度 ↔ 驱动度）方向/零位核对工具。

为什么需要：首次上机 / 换 URDF / 换驱动版本后，必须确认 Placo 模型关节方向与真机一致。
2026-11-12 已用 huaqin_20260924 URDF + 恒等 joint_map 在 ms-thor 上 14 轴全部验证通过（余弦 +1.00）。

两种模式（均需驱动已启动、已 source iNex 工作区）：

  passive（默认，不动机器人）
      读 /inex/joint_states → JointMap → Placo，打印每个关节的 驱动角 / 模型角 / 末端位置，
      并与 home 位姿比较。机器人应先处于 home 位姿。

  jog（会动机器人！每次一个关节，小角度）
      python scripts/check_joint_map.py --mode jog --arm left --joint 4 --deg 5
      对指定关节发 +deg 的 S 曲线 ServoJ（2 s），再回到原位。独立真值来自驱动自己的正解
      /inex/tcp_pose（TcpPosition，chassis_link 系）：比较"Placo 模型按 sign 预测的末端位移"与
      "驱动 tcp_pose 实测位移"的方向：一致 = 该关节 sign 正确；相反 = 需把 sign 取反。
      两套模型的参考系不同，只比方向（余弦），不比绝对坐标。
      注意：jog 前请确认该方向有 deg° 的安全空间，先从 wrist (j5~j7) 开始。
"""

from __future__ import annotations

import os
import sys
import threading
import time

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
# 加密发布版 f1_teleop_dist/（pyarmor）存在时优先用它，否则用源码包 f1_teleop/
sys.path.insert(0, os.path.join(_ROOT, "f1_teleop_dist") if os.path.isdir(os.path.join(_ROOT, "f1_teleop_dist")) else _ROOT)

import numpy as np  # noqa: E402
import placo  # noqa: E402
import tyro  # noqa: E402
from inex_ros2_interface.msg import TcpPosition  # noqa: E402

from f1_teleop.config import load_config  # noqa: E402
from f1_teleop.hardware.interface import InexArmController, InexClient, RosContext  # noqa: E402
from f1_teleop.utils.control_utils import JointMap  # noqa: E402

ARMS = {"left": ("left_arm", 0), "right": ("right_arm", 7)}


class TcpPoseListener:
    def __init__(self, ros: RosContext, topic: str = "/inex/tcp_pose") -> None:
        self._lock = threading.Lock()
        self._xyz = {"left_arm": None, "right_arm": None}
        self._rot = {"left_arm": None, "right_arm": None}
        ros.node.create_subscription(TcpPosition, topic, self._cb, 10)

    def _cb(self, msg: TcpPosition) -> None:
        with self._lock:
            for arm, p in (("left_arm", msg.left_arm), ("right_arm", msg.right_arm)):
                self._xyz[arm] = np.array([p.position.x, p.position.y, p.position.z])
                self._rot[arm] = quat_to_rot(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)

    def get(self, arm: str, settle: float = 0.5):
        time.sleep(settle)
        with self._lock:
            v = self._xyz[arm]
        return None if v is None else v.copy()

    def get_rot(self, arm: str):
        with self._lock:
            v = self._rot[arm]
        return None if v is None else v.copy()


def quat_to_rot(w: float, x: float, y: float, z: float) -> np.ndarray:
    n = np.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_axis_angle(R0: np.ndarray, R1: np.ndarray):
    """R0→R1 的相对旋转（世界系）：返回 (单位轴, 角度 deg)。"""
    dR = R1 @ R0.T
    ang = float(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
    axis = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
    n = np.linalg.norm(axis)
    return (axis / n if n > 1e-9 else axis), np.degrees(ang)


def fk_ee(robot: placo.RobotWrapper, offsets: dict, q_rad_by_arm: dict, ee_link: dict) -> dict:
    for arm, q in q_rad_by_arm.items():
        for off, v in zip(offsets[arm], q):
            robot.state.q[off] = float(v)
    robot.update_kinematics()
    return {arm: robot.get_T_world_frame(ee_link[arm])[:3, 3].copy() for arm in q_rad_by_arm}


def fk_rot(robot: placo.RobotWrapper, offsets: dict, q_rad_by_arm: dict, ee_link: dict, arm: str) -> np.ndarray:
    fk_ee(robot, offsets, q_rad_by_arm, ee_link)
    return robot.get_T_world_frame(ee_link[arm])[:3, :3].copy()


def main(mode: str = "passive", arm: str = "left", joint: int = 4, deg: float = 5.0,
         duration: float = 2.0, config: str | None = None, yes: bool = False) -> None:
    """
    Args:
        mode:     passive（只读）| jog（点动一个关节，会动机器人）
        arm:      left | right
        joint:    1..7
        deg:      点动角度（度，驱动坐标，正向）
        duration: 点动去/回各自的 S 曲线时长（秒）
        config:   配置文件路径
        yes:      跳过回车确认（仅在已确认现场安全时使用）
    """
    cfg = load_config(config)
    r = cfg.robot
    jm = {
        "left_arm": JointMap.from_lists(r.joint_map.left.sign, r.joint_map.left.offset_deg),
        "right_arm": JointMap.from_lists(r.joint_map.right.sign, r.joint_map.right.offset_deg),
    }
    robot = placo.RobotWrapper(r.urdf_path)
    offsets = {
        "left_arm": [robot.get_joint_offset(j) for j in r.urdf_left_joint_names],
        "right_arm": [robot.get_joint_offset(j) for j in r.urdf_right_joint_names],
    }
    ee_link = {"left_arm": r.left_ee_link, "right_arm": r.right_ee_link}
    home = np.array([*r.home_pose.left, *r.home_pose.right], dtype=float)

    ros = RosContext("check_joint_map")
    arm_if = InexArmController(ros, cfg.ros.servo_cmd_topic, cfg.ros.joint_state_topic,
                               cfg.ros.gripper_left_cmd_topic, cfg.ros.gripper_right_cmd_topic)
    tcp = TcpPoseListener(ros)
    if not arm_if.wait_for_state(5.0):
        print("❌ 未收到 /inex/joint_states"); ros.shutdown(); return
    time.sleep(0.5)
    fb = arm_if.get_arm_state_deg()

    def to_model(deg14):
        return {"left_arm": jm["left_arm"].deg_to_rad(deg14[:7]), "right_arm": jm["right_arm"].deg_to_rad(deg14[7:])}

    q_model = to_model(fb)
    ee = fk_ee(robot, offsets, q_model, ee_link)
    print("\n=== passive 核对（驱动反馈 → JointMap → Placo）===")
    print(f"反馈 L(deg) = {np.round(fb[:7], 2)}\n反馈 R(deg) = {np.round(fb[7:], 2)}")
    print(f"与 home 最大偏差 = {np.max(np.abs(fb - home)):.2f}°（>10° 说明机器人不在 home 位姿）")
    for a in ("left_arm", "right_arm"):
        drv = tcp.get(a, settle=0.0)
        print(f"{a}: 模型 q(deg) = {np.round(np.rad2deg(q_model[a]), 2)} | Placo 末端 xyz(m) = {np.round(ee[a], 4)}"
              + (f" | 驱动 tcp_pose xyz(m) = {np.round(drv, 4)}" if drv is not None else " | 驱动 tcp_pose 未收到"))
    back = np.concatenate([jm["left_arm"].rad_to_deg(q_model["left_arm"]), jm["right_arm"].rad_to_deg(q_model["right_arm"])])
    print(f"JointMap 往返误差 = {np.max(np.abs(back - fb)):.2e}°（应≈0）")

    if mode != "jog":
        ros.shutdown(); return

    # ---------------- jog ----------------
    if arm not in ARMS or not 1 <= joint <= 7:
        print("参数错误：--arm left|right --joint 1..7"); ros.shutdown(); return
    name, base = ARMS[arm]
    idx = base + joint - 1
    inex = InexClient(ros)
    if not inex.prepare_arms():
        print("❌ 双臂未确认上电，放弃点动"); ros.shutdown(); return
    if not arm_if.wait_servo_subscriber(3.0):
        ros.shutdown(); return
    if tcp.get(name) is None:
        print("❌ 未收到 /inex/tcp_pose，无独立真值，放弃点动"); ros.shutdown(); return

    start = arm_if.get_arm_state_deg()
    target = start.copy(); target[idx] += deg
    sign = jm[name].sign[joint - 1]
    q_pred = to_model(start)
    ee0 = fk_ee(robot, offsets, q_pred, ee_link)[name]
    R0_pred = fk_rot(robot, offsets, q_pred, ee_link, name)
    q_pred[name] = q_pred[name].copy(); q_pred[name][joint - 1] += sign * np.deg2rad(deg)
    ee_pred = fk_ee(robot, offsets, q_pred, ee_link)[name] - ee0
    R1_pred = fk_rot(robot, offsets, q_pred, ee_link, name)
    ax_pred, ang_pred = rot_axis_angle(R0_pred, R1_pred)
    tcp0 = tcp.get(name)
    rot0 = tcp.get_rot(name)

    print(f"\n=== jog {arm} j{joint} +{deg}°（驱动坐标），当前 sign={sign:+.0f} ===")
    if yes:
        print("--yes：跳过回车确认，开始点动")
    else:
        input("确认该方向有安全空间后按回车开始，Ctrl+C 取消 > ")
    arm_if.move_to(target, duration, r.move_rate_hz)
    fb1 = arm_if.get_arm_state_deg()
    tcp1 = tcp.get(name, settle=0.8)
    rot1 = tcp.get_rot(name)
    moved = fb1[idx] - start[idx]
    ee_meas = tcp1 - tcp0
    ax_meas, ang_meas = rot_axis_angle(rot0, rot1)
    print(f"反馈该关节实际变化 = {moved:+.2f}°（期望 {deg:+.1f}）")
    print(f"Placo 模型预测末端位移 = {np.round(ee_pred * 1e3, 1)} mm | 预测旋转 {ang_pred:.1f}° 轴 {np.round(ax_pred, 2)}")
    print(f"驱动 tcp_pose 实测位移 = {np.round(ee_meas * 1e3, 1)} mm | 实测旋转 {ang_meas:.1f}° 轴 {np.round(ax_meas, 2)}")

    def verdict(cos: float) -> str:
        return ("✅ 方向一致（sign 正确）" if cos > 0.7 else
                "❌ 方向相反：把该关节 sign 取反" if cos < -0.7 else
                "⚠️ 不确定（两模型参考系轴向可能不同，或存在耦合）；请结合 passive 的 xyz 对照判断")

    n_pred, n_meas = np.linalg.norm(ee_pred), np.linalg.norm(ee_meas)
    if n_pred >= 2e-3 and n_meas >= 2e-3:
        cos = float(np.dot(ee_pred, ee_meas) / (n_pred * n_meas))
        print(f"位移方向余弦 = {cos:+.2f} → {verdict(cos)}")
    else:
        print("位移太小（<2 mm），改看旋转轴")
    if ang_pred >= 1.0 and ang_meas >= 1.0:
        cos_r = float(np.dot(ax_pred, ax_meas))
        print(f"旋转轴方向余弦 = {cos_r:+.2f} → {verdict(cos_r)}")
    arm_if.move_to(start, duration, r.move_rate_hz)
    inex.halt()
    ros.shutdown()


if __name__ == "__main__":
    tyro.cli(main)
