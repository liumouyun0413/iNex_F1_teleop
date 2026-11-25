#!/usr/bin/env python3
"""
check_com.py —— iNex 整机质心 /inex/com_state 校核工具（配合 com_guard 参数标定）。

  static（默认，只读）
      读当前关节反馈 → Placo(URDF 惯性参数) 计算整机质心，与驱动 /inex/com_state 比较；
      打印质心到 warn/stop 盒及四轮支撑矩形边界的裕度。
  sweep（会动机器人！需先 ./stop_teleop.sh）
      双臂从当前位姿按 S 曲线缓慢运动到 --pose 指定的"前伸/侧举"位姿再返回，全程记录
      驱动质心与模型质心，输出灵敏度（质心随臂前伸的位移）与两者偏差。

  python scripts/check_com.py
  python scripts/check_com.py --mode sweep --pose forward --duration 6 --yes
"""

from __future__ import annotations

import os
import sys
import time

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
# 加密发布版 f1_teleop_dist/（pyarmor）存在时优先用它，否则用源码包 f1_teleop/
sys.path.insert(0, os.path.join(_ROOT, "f1_teleop_dist") if os.path.isdir(os.path.join(_ROOT, "f1_teleop_dist")) else _ROOT)

import numpy as np  # noqa: E402
import placo  # noqa: E402
import tyro  # noqa: E402

from f1_teleop.config import load_config  # noqa: E402
from f1_teleop.hardware.interface import ComGuard, InexArmController, InexClient, RosContext  # noqa: E402

# 四轮触地点（URDF chassis_*_wheel_*_joint1 原点，base_link 系），轮半径 0.07
SUPPORT_X = (-0.33156, 0.12828)
SUPPORT_Y = (-0.218, 0.218)
GROUND_Z = -0.13519 - 0.07
TOOL_LEN = 0.20            # 法兰以外的夹爪长度（沿法兰 +z）
FLOOR_CLEARANCE = 0.15     # sweep 位姿任一臂 link / 工具尖离地至少这么多，否则拒绝运动

# sweep 目标位姿（驱动度，左 7 + 右 7）。★ 每个位姿都在运动前由 check_floor_clearance() 用 FK 复核。
# 2026-11-14 教训：曾用“肘 J4 90→20”当作前伸，实际是前臂向下折，夹爪触地（J4≈34° 时法兰 z=-0.057）。
POSES = {
    # 肩 J1 90→45（右臂镜像 -45）：双臂向前上方抬起，法兰 (0.61, ±0.30, 0.78)，模型质心 x 前移 ≈ +2.4 cm
    "forward": [45, 90, -90, 90, 0, 0, 0, -45, 90, 90, 90, 0, 0, 0],
    # 回零位
    "home": [90, 90, -90, 90, 0, 0, 0, -90, 90, 90, 90, 0, 0, 0],
}
ARM_FRAMES = ("link3", "link4", "link5", "link6", "link7", "flan_link")


def check_floor_clearance(robot: placo.RobotWrapper, r, names: list[str], q_deg: np.ndarray) -> float:
    """返回该位姿下所有臂 link 原点与工具尖的最小离地高度（m）。"""
    for n, v in zip(names, q_deg):
        robot.set_joint(n, float(np.deg2rad(v)))
    robot.update_kinematics()
    zmin = np.inf
    for prefix in (r.left_ee_link.rsplit("_", 2)[0], r.right_ee_link.rsplit("_", 2)[0]):
        for fr in ARM_FRAMES:
            zmin = min(zmin, robot.get_T_world_frame(f"{prefix}_{fr}")[2, 3])
        T = robot.get_T_world_frame(f"{prefix}_flan_link")
        zmin = min(zmin, (T[:3, 3] + T[:3, 2] * TOOL_LEN)[2])
    return float(zmin - GROUND_Z)


def model_com(robot: placo.RobotWrapper, names: list[str], q_deg: np.ndarray) -> np.ndarray:
    for n, v in zip(names, q_deg):
        robot.set_joint(n, float(np.deg2rad(v)))
    robot.update_kinematics()
    return np.asarray(robot.com_world()).copy()


def main(mode: str = "static", pose: str = "forward", duration: float = 6.0, yes: bool = False,
         config: str | None = None) -> None:
    cfg = load_config(config)
    r, cg = cfg.robot, cfg.com_guard
    robot = placo.RobotWrapper(r.urdf_path)
    arm_names = [*r.urdf_left_joint_names, *r.urdf_right_joint_names]
    body_names = [f"body_joint_{i}" for i in range(1, 6)]
    head_names = [f"head_joint_{i}" for i in range(1, 4)]

    ros = RosContext("check_com")
    arm = InexArmController(ros, cfg.ros.servo_cmd_topic, cfg.ros.joint_state_topic,
                            cfg.ros.gripper_left_cmd_topic, cfg.ros.gripper_right_cmd_topic)
    guard = ComGuard(ros, cg.topic, cg.stop_min, cg.stop_max, cg.warn_min, cg.warn_max, cg.timeout_ms)
    if not arm.wait_for_state(5.0) or not guard.wait_for_data(5.0):
        print("❌ 未收到 joint_states / com_state"); ros.shutdown(); return
    time.sleep(0.3)

    def snapshot():
        q = np.concatenate([arm.get_arm_state_deg(), arm.get_part_deg("body", 5), arm.get_part_deg("head", 3)])
        c_model = model_com(robot, arm_names + body_names + head_names, q)
        return q, c_model, guard.position

    def report(c_drv, c_model, title):
        d = c_drv - c_model
        mw, ms = guard.margin(c_drv)
        sx = min(c_drv[0] - SUPPORT_X[0], SUPPORT_X[1] - c_drv[0]); sy = SUPPORT_Y[1] - abs(c_drv[1])
        print(f"\n=== {title} ===")
        print(f"驱动质心   = {np.round(c_drv, 4)} m  (M={guard.mass:.1f} kg)")
        print(f"模型质心   = {np.round(c_model, 4)} m  (URDF 惯性参数, Placo com_world)")
        print(f"差 (驱动-模型) = {np.round(d * 1000, 1)} mm  |xy| = {np.linalg.norm(d[:2]) * 1000:.1f} mm")
        print(f"离地高度 h ≈ {c_drv[2] - GROUND_Z:.3f} m | 到支撑矩形边界: x {sx * 100:.1f} cm, y {sy * 100:.1f} cm"
              f" | 到 warn/stop 盒边界: {mw * 100:.1f}/{ms * 100:.1f} cm")

    q0, cm0, cd0 = snapshot()
    report(cd0, cm0, "static：当前位姿")
    print(f"关节(deg) L={np.round(q0[:7], 1)} R={np.round(q0[7:14], 1)} body={np.round(q0[14:19], 1)}")

    if mode != "sweep":
        ros.shutdown(); return

    # ---------------- sweep ----------------
    if pose not in POSES:
        print(f"--pose 应为 {list(POSES)}"); ros.shutdown(); return
    target = np.array(POSES[pose], dtype=float)
    # 运动前必做：目标位姿与起点→目标的线性插值路径上，臂与工具尖离地高度复核
    clearance = min(check_floor_clearance(robot, r, arm_names + body_names + head_names,
                                          np.concatenate([q0[:14] + s * (target - q0[:14]), q0[14:]]))
                    for s in np.linspace(0.0, 1.0, 21))
    print(f"\n[安全] 路径上臂/工具尖最小离地 {clearance:.2f} m（要求 ≥ {FLOOR_CLEARANCE}）")
    if clearance < FLOOR_CLEARANCE:
        print("❌ 位姿会贴近/碰到地面，拒绝运动"); ros.shutdown(); return
    inex = InexClient(ros)
    if not inex.prepare_arms():
        print("❌ 双臂未确认上电"); ros.shutdown(); return
    if not arm.wait_servo_subscriber(3.0):
        ros.shutdown(); return
    # 预测：模型质心在目标位姿的位置
    cm_t = model_com(robot, arm_names + body_names + head_names, np.concatenate([target, q0[14:]]))
    print(f"\n=== sweep → {pose}: 模型预测质心位移 {np.round((cm_t - cm0) * 1000, 1)} mm ===")
    if not yes:
        input("确认双臂前方/侧方有安全空间后按回车开始，Ctrl+C 取消 > ")

    log = []
    stop = False

    def sampler():
        while not stop:
            q, cm, cd = snapshot()
            log.append((time.monotonic(), cd.copy(), cm.copy(), guard.evaluate()))
            time.sleep(0.05)

    import threading
    th = threading.Thread(target=sampler, daemon=True); th.start()
    arm.move_to(target, duration, r.move_rate_hz)
    time.sleep(1.0)
    q1, cm1, cd1 = snapshot()
    report(cd1, cm1, f"sweep：到达 {pose}")
    print(f"实测质心位移 (驱动) = {np.round((cd1 - cd0) * 1000, 1)} mm | 模型预测 = {np.round((cm_t - cm0) * 1000, 1)} mm")
    arm.move_to(q0[:14], duration, r.move_rate_hz)
    stop = True; th.join(1.0)
    inex.halt()
    diffs = np.array([l[1] - l[2] for l in log]); states = [l[3] for l in log]
    print(f"\n全程 {len(log)} 采样：驱动-模型 差 xy p50/p90/max = {np.percentile(np.linalg.norm(diffs[:, :2], axis=1), 50) * 1000:.1f}/"
          f"{np.percentile(np.linalg.norm(diffs[:, :2], axis=1), 90) * 1000:.1f}/{np.linalg.norm(diffs[:, :2], axis=1).max() * 1000:.1f} mm"
          f" | 质心 x 范围 {min(l[1][0] for l in log):.3f}~{max(l[1][0] for l in log):.3f} m"
          f" | guard 状态计数 OK/WARN/STOP = {states.count('OK')}/{states.count('WARN')}/{states.count('STOP')}")
    ros.shutdown()


if __name__ == "__main__":
    tyro.cli(main)
