#!/usr/bin/env python3
"""
F1（珞石双臂 + iNex ROS2）PICO 遥操作入口（对应官方 scripts/hardware/teleop_r1lite_hardware.py）。

    python scripts/teleop_f1_hardware.py
    python scripts/teleop_f1_hardware.py --config /path/to/f1_teleop.yaml --visualize-placo
    python scripts/teleop_f1_hardware.py --no-planner       # 调试：关闭 ServoJ 速度规划器，直接下发 IK 解
    python scripts/teleop_f1_hardware.py --speed-scale 0.5  # 限速版：所有速度/加速度上限 ×0.5（跟随滞后但更温和）
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import tyro  # noqa: E402

from f1_teleop.config import apply_speed_scale, load_config  # noqa: E402
from f1_teleop.hardware.f1_teleop_controller import F1RokaeTeleopController  # noqa: E402


def main(
    config: str | None = None,
    visualize_placo: bool = False,
    scale_factor: float | None = None,
    control_rate_hz: int | None = None,
    no_safety: bool = False,
    no_recording: bool = False,
    no_planner: bool = False,
    no_gripper_init: bool = False,
    speed_scale: float | None = None,
) -> None:
    """
    Args:
        config:           配置文件路径，默认 f1_teleop/config/f1_teleop.yaml
        visualize_placo:  打开 Meshcat 可视化 Placo 模型
        scale_factor:     覆盖配置中的手柄位移放大倍数
        control_rate_hz:  覆盖配置中的控制频率
        no_safety:        禁用质心/碰撞安全门
        no_recording:     禁用 ros2 bag 录制
        no_planner:       禁用 ServoJ 速度受限规划器（直接下发 IK 解，仅调试）
        no_gripper_init:  跳过 /inex/init_gripper
        speed_scale:      限速系数 (0,1]，覆盖 yaml teleop.speed_scale：规划器 vmax/amax、IK 关节速度、末端摆率统一乘以该值
    """
    cfg = load_config(config)
    if speed_scale is not None:
        apply_speed_scale(cfg, speed_scale)
    if scale_factor is not None:
        cfg.teleop.scale_factor = scale_factor
    if control_rate_hz is not None:
        cfg.teleop.control_rate_hz = control_rate_hz
    if no_safety:
        cfg.safety.enable = False
    if no_recording:
        cfg.recording.enable = False
    if no_planner:
        cfg.inex.servo_planner.enable = False
    if no_gripper_init:
        cfg.inex.init_grippers = False

    print(f"[Main] 配置: {cfg.config_path}")
    print(f"[Main] URDF: {cfg.robot.urdf_path}")
    ss = float(getattr(cfg.teleop, "speed_scale", 1.0))
    if ss < 1.0:
        pl, t = cfg.inex.servo_planner, cfg.teleop
        r = lambda v: [round(x) for x in v] if isinstance(v, list) else v  # noqa: E731
        print(f"[Main] ⚠️ 限速版 speed_scale={ss:.2f}：规划器 vmax={r(pl.vmax_deg_s)}°/s amax={r(pl.amax_deg_s2)}°/s²，"
              f"IK 关节速度 {r(t.ik.joint_velocity_limits_deg_s)}°/s，末端 {t.max_ee_speed_m_s} m/s / {t.max_ee_ang_speed_rad_s} rad/s")
    controller = F1RokaeTeleopController(cfg, visualize_placo=visualize_placo)
    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
