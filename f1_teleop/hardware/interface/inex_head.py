"""
InexHeadController —— 头部增量位置控制（ServoJ part=3，可选）。

lxl 珞石版没有头部控制；本类按 f1_tianji_teleop 的 F1HeadController 接口保留同样的
摇杆增量逻辑，输出改为 /inex/servo_cmd(part=PART_HEAD)。启用前提（见 README）：
  * 机器人模式须含头部（WholeBody=3），头部分区已上电；
  * 已对 part=3 调 set_motion_profile(JOINT_POSITION) 与 set_motion_servo_mode(ServoJ)。
`enable=False` 时所有方法为空操作，只保存/返回目标角，供 /lead/joint_states 镜像使用。
"""

from __future__ import annotations

import threading
from typing import Optional, Sequence

import numpy as np
from inex_ros2_interface.msg import ServoCommand
from rclpy.qos import qos_profile_sensor_data

from f1_teleop.hardware.interface.inex_client import (
    PART_HEAD,
    PROFILE_JOINT_POSITION,
    SERVO_MODE_J,
    InexClient,
)
from f1_teleop.hardware.interface.ros_context import RosContext


class InexHeadController:
    def __init__(
        self,
        ros: RosContext,
        limits_deg: Sequence[Sequence[float]],
        step_deg_per_frame: float,
        servo_cmd_topic: str,
        enable: bool = False,
        inex: Optional[InexClient] = None,
        servo_axes: int = 6,
    ) -> None:
        self.enable = bool(enable)
        self.limits = np.asarray(limits_deg, dtype=float)
        self.step = float(step_deg_per_frame)
        self.n = self.limits.shape[0]
        self.servo_axes = int(servo_axes)        # 驱动 head 缓冲长度（v1.0 为 6，不足补零）
        if self.limits.shape != (self.n, 2):
            raise ValueError("头部 limits_deg 维度应为 (关节数, 2)")

        self._lock = threading.Lock()
        self._target = np.zeros(self.n)
        self._pub = None
        if self.enable:
            self._pub = ros.node.create_publisher(ServoCommand, servo_cmd_topic, qos_profile_sensor_data)
            self._msg = ServoCommand()
            self._msg.part = PART_HEAD
            self._msg.cmd_type = ServoCommand.CMD_TYPE_J
            if inex is not None:
                inex.set_profile(PART_HEAD, PROFILE_JOINT_POSITION)
                inex.set_servo_mode(PART_HEAD, SERVO_MODE_J)
            print("[Head] 头部 ServoJ 控制已启用（需机器人模式含头部且头部已上电）")
        else:
            print("[Head] 头部控制未启用（head.enable=false），摇杆头部模式仅更新目标不下发")

    def set_target(self, target_deg: Sequence[float]) -> None:
        with self._lock:
            self._target = np.clip(np.asarray(target_deg, dtype=float)[: self.n], self.limits[:, 0], self.limits[:, 1])

    def get_target(self) -> list[float]:
        with self._lock:
            return self._target.tolist()

    def step_by(self, axis_values: Sequence[float]) -> bool:
        axes = np.asarray(axis_values, dtype=float)[: self.n]
        if not np.any(axes):
            return False
        with self._lock:
            new = np.clip(self._target + axes * self.step, self.limits[:, 0], self.limits[:, 1])
            changed = not np.array_equal(new, self._target)
            self._target = new
        return changed

    def publish(self) -> None:
        if self._pub is None:
            return
        q = np.zeros(self.servo_axes)
        tgt = self.get_target()
        q[: len(tgt)] = tgt
        self._msg.q_deg = q.tolist()
        self._pub.publish(self._msg)
