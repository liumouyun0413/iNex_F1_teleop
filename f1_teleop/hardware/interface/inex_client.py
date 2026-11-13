"""
InexClient —— iNex ROS2 (inex-ros2-v1.0.0_1.1.4) 服务/状态封装。

把 lxl 版 init_f1.py 中与"系统准备"有关的服务调用收拢到一个薄客户端：
  * 上电（按 /inex/system_status 的 powered_mask 逐臂补齐）
  * 机器人模式（DualArmOnly=2 / WholeBody=3，已含双臂则跳过）
  * 运动 profile（JOINT_POSITION）与 ServoJ 流式模式（按分区）
  * 关闭单点心跳（ServoJ 之前必须）
  * 软限位查询、夹爪初始化 / 状态发布频率、原生碰撞检测查询
  * 平滑制动 Halt

所有调用都是"查询后设置、幂等"，失败只告警不抛（启动不被单项服务卡死），
但上电失败/复核失败会明确打印【严重】。
"""

from __future__ import annotations

import time
from typing import List, Optional, Tuple

from f1_teleop.hardware.interface.ros_context import RosContext

try:
    from inex_ros2_interface.msg import RobotSystemStatus
    from inex_ros2_interface.srv import (
        GetCollisionConfig,
        GetSoftLimits,
        Halt,
        InitGripper,
        PowerChange,
        RobotMode,
        SetGripperForceSpeed,
        SetGripperPublishFrequency,
        SetMotionProfile,
        SetMotionServoMode,
        SetRobotMode,
        StopSinglePointHeartbeat,
    )
except ImportError as e:  # pragma: no cover
    raise ImportError(
        "无法导入 inex_ros2_interface：请先 source iNex 工作区 "
        "(例如 ~/workspace/inex_ros/install/setup.bash)，接口包需为 v1.0（含 ServoCommand）"
    ) from e

# ---- RobotPart（ServoCommand / SetMotionProfile / Halt 共用）----
PART_BODY = 0
PART_LEFT_ARM = 1
PART_RIGHT_ARM = 2
PART_HEAD = 3
PART_ALL = 0xFF
# ---- PowerChange.group（ServoGroup）----
GROUP_ALL = 0
GROUP_LEFT_ARM = 1
GROUP_RIGHT_ARM = 2
GROUP_BODY = 3
# ---- RobotSystemStatus.powered_mask 位 ----
MASK_LEFT_ARM = 0x01
MASK_RIGHT_ARM = 0x02
MASK_BODY = 0x04
MASK_HEAD = 0x08
# ---- controller::RobotMode ----
ROBOT_MODE_DUAL_ARM = 2
ROBOT_MODE_WHOLE_BODY = 3
# ---- MotionProfile / MotionServoMode ----
PROFILE_JOINT_POSITION = 1
SERVO_MODE_J = 4
# ---- RobotSystemStatus.state ----
STATE_IDLE, STATE_READY, STATE_RUNNING, STATE_ERROR = 0, 1, 2, 3
# ---- 驱动常见拒绝码（robot_manager_user_error.h）----
ERR_SOFT_LIMIT = 40002
ERR_MOTION_REJECTED = 41020

PART_NAME = {PART_BODY: "躯干", PART_LEFT_ARM: "左臂", PART_RIGHT_ARM: "右臂", PART_HEAD: "头部"}


class InexClient:
    def __init__(self, ros: RosContext, service_timeout: float = 3.0) -> None:
        self.ros = ros
        self.service_timeout = service_timeout
        node = ros.node
        self._power_change = node.create_client(PowerChange, "/inex/power_change")
        self._robot_mode = node.create_client(RobotMode, "/inex/robot_mode")
        self._set_robot_mode = node.create_client(SetRobotMode, "/inex/set_robot_mode")
        self._set_profile = node.create_client(SetMotionProfile, "/inex/set_motion_profile")
        self._set_servo_mode = node.create_client(SetMotionServoMode, "/inex/set_motion_servo_mode")
        self._stop_heartbeat = node.create_client(StopSinglePointHeartbeat, "/inex/stop_single_point_heartbeat")
        self._halt = node.create_client(Halt, "/inex/halt")
        self._soft_limits = node.create_client(GetSoftLimits, "/inex/get_soft_limits")
        self._init_gripper = node.create_client(InitGripper, "/inex/init_gripper")
        self._gripper_freq = node.create_client(SetGripperPublishFrequency, "/inex/set_gripper_publish_frequency")
        self._gripper_fs = node.create_client(SetGripperForceSpeed, "/inex/set_gripper_force_speed")
        self._get_collision = node.create_client(GetCollisionConfig, "/inex/get_collision_config")

        self._powered_mask: Optional[int] = None
        self._robot_mode_cache: Optional[int] = None
        self._state: Optional[int] = None
        node.create_subscription(RobotSystemStatus, "/inex/system_status", self._on_status, 10)

    # ------------------------------------------------------------------ #
    # 基础
    # ------------------------------------------------------------------ #
    def _on_status(self, msg: RobotSystemStatus) -> None:
        self._powered_mask = int(msg.powered_mask)
        self._robot_mode_cache = int(msg.robot_mode)
        self._state = int(msg.state)

    def _call(self, client, request, name: str, timeout: Optional[float] = None):
        """同步服务调用；服务不可用或超时返回 None（应答由 RosContext 的 spin 线程处理）。"""
        if not client.wait_for_service(timeout_sec=self.service_timeout):
            print(f"[iNex] ❌ 服务不可用: {name}")
            return None
        fut = client.call_async(request)
        deadline = time.monotonic() + (timeout or 5.0)
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not fut.done():
            print(f"[iNex] ⚠️ 服务超时: {name}")
            return None
        try:
            return fut.result()
        except Exception as e:
            print(f"[iNex] ⚠️ 服务异常 {name}: {e!r}")
            return None

    @staticmethod
    def _code(res) -> str:
        return "超时" if res is None else str(res.code)

    def powered_mask(self, wait: float = 2.0) -> Optional[int]:
        deadline = time.monotonic() + wait
        while self._powered_mask is None and time.monotonic() < deadline:
            time.sleep(0.05)
        return self._powered_mask

    @property
    def state(self) -> Optional[int]:
        return self._state

    def recover_from_error(self, group: int = GROUP_LEFT_ARM) -> bool:
        """state==ERROR 时用 power_on 重试清错（SDK 状态表：ERROR → power_on → READY）；非 ERROR 直接返回 True。"""
        if self._state != STATE_ERROR:
            return True
        print("[iNex] ⚠️ 系统处于 ERROR（上次运动被拒/故障），用 power_on 重试清错...")
        req = PowerChange.Request(); req.power_on = True; req.group = group
        res = self._call(self._power_change, req, "power_change", timeout=30.0)
        deadline = time.monotonic() + 3.0
        while self._state == STATE_ERROR and time.monotonic() < deadline:
            time.sleep(0.05)
        ok = res is not None and res.code == 0 and self._state != STATE_ERROR
        print(f"[iNex] {'✅ ERROR 已清除' if ok else '❌ ERROR 未清除'} code={self._code(res)} state={self._state}")
        return ok

    # ------------------------------------------------------------------ #
    # 准备时序（lxl init_f1._prepare_system 的等价实现）
    # ------------------------------------------------------------------ #
    def prepare_arms(self, parts: Tuple[int, ...] = (PART_LEFT_ARM, PART_RIGHT_ARM)) -> bool:
        """关闭心跳 → 上电（按掩码补齐）→ 机器人模式 → JOINT_POSITION → ServoJ。返回双臂是否确认上电。"""
        res = self._call(self._stop_heartbeat, StopSinglePointHeartbeat.Request(), "stop_single_point_heartbeat")
        print(f"[iNex] 关闭单点心跳 code={self._code(res)}")

        self.powered_mask()  # 等首帧 system_status，顺带拿到 state
        self.recover_from_error()
        powered = self._power_on_arms()
        self._ensure_robot_mode(ROBOT_MODE_DUAL_ARM)
        for part in parts:
            self.set_profile(part, PROFILE_JOINT_POSITION)
            self.set_servo_mode(part, SERVO_MODE_J)
        self.print_collision_config(parts)
        return powered

    def _power_on_arms(self) -> bool:
        mask = self.powered_mask()
        if mask is None:
            # 收不到 system_status 时退回整体上电（旧行为）
            print("[iNex] ⚠️ 未收到 /inex/system_status，退回整体上电")
            req = PowerChange.Request(); req.power_on = True; req.group = GROUP_ALL
            res = self._call(self._power_change, req, "power_change", timeout=30.0)
            print(f"[iNex] 整体上电 code={self._code(res)}")
            return res is not None and res.code == 0

        need = [(GROUP_LEFT_ARM, MASK_LEFT_ARM, "左臂"), (GROUP_RIGHT_ARM, MASK_RIGHT_ARM, "右臂")]
        print(f"[iNex] 上电状态 mask={mask:#04x} → "
              + " ".join(f"{n}{'✅' if mask & m else '❌'}" for _, m, n in need))
        for group, bit, name in need:
            if mask & bit:
                continue
            req = PowerChange.Request(); req.power_on = True; req.group = group
            res = self._call(self._power_change, req, "power_change", timeout=30.0)
            print(f"[iNex] {name}上电 code={self._code(res)}")
        return self._verify_arms_powered()

    def _verify_arms_powered(self, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            m = self._powered_mask
            if m is not None and (m & MASK_LEFT_ARM) and (m & MASK_RIGHT_ARM):
                print(f"[iNex] ✅ 双臂均已上电 mask={m:#04x}")
                return True
            time.sleep(0.1)
        m = self._powered_mask
        print(f"[iNex] ❌【严重】双臂上电复核失败 mask={m if m is None else hex(m)}，未上电的臂遥操无响应！")
        return False

    def _ensure_robot_mode(self, wanted: int) -> None:
        res = self._call(self._robot_mode, RobotMode.Request(), "robot_mode")
        mode = res.robot_mode if (res is not None and res.code == 0) else self._robot_mode_cache
        if mode in (ROBOT_MODE_DUAL_ARM, ROBOT_MODE_WHOLE_BODY):
            print(f"[iNex] 机器人模式={mode}，已包含双臂，跳过")
            return
        req = SetRobotMode.Request(); req.robot_mode = wanted
        res = self._call(self._set_robot_mode, req, "set_robot_mode")
        print(f"[iNex] 设置机器人模式={wanted} code={self._code(res)}")

    def set_profile(self, part: int, profile: int = PROFILE_JOINT_POSITION) -> bool:
        req = SetMotionProfile.Request(); req.profile = profile; req.part = part
        res = self._call(self._set_profile, req, "set_motion_profile")
        ok = res is not None and res.code == 0
        print(f"[iNex] {PART_NAME.get(part, part)} profile={profile} {'✅' if ok else '❌ code=' + self._code(res)}")
        return ok

    def set_servo_mode(self, part: int, mode: int = SERVO_MODE_J) -> bool:
        req = SetMotionServoMode.Request(); req.mode = mode; req.part = part
        res = self._call(self._set_servo_mode, req, "set_motion_servo_mode")
        ok = res is not None and res.code == 0
        print(f"[iNex] {PART_NAME.get(part, part)} servo_mode={mode} {'✅' if ok else '❌ code=' + self._code(res)}")
        return ok

    def print_collision_config(self, parts) -> None:
        for part in parts:
            req = GetCollisionConfig.Request(); req.part = part
            res = self._call(self._get_collision, req, "get_collision_config", timeout=2.0)
            if res is not None and res.code == 0:
                print(f"[iNex] {PART_NAME.get(part, part)} 原生碰撞检测 enabled={res.enabled} params={list(res.params)}")

    # ------------------------------------------------------------------ #
    # 查询 / 夹爪 / 制动
    # ------------------------------------------------------------------ #
    def get_soft_limits_deg(self, part: int) -> Optional[List[Tuple[float, float]]]:
        """返回 7 组 (min, max)（度）；失败返回 None。"""
        req = GetSoftLimits.Request(); req.part = part
        res = self._call(self._soft_limits, req, "get_soft_limits", timeout=2.0)
        if res is None or res.code != 0 or len(res.q_min_deg) != 7:
            return None
        return [(float(lo), float(hi)) for lo, hi in zip(res.q_min_deg, res.q_max_deg)]

    def init_grippers(self, publish_hz: int = 30, force_speed: Optional[Tuple[int, int]] = None,
                      timeout: float = 30.0) -> bool:
        """双夹爪初始化（需双臂已上电）；完成后设置状态发布频率与可选的力/速度。"""
        ok = True
        for side, name in ((1, "左夹爪"), (2, "右夹爪")):
            req = InitGripper.Request(); req.side = side
            res = self._call(self._init_gripper, req, "init_gripper", timeout=timeout)
            good = res is not None and res.code == 0
            ok &= good
            print(f"[iNex] {name}初始化 {'✅' if good else '❌ code=' + self._code(res) + '（1 参数非法 2 通信失败 3 超时）'}")
        req = SetGripperPublishFrequency.Request(); req.frequency_hz = int(publish_hz)
        res = self._call(self._gripper_freq, req, "set_gripper_publish_frequency")
        print(f"[iNex] 夹爪状态发布 {publish_hz}Hz code={self._code(res)}")
        if force_speed is not None:
            for side in (1, 2):
                req = SetGripperForceSpeed.Request()
                req.side = side; req.force = int(force_speed[0]); req.speed = int(force_speed[1])
                res = self._call(self._gripper_fs, req, "set_gripper_force_speed")
                print(f"[iNex] 夹爪{side} 力/速度={force_speed} code={self._code(res)}")
        return ok

    def halt(self, part: int = PART_ALL) -> None:
        """平滑制动（不下电）；退出时调用，ServoJ 流停止后必须显式停。"""
        req = Halt.Request(); req.part = part
        res = self._call(self._halt, req, "halt", timeout=3.0)
        print(f"[iNex] 平滑制动 part={part} code={self._code(res)}")
