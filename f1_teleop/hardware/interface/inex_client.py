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
from typing import List, Optional, Sequence, Tuple

from f1_teleop.hardware.interface.ros_context import RosContext

try:
    from inex_ros2_interface.msg import RobotSystemStatus
    from inex_ros2_interface.srv import (
        GetCollisionConfig,
        GetPayload,
        GetSoftLimits,
        Halt,
        HotReset,
        InitGripper,
        PowerChange,
        RobotMode,
        SetGripperForceSpeed,
        SetGripperPublishFrequency,
        SetJointStiffness,
        SetMotionProfile,
        SetMotionServoMode,
        SetPayload,
        SetRobotMode,
        StartImpedance,
        StopImpedanceSession,
        StopSinglePointHeartbeat,
    )
except ImportError as e:  # pragma: no cover
    raise ImportError(
        "无法导入 inex_ros2_interface：请先 source iNex 工作区 "
        "(例如 ~/workspace/inex_ros/install/setup.bash)，接口包需为 v1.0（含 ServoCommand）"
    ) from e

try:  # 持物质心绑定：inex-ros2 v1.0.2 held-object-com-1.0.0 起才有，旧接口包上为 None
    from inex_ros2_interface.srv import ClearHeldObject, GetHeldObject, SetHeldObject
except ImportError:  # pragma: no cover
    ClearHeldObject = GetHeldObject = SetHeldObject = None

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
PROFILE_JOINT_IMPEDANCE = 2
SERVO_MODE_J = 4
# ---- RobotSystemStatus.state ----
STATE_IDLE, STATE_READY, STATE_RUNNING, STATE_ERROR = 0, 1, 2, 3
# ---- 驱动常见拒绝码（robot_manager_user_error.h）----
ERR_SOFT_LIMIT = 40002
ERR_MOTION_REJECTED = 41020
ERR_BUSY = 10034
ERR_POWER_IN_PROGRESS = -3   # power_change 长操作未在 RPC 窗口内完成，实际仍在执行（见 _wait_powered）

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
        self._hot_reset = node.create_client(HotReset, "/inex/hot_reset")
        self._soft_limits = node.create_client(GetSoftLimits, "/inex/get_soft_limits")
        self._init_gripper = node.create_client(InitGripper, "/inex/init_gripper")
        self._gripper_freq = node.create_client(SetGripperPublishFrequency, "/inex/set_gripper_publish_frequency")
        self._gripper_fs = node.create_client(SetGripperForceSpeed, "/inex/set_gripper_force_speed")
        self._get_collision = node.create_client(GetCollisionConfig, "/inex/get_collision_config")
        self._set_stiffness = node.create_client(SetJointStiffness, "/inex/set_joint_stiffness")
        self._start_impedance = node.create_client(StartImpedance, "/inex/start_impedance")
        self._stop_impedance = node.create_client(StopImpedanceSession, "/inex/stop_impedance_session")
        self._get_payload = node.create_client(GetPayload, "/inex/get_payload")
        self._set_payload = node.create_client(SetPayload, "/inex/set_payload")
        self._held_available = SetHeldObject is not None
        if self._held_available:
            self._get_held = node.create_client(GetHeldObject, "/inex/get_held_object")
            self._set_held = node.create_client(SetHeldObject, "/inex/set_held_object")
            self._clear_held = node.create_client(ClearHeldObject, "/inex/clear_held_object")
        self.impedance_parts: Tuple[int, ...] = ()   # 已开启阻抗会话的分区（退出时 stop_impedance_session）

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
        """state==ERROR 时清错。实测有效顺序（2026-11-13）：hot_reset（= UI「报警复位」，清伺服故障/闩锁）
        → 逐臂 power_on（Busy 10034 则稍等重试）。单纯 power_on / power_off→on 都不能清掉伺服故障。"""
        if self._state != STATE_ERROR:
            return True
        print("[iNex] ⚠️ 系统处于 ERROR（上次运动被拒/故障），执行报警复位 hot_reset...")
        res = self._call(self._hot_reset, HotReset.Request(), "hot_reset", timeout=40.0)
        print(f"[iNex] hot_reset code={self._code(res)}")
        time.sleep(1.0)
        ok = True
        for g in (GROUP_LEFT_ARM, GROUP_RIGHT_ARM):
            ok &= self._power_on_retry(g)
        deadline = time.monotonic() + 3.0
        while self._state == STATE_ERROR and time.monotonic() < deadline:
            time.sleep(0.05)
        ok = ok and self._state != STATE_ERROR
        print(f"[iNex] {'✅ ERROR 已清除' if ok else '❌ ERROR 仍未清除，请在 UI 做报警复位/检查驱动日志'} state={self._state}")
        return ok

    def _power_on_retry(self, group: int, attempts: int = 3) -> bool:
        bit = {GROUP_LEFT_ARM: MASK_LEFT_ARM, GROUP_RIGHT_ARM: MASK_RIGHT_ARM, GROUP_BODY: MASK_BODY}.get(group, 0)
        for i in range(attempts):
            req = PowerChange.Request(); req.power_on = True; req.group = group
            res = self._call(self._power_change, req, "power_change", timeout=30.0)
            code = None if res is None else int(res.code)
            print(f"[iNex] power_on group={group} code={self._code(res)}")
            if code == 0:
                return True
            if code == ERR_POWER_IN_PROGRESS and self._wait_powered(bit, 3.0):
                return True
            if code not in (ERR_BUSY, ERR_POWER_IN_PROGRESS):
                return False
            time.sleep(1.5)   # Busy：上一组上电/复位尚未完成
        return False

    def _wait_powered(self, bit: int, timeout: float) -> bool:
        """等 system_status.powered_mask 置位。上电在驱动内约 0.8 s，ROS2 桥接的 RPC 对这类长操作常先返回 -3（实际仍在执行并成功，
        2026-11-17 robot_manager 日志核实），所以以掩码为准而不是返回码。"""
        if not bit:
            return False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._powered_mask is not None and self._powered_mask & bit:
                print(f"[iNex] power_on 已按 powered_mask 确认完成 mask={self._powered_mask:#04x}")
                return True
            time.sleep(0.1)
        return False

    # ------------------------------------------------------------------ #
    # 准备时序（lxl init_f1._prepare_system 的等价实现）
    # ------------------------------------------------------------------ #
    def prepare_arms(self, parts: Tuple[int, ...] = (PART_LEFT_ARM, PART_RIGHT_ARM),
                     impedance_parts: Tuple[int, ...] = (),
                     joint_stiffness: Optional[Sequence[float]] = None,
                     payload_cfg=None, held_cfg=None) -> bool:
        """关闭心跳 → 上电（按掩码补齐）→ 机器人模式 → 负载 → 运动档 → ServoJ。返回双臂是否确认上电。

        impedance_parts 中的分区走关节阻抗档：READY 下热切 JOINT_IMPEDANCE → （可选）设关节刚度 → start_impedance（HOLD）→ ServoJ。
        2026-11-14 ms-thor 实测：阻抗 HOLD（activity=3）下 set_motion_servo_mode(ServoJ) 返回 0。不标定力矩传感器。"""
        res = self._call(self._stop_heartbeat, StopSinglePointHeartbeat.Request(), "stop_single_point_heartbeat")
        print(f"[iNex] 关闭单点心跳 code={self._code(res)}")

        self.powered_mask()  # 等首帧 system_status，顺带拿到 state
        self.recover_from_error()
        powered = self._power_on_arms()
        self._ensure_robot_mode(ROBOT_MODE_DUAL_ARM)
        self.apply_payloads(payload_cfg)
        self.apply_held_objects(held_cfg)
        # 先把所有分区的运动档设好，再开阻抗会话：set_motion_profile 会重新 apply 控制模式，
        # 在另一臂会话已开启时调用会把那只臂打回位置控制（2026-11-14 左臂先 start、再设右臂档 → 左臂不再柔顺）
        for part in parts:
            self.set_profile(part, PROFILE_JOINT_IMPEDANCE if part in impedance_parts else PROFILE_JOINT_POSITION)
        failed = []
        for part in parts:
            if part in impedance_parts:
                if joint_stiffness is not None:
                    self.set_joint_stiffness(part, joint_stiffness)
                if not self.start_impedance(part):     # 文档：part 禁止 255，双臂顺序两次
                    failed.append(part)
        if failed:
            # 阻抗档“待命”(ARMED, execution_mask=0) 下关节无保持力矩且忽略 joint/ref：2026-11-14 配置了未装上的 5 kg
            # 负载 → 30003 力矩偏差拒绝 → 双臂在待命态下垂 77°。失败必须退回位置档让臂保持住；set_motion_profile 会
            # 打断另一臂已开启的会话，所以整组一起退回（先停会话再切档）。
            print(f"[iNex] ❌ {'/'.join(PART_NAME.get(p, str(p)) for p in failed)} 阻抗启动失败，双臂退回 JOINT_POSITION 以保持位置"
                  "（30003=力矩偏差过大：负载参数与实际不符或力矩传感器需标定）")
            self.stop_impedance_sessions()
            for part in impedance_parts:
                self.set_profile(part, PROFILE_JOINT_POSITION)
        for part in parts:
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

    def set_joint_stiffness(self, part: int, stiffness: Sequence[float]) -> bool:
        req = SetJointStiffness.Request(); req.part = part; req.stiffness = [float(k) for k in stiffness]
        res = self._call(self._set_stiffness, req, "set_joint_stiffness")
        ok = res is not None and res.code == 0
        print(f"[iNex] {PART_NAME.get(part, part)} 关节刚度={list(req.stiffness)} Nm/rad {'✅' if ok else '❌ code=' + self._code(res)}")
        return ok

    def start_impedance(self, part: int) -> bool:
        req = StartImpedance.Request(); req.part = part
        res = self._call(self._start_impedance, req, "start_impedance", timeout=10.0)
        ok = res is not None and res.code == 0
        print(f"[iNex] {PART_NAME.get(part, part)} start_impedance {'✅' if ok else '❌ code=' + self._code(res)}")
        if ok and part not in self.impedance_parts:
            self.impedance_parts = self.impedance_parts + (part,)
        return ok

    def stop_impedance_sessions(self) -> None:
        for part in self.impedance_parts:
            req = StopImpedanceSession.Request(); req.part = part
            res = self._call(self._stop_impedance, req, "stop_impedance_session", timeout=10.0)
            print(f"[iNex] {PART_NAME.get(part, part)} stop_impedance_session code={self._code(res)}")
        self.impedance_parts = ()

    # ------------------------------------------------------------------ #
    # 负载（影响 /inex/com_state 整机质心与阻抗重力补偿）
    # ------------------------------------------------------------------ #
    def get_payload(self, part: int):
        req = GetPayload.Request(); req.part = part
        res = self._call(self._get_payload, req, "get_payload", timeout=2.0)
        return res if (res is not None and res.code == 0) else None

    def set_payload(self, part: int, mass: float, com: Sequence[float],
                    inertia: Optional[Sequence[float]] = None) -> bool:
        req = SetPayload.Request()
        req.part = part; req.mass = float(mass)
        req.com = [float(v) for v in com]
        req.inertia = [float(v) for v in (inertia if inertia is not None else (0.0,) * 6)]
        if len(req.com) != 3 or len(req.inertia) != 6:
            raise ValueError(f"payload com 需 3 个值、inertia 需 6 个值（{PART_NAME.get(part, part)}）")
        res = self._call(self._set_payload, req, "set_payload", timeout=5.0)
        ok = res is not None and res.code == 0
        print(f"[iNex] {PART_NAME.get(part, part)} set_payload mass={req.mass} com={list(req.com)} {'✅' if ok else '❌ code=' + self._code(res)}")
        return ok

    def apply_payloads(self, payload_cfg) -> None:
        """按配置（inex.payload.left/right：mass, com[3], inertia[6]；null=不改）设置负载，并打印驱动当前值。"""
        for side, part in (("left", PART_LEFT_ARM), ("right", PART_RIGHT_ARM)):
            spec = getattr(payload_cfg, side, None) if payload_cfg is not None else None
            if spec is not None:
                self.set_payload(part, spec.mass, spec.com, getattr(spec, "inertia", None))
            cur = self.get_payload(part)
            if cur is None:
                print(f"[iNex] {PART_NAME[part]} 负载查询失败")
            else:
                print(f"[iNex] {PART_NAME[part]} 负载: configured={cur.configured} mass={cur.mass:.3f} kg "
                      f"com={[round(float(v), 4) for v in cur.com]} m" + ("" if spec is not None else "（沿用驱动当前值）"))

    # ------------------------------------------------------------------ #
    # 持物质心绑定（/inex/*_held_object）：只参与 /inex/com_state 整机质心（质心保护的输入），
    # 不进重力补偿（文档"动力学接入另行设计"）；控制器重启后清零，遥操重启不会清，因此 null 时显式解绑。
    # 注意：同一件重物不要既算进 payload 又登记为 held_object，会被计两次。
    # ------------------------------------------------------------------ #
    def get_held_objects(self):
        if not self._held_available:
            return None
        res = self._call(self._get_held, GetHeldObject.Request(), "get_held_object", timeout=2.0)
        return res if (res is not None and res.code == 0) else None

    def set_held_object(self, part: int, mass: float, com: Sequence[float],
                        inertia: Optional[Sequence[float]] = None) -> bool:
        req = SetHeldObject.Request()
        req.part = part; req.mass = float(mass)
        req.com = [float(v) for v in com]
        req.inertia = [float(v) for v in (inertia if inertia is not None else (0.0,) * 6)]
        if len(req.com) != 3 or len(req.inertia) != 6 or not req.mass > 0:
            raise ValueError(f"held_object 需 mass>0、com 3 个值、inertia 6 个值（{PART_NAME.get(part, part)}）")
        res = self._call(self._set_held, req, "set_held_object", timeout=5.0)
        ok = res is not None and res.code == 0
        print(f"[iNex] {PART_NAME.get(part, part)} set_held_object mass={req.mass} com={list(req.com)}（法兰系） "
              f"{'✅' if ok else '❌ code=' + self._code(res)}")
        return ok

    def clear_held_object(self, part: int) -> bool:
        req = ClearHeldObject.Request(); req.part = part
        res = self._call(self._clear_held, req, "clear_held_object", timeout=5.0)
        ok = res is not None and res.code == 0
        if not ok:
            print(f"[iNex] {PART_NAME.get(part, part)} clear_held_object ❌ code={self._code(res)}")
        return ok

    def apply_held_objects(self, held_cfg) -> None:
        """inex.held_object.left/right：{mass, com[3], inertia[6]} 登记，null 解绑；之后打印两臂当前绑定。"""
        if not self._held_available:
            if held_cfg is not None and any(getattr(held_cfg, s, None) is not None for s in ("left", "right")):
                print("[iNex] ⚠️ 接口包无 SetHeldObject（需 inex-ros2 held-object-com 版本），忽略 held_object 配置")
            return
        for side, part in (("left", PART_LEFT_ARM), ("right", PART_RIGHT_ARM)):
            spec = getattr(held_cfg, side, None) if held_cfg is not None else None
            if spec is not None:
                self.set_held_object(part, spec.mass, spec.com, getattr(spec, "inertia", None))
            else:
                self.clear_held_object(part)
        cur = self.get_held_objects()
        if cur is None:
            print("[iNex] 持物绑定查询失败")
            return
        for side, cfgd, m, c in (("左臂", cur.left_configured, cur.left_mass, cur.left_com),
                                 ("右臂", cur.right_configured, cur.right_mass, cur.right_com)):
            print(f"[iNex] {side} 持物: " + (f"{m:.3f} kg com={[round(float(v), 4) for v in c]} m（法兰系）" if cfgd else "无"))

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
