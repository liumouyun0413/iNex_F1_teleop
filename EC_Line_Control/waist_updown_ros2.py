#!/usr/bin/env python3
"""ROS 2 Cartesian up/down controller for the iNex (珞石) waist joints.

Subscribes:
    /inex/joint_states      sensor_msgs/msg/JointState   (positions in RADIANS)
    /control/waist/updown   std_msgs/msg/Bool

Uses the iNex driver interfaces (RobotPart 0 = body):
    /inex/stop_single_point_heartbeat
    /inex/power_change
    /inex/set_robot_mode
    /inex/set_motion_profile      (JOINT_POSITION for PTP)
    /inex/move_to                 (action, motion_type=JOINT, q in DEGREES)
    /inex/halt

`True` moves the L3 tip in the positive world-Y direction; `False` moves it in
the negative world-Y direction. Each command moves by `distance_m` relative to
the latest measured pose while preserving L3's current world orientation.

The planar waist kinematics (link lengths, angle offsets, IK, joint limits and
trajectory shaping) are unchanged from the previous 天机-arm version; only the
transport layer is swapped to the iNex service interface. Two transport details
differ from the old node and are handled here:

  * /inex/joint_states reports positions in radians, so feedback is converted
    to degrees before entering the (degree based) kinematics.  Joint names are
    body_joint_N (0.1.0 with mechanical_description) or body_N (1.0.0 fallback);
    both are accepted.
  * Motion runs as a JOINT PTP through the /inex/move_to action: the node
    solves the three waist joint angles for the end point and the controller
    plans the accel/decel and interpolates internally (no host-side streaming,
    so no ServoJ timing jitter).  All five body joints are sent in degrees;
    body_joint_4/5 are held at their measured value.
"""

import configparser
import math
import signal
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from inex_ros2_interface.action import MoveTo
from inex_ros2_interface.srv import (
    Halt,
    JointPublish,
    OperationState,
    PowerChange,
    RobotMode,
    SetMotionProfile,
    SetRobotMode,
    StopSinglePointHeartbeat,
)


# Link lengths in metres.
L1 = 0.352
L2 = 0.385
L3 = 1.000
DISTANCE_REDUCTION_STEP_M = 0.005

# Mathematical-angle to joint-command mapping.  The command interface uses
# degrees; /inex/joint_states feedback is converted from radians to degrees.
THETA1_OFFSET_DEG = 155.0
THETA2_OFFSET_DEG = -133.0
THETA3_OFFSET_DEG = 68.0

# iNex body joint names.  The planar waist is driven by the first three body
# joints; the remaining two are held still during a move.
COMMAND_JOINT_NAMES = ('body_joint_1', 'body_joint_2', 'body_joint_3')
HELD_JOINT_NAMES = ('body_joint_4', 'body_joint_5')
ALL_BODY_JOINT_NAMES = COMMAND_JOINT_NAMES + HELD_JOINT_NAMES

# iNex ROS2 v1.0 dropped inex_ros2_mechanical_description, so the driver falls
# back to generic names body_1..body_6 (6 = hardware buffer, the 6th axis is
# zero-padded on this 5-joint waist).  Map them onto the canonical names used
# throughout this file; the legacy yaml names pass through unchanged.
_BODY_NAME_ALIASES = {f'body_{i}': f'body_joint_{i}' for i in range(1, 6)}

# /inex/joint_states reports radians; the kinematics below work in degrees.
FEEDBACK_IN_RADIANS = True

# Joint limits in degrees, taken from this robot's authoritative soft limits
# via /inex/get_soft_limits (part=0 body):
#   body_joint_1 [-36, 0], body_joint_2 [-94, 0], body_joint_3 [-5, 88].
# The previous 天机-arm numbers (notably body_joint_3 [-40, 0]) do NOT match
# this robot: its body_joint_3 range is mostly positive, so capping it at 0
# rejected every trajectory at the current pose (+0.01 deg reads as over-limit).
#
# Start limits add a tolerance to accept measured feedback overshoot; end
# limits are the strict soft limits applied to all newly planned command
# points.  The motor numbers map directly as body_joint_1=m1,
# body_joint_2=m2, body_joint_3=m3, so trajectory tuples are ordered
# (m1, m2, m3) to line up with COMMAND_JOINT_NAMES.
#
# 2026-11-13 on ms-thor: the waist parked at the mechanical end reads
# body_joint_2 = +1.04 deg (soft limit 0), so a 1.0 deg tolerance rejected
# every command ("Rejected start check").  The end target is still checked
# against the strict limits, so a JOINT PTP from a slightly over-limit start
# can only move back inside the range; a wider start tolerance is safe.
START_TOLERANCE_DEG = 3.0

JOINT_LIMITS_E_DEG = {
    'body_joint_1': (-36.0, 0.0),
    'body_joint_2': (-94.0, 0.0),
    'body_joint_3': (-5.0, 88.0),
}

JOINT_LIMITS_S_DEG = {
    name: (lo - START_TOLERANCE_DEG, hi + START_TOLERANCE_DEG)
    for name, (lo, hi) in JOINT_LIMITS_E_DEG.items()
}

# iNex interface enums (see inex_ros2_interface/srv definitions).
BODY_PART = 0                 # RobotPart: 0 = body
# ServoGroup (hardware_types.h): 0=ALL, 1=left_arm, 2=right_arm, 3=body,
# 4=external.  The wiring differs from RobotPart (where body=0): the body servo
# group is 3.  This node drives only the waist, so power just the 5 body motors
# -- using ALL(0) would also energize both arms and the head.
SERVO_GROUP_BODY = 3
# 启动上电重试：stop 后立即 start 时，上一个节点 teardown 的 body 下电还在驱动内执行，此时 power_on 被拒（实测 code=-3，
# 2026-11-17 六次启动三次复现）；等几秒再试即可。
POWER_ON_ATTEMPTS = 4
POWER_RETRY_DELAY_S = 2.0
ROBOT_MODE_WHOLE_BODY = 3     # controller::RobotMode WholeBody
MOTION_PROFILE_JOINT_POSITION = 1
# /inex/move_to action motion_type: JOINT(0) interpolates in joint space (the
# controller plans the accel/decel); CARTESIAN(1) would need a 7-D pose.
MOTION_TYPE_JOINT = 0
# RobotSystemState.ERROR: power_on retries from ERROR back to READY (SDK state
# table); any rejected PTP (e.g. soft-limit) can leave the system here.
SYSTEM_STATE_ERROR = 3
# robot_manager_user_error.h codes seen in MoveTo results.
PTP_ERROR_HINTS = {
    40002: 'SoftLimit: start or target outside the driver soft limits '
           '(check /inex/get_soft_limits part=0 vs. measured body joints)',
    41020: 'MotionRejected: system not READY (ERROR / busy); cleared on next '
           'command via power_on retry',
}

CONFIG_PATH = Path(__file__).with_name('waist_updown_config.ini')


def read_motion_defaults() -> Tuple[float, float, float, float, float]:
    """Read distance, duration, publish-rate, idle-power-off and speed_scale."""
    config = configparser.ConfigParser()
    if not config.read(CONFIG_PATH):
        raise FileNotFoundError(f'Configuration file not found: {CONFIG_PATH}')
    try:
        distance_m = config.getfloat('motion', 'distance_m')
        # duration_s is unused in JOINT PTP mode; keep it optional so a
        # PTP-era config that dropped it still loads.
        duration_s = config.getfloat('motion', 'duration_s', fallback=1.0)
        publish_rate_hz = config.getfloat(
            'motion', 'publish_rate_hz', fallback=50.0
        )
        # Optional: idle auto-power-off timeout for the waist (0 disables).
        idle_power_off_s = config.getfloat(
            'motion', 'idle_power_off_s', fallback=300.0
        )
        # PTP speed as a fraction (0, 1] of the controller's max speed.
        speed_scale = config.getfloat(
            'motion', 'speed_scale', fallback=0.05
        )
    except (configparser.Error, ValueError) as error:
        raise ValueError(
            f'Invalid motion configuration in {CONFIG_PATH}: {error}'
        ) from error
    if distance_m <= 0.0:
        raise ValueError('motion.distance_m must be greater than zero')
    if publish_rate_hz <= 0.0:
        raise ValueError('motion.publish_rate_hz must be greater than zero')
    if idle_power_off_s < 0.0:
        raise ValueError('motion.idle_power_off_s must not be negative')
    if not 0.0 < speed_scale <= 1.0:
        raise ValueError('motion.speed_scale must be in (0, 1]')
    return distance_m, duration_s, publish_rate_hz, idle_power_off_s, speed_scale


def motor_to_math(m1_deg: float, m2_deg: float) -> Tuple[float, float]:
    """Convert former m1/m2 command angles (degrees) to math angles (rad)."""
    theta1 = math.radians(m1_deg + THETA1_OFFSET_DEG)
    theta2 = math.radians(-m2_deg + THETA2_OFFSET_DEG)
    return theta1, theta2


def math_to_motor(theta1: float, theta2: float) -> Tuple[float, float]:
    """Convert math angles (rad) to former m1/m2 command angles (degrees)."""
    m1 = math.degrees(theta1) - THETA1_OFFSET_DEG
    m2 = -(math.degrees(theta2) - THETA2_OFFSET_DEG)
    return m1, m2


def motor3_to_math(m3_deg: float) -> float:
    """Convert former m3 command angle (degrees) to its math angle (rad).

    The m3 motor (body_joint_1) spins in the opposite sense to the math angle,
    so its command value is negated -- mirroring how m2 is handled in
    motor_to_math / math_to_motor.
    """
    return math.radians(-m3_deg + THETA3_OFFSET_DEG)


def math_to_motor3(theta3: float) -> float:
    """Convert a third-joint math angle (rad) to former m3 degrees.

    Exact inverse of motor3_to_math, including the m3 direction reversal.
    """
    return -(math.degrees(theta3) - THETA3_OFFSET_DEG)


def forward_kinematics(
    theta1: float, theta2: float, tool_orientation: float
) -> Tuple[float, float]:
    """Return the L3-tip position in metres for a fixed world orientation."""
    return (
        L1 * math.cos(theta1)
        + L2 * math.cos(theta1 + theta2)
        + L3 * math.cos(tool_orientation),
        L1 * math.sin(theta1)
        + L2 * math.sin(theta1 + theta2)
        + L3 * math.sin(tool_orientation),
    )


def inverse_kinematics(
    x: float, y: float, elbow_up: bool
) -> Optional[Tuple[float, float]]:
    """Return an IK solution, or None if the target is outside the workspace."""
    radius_sq = x * x + y * y
    radius = math.sqrt(radius_sq)
    if radius > L1 + L2 or radius < abs(L1 - L2):
        return None

    cos_theta2 = (radius_sq - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))
    theta2 = math.acos(cos_theta2)
    if not elbow_up:
        theta2 = -theta2
    theta1 = math.atan2(y, x) - math.atan2(
        L2 * math.sin(theta2), L1 + L2 * math.cos(theta2)
    )
    return theta1, theta2


class WaistUpDownController(Node):
    """Convert Boolean up/down commands into limit-checked waist trajectories."""

    def __init__(self) -> None:
        super().__init__('waist_updown_controller')

        (default_distance_m, default_duration_s, default_rate_hz,
         default_idle_off_s, default_speed_scale) = read_motion_defaults()
        self.declare_parameter('distance_m', default_distance_m)
        self.declare_parameter('duration_s', default_duration_s)
        # ServoJ stream rate.  50 Hz matches the proven iNex example; the
        # joint_publish round-trip cannot reliably sustain very high rates.
        self.declare_parameter('publish_rate_hz', default_rate_hz)
        # The measured robot waist posture uses the negative-theta2 branch.
        self.declare_parameter('elbow_up', False)
        self.declare_parameter('service_timeout_s', 10.0)
        self.declare_parameter('power_group', SERVO_GROUP_BODY)
        self.declare_parameter('robot_mode', ROBOT_MODE_WHOLE_BODY)
        self.declare_parameter('power_on_at_startup', True)
        # Waist idle auto-power-off: each updown command restarts this
        # countdown; when it lapses with no new command the body group is
        # powered off.  Set to 0 to disable.
        self.declare_parameter('idle_power_off_s', default_idle_off_s)
        # PTP speed fraction in (0, 1]; replaces duration_s/publish_rate_hz,
        # which the ServoJ streamer used but JOINT PTP does not.
        self.declare_parameter('speed_scale', default_speed_scale)
        # Generous ceiling for one PTP move (controller-planned, can be slow).
        self.declare_parameter('action_timeout_s', 60.0)

        self.distance_m = float(self.get_parameter('distance_m').value)
        self.duration_s = float(self.get_parameter('duration_s').value)
        self.publish_rate_hz = float(
            self.get_parameter('publish_rate_hz').value
        )
        self.elbow_up = bool(self.get_parameter('elbow_up').value)
        self.service_timeout_s = float(
            self.get_parameter('service_timeout_s').value
        )
        self.power_group = int(self.get_parameter('power_group').value)
        self.robot_mode = int(self.get_parameter('robot_mode').value)
        self.power_on_at_startup = bool(
            self.get_parameter('power_on_at_startup').value
        )
        self.idle_power_off_s = float(
            self.get_parameter('idle_power_off_s').value
        )
        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.action_timeout_s = float(
            self.get_parameter('action_timeout_s').value
        )
        if self.distance_m <= 0.0:
            raise ValueError('distance_m must be greater than zero')
        if not 0.0 < self.speed_scale <= 1.0:
            raise ValueError('speed_scale must be in (0, 1]')

        self.joint_positions: Dict[str, float] = {}
        self._moving = False
        self._move_lock = threading.Lock()
        self._ready = False
        self._powered = False
        # ServoJ worker lifecycle: _stream_stop signals the streaming loop to
        # exit; _worker holds the thread so teardown can join it before halting.
        self._stream_stop = threading.Event()
        self._worker: Optional[threading.Thread] = None
        # Idle auto-power-off watchdog: _idle_deadline is the monotonic time at
        # which the waist powers off if no new command arrived; every command
        # pushes it out.  _power_lock serialises the main-thread power-off
        # against the worker-thread re-power so the two never overlap.
        self._idle_deadline: Optional[float] = None
        self._power_lock = threading.Lock()

        # iNex driver service clients (RobotPart 0 = body).
        self.cli_heartbeat = self.create_client(
            StopSinglePointHeartbeat, '/inex/stop_single_point_heartbeat'
        )
        self.cli_power = self.create_client(PowerChange, '/inex/power_change')
        self.cli_mode = self.create_client(SetRobotMode, '/inex/set_robot_mode')
        # Getter for the current global session mode, so _apply_motion_mode can
        # skip the (BUSY-prone) set_robot_mode write when it already matches --
        # an idle re-power landing while the arms teleoperate must not fail on a
        # redundant no-op mode write.
        self.cli_robot_mode = self.create_client(RobotMode, '/inex/robot_mode')
        self.cli_profile = self.create_client(
            SetMotionProfile, '/inex/set_motion_profile'
        )
        self.cli_publish = self.create_client(
            JointPublish, '/inex/joint_publish'
        )
        self.cli_halt = self.create_client(Halt, '/inex/halt')
        self.cli_op_state = self.create_client(
            OperationState, '/inex/operation_state'
        )
        # JOINT PTP action client: the controller plans and executes the move.
        self.act_move_to = ActionClient(self, MoveTo, '/inex/move_to')
        self._goal_handle = None

        self.create_subscription(
            JointState, '/inex/joint_states', self.joint_state_callback, 10
        )
        self.create_subscription(
            Bool, '/control/waist/updown', self.updown_callback, 10
        )

    # ------------------------------------------------------------------ #
    # Blocking service helper.  Runs in the caller's thread (main thread
    # during setup/teardown, worker thread during a move) while the node's
    # executor spins elsewhere and delivers the response.
    # ------------------------------------------------------------------ #
    def _call_service(self, client, request, wait: bool = False):
        if wait and not client.wait_for_service(self.service_timeout_s):
            self.get_logger().error(
                '服务 %s 不可用' % client.service_name
            )
            return None
        future = client.call_async(request)
        done = threading.Event()
        holder: Dict[str, object] = {}

        def _on_done(fut):
            try:
                holder['res'] = fut.result()
            except Exception as error:  # pragma: no cover - defensive
                holder['err'] = error
            finally:
                done.set()

        future.add_done_callback(_on_done)
        if not done.wait(self.service_timeout_s):
            self.get_logger().error(
                '服务 %s 请求超时' % client.service_name
            )
            future.cancel()
            return None
        if 'err' in holder:
            self.get_logger().error(
                '服务 %s 调用异常: %s' % (client.service_name, holder['err'])
            )
            return None
        return holder.get('res')

    # ------------------------------------------------------------------ #
    # Lifecycle: power on + mode/profile setup, and safe teardown.
    # ------------------------------------------------------------------ #
    def _apply_motion_mode(self) -> bool:
        """Ensure WholeBody mode + JointPosition profile on the body (PTP).

        set_robot_mode is a GLOBAL write the controller rejects with BUSY while
        any partition is RUNNING (e.g. the arms teleoperating).  The session
        mode is normally already WholeBody(3), so query it first and skip the
        write when it already matches -- otherwise an idle re-power that lands
        while the arms are moving would fail on a redundant no-op set_robot_mode.
        If the query fails we fall back to writing (never assume the mode).
        set_motion_profile is bound per partition, so with the body freshly
        powered and stationary it is not blocked by the arms.
        """
        need_set_mode = True
        cur = self._call_service(
            self.cli_robot_mode, RobotMode.Request(), wait=True
        )
        if cur is not None and cur.code == 0 and cur.robot_mode == self.robot_mode:
            need_set_mode = False
            self.get_logger().info(
                '机器人模式已=%d（含躯干），跳过 set_robot_mode' % cur.robot_mode
            )
        if need_set_mode:
            mode_req = SetRobotMode.Request()
            mode_req.robot_mode = self.robot_mode
            res = self._call_service(self.cli_mode, mode_req, wait=True)
            if res is None or res.code != 0:
                # A non-zero code here is commonly BUSY: another partition
                # (e.g. the arms) is RUNNING, so the global mode cannot change
                # right now -- retry once the arms are idle.
                self.get_logger().error(
                    '设置全身模式失败 code=%s（双臂运动中会 BUSY，待其空闲重试）'
                    % (res.code if res else 'NA')
                )
                return False
            self.get_logger().info('机器人模式=全身(%d)' % self.robot_mode)

        profile_req = SetMotionProfile.Request()
        profile_req.profile = MOTION_PROFILE_JOINT_POSITION
        profile_req.part = BODY_PART
        res = self._call_service(self.cli_profile, profile_req, wait=True)
        if res is None or res.code != 0:
            self.get_logger().error(
                '设置躯干关节位置模式失败 code=%s' % (res.code if res else 'NA')
            )
            return False
        self.get_logger().info('躯干运动模式=关节位置')
        # JOINT PTP (/inex/move_to) only needs WholeBody mode + the
        # JOINT_POSITION profile; the ServoJ servo mode is not required (and
        # the PTP example does not set it).
        return True

    def _arm_motion(self) -> bool:
        """Power the body group on and (re)apply the PTP motion mode/profile.

        Used at startup and to re-energise the waist after an idle
        auto-power-off.  Runs in the main or worker thread (never the executor
        thread), because _call_service blocks waiting for the response.
        """
        if not self._set_power(True):
            return False
        return self._apply_motion_mode()

    def setup(self) -> bool:
        """Prepare the robot for JOINT PTP waist moves. Returns success."""
        hb = StopSinglePointHeartbeat.Request()
        res = self._call_service(self.cli_heartbeat, hb, wait=True)
        if res is None or res.code != 0:
            self.get_logger().error(
                '关闭单点运动心跳失败 code=%s' % (res.code if res else 'NA')
            )
            return False
        self.get_logger().info('已关闭单点运动心跳')

        if self.power_on_at_startup and not self._set_power(True, attempts=POWER_ON_ATTEMPTS):
            return False
        if not self._apply_motion_mode():
            return False

        self._ready = True
        if self._powered:
            # Arm the idle watchdog: with no command the waist powers itself
            # off after idle_power_off_s instead of staying energized forever.
            self._reset_idle_timer()
        self.get_logger().info(
            'Ready: True moves +Y and False moves -Y by %.3f m '
            '(JOINT PTP, speed_scale=%.3f).'
            % (self.distance_m, self.speed_scale)
        )
        if self.idle_power_off_s > 0.0:
            self.get_logger().info(
                '腰部空闲自动下电已启用：无新指令 %.1f 分钟后给 body 组下电。'
                % (self.idle_power_off_s / 60.0)
            )
        return True

    def teardown(self) -> None:
        """Stop any active PTP move, then smooth-brake and power the robot down."""
        self._ready = False
        self._idle_deadline = None  # disarm the idle auto-power-off watchdog
        # Signal the worker to stop, cancel any in-flight PTP goal so its result
        # wait unblocks, then join it BEFORE halting so a move cannot still be
        # running when we brake/power down.  The worker's waits are bounded, so
        # join with a bounded timeout while the executor is still spinning.
        self._stream_stop.set()
        goal_handle = self._goal_handle
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception:  # pragma: no cover - defensive
                pass
        worker = self._worker
        if (worker is not None and worker.is_alive()
                and worker is not threading.current_thread()):
            worker.join(timeout=self.service_timeout_s + 2.0)
            if worker.is_alive():
                self.get_logger().error(
                    '推流线程未在 %.1fs 内退出，继续停机流程。'
                    % (self.service_timeout_s + 2.0)
                )
        halt_req = Halt.Request()
        halt_req.part = BODY_PART
        res = self._call_service(self.cli_halt, halt_req)
        if res is None or res.code != 0:
            self.get_logger().error(
                '躯干平滑制动失败 code=%s' % (res.code if res else 'NA')
            )
        else:
            self.get_logger().info('躯干已平滑制动')
        if self._powered:
            self._set_power(False)

    def _recover_from_error(self) -> bool:
        """If the driver is in ERROR, retry power_on on the body group to clear it."""
        res = self._call_service(
            self.cli_op_state, OperationState.Request(), wait=True
        )
        if res is None or res.code != 0 or res.state != SYSTEM_STATE_ERROR:
            return True
        self.get_logger().warning(
            '驱动处于 ERROR 状态（上次运动被拒），用 power_on 重试清错...'
        )
        if not self._set_power(True):
            return False
        res = self._call_service(
            self.cli_op_state, OperationState.Request(), wait=True
        )
        ok = res is not None and res.code == 0 and res.state != SYSTEM_STATE_ERROR
        self.get_logger().info(
            'ERROR %s (state=%s)' % ('已清除' if ok else '未清除',
                                   getattr(res, 'state', 'NA'))
        )
        return ok

    def _set_power(self, power_on: bool, attempts: int = 1) -> bool:
        req = PowerChange.Request()
        req.power_on = power_on
        req.group = self.power_group
        action = '上电' if power_on else '下电'
        target = '整机' if self.power_group == 0 else '躯干(body)'
        for i in range(max(1, attempts)):
            res = self._call_service(self.cli_power, req, wait=True)
            if res is not None and res.code == 0:
                self._powered = power_on
                self.get_logger().info('%s%s返回 code=0' % (target, action))
                return True
            self.get_logger().error(
                '%s%s失败 code=%s%s' % (target, action, res.code if res else 'NA',
                                      '，%.1fs 后重试 (%d/%d)' % (POWER_RETRY_DELAY_S, i + 1, attempts)
                                      if i + 1 < attempts else '')
            )
            if i + 1 < attempts:
                time.sleep(POWER_RETRY_DELAY_S)
        return False

    # ------------------------------------------------------------------ #
    # Idle auto-power-off watchdog.  The waist motors must not stay energized
    # for long, so each updown command restarts a countdown; when it lapses the
    # body group is powered off, and the next command re-powers it on demand.
    # ------------------------------------------------------------------ #
    def _reset_idle_timer(self) -> None:
        """(Re)start the idle auto-power-off countdown from now."""
        if self.idle_power_off_s > 0.0:
            self._idle_deadline = time.monotonic() + self.idle_power_off_s

    def check_idle_power_off(self) -> None:
        """Power the waist off after idle_power_off_s with no new command.

        Polled from the main loop (never the executor thread, since _set_power
        blocks on a service response).  Holds _power_lock across the power-off
        so it cannot overlap a worker-thread re-power, and skips while a move
        is streaming.
        """
        if self.idle_power_off_s <= 0.0:
            return
        deadline = self._idle_deadline
        if deadline is None or time.monotonic() < deadline:
            return
        with self._power_lock:
            # Re-check under the lock: a command may have reset the timer or
            # started a move since the quick, lock-free check above.
            deadline = self._idle_deadline
            if (deadline is None or time.monotonic() < deadline
                    or self._moving or not self._powered):
                return
            self._idle_deadline = None
            self.get_logger().info(
                '腰部空闲已满 %.1f 分钟且无新指令，自动给 body 组下电。'
                % (self.idle_power_off_s / 60.0)
            )
            self._set_power(False)

    # ------------------------------------------------------------------ #
    # Callbacks.
    # ------------------------------------------------------------------ #
    def joint_state_callback(self, message: JointState) -> None:
        """Cache the latest body joint state, in degrees, by canonical name."""
        for name, position in zip(message.name, message.position):
            name = _BODY_NAME_ALIASES.get(name, name)
            if name in ALL_BODY_JOINT_NAMES:
                self.joint_positions[name] = (
                    math.degrees(position) if FEEDBACK_IN_RADIANS else position
                )

    def updown_callback(self, message: Bool) -> None:
        """Start a move only when no previously commanded move is active."""
        if not self._ready:
            self.get_logger().error(
                'Robot not ready (setup incomplete); ignoring command.'
            )
            return
        # Any new command restarts the idle auto-power-off countdown.
        self._reset_idle_timer()
        with self._move_lock:
            if self._moving:
                direction_name = 'up' if message.data else 'down'
                self.get_logger().info(
                    '%s command ignored because a Cartesian waist move is '
                    'active.' % direction_name
                )
                return
            self._moving = True
        if not self.start_move(message.data):
            with self._move_lock:
                self._moving = False

    @property
    def is_moving(self) -> bool:
        """Whether a trajectory is currently being streamed."""
        return self._moving

    # ------------------------------------------------------------------ #
    # Planning (kinematics unchanged from the previous version).
    # ------------------------------------------------------------------ #
    def start_move(self, move_up: bool) -> bool:
        """Solve one relative vertical move and run it as a JOINT PTP."""
        # Require ALL five body joints (not just the three waist joints): the
        # two held joints are commanded too, so a missing body_joint_4/5 must
        # never silently default to 0.0 deg and cause a large jump.
        required = set(ALL_BODY_JOINT_NAMES)
        if not required.issubset(self.joint_positions):
            self.get_logger().error(
                'No complete body state yet (need body_joint_1..5); '
                'waiting for /inex/joint_states.'
            )
            return False

        start_positions = tuple(
            self.joint_positions[name] for name in COMMAND_JOINT_NAMES
        )
        if not self.check_joint_limits(
                start_positions, JOINT_LIMITS_S_DEG, 'start'):
            return False

        # The state/order mapping is direct: body_joint_1=m1, body_joint_2=m2,
        # body_joint_3=m3 (matching COMMAND_JOINT_NAMES).
        m1 = self.joint_positions['body_joint_1']
        m2 = self.joint_positions['body_joint_2']
        m3 = self.joint_positions['body_joint_3']
        theta1, theta2 = motor_to_math(m1, m2)
        theta3 = motor3_to_math(m3)
        tool_orientation = theta1 + theta2 + theta3
        x_start, y_start = forward_kinematics(
            theta1, theta2, tool_orientation
        )
        direction = 1.0 if move_up else -1.0
        requested_distance_m = self.distance_m
        used_distance_m = requested_distance_m
        end_target: Optional[Tuple[float, float, float]] = None

        # JOINT PTP interpolates each joint linearly from its current value to
        # the target.  The start was already accepted against the tolerant S
        # limits above, so if the end target is within the strict E limits the
        # whole interpolated joint path stays within them too.  Only the end
        # needs solving/validating; reduce the distance until one is compliant.
        while used_distance_m > 0.0:
            end_target = self._solve_end_target(
                x_start, y_start, direction, used_distance_m, tool_orientation
            )
            if end_target is not None:
                break
            used_distance_m = max(
                0.0, used_distance_m - DISTANCE_REDUCTION_STEP_M
            )

        if end_target is None:
            self.get_logger().error(
                'Rejected %s move: no limit-compliant target remains '
                'after reducing %.3f m in %.3f m steps.'
                % (('up' if move_up else 'down'), requested_distance_m,
                   DISTANCE_REDUCTION_STEP_M)
            )
            return False

        # Hold the two non-waist body joints at their current measured value.
        # The start precondition guarantees all five body joints were received,
        # so index directly -- never default a held joint to 0.0 deg.
        held = tuple(
            self.joint_positions[name] for name in HELD_JOINT_NAMES
        )

        direction_name = 'up' if move_up else 'down'
        y_end = y_start + direction * used_distance_m
        distance_note = ''
        if used_distance_m != requested_distance_m:
            distance_note = ' (reduced from %.3f m for joint limits)' % (
                requested_distance_m
            )
        self.get_logger().info(
            'Starting %s move by %.3f m%s: (%.4f, %.4f) -> (%.4f, %.4f) m.'
            % (direction_name, used_distance_m, distance_note,
               x_start, y_start, x_start, y_end)
        )

        self._stream_stop.clear()
        worker = threading.Thread(
            target=self._ptp_move,
            args=(end_target, held, direction_name),
            daemon=True,
        )
        self._worker = worker
        worker.start()
        return True

    def _solve_end_target(
        self,
        x_start: float,
        y_start: float,
        direction: float,
        distance_m: float,
        tool_orientation: float,
    ) -> Optional[Tuple[float, float, float]]:
        """Solve the (m1, m2, m3) end target for a vertical move, or None.

        Only the end point is solved and validated against the strict E limits:
        JOINT PTP interpolates each joint linearly from its current value to
        this target, and the start was already accepted by the tolerant S-limit
        check in start_move, so the interpolated path stays within limits as
        long as the end does.  Returns None when the end is unreachable or out
        of limits, so the caller reduces the distance and retries.
        """
        x_end = x_start
        y_end = y_start + direction * distance_m
        wrist_x_end = x_end - L3 * math.cos(tool_orientation)
        wrist_y_end = y_end - L3 * math.sin(tool_orientation)
        end_ik = inverse_kinematics(
            wrist_x_end, wrist_y_end, self.elbow_up
        )
        if end_ik is None:
            return None

        end_theta1, end_theta2 = end_ik
        end_m1, end_m2 = math_to_motor(end_theta1, end_theta2)
        end_m3 = math_to_motor3(
            tool_orientation - end_theta1 - end_theta2
        )
        if not self.check_joint_limits(
                (end_m1, end_m2, end_m3), JOINT_LIMITS_E_DEG,
                'end', log_error=False):
            return None
        return (end_m1, end_m2, end_m3)

    def check_joint_limits(
        self,
        positions: Tuple[float, float, float],
        limits: Dict[str, Tuple[float, float]],
        check_name: str,
        log_error: bool = True,
    ) -> bool:
        """Validate a joint tuple against the supplied start or end limits."""
        for name, position in zip(COMMAND_JOINT_NAMES, positions):
            lower, upper = limits[name]
            if not lower <= position <= upper:
                if log_error:
                    self.get_logger().error(
                        'Rejected %s check: %s=%.2f deg is outside '
                        '[%.2f, %.2f] deg.'
                        % (check_name, name, position, lower, upper)
                    )
                return False
        return True

    # ------------------------------------------------------------------ #
    # JOINT PTP execution.
    # ------------------------------------------------------------------ #
    def _ptp_move(
        self,
        end_target: Tuple[float, float, float],
        held: Tuple[float, float],
        direction_name: str,
    ) -> None:
        """Run one JOINT PTP move via the /inex/move_to action (worker thread).

        The controller plans the accel/decel and interpolates internally, so
        there is no host-side frame streaming (and no ServoJ timing jitter).
        The blocking waits here run in the worker thread while the executor
        spins in spin_thread and completes the action futures.
        """
        try:
            # The waist may have been powered off by the idle watchdog.  This
            # runs in the worker thread, so blocking on services here is safe
            # (never do this from the executor thread).  Re-energise and
            # re-apply WholeBody + JOINT_POSITION before moving.
            with self._power_lock:
                if not self._powered:
                    self.get_logger().info(
                        '腰部处于下电状态，收到指令重新上电并复位运动模式...'
                    )
                    if not self._arm_motion():
                        self.get_logger().error(
                            '%s move aborted: re-power/mode setup failed.'
                            % direction_name
                        )
                        return
                if not self._recover_from_error():
                    self.get_logger().error(
                        '%s move aborted: driver stuck in ERROR.' % direction_name
                    )
                    return
            if not self.act_move_to.wait_for_server(self.service_timeout_s):
                self.get_logger().error(
                    '%s move aborted: /inex/move_to action server unavailable.'
                    % direction_name
                )
                return
            self._reset_idle_timer()

            goal = MoveTo.Goal()
            goal.part = BODY_PART
            goal.motion_type = MOTION_TYPE_JOINT
            # joint_body order: body_joint_1..5 == (m1, m2, m3, held4, held5).
            goal.joint_body = [
                float(end_target[0]),
                float(end_target[1]),
                float(end_target[2]),
                float(held[0]),
                float(held[1]),
            ]
            goal.speed_scale = self.speed_scale

            # Send the goal; the executor (spin_thread) completes the future.
            send_future = self.act_move_to.send_goal_async(goal)
            send_done = threading.Event()
            send_future.add_done_callback(lambda fut: send_done.set())
            if not send_done.wait(self.service_timeout_s):
                self.get_logger().error(
                    '%s move aborted: send_goal timeout.' % direction_name
                )
                return
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error(
                    '%s move aborted: goal rejected by /inex/move_to.'
                    % direction_name
                )
                return
            self._goal_handle = goal_handle

            # Wait for the result.  A stop request (teardown) cancels the goal
            # so this wait unblocks; the bounded loop also enforces a timeout.
            result_future = goal_handle.get_result_async()
            result_done = threading.Event()
            result_future.add_done_callback(lambda fut: result_done.set())
            deadline = time.monotonic() + self.action_timeout_s
            while not result_done.wait(0.2):
                if self._stream_stop.is_set():
                    goal_handle.cancel_goal_async()
                elif time.monotonic() > deadline:
                    self.get_logger().error(
                        '%s move aborted: PTP result timeout after %.1fs.'
                        % (direction_name, self.action_timeout_s)
                    )
                    goal_handle.cancel_goal_async()
                    break

            if self._stream_stop.is_set():
                self.get_logger().info(
                    '%s move aborted: stop requested (shutdown).'
                    % direction_name
                )
                return
            if not result_done.is_set():
                return  # timed out above; already logged
            wrapped = result_future.result()
            res = wrapped.result if wrapped is not None else None
            if res is not None and res.success and res.code == 0:
                self.get_logger().info(
                    'Cartesian waist %s move completed (JOINT PTP).'
                    % direction_name
                )
            else:
                code = getattr(res, 'code', None)
                self.get_logger().error(
                    '%s move aborted: PTP result success=%s code=%s message=%s. %s'
                    % (direction_name,
                       getattr(res, 'success', None),
                       code,
                       getattr(res, 'message', ''),
                       PTP_ERROR_HINTS.get(code, ''))
                )
        finally:
            self._goal_handle = None
            with self._move_lock:
                self._moving = False


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = WaistUpDownController()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Graceful shutdown on SIGINT/SIGTERM so teardown (halt + power off) runs.
    stop_event = threading.Event()

    def _request_stop(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    try:
        if not node.setup():
            node.get_logger().error(
                '初始化失败：节点继续运行但不会执行运动，请检查驱动后重启。'
            )
        while rclpy.ok() and not stop_event.is_set():
            node.check_idle_power_off()
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('正在安全停机...')
        node.teardown()
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
