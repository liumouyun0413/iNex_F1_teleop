#!/usr/bin/env python3
"""
F1 机器人腰部柔顺轨迹控制脚本（基于 Ruckig 算法）

使用 Ruckig (jerk-limited) 轨迹规划器，对腰部 5 个电机进行独立的速度、加速度、
jerk 限制，避免多电机同步运动时因加速度剧变导致的抖动问题。

常用测试范围5个腰部电机：
ruckig==0.19.4
python3 solve_F1_waist.py --motors "3,4,5,6,7" --ends="0,0,-2,-6,-2" 
python3 solve_F1_waist.py --motors "3,4,5,6,7" --ends="10,10,-22,-46,-22"


用法：
    # 单轴控制（自动获取当前角度）
    python3 solve_F1_waist.py --motor 3 --end 20 --duration 3

    # 单轴往复运动
    python3 solve_F1_waist.py --motor 5 --end -30 --duration 3 --roundtrip --cycles 3

    # 多轴控制
    python3 solve_F1_waist.py --motors "3,4,5" --ends="-10,5,-30" --duration 3

    # 全部腰部 (3-7)
    python3 solve_F1_waist.py --all-waist --ends="10,5,-30,-45,-20" --duration 3

    # 全部腰部往复运动
    python3 solve_F1_waist.py --all-waist --ends="10,5,-30,-45,-20" --duration 3 --roundtrip --cycles 2

    # 跳过确认
    python3 solve_F1_waist.py --motor 5 --end -30 --duration 3 --no-confirm

    # 自定义 Ruckig 周期
    python3 solve_F1_waist.py --all-waist --ends="10,5,-30,-45,-20" --ruckig-dt 0.005

依赖安装：
    pip install ruckig
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import time
import threading
import argparse
import math

# ============================================================
# 尝试导入 Ruckig
# ============================================================
try:
    from ruckig import InputParameter, OutputParameter, Ruckig, Result
    RUCKIG_AVAILABLE = True
except ImportError:
    RUCKIG_AVAILABLE = False
    print("[ERROR] ruckig 未安装！请执行: pip install ruckig")


# ============================================================
# 腰部关节限位定义 (ID 3-7)
# ============================================================
WAIST_LIMITS = {
    3: {'min': -130.0, 'max': 130.0},   # body_j1 (腰部Yaw)
    4: {'min': -10.0,  'max': 10.0},    # body_j2 (腰部Roll)
    5: {'min': -70.0,  'max': 0.0},     # body_j3 (腰部Pitch)
    6: {'min': -95.0,  'max': 0.0},     # body_j4 (屈膝)
    7: {'min': -40.0,  'max': 0.0},     # body_j5 (脚踝)
}

# ============================================================
# 电机名称映射
# ============================================================
MOTOR_NAMES = {
    3: "body_j1",
    4: "body_j2",
    5: "body_j3",
    6: "body_j4",
    7: "body_j5",
}

MOTOR_DISPLAY_NAMES = {
    3: "body_j1 (腰部Yaw)",
    4: "body_j2 (腰部Roll)",
    5: "body_j3 (腰部Pitch)",
    6: "body_j4 (屈膝)",
    7: "body_j5 (脚踝)",
}

JOINT_NAME_TO_ID = {v: k for k, v in MOTOR_NAMES.items()}

# ============================================================
# 腰部各电机的运动学限制（单位: °/s, °/s², °/s³）
# ============================================================
# 说明：
#   - 两种类型的电机具有不同的动力学特性，因此需要分别设置限制
#   - 类型 A（body_j1, body_j2）：大扭矩电机，响应较快
#   - 类型 B（body_j3, body_j4, body_j5）：较小电机，响应相对较慢
#   - 以下数值为建议初始值，请根据实际电机规格和测试结果调整
#   - 如果某类电机抖动明显，可适当降低其 max_velocity / max_acceleration / max_jerk

# 类型 A: 大扭矩腰部电机 (body_j1, body_j2)
WAIST_VELOCITY_LIMITS = {
    3: 150.0,    # body_j1: °/s
    4: 150.0,    # body_j2: °/s
    5: 150.0,    # body_j3: °/s
    6: 150.0,    # body_j4: °/s
    7: 150.0,    # body_j5: °/s
}

WAIST_ACCELERATION_LIMITS = {
    3: 200.0,    # body_j1: °/s²
    4: 200.0,    # body_j2: °/s²
    5: 200.0,    # body_j3: °/s²
    6: 200.0,    # body_j4: °/s²
    7: 200.0,    # body_j5: °/s²
}

WAIST_JERK_LIMITS = {
    3: 600.0,   # body_j1: °/s³
    4: 600.0,   # body_j2: °/s³
    5: 600.0,   # body_j3: °/s³
    6: 600.0,   # body_j4: °/s³
    7: 600.0,   # body_j5: °/s³
}


# ============================================================
# 工具函数
# ============================================================
def deg2rad(deg):
    """角度转弧度"""
    return deg * math.pi / 180.0


def rad2deg(rad):
    """弧度转角度"""
    return rad * 180.0 / math.pi


def check_limits(motor_ids, angles):
    """检查角度是否在限位范围内"""
    errors = []
    for motor_id, angle in zip(motor_ids, angles):
        if motor_id not in WAIST_LIMITS:
            errors.append(f"Motor {motor_id} 不在腰部关节范围内")
            continue
        limits = WAIST_LIMITS[motor_id]
        if angle < limits['min'] or angle > limits['max']:
            errors.append(
                f"Motor {motor_id} ({MOTOR_DISPLAY_NAMES.get(motor_id, 'unknown')}) "
                f"角度 {angle:.2f}° 超出范围 [{limits['min']:.2f}°, {limits['max']:.2f}°]"
            )
    if errors:
        return False, "\n".join(errors)
    return True, "OK"


# ============================================================
# Ruckig 轨迹规划器
# ============================================================
class RuckigTrajectoryGenerator:
    """
    基于 Ruckig 的腰部轨迹规划器

    核心优势：
    - 对每个电机独立限制 velocity / acceleration / jerk
    - 生成 C² 连续的轨迹（位置、速度、加速度均连续）
    - 时间最优：在约束范围内尽快到达目标
    - 避免加速度突变（jerk 有限），减少不同类型电机间的响应差异
    """

    def __init__(self, motor_ids, ruckig_dt=0.01):
        """
        Args:
            motor_ids: 要控制的电机 ID 列表，如 [3, 4, 5, 6, 7]
            ruckig_dt: Ruckig 插值周期（秒），默认 0.01s = 10ms
        """
        if not RUCKIG_AVAILABLE:
            raise RuntimeError("ruckig 未安装，请执行: pip install ruckig")

        self.motor_ids = list(motor_ids)
        self.dof = len(self.motor_ids)
        self.ruckig_dt = ruckig_dt

        # 创建 Ruckig 实例
        self.op = Ruckig(self.dof, ruckig_dt)

    def compute_trajectory(self, start_angles, end_angles,
                           start_vels=None, start_accels=None):
        """
        计算 Ruckig 轨迹

        Args:
            start_angles: 起始角度列表 (度)，顺序对应 motor_ids
            end_angles:   目标角度列表 (度)，顺序对应 motor_ids
            start_vels:   起始速度列表 (°/s)，默认全 0
            start_accels: 起始加速度列表 (°/s²)，默认全 0

        Returns:
            dict: {
                'positions': [[p1, p2, ...], [p1, p2, ...], ...],  # 每步各关节角度(度)
                'velocities': [...],   # 同上
                'accelerations': [...],# 同上
                'duration': float,     # 轨迹总时长(秒)
                'dt': float,           # 插值周期(秒)
            }
            失败返回 None
        """
        inp = InputParameter(self.dof)

        # ---- 起始状态（单位：度）----
        inp.current_position = list(start_angles)
        if start_vels is not None:
            inp.current_velocity = list(start_vels)
        else:
            inp.current_velocity = [0.0] * self.dof
        if start_accels is not None:
            inp.current_acceleration = list(start_accels)
        else:
            inp.current_acceleration = [0.0] * self.dof

        # ---- 目标状态（速度、加速度归零）----
        inp.target_position = list(end_angles)
        inp.target_velocity = [0.0] * self.dof
        inp.target_acceleration = [0.0] * self.dof

        # ---- 各电机的运动学限制（核心：每个电机独立限制，单位：度）----
        inp.max_velocity = [
            WAIST_VELOCITY_LIMITS[mid] for mid in self.motor_ids
        ]
        inp.max_acceleration = [
            WAIST_ACCELERATION_LIMITS[mid] for mid in self.motor_ids
        ]
        inp.max_jerk = [
            WAIST_JERK_LIMITS[mid] for mid in self.motor_ids
        ]

        # ---- 迭代计算轨迹 ----
        positions = []
        velocities = []
        accelerations = []
        out = OutputParameter(self.dof)

        max_iterations = 100000  # 安全上限
        for i in range(max_iterations):
            result = self.op.update(inp, out)

            # 记录当前步（单位：度）
            positions.append(list(out.new_position))
            velocities.append(list(out.new_velocity))
            accelerations.append(list(out.new_acceleration))

            if result == Result.Finished:
                # Result.Finished 时 out.new_position 已是目标位置，已在上方记录
                break

            # 将输出作为下一步输入
            out.pass_to_input(inp)
        else:
            print(f"[ERROR] Ruckig 计算超过 {max_iterations} 次迭代，可能无法到达目标")
            print("  请检查：1) 目标角度是否在限位内  2) 速度/加速度限制是否过小")
            return None

        duration = len(positions) * self.ruckig_dt
        return {
            'positions': positions,
            'velocities': velocities,
            'accelerations': accelerations,
            'duration': duration,
            'dt': self.ruckig_dt,
        }


# ============================================================
# ROS2 Node
# ============================================================
class MotionControllerNode(Node):
    def __init__(self):
        super().__init__('f1_waist_ruckig_controller')
        self.joint_states = {}
        self.joint_names = []
        self.state_received = False
        self.state_lock = threading.Lock()

        # 订阅关节状态
        self.subscription = self.create_subscription(
            JointState,
            '/hal/joint_states',
            self.joint_states_callback,
            10
        )

        # 发布控制命令
        self.publisher = self.create_publisher(
            JointState,
            '/motion_ctl/joint_ctl',
            10
        )

        self.get_logger().info("F1 Waist Ruckig Controller 已初始化，等待关节状态...")

    def joint_states_callback(self, msg):
        with self.state_lock:
            self.joint_names = msg.name
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self.joint_states[name] = msg.position[i]
            self.state_received = True

    def get_joint_angle(self, motor_id, timeout=2.0, default_zero=False):
        """获取指定电机的当前角度（度）
        
        Args:
            motor_id: 电机 ID
            timeout: 超时时间（秒）
            default_zero: 获取失败时是否返回 0°
        """
        joint_name = MOTOR_NAMES.get(motor_id)
        if joint_name is None:
            self.get_logger().error(f"未知电机 ID: {motor_id}")
            return 0.0 if default_zero else None

        start_time = time.time()
        while time.time() - start_time < timeout:
            with self.state_lock:
                if joint_name in self.joint_states:
                    return self.joint_states[joint_name]
            time.sleep(0.01)

        if default_zero:
            self.get_logger().warn(f"获取电机 {motor_id} ({joint_name}) 角度失败，使用默认值 0°")
            return 0.0
        self.get_logger().error(f"获取电机 {motor_id} ({joint_name}) 角度超时")
        return None

    def get_multiple_joint_angles(self, motor_ids, timeout=2.0, default_zero=False):
        """获取多个电机的当前角度（度）
        
        Args:
            motor_ids: 电机 ID 列表
            timeout: 超时时间（秒）
            default_zero: 获取失败时是否以 0° 填充
        """
        angles = {}
        remaining = set(motor_ids)

        start_time = time.time()
        while remaining and time.time() - start_time < timeout:
            with self.state_lock:
                for motor_id in list(remaining):
                    joint_name = MOTOR_NAMES.get(motor_id)
                    if joint_name and joint_name in self.joint_states:
                        angles[motor_id] = self.joint_states[joint_name]
                        remaining.remove(motor_id)
            time.sleep(0.01)

        for motor_id in remaining:
            if default_zero:
                self.get_logger().warn(f"获取电机 {motor_id} 当前角度失败，使用默认值 0°")
                angles[motor_id] = 0.0
            else:
                self.get_logger().warn(f"获取电机 {motor_id} 当前角度失败")
                angles[motor_id] = None

        return angles

    def send_command(self, motor_ids, angles):
        """
        发送控制命令

        Args:
            motor_ids: 电机 ID 列表
            angles: 角度列表（度）
        """
        if not isinstance(motor_ids, list):
            motor_ids = [motor_ids]
        if not isinstance(angles, list):
            angles = [angles]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []

        for motor_id, angle in zip(motor_ids, angles):
            joint_name = MOTOR_NAMES.get(motor_id)
            if joint_name is None:
                self.get_logger().error(f"未知电机 ID: {motor_id}")
                return False
            msg.name.append(joint_name)
            msg.position.append(float(angle))

        self.publisher.publish(msg)
        return True


# ============================================================
# 交互确认函数
# ============================================================
def confirm_motion(axes_info, duration, ruckig_dt, roundtrip=False, cycles=1):
    """打印运动信息并等待用户确认"""
    print("\n" + "=" * 70)
    print("            F1 腰部 Ruckig 轨迹控制 - 运动参数确认")
    print("=" * 70)

    mode_str = "往复运动" if roundtrip else "单次运动"
    if roundtrip:
        mode_str += f" (来回 {cycles} 次)"

    print(f"\n  [运动参数]")
    print(f"    运动模式:      {mode_str}")
    print(f"    轨迹规划:      Ruckig (jerk-limited, C² 连续)")
    print(f"    参考时长:      {duration:.2f} 秒")
    print(f"    Ruckig 周期:   {ruckig_dt * 1000:.1f} ms")
    print(f"    插值方式:      各电机独立速度/加速度/jerk 限制")

    print(f"\n  [电机列表]")
    print(f"    {'ID':<4} {'名称':<25} {'起始角度':>10} {'目标角度':>10} {'变化量':>10}")
    print(f"    {'-'*4} {'-'*25} {'-'*10} {'-'*10} {'-'*10}")

    for info in axes_info:
        motor_id = info['motor_id']
        name = MOTOR_DISPLAY_NAMES.get(motor_id, f"motor_{motor_id}")
        start = info['start']
        end = info['end']
        delta = end - start
        print(f"    {motor_id:<4} {name:<25} {start:>8.2f}°  {end:>8.2f}°  {delta:>+8.2f}°")

    print(f"\n  [各电机运动学限制]")
    print(f"    {'ID':<4} {'名称':<20} {'最大速度(°/s)':>14} {'最大加速度(°/s²)':>16} {'最大jerk(°/s³)':>16}")
    print(f"    {'-'*4} {'-'*20} {'-'*14} {'-'*16} {'-'*16}")

    shown_ids = set()
    for info in axes_info:
        mid = info['motor_id']
        if mid in shown_ids:
            continue
        shown_ids.add(mid)
        name = MOTOR_DISPLAY_NAMES.get(mid, f"motor_{mid}")
        print(f"    {mid:<4} {name:<20} "
              f"{WAIST_VELOCITY_LIMITS[mid]:>12.1f}  "
              f"{WAIST_ACCELERATION_LIMITS[mid]:>14.1f}  "
              f"{WAIST_JERK_LIMITS[mid]:>14.1f}")

    print("\n" + "=" * 70)
    print("[确认] 输入 'y' 开始运动，'n' 取消")
    print("=" * 70)

    while True:
        try:
            user_input = input("\n请确认: ").strip().lower()
            if user_input == 'y':
                print("\n[INFO] 开始运动...")
                return True
            elif user_input == 'n':
                print("\n[INFO] 用户取消运动")
                return False
            else:
                print("[提示] 请输入 'y' 确认 或 'n' 取消")
        except KeyboardInterrupt:
            print("\n[INFO] 用户取消运动")
            return False


# ============================================================
# 主控制类
# ============================================================
class WaistMotionController:
    """腰部运动控制器（基于 Ruckig）"""

    def __init__(self, node, ruckig_dt=0.01, use_default_zero=False):
        self.node = node
        self.ruckig_dt = ruckig_dt
        self.use_default_zero = use_default_zero

    def execute_trajectory(self, motor_ids, trajectory, label=""):
        """
        执行单段轨迹

        Args:
            motor_ids: 电机 ID 列表
            trajectory: Ruckig 计算出的轨迹 dict
            label: 日志标签
        """
        positions = trajectory['positions']
        dt = trajectory['dt']
        total_steps = len(positions)

        self.node.get_logger().info(
            f"{label}开始执行: {total_steps} 步, "
            f"时长 {trajectory['duration']:.2f}s, 周期 {dt*1000:.1f}ms"
        )

        start_time = time.perf_counter()
        for step, pos in enumerate(positions):
            self.node.send_command(motor_ids, pos)

            # 进度打印
            if step % max(1, total_steps // 10) == 0 or step == total_steps - 1:
                progress = step * 100 // (total_steps - 1)
                angles_str = ", ".join([f"{p:.2f}°" for p in pos])
                print(f"  [{label}] [{progress:3}%] step={step:5d}, angles=[{angles_str}]")

            # 精确等待到下一个周期
            next_time = start_time + (step + 1) * dt
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

    def run_motion(self, axes_configs, duration, roundtrip=False, cycles=1, no_confirm=False):
        """
        执行腰部运动

        Args:
            axes_configs: [{'motor_id': int, 'start': float, 'end': float}, ...]
            duration: 参考运动时长（秒），Ruckig 会根据限制自动计算实际最优时长
            roundtrip: 是否往复运动
            cycles: 往复次数
            no_confirm: 是否跳过确认
        """
        motor_ids = [c['motor_id'] for c in axes_configs]
        starts = [c['start'] for c in axes_configs]
        ends = [c['end'] for c in axes_configs]

        # 限位检查
        ok, msg = check_limits(motor_ids, ends)
        if not ok:
            self.node.get_logger().error(f"限位检查失败:\n{msg}")
            return False

        # 交互确认
        if not no_confirm:
            if not confirm_motion(axes_configs, duration, self.ruckig_dt, roundtrip, cycles):
                return False

        # ---- 创建 Ruckig 规划器 ----
        generator = RuckigTrajectoryGenerator(motor_ids, self.ruckig_dt)

        if roundtrip:
            return self._run_roundtrip(generator, axes_configs, cycles)
        else:
            return self._run_single(generator, axes_configs)

    def _run_single(self, generator, axes_configs):
        """单次运动"""
        motor_ids = [c['motor_id'] for c in axes_configs]
        starts = [c['start'] for c in axes_configs]
        ends = [c['end'] for c in axes_configs]

        self.node.get_logger().info(
            f"Ruckig 轨迹计算中: {len(motor_ids)} 个电机"
        )

        trajectory = generator.compute_trajectory(starts, ends)
        if trajectory is None:
            self.node.get_logger().error("Ruckig 轨迹计算失败！")
            return False

        self.node.get_logger().info(
            f"Ruckig 计算完成: 实际时长 {trajectory['duration']:.2f}s, "
            f"共 {len(trajectory['positions'])} 个插值点"
        )

        self.execute_trajectory(motor_ids, trajectory, label="")
        self.node.get_logger().info("腰部运动完成！")
        return True

    def _run_roundtrip(self, generator, axes_configs, cycles):
        """往复运动"""
        motor_ids = [c['motor_id'] for c in axes_configs]
        starts = [c['start'] for c in axes_configs]
        ends = [c['end'] for c in axes_configs]

        for cycle in range(cycles):
            # ---- 正向 ----
            self.node.get_logger().info(
                f"[Cycle {cycle+1}/{cycles}] 正向: {starts} -> {ends}"
            )
            traj_fwd = generator.compute_trajectory(starts, ends)
            if traj_fwd is None:
                self.node.get_logger().error(f"Cycle {cycle+1} 正向轨迹计算失败")
                return False

            self.node.get_logger().info(
                f"  正向时长: {traj_fwd['duration']:.2f}s, "
                f"{len(traj_fwd['positions'])} 步"
            )
            self.execute_trajectory(motor_ids, traj_fwd, label=f"正向{cycle+1}")

            # 正向结束，短暂停顿让系统稳定
            time.sleep(0.1)

            # ---- 反向 ----
            self.node.get_logger().info(
                f"[Cycle {cycle+1}/{cycles}] 反向: {ends} -> {starts}"
            )
            # 使用正向轨迹的终态（速度/加速度为0）作为反向起点
            traj_bwd = generator.compute_trajectory(ends, starts)
            if traj_bwd is None:
                self.node.get_logger().error(f"Cycle {cycle+1} 反向轨迹计算失败")
                return False

            self.node.get_logger().info(
                f"  反向时长: {traj_bwd['duration']:.2f}s, "
                f"{len(traj_bwd['positions'])} 步"
            )
            self.execute_trajectory(motor_ids, traj_bwd, label=f"反向{cycle+1}")

            # 反向结束，短暂停顿
            time.sleep(0.1)

        self.node.get_logger().info(f"腰部往复运动完成！({cycles} 个来回)")
        return True


# ============================================================
# 主函数
# ============================================================
def main():
    if not RUCKIG_AVAILABLE:
        print("\n[ERROR] ruckig 库未安装！")
        print("  请执行: pip install ruckig")
        print("  或:     pip install ruckig==0.11.0\n")
        sys.exit(1)

    parser = argparse.ArgumentParser(
        description='F1 机器人腰部 Ruckig 柔顺轨迹控制'
    )

    # 单轴模式
    parser.add_argument('--motor', '-m', type=int,
                        help='单个电机 ID (3-7)')
    parser.add_argument('--end', '-e', type=float,
                        help='目标角度（度）')
    parser.add_argument('--start', '-s', type=float, default=None,
                        help='起始角度（度，可选，默认自动获取）')

    # 多轴模式
    parser.add_argument('--motors', type=str,
                        help='电机 ID，逗号分隔 (如 "3,4,5")')
    parser.add_argument('--ends', type=str,
                        help='目标角度，逗号分隔 (如 "10,5,-30")')
    parser.add_argument('--starts', type=str, default=None,
                        help='起始角度，逗号分隔（可选，默认自动获取）')

    # 全部腰部
    parser.add_argument('--all-waist', action='store_true',
                        help='控制全部腰部关节 (3-7)')

    # 通用参数
    parser.add_argument('--duration', '-d', type=float, default=3.0,
                        help='参考运动时长/秒 (默认: 3.0, Ruckig 会自动计算最优时长)')
    parser.add_argument('--ruckig-dt', type=float, default=0.001,
                        help='Ruckig 插值周期/秒 (默认: 0.015 = 15ms, 与源文件一致)')
    parser.add_argument('--no-confirm', action='store_true',
                        help='跳过确认直接执行')
    parser.add_argument('--default-zero', action='store_true',
                        help='无法获取关节状态时默认以 0° 为起始角度（不弹窗询问）')
    parser.add_argument('--roundtrip', '-r', action='store_true',
                        help='往复运动')
    parser.add_argument('--cycles', type=int, default=1,
                        help='往复次数 (默认: 1)')

    args = parser.parse_args()

    # 初始化 ROS2
    rclpy.init(args=sys.argv)
    node = MotionControllerNode()

    # 等待关节状态
    use_default_zero = args.default_zero  # 命令行指定了 --default-zero
    if not node.state_received:
        node.get_logger().info("等待关节状态数据 (5秒)...")
        timeout = 5.0
        start_time = time.time()
        while not node.state_received and time.time() - start_time < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.state_received:
            if use_default_zero:
                node.get_logger().warn("未收到关节状态，使用 --default-zero，起始角度全部为 0°")
            else:
                print("\n" + "=" * 70)
                print("  [警告] 5秒内未收到关节状态数据！")
                print("=" * 70)
                print("  可能原因：")
                print("    1. 机器人未启动 / joint_states 话题未发布")
                print("    2. ROS2 网络不通 (RMW/Domain ID 不匹配)")
                print("    3. 话题名称不匹配 (期望: /hal/joint_states)")
                print()
                print("  选择操作：")
                print("    [y] 以 0° 作为所有腰部关节当前角度，继续执行")
                print("    [n] 退出程序")
                print("=" * 70)
                try:
                    user_input = input("\n请选择 (y/n): ").strip().lower()
                    if user_input == 'y':
                        use_default_zero = True
                        node.get_logger().warn("用户选择以 0° 为起始角度继续执行")
                    else:
                        print("已退出。")
                        rclpy.shutdown()
                        sys.exit(0)
                except KeyboardInterrupt:
                    print("\n已退出。")
                    rclpy.shutdown()
                    sys.exit(0)

    controller = WaistMotionController(node, ruckig_dt=args.ruckig_dt, use_default_zero=use_default_zero)
    success = False

    try:
        # ========== 单轴模式 ==========
        if args.motor is not None and args.end is not None:
            motor_id = args.motor
            if motor_id not in WAIST_LIMITS:
                node.get_logger().error(
                    f"电机 {motor_id} 不是腰部关节！腰部关节 ID 为 3-7"
                )
                sys.exit(1)

            # 获取起始角度
            if args.start is None:
                start_angle = node.get_joint_angle(motor_id, default_zero=use_default_zero)
                if start_angle is None:
                    node.get_logger().error(f"获取电机 {motor_id} 当前角度失败")
                    sys.exit(1)
                node.get_logger().info(f"电机 {motor_id} 当前角度: {start_angle:.2f}°")
            else:
                start_angle = args.start

            axes_configs = [{
                'motor_id': motor_id,
                'start': start_angle,
                'end': args.end
            }]

            success = controller.run_motion(
                axes_configs=axes_configs,
                duration=args.duration,
                roundtrip=args.roundtrip,
                cycles=args.cycles,
                no_confirm=args.no_confirm
            )

        # ========== 全部腰部 ==========
        elif args.all_waist:
            if args.ends is None:
                node.get_logger().error("--all-waist 模式需要 --ends 参数 (5个角度)")
                sys.exit(1)

            ends = [float(x.strip()) for x in args.ends.split(',')]
            if len(ends) != 5:
                node.get_logger().error(f"全部腰部需要 5 个目标角度，收到 {len(ends)} 个")
                sys.exit(1)

            motor_ids = [3, 4, 5, 6, 7]

            if args.starts is not None:
                starts = [float(x.strip()) for x in args.starts.split(',')]
                if len(starts) != 5:
                    node.get_logger().error(f"需要 5 个起始角度，收到 {len(starts)} 个")
                    sys.exit(1)
            else:
                node.get_logger().info("获取腰部全部关节当前角度...")
                angles = node.get_multiple_joint_angles(motor_ids, default_zero=use_default_zero)
                starts = [angles.get(mid) for mid in motor_ids]
                for i, mid in enumerate(motor_ids):
                    if starts[i] is None:
                        node.get_logger().error(f"获取电机 {mid} 当前角度失败")
                        sys.exit(1)
                node.get_logger().info(
                    f"当前角度: {[f'{s:.2f}°' for s in starts]}"
                )

            axes_configs = []
            for i in range(5):
                axes_configs.append({
                    'motor_id': motor_ids[i],
                    'start': starts[i],
                    'end': ends[i]
                })

            success = controller.run_motion(
                axes_configs=axes_configs,
                duration=args.duration,
                roundtrip=args.roundtrip,
                cycles=args.cycles,
                no_confirm=args.no_confirm
            )

        # ========== 多轴模式 ==========
        elif args.motors is not None and args.ends is not None:
            motor_ids = [int(x.strip()) for x in args.motors.split(',')]
            ends = [float(x.strip()) for x in args.ends.split(',')]

            if len(motor_ids) != len(ends):
                node.get_logger().error("motors 和 ends 数量不匹配")
                sys.exit(1)

            # 验证都是腰部关节
            for mid in motor_ids:
                if mid not in WAIST_LIMITS:
                    node.get_logger().error(
                        f"电机 {mid} 不是腰部关节！腰部关节 ID 为 3-7"
                    )
                    sys.exit(1)

            if args.starts is not None:
                starts = [float(x.strip()) for x in args.starts.split(',')]
                if len(starts) != len(motor_ids):
                    node.get_logger().error("starts 数量与 motors 不匹配")
                    sys.exit(1)
            else:
                node.get_logger().info(f"获取电机 {motor_ids} 当前角度...")
                angles = node.get_multiple_joint_angles(motor_ids, default_zero=use_default_zero)
                starts = [angles.get(mid) for mid in motor_ids]
                for i, mid in enumerate(motor_ids):
                    if starts[i] is None:
                        node.get_logger().error(f"获取电机 {mid} 当前角度失败")
                        sys.exit(1)

            axes_configs = []
            for i in range(len(motor_ids)):
                axes_configs.append({
                    'motor_id': motor_ids[i],
                    'start': starts[i],
                    'end': ends[i]
                })

            success = controller.run_motion(
                axes_configs=axes_configs,
                duration=args.duration,
                roundtrip=args.roundtrip,
                cycles=args.cycles,
                no_confirm=args.no_confirm
            )

        else:
            print_help()

    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        rclpy.shutdown()


def print_help():
    """打印帮助信息"""
    print("\n" + "=" * 70)
    print("  F1 腰部 Ruckig 柔顺轨迹控制 - 使用示例")
    print("=" * 70)

    print("\n  # ===== 单轴控制 ===== #")
    print("  python3 solve_F1_waist.py --motor 3 --end 20 --duration 3")
    print("  python3 solve_F1_waist.py --motor 5 --end -30 --duration 3 --no-confirm")
    print()
    print("  # ===== 单轴往复运动 ===== #")
    print("  python3 solve_F1_waist.py --motor 5 --end -30 --duration 3 --roundtrip --cycles 3")
    print()
    print("  # ===== 多轴控制 ===== #")
    print('  python3 solve_F1_waist.py --motors "3,4,5" --ends="10,5,-30" --duration 3')
    print('  python3 solve_F1_waist.py --motors "3,4" --starts "10,5" --ends "-10,-5" --duration 3')
    print()
    print("  # ===== 全部腰部 (3-7) ===== #")
    print('  python3 solve_F1_waist.py --all-waist --ends="10,5,-30,-45,-20" --duration 3')
    print('  python3 solve_F1_waist.py --all-waist --ends="10,5,-30,-45,-20" --duration 3 --roundtrip --cycles 2')
    print()
    print("  # ===== 自定义 Ruckig 周期 ===== #")
    print('  python3 solve_F1_waist.py --all-waist --ends="10,5,-30,-45,-20" --ruckig-dt 0.005')
    print()
    print("=" * 70)
    print("  腰部关节 ID 参考")
    print("=" * 70)
    print("""
  身体 (3-7):
    3: body_j1 (腰部Yaw)   |  范围: -130° ~ 130°  |  类型: A (大扭矩)
    4: body_j2 (腰部Roll)  |  范围:  -10° ~  10°  |  类型: A (大扭矩)
    5: body_j3 (腰部Pitch) |  范围:  -70° ~   0°  |  类型: B
    6: body_j4 (屈膝)      |  范围:  -95° ~   0°  |  类型: B
    7: body_j5 (脚踝)      |  范围:  -40° ~   0°  |  类型: B
    """)
    print("  运动学限制（可在代码中修改 WAIST_VELOCITY/ACCELERATION/JERK_LIMITS）:")
    print(f"    {'ID':<4} {'最大速度(°/s)':>14} {'最大加速度(°/s²)':>16} {'最大jerk(°/s³)':>16}")
    for mid in [3, 4, 5, 6, 7]:
        print(f"    {mid:<4} {WAIST_VELOCITY_LIMITS[mid]:>14.1f} "
              f"{WAIST_ACCELERATION_LIMITS[mid]:>16.1f} "
              f"{WAIST_JERK_LIMITS[mid]:>16.1f}")
    print("=" * 70)


if __name__ == "__main__":
    main()
