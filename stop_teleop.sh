#!/bin/bash
# 停止 F1-Rokae 遥操作相关进程（遥操作主程序 / 碰撞检测节点 / RoboticsService / 腰部节点）
# 主程序收到 SIGINT 后会先 /inex/halt 平滑制动再退出，所以先 INT 再等待，最后才 KILL。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STOP_WAIST_NODE="${STOP_WAIST_NODE:-1}"

echo "=== 停止 F1-Rokae teleop ==="
# 先 TERM（主程序装了 handler；某些 SDK 会吞掉 SIGINT），给足时间做 halt / stop_impedance_session
_wait_exit() { for _ in $(seq 1 "$1"); do pgrep -f "teleop_f1_hardware.py" >/dev/null || return 0; sleep 0.5; done; return 1; }
pkill -TERM -f "teleop_f1_hardware.py" 2>/dev/null
_wait_exit 20 || { pkill -INT -f "teleop_f1_hardware.py" 2>/dev/null; _wait_exit 10; } || echo "⚠️ 主程序未在 15 s 内退出，强制结束（可能未执行 halt/停阻抗会话）"
pkill -9 -f "teleop_f1_hardware.py" 2>/dev/null
pkill -f "realtime_joint_safety_monitor.py" 2>/dev/null
pkill -f "runService.sh" 2>/dev/null
pkill -f "RoboticsServiceProcess" 2>/dev/null
rm -f "${SCRIPT_DIR}/log/teleop.pid"

if [[ "${STOP_WAIST_NODE}" == "1" && -f "${SCRIPT_DIR}/EC_Line_Control/stop_waist_updown_ros2.sh" ]]; then
    bash "${SCRIPT_DIR}/EC_Line_Control/stop_waist_updown_ros2.sh" || true
fi

sleep 1
if ps aux | grep -E 'teleop_f1_hardware|realtime_joint_safety_monitor|runService|RoboticsServiceProcess' | grep -v grep; then
    echo "⚠️ 仍有残留进程，请再执行一次"
else
    echo "✅ 已全部停止"
fi
