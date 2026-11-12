#!/bin/bash
# 停止 F1-Rokae 遥操作相关进程（遥操作主程序 / RoboticsService / 腰部节点）
# 主程序收到 SIGINT 后会先 /inex/halt 平滑制动再退出，所以先 INT 再等待，最后才 KILL。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STOP_WAIST_NODE="${STOP_WAIST_NODE:-1}"

echo "=== 停止 F1-Rokae teleop ==="
pkill -INT -f "teleop_f1_hardware.py" 2>/dev/null
for _ in 1 2 3 4 5 6; do
    pgrep -f "teleop_f1_hardware.py" >/dev/null || break
    sleep 0.5
done
pkill -9 -f "teleop_f1_hardware.py" 2>/dev/null
pkill -f "runService.sh" 2>/dev/null
pkill -f "RoboticsServiceProcess" 2>/dev/null
rm -f "${SCRIPT_DIR}/log/teleop.pid"

if [[ "${STOP_WAIST_NODE}" == "1" && -f "${SCRIPT_DIR}/EC_Line_Control/stop_waist_updown_ros2.sh" ]]; then
    bash "${SCRIPT_DIR}/EC_Line_Control/stop_waist_updown_ros2.sh" || true
fi

sleep 1
if ps aux | grep -E 'teleop_f1_hardware|runService|RoboticsServiceProcess' | grep -v grep; then
    echo "⚠️ 仍有残留进程，请再执行一次"
else
    echo "✅ 已全部停止"
fi
