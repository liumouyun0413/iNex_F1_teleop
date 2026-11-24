#!/bin/bash
# 启动/重启碰撞检测节点 realtime_joint_safety_monitor.py（cyb SafetyProtector），发布 /f1_safety/is_safe。
# 遥操 safety.enable=true 时必须先于 ./start_teleop.sh 运行。日志 ./log/safety_monitor_<时间>.log。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CYB_DIR="${CYB_DIR:-${HOME}/cyb_safe_control_real/src/python}"
MONITOR="${CYB_DIR}/realtime_joint_safety_monitor.py"
INEX_SETUP="${INEX_SETUP:-${HOME}/workspace/inex_ros/install/setup.bash}"
LOG_DIR="${SCRIPT_DIR}/log"; mkdir -p "${LOG_DIR}"
TS="$(date +%Y%m%d_%H%M%S)"

[[ -f "${MONITOR}" ]] || { echo "❌ 未找到 ${MONITOR}（可用 CYB_DIR=... 指定）"; exit 1; }
pkill -f "python3 ${MONITOR}" 2>/dev/null && sleep 1

source /opt/ros/jazzy/setup.bash
[[ -f "${INEX_SETUP}" ]] && source "${INEX_SETUP}" || echo "⚠️ 未找到 iNex 工作区 ${INEX_SETUP}，ServoCommand 不可用，只能按反馈检查"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export RCUTILS_LOGGING_MAX_SEVERITY=40

cd "${CYB_DIR}"
nohup python3 "${MONITOR}" "$@" > "${LOG_DIR}/safety_monitor_${TS}.log" 2>&1 < /dev/null &
echo $! > "${LOG_DIR}/safety_monitor.pid"
sleep 6
if pgrep -f "python3 ${MONITOR}" >/dev/null; then
    echo "✅ 碰撞检测节点已启动 (pid $(cat "${LOG_DIR}/safety_monitor.pid"))，日志 ${LOG_DIR}/safety_monitor_${TS}.log"
    grep -v "type hash" "${LOG_DIR}/safety_monitor_${TS}.log" | tail -3
else
    echo "❌ 启动失败："; grep -v "type hash" "${LOG_DIR}/safety_monitor_${TS}.log" | tail -20; exit 1
fi
