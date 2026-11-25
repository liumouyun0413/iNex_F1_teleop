#!/bin/bash
# ============================================================================
# F1（珞石双臂 + iNex ROS2）遥操作一键启动（替代 lxl 珞石版的 start_all_pico2_log.sh）
#   1) 腰部升降节点（EC_Line_Control，可选）
#   2) PICO 数据服务 RoboticsService
#   3) 碰撞检测节点 realtime_joint_safety_monitor.py（cyb，发布 /f1_safety/is_safe；可选）
#   4) 遥操作主程序 teleop_f1_hardware.py（单进程：iNex 双臂/夹爪/头/腰/底盘/安全门/数采）
# 日志按时间戳写入 ./log/，重启不覆盖。
# 注意：不要开 set -u，ROS/conda 的 setup 脚本会引用未定义变量。
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/log"
TS="$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}"
find "${LOG_DIR}" -maxdepth 1 -name '*.log' -mtime +7 -delete 2>/dev/null   # 日志只保留 7 天

# ---------------- 可按现场环境修改 ----------------
ROS_SETUP="/opt/ros/jazzy/setup.bash"
# iNex 工作区（inex_ros2_interface v1.0：ServoCommand / RobotSystemStatus 等）—— 必需
INEX_SETUP="${INEX_SETUP:-${HOME}/workspace/inex_ros/install/setup.bash}"
# interface_20260623 编译产物（底盘 move/srv）：IFACE_SETUP=... 指定 > ~/interface_20260623 > 本目录自带
IFACE_SETUP_CANDIDATES=(
    "${IFACE_SETUP:-}"
    "${HOME}/interface_20260623/install/setup.bash"
    "${SCRIPT_DIR}/interface_20260623/install/setup.bash"
)
CONDA_BASE="${HOME}/miniconda3"
CONDA_ENV="pico4ultra_py312"
ROBOTICS_SERVICE_DIR="/opt/apps/roboticsservice"
START_WAIST_NODE="${START_WAIST_NODE:-1}"   # 腰部节点（EC_Line_Control/waist_updown_ros2.py → /inex/move_to）
START_SAFETY_MONITOR="${START_SAFETY_MONITOR:-1}"   # 碰撞检测节点（yaml safety.enable=true 时遥操启动必须收到 /f1_safety/is_safe）
CYB_DIR="${CYB_DIR:-${HOME}/cyb_safe_control_real/cyb_dist/src/python}"   # cyb SafetyProtector + realtime_joint_safety_monitor.py 所在目录
SAFETY_MONITOR_ARGS="${SAFETY_MONITOR_ARGS:-}"   # 例如 "--check-hz 500"
TELEOP_ARGS="${TELEOP_ARGS:-}"              # 例如 TELEOP_ARGS="--visualize-placo"
# --------------------------------------------------

echo "=== F1-Rokae teleop 启动 (${TS}) ==="

# ROS2 环境（nohup 非交互 shell 不读 bashrc，必须显式导出）
source "${ROS_SETUP}"
if [[ -f "${INEX_SETUP}" ]]; then
    source "${INEX_SETUP}"; echo "已 source iNex 工作区: ${INEX_SETUP}"
else
    echo "❌ 未找到 iNex 工作区 ${INEX_SETUP}（可用 INEX_SETUP=... 指定），inex_ros2_interface 无法导入"; exit 1
fi
IFACE_OK=0
for f in "${IFACE_SETUP_CANDIDATES[@]}"; do
    if [[ -f "$f" ]]; then source "$f"; IFACE_OK=1; echo "已 source 接口包: $f"; break; fi
done
[[ "${IFACE_OK}" == "1" ]] || echo "⚠️ 未找到 interface_20260623 编译产物，move/srv 无法导入（底盘使能会失败）"

# 驱动 inex_ros2_driver 以 root 运行，FastDDS 共享内存跨用户失败 → 统一 Cyclone DDS（与腰部节点一致）
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export RCUTILS_LOGGING_MAX_SEVERITY=2      # 抑制 DDS type-hash WARN 刷屏

# 1) 腰部节点
if [[ "${START_WAIST_NODE}" == "1" ]]; then
    if pgrep -f waist_updown_ros2.py >/dev/null; then
        echo "腰部节点已在运行"
    elif [[ -f "${SCRIPT_DIR}/EC_Line_Control/start_waist_updown_ros2.sh" ]]; then
        WS_SETUP="${INEX_SETUP}" bash "${SCRIPT_DIR}/EC_Line_Control/start_waist_updown_ros2.sh" || echo "⚠️ 腰部节点启动失败"
    fi
elif ! pgrep -f waist_updown_ros2.py >/dev/null; then
    echo "⚠️ 未检测到腰部节点 waist_updown_ros2.py，Y/X 升降腰将无响应"
fi

# 2) RoboticsService（PICO 手柄数据）
if [[ -d "${ROBOTICS_SERVICE_DIR}" ]]; then
    if pgrep -f RoboticsServiceProcess >/dev/null; then
        echo "RoboticsService 已在运行"
    else
        (cd "${ROBOTICS_SERVICE_DIR}" && nohup ./runService.sh > "${LOG_DIR}/roboticsservice_${TS}.log" 2>&1 &)
        echo "RoboticsService 已启动，等待就绪..."
        sleep 3
    fi
else
    echo "⚠️ 未找到 ${ROBOTICS_SERVICE_DIR}"
fi

# 3) 碰撞检测节点（系统 python3 + ROS 环境，须在 conda 激活前启动）
SAFETY_MONITOR="${CYB_DIR}/realtime_joint_safety_monitor.py"
if [[ "${START_SAFETY_MONITOR}" == "1" ]]; then
    if pgrep -f "realtime_joint_safety_monitor.py" >/dev/null; then
        echo "碰撞检测节点已在运行"
    elif [[ -f "${SAFETY_MONITOR}" ]]; then
        SM_LOG="${LOG_DIR}/safety_monitor_${TS}.log"
        (cd "${CYB_DIR}" && RCUTILS_LOGGING_MAX_SEVERITY=40 nohup /usr/bin/python3 "${SAFETY_MONITOR}" ${SAFETY_MONITOR_ARGS} > "${SM_LOG}" 2>&1 < /dev/null &)
        for _ in $(seq 1 30); do grep -q "\[Safety\] 等待" "${SM_LOG}" 2>/dev/null && break; sleep 0.5; done   # MuJoCo 模型加载数秒
        if pgrep -f "realtime_joint_safety_monitor.py" >/dev/null; then
            echo "碰撞检测节点已启动，日志 ${SM_LOG}"
        else
            echo "⚠️ 碰撞检测节点启动失败（见 ${SM_LOG}）；yaml safety.enable=true 时遥操会因收不到 /f1_safety/is_safe 终止"
        fi
    else
        echo "⚠️ 未找到 ${SAFETY_MONITOR}（可用 CYB_DIR=... 指定），跳过碰撞检测节点"
    fi
elif ! pgrep -f "realtime_joint_safety_monitor.py" >/dev/null; then
    echo "⚠️ 未检测到碰撞检测节点，yaml safety.enable=true 时遥操无法启动"
fi

# 4) 遥操作主程序（conda 环境）
source "${CONDA_BASE}/etc/profile.d/conda.sh"
conda activate "${CONDA_ENV}" || { echo "❌ conda 环境 ${CONDA_ENV} 激活失败"; exit 1; }

# 清理 conda 对 ROS2 库加载的污染（lxl 珞石版实测：conda 库路径抢先于 /opt/ros/jazzy/lib 时，
# 所有 /inex/* 服务超时、话题收不到）。只剔除库路径，不退出环境。
_clean_ldp=""
IFS=':' read -ra _dirs <<< "${LD_LIBRARY_PATH:-}"
for _d in "${_dirs[@]}"; do
    case "$_d" in
        *miniforge3*|*miniconda3*|*anaconda3*|"${CONDA_PREFIX:-__none__}/lib") ;;
        "") ;;
        *) _clean_ldp="${_clean_ldp:+${_clean_ldp}:}$_d" ;;
    esac
done
unset IFS
INEX_IFACE_LIB="$(dirname "${INEX_SETUP}")/inex_ros2_interface/lib"
export LD_LIBRARY_PATH="/opt/ros/jazzy/lib:${INEX_IFACE_LIB}:${_clean_ldp}"

# PICO SDK 原生库：xrobotoolkit_sdk 的 RUNPATH 指向编译机路径，换机器后失效（ImportError: libPXREARobotSDK.so）；
# 统一用 RoboticsService 安装包自带的同一库（SDK/<arch>/）。
PXREA_FOUND=0
for _d in "${ROBOTICS_SERVICE_DIR}"/SDK/*/; do
    if [[ -f "${_d}libPXREARobotSDK.so" ]]; then
        export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${_d%/}"; PXREA_FOUND=1; break
    fi
done
[[ "${PXREA_FOUND}" == "1" ]] || echo "⚠️ 未找到 ${ROBOTICS_SERVICE_DIR}/SDK/*/libPXREARobotSDK.so（RoboticsService 未安装？），xrobotoolkit_sdk 可能无法导入"

if pgrep -f teleop_f1_hardware.py >/dev/null; then
    echo "❌ teleop_f1_hardware.py 已在运行，请先 ./stop_teleop.sh"; exit 1
fi
if pgrep -f "teleop_lxl.py|all_remote_lxl.py" >/dev/null; then
    echo "❌ 旧版 lxl 遥操进程仍在运行，会与本程序争抢 /inex/servo_cmd，请先停掉"; exit 1
fi

cd "${SCRIPT_DIR}"
nohup python -u scripts/teleop_f1_hardware.py ${TELEOP_ARGS} > "${LOG_DIR}/teleop_${TS}.log" 2>&1 &
echo $! > "${LOG_DIR}/teleop.pid"
sleep 2

echo ""
echo "进程检查："
ps aux | grep -E 'teleop_f1_hardware|realtime_joint_safety_monitor|runService|RoboticsServiceProcess|waist_updown_ros2' | grep -v grep | cut -c1-160
echo ""
echo "日志：tail -f ${LOG_DIR}/teleop_${TS}.log"
echo "停止：./stop_teleop.sh"
