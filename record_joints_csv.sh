#!/bin/bash
# 录制 ServoJ 关节指令 + 反馈到 CSV（只读，可与遥操同时运行）。参数原样透传给 scripts/record_joints_csv.py。
#   ./record_joints_csv.sh                      # Ctrl+C 结束 → data/joint_csv/joints_<时间>.csv
#   ./record_joints_csv.sh --duration 60 --ik   # 录 60 s，含 IK 目标
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
INEX_SETUP="${INEX_SETUP:-${HOME}/workspace/inex_ros/install/setup.bash}"
CONDA_BASE="${HOME}/miniconda3"
CONDA_ENV="pico4ultra_py312"

source "${ROS_SETUP}"
[[ -f "${INEX_SETUP}" ]] && source "${INEX_SETUP}" || { echo "❌ 未找到 iNex 工作区 ${INEX_SETUP}"; exit 1; }
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export RCUTILS_LOGGING_MAX_SEVERITY=2

source "${CONDA_BASE}/etc/profile.d/conda.sh" && conda activate "${CONDA_ENV}" || { echo "❌ conda 环境 ${CONDA_ENV} 激活失败"; exit 1; }
_clean_ldp=""
IFS=':' read -ra _dirs <<< "${LD_LIBRARY_PATH:-}"
for _d in "${_dirs[@]}"; do
    case "$_d" in *miniforge3*|*miniconda3*|*anaconda3*|"${CONDA_PREFIX:-__none__}/lib"|"") ;; *) _clean_ldp="${_clean_ldp:+${_clean_ldp}:}$_d" ;; esac
done
unset IFS
export LD_LIBRARY_PATH="/opt/ros/jazzy/lib:$(dirname "${INEX_SETUP}")/inex_ros2_interface/lib:${_clean_ldp}"

cd "${SCRIPT_DIR}"
exec python -u scripts/record_joints_csv.py "$@"
