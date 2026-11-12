#!/usr/bin/env bash
# ============================================================================
# fatigue_waist_updown.sh — EC 腰部直线运动 up/down 疲劳测试
#
# 每轮循环 (REPEAT 默认 3, 即"连续升 3 次 → 连续降 3 次"为一轮):
#   1) 连续上升 REPEAT 次: 每次发 ros2 topic pub --once ... "{data: true}" 并等"到位"
#   2) 连续下降 REPEAT 次: 每次发 ros2 topic pub --once ... "{data: false}" 并等"到位"
#   3) 回到 1) 反复, 直到达到设定轮数(CYCLES)或被 Ctrl-C / SIGTERM 停止
#   单步距离由节点 config 的 distance_m 决定(默认 0.02m=20mm); "到位" = 节点日志出现
#   "move completed"。注意: 连续同向 REPEAT 次 = REPEAT×distance_m 的单向行程(默认
#   60mm), 起点须在该方向留足余量, 否则末几次会被软限位拒绝/缩减(详见部署说明)。
#
# "到位"判定: 监视控制器节点自己的日志(waist_updown_ros2.log)。节点每完成一次
#   轨迹推流会打印 "Cartesian waist <dir> move completed (N frames)."；被限位拒绝
#   打印 "Rejected ..."；异常打印 "... move aborted ..."。脚本据此判断成功/失败,
#   避免在运动过程中重复发指令(节点会 "command ignored")。
#
# 前置条件: 先用 start_waist_updown_ros2.sh 启动节点, 且界面(HMI)已释放控制权。
#
# 用法:
#   ./fatigue_waist_updown.sh                 # 无限循环(每轮升3降3), Ctrl-C 停止
#   CYCLES=200 ./fatigue_waist_updown.sh      # 跑 200 轮后停止(1 轮 = 升3+降3 = 6 次)
#   REPEAT=5 ./fatigue_waist_updown.sh        # 改成每轮"连续升5次→连续降5次"
#   DWELL=1.0 ./fatigue_waist_updown.sh       # 每次到位后停留 1.0s
# 可用环境变量:
#   CYCLES(0=无限,单位:轮) REPEAT(每方向连续次数,默认3) DWELL(每次到位后停留秒,默认0.5)
#   ACK_TIMEOUT(默认5) DONE_TIMEOUT(默认15) MAX_PUB_RETRY(默认3)
#   TOPIC LOG_FILE RESULT_LOG WS_SETUP RMW_IMPLEMENTATION
# ============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPIC="${TOPIC:-/control/waist/updown}"
LOG_FILE="${LOG_FILE:-$SCRIPT_DIR/waist_updown_ros2.log}"
RESULT_LOG="${RESULT_LOG:-$SCRIPT_DIR/fatigue_waist_updown.log}"
CYCLES="${CYCLES:-0}"                 # 0 = 无限循环(单位: 轮)
DWELL="${DWELL:-0.5}"                 # 每次到位后停留秒数
ACK_TIMEOUT="${ACK_TIMEOUT:-5}"       # 等节点"收到指令"的秒数(超时则重发)
DONE_TIMEOUT="${DONE_TIMEOUT:-15}"    # 等一次运动"到位"的秒数
MAX_PUB_RETRY="${MAX_PUB_RETRY:-3}"   # 单条指令最多重发次数
REPEAT="${REPEAT:-3}"                 # 每方向连续运动次数: 升 REPEAT 次后再降 REPEAT 次为一轮
[[ "$REPEAT" =~ ^[1-9][0-9]*$ ]] || { echo "Error: REPEAT 必须是 >=1 的整数 (当前: '$REPEAT')" >&2; exit 1; }

# ---- 复刻节点的 ROS / DDS 环境 (必须与 start_waist_updown_ros2.sh 一致) ----
ros_setup="$(compgen -G '/opt/ros/*/setup.bash' | head -n 1 || true)"
if [[ -z "$ros_setup" ]]; then
    echo "Error: 找不到 /opt/ros/*/setup.bash" >&2
    exit 1
fi
ws_setup="${WS_SETUP:-$HOME/workspace/inex_ros/install/setup.bash}"
# ROS 的 setup.bash 不是 set -u(nounset) 安全的: 它内部会引用 AMENT_TRACE_SETUP_FILES
# 等未初始化变量, 在 set -u 下会报 "unbound variable" 并中断脚本。故 source 期间临时
# 关闭 -u, 两个 setup 都 source 完再恢复 -u(不影响本脚本其余部分的 nounset 保护)。
set +u
# shellcheck disable=SC1090
source "$ros_setup"
if [[ -f "$ws_setup" ]]; then
    # shellcheck disable=SC1090
    source "$ws_setup"
fi
set -u
# 节点用 CycloneDDS 与 root 下的 driver 通信; 发布端必须一致, 否则消息到不了节点。
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# ---- 前置检查 ----
pid_file="$SCRIPT_DIR/waist_updown_ros2.pid"
if [[ -f "$pid_file" ]]; then
    pid="$(cat "$pid_file")"
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "Error: 节点未在运行 (PID $pid 不存在), 请先执行 start_waist_updown_ros2.sh" >&2
        exit 1
    fi
else
    echo "Error: 找不到 PID 文件 $pid_file, 节点可能没启动" >&2
    exit 1
fi
command -v ros2 >/dev/null 2>&1 || { echo "Error: ros2 命令不可用" >&2; exit 1; }
[[ -f "$LOG_FILE" ]] || { echo "Error: 找不到节点日志 $LOG_FILE, 无法判定到位" >&2; exit 1; }

# ---- 工具函数 ----
ts() { date '+%Y-%m-%d %H:%M:%S'; }
say() { echo "[$(ts)] $*" | tee -a "$RESULT_LOG"; }
log_offset() { wc -c < "$LOG_FILE" 2>/dev/null || echo 0; }
new_log() { tail -c +"$(( $1 + 1 ))" "$LOG_FILE" 2>/dev/null; }

RUNNING=1
on_stop() { RUNNING=0; say "收到停止信号, 完成当前动作后退出..."; }
trap on_stop INT TERM

# 发一条指令并等到位。返回: 0=到位, 2=被拒/中止, 1=等到位超时, 3=指令未被节点接收
do_move() {
    local data="$1" name="$2"
    local off; off="$(log_offset)"
    local attempt=1 acked=0

    # 阶段1: 发指令并确认节点已收到(应对 ros2 topic pub --once 的发现竞态, 未收到则重发)
    while [[ "$attempt" -le "$MAX_PUB_RETRY" && "$acked" -eq 0 && "$RUNNING" -eq 1 ]]; do
        ros2 topic pub --once "$TOPIC" std_msgs/msg/Bool "{data: $data}" >/dev/null 2>&1
        local ack_deadline=$(( $(date +%s) + ACK_TIMEOUT ))
        while [[ "$(date +%s)" -lt "$ack_deadline" && "$RUNNING" -eq 1 ]]; do
            if new_log "$off" | grep -qE 'Starting .*move by|Rejected .*move|command ignored|Robot not ready|move aborted|move completed'; then
                acked=1; break
            fi
            sleep 0.2
        done
        if [[ "$acked" -eq 0 ]]; then
            say "  $name: 未确认节点收到, 重发 ($attempt/$MAX_PUB_RETRY)"
            attempt=$(( attempt + 1 ))
        fi
    done
    if [[ "$acked" -eq 0 ]]; then
        say "  $name: 连续 $MAX_PUB_RETRY 次无节点响应 — 请检查节点是否运行、RMW_IMPLEMENTATION/ROS_DOMAIN_ID/CYCLONEDDS_URI 是否与节点一致"
        return 3
    fi

    # 阶段2: 等"到位"(completed) 或失败(Rejected/aborted/ignored/not ready)
    local done_deadline=$(( $(date +%s) + DONE_TIMEOUT ))
    while [[ "$(date +%s)" -lt "$done_deadline" && "$RUNNING" -eq 1 ]]; do
        local n; n="$(new_log "$off")"
        if printf '%s' "$n" | grep -q 'move completed'; then
            say "  $name: 到位"; return 0
        fi
        if printf '%s' "$n" | grep -qE 'Rejected .*move|move aborted|command ignored|Robot not ready'; then
            local reason; reason="$(printf '%s' "$n" | grep -E 'Rejected .*move|move aborted|command ignored|Robot not ready' | tail -1)"
            say "  $name: 失败 -> $reason"; return 2
        fi
        sleep 0.2
    done
    [[ "$RUNNING" -eq 1 ]] && say "  $name: 等待到位超时 (${DONE_TIMEOUT}s)"
    return 1
}

# ---- 主循环: 每轮 = 连续升 REPEAT 次 → 连续降 REPEAT 次 ----
say "==== EC 腰部直线运动疲劳测试开始 ===="
say "模式: 每轮 连续升 ${REPEAT} 次 → 连续降 ${REPEAT} 次 (单步距离由节点 config 的 distance_m 决定)"
say "topic=$TOPIC  cycles=$([[ "$CYCLES" -gt 0 ]] && echo "$CYCLES 轮" || echo 无限)  dwell=${DWELL}s  node_log=$LOG_FILE"
say "提示: 期间请勿重启节点(日志偏移会失效); 按 Ctrl-C 可安全停止。"

done_cycles=0
up_ok=0; down_ok=0
t_start="$(date +%s)"

while [[ "$RUNNING" -eq 1 ]]; do
    n=$(( done_cycles + 1 ))
    fail=0

    say "---- 轮 #$n: 连续升 ${REPEAT} 次 ----"
    for (( i=1; i<=REPEAT; i++ )); do
        [[ "$RUNNING" -eq 1 ]] || { fail=1; break; }
        if ! do_move true "up#$i"; then say "!! up#$i 未到位, 终止疲劳测试"; fail=1; break; fi
        up_ok=$(( up_ok + 1 ))
        [[ "$RUNNING" -eq 1 && "$DWELL" != "0" ]] && sleep "$DWELL"
    done
    [[ "$fail" -eq 1 ]] && break

    say "---- 轮 #$n: 连续降 ${REPEAT} 次 ----"
    for (( i=1; i<=REPEAT; i++ )); do
        [[ "$RUNNING" -eq 1 ]] || { fail=1; break; }
        if ! do_move false "down#$i"; then say "!! down#$i 未到位, 终止疲劳测试"; fail=1; break; fi
        down_ok=$(( down_ok + 1 ))
        [[ "$RUNNING" -eq 1 && "$DWELL" != "0" ]] && sleep "$DWELL"
    done
    [[ "$fail" -eq 1 ]] && break

    done_cycles=$n
    if [[ "$CYCLES" -gt 0 && "$done_cycles" -ge "$CYCLES" ]]; then
        say "已达到设定轮数 $CYCLES, 正常结束"; break
    fi
done

t_end="$(date +%s)"
say "==== 疲劳测试结束: 完整轮数 $done_cycles (up 成功 $up_ok, down 成功 $down_ok), 用时 $(( t_end - t_start ))s ===="
