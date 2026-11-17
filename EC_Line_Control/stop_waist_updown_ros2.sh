#!/usr/bin/env bash
# Stop the ROS 2 waist up/down controller started by start_waist_updown_ros2.sh.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODE_SCRIPT="$SCRIPT_DIR/waist_updown_ros2.py"
PID_FILE="$SCRIPT_DIR/waist_updown_ros2.pid"

if [[ ! -f "$PID_FILE" ]]; then
    echo "Controller is not running: PID file not found."
    exit 0
fi

pid="$(cat "$PID_FILE")"
if ! kill -0 "$pid" 2>/dev/null; then
    echo "Controller is not running (stale PID: $pid)."
    rm -f "$PID_FILE"
    exit 0
fi

command="$(ps -p "$pid" -o args= 2>/dev/null || true)"
if [[ "$command" != *"$NODE_SCRIPT"* ]]; then
    echo "Error: PID $pid does not belong to $NODE_SCRIPT; refusing to stop it." >&2
    exit 1
fi

kill "$pid"
rm -f "$PID_FILE"
echo "Controller stop signal sent (PID: $pid)."
# 等 teardown（halt + body 下电）做完再返回，否则紧接着的 start 会在下电进行中请求上电而被拒（code=-3）
for _ in $(seq 1 30); do
    kill -0 "$pid" 2>/dev/null || { echo "Controller exited."; exit 0; }
    sleep 0.5
done
echo "Controller did not exit within 15 s; killing." >&2
kill -9 "$pid" 2>/dev/null || true
