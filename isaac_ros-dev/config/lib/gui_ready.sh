# Sourced helper for GUI launch scripts.
#
# Provides gui_ready_wait(): block on wait_for_topic.py and, on success,
# print the [GUI_READY] <label> sentinel the HUD reader watches for.
#
# Pattern:
#   source "$(dirname "${BASH_SOURCE[0]}")/lib/gui_ready.sh"
#   ros2 launch ... &
#   launchpid=$!
#   trap 'kill -INT $launchpid 2>/dev/null' INT TERM
#   gui_ready_wait "DeviceLabel" /some_topic --type sensor_msgs/msg/X --qos sensor --timeout 60
#   wait "$launchpid"

_GUI_READY_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GUI_READY_WAIT_FOR_TOPIC_PY="${_GUI_READY_LIB_DIR}/wait_for_topic.py"

gui_ready_wait() {
    local label="$1"; shift
    if python3 "$GUI_READY_WAIT_FOR_TOPIC_PY" "$@"; then
        echo "[GUI_READY] $label"
    else
        echo "[GUI_READY_TIMEOUT] $label" >&2
    fi
}
