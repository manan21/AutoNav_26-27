#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
NAV_PATH="${AUTONAV_RVIZ_CONFIG:-${SCRIPT_DIR}/../src/sim/config/view_bot.rviz}"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

in_container() {
    [[ -f /.dockerenv ]] || grep -qaE '/docker/|/containerd/' /proc/1/cgroup 2>/dev/null
}

if ! in_container; then
    echo "ERROR: run-rviz-jetson.sh is intended to run inside the Jetson container."
    echo "From the laptop, connect with X11 and start/attach the container like this:"
    echo "  ssh -Y jetson"
    echo "  echo \$DISPLAY   # should look like localhost:10.0"
    echo "  cd AutoNav_25-26"
    echo "  AUTONAV_CONTAINER_GUI=1 AUTONAV_KEEP_SSH_X11=1 ./env/docker/run-container.sh"
    echo "Then inside the container:"
    echo "  ./config/run-rviz-jetson.sh"
    exit 1
fi

if [[ -z "${DISPLAY:-}" ]]; then
    echo "ERROR: DISPLAY is not set inside the container; X11 forwarding is not active."
    echo "Restart the container from an SSH-X11 session:"
    echo "  ssh -Y jetson"
    echo "  echo \$DISPLAY   # should look like localhost:10.0"
    echo "  cd AutoNav_25-26"
    echo "  docker rm -f koopa-kingdom"
    echo "  AUTONAV_CONTAINER_GUI=1 AUTONAV_KEEP_SSH_X11=1 ./env/docker/run-container.sh"
    echo "Then run ./config/run-rviz-jetson.sh inside the container."
    exit 2
fi

if [[ -n "${XAUTHORITY:-}" && ! -r "${XAUTHORITY}" ]]; then
    echo "ERROR: XAUTHORITY=${XAUTHORITY} is not readable inside the container."
    echo "Restart the container from the Jetson host with AUTONAV_CONTAINER_GUI=1 AUTONAV_KEEP_SSH_X11=1."
    exit 2
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
fi

if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
    source "${WORKSPACE_ROOT}/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${REPO_ROOT}/env/docker/fastdds_udp.xml}"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-${FASTRTPS_DEFAULT_PROFILES_FILE}}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

for passthrough_var in RMW_IMPLEMENTATION ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI; do
    if [[ -n "${!passthrough_var:-}" ]]; then
        export "${passthrough_var}=${!passthrough_var}"
    fi
done

if ! command -v rviz2 >/dev/null 2>&1; then
    echo "ERROR: rviz2 is not installed or not on PATH inside the container."
    exit 1
fi

echo "run-rviz-jetson.sh: launching container RViz over forwarded X11"
echo "DISPLAY=${DISPLAY}"
if [[ -n "${XAUTHORITY:-}" ]]; then
    echo "XAUTHORITY=${XAUTHORITY}"
fi
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
echo "FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE}"
echo "QT_X11_NO_MITSHM=${QT_X11_NO_MITSHM}"
echo "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}"

rviz2 -d "${NAV_PATH}" "$@"
