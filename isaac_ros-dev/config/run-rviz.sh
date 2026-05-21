#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_NAV_RVIZ_PATH="${SCRIPT_DIR}/../src/sim/config/view_bot.rviz"
DEFAULT_SENSOR_RVIZ_PATH="${SCRIPT_DIR}/../src/sim/config/sensors.rviz"
NAV_PATH="${AUTONAV_RVIZ_CONFIG:-}"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CONTAINER_WORKSPACE_ROOT="${AUTONAV_CONTAINER_WORKSPACE_ROOT:-/autonav/isaac_ros-dev}"
CONTAINER_NAV_PATH="${AUTONAV_RVIZ_CONTAINER_CONFIG:-}"
CONTAINER_NAME="${AUTONAV_CONTAINER_NAME:-koopa-kingdom}"
CONTAINER_USER="${AUTONAV_CONTAINER_USER:-admin}"
RVIZ_MODE="${AUTONAV_RVIZ_MODE:-auto}"
RVIZ_CONFIG_MODE="${AUTONAV_RVIZ_CONFIG_MODE:-auto}"

while (($# > 0)); do
    case "${1}" in
        --wifi|--network)
            shift
            ;;
        --usb)
            echo "ERROR: --usb is not used for the no-Wi-Fi workflow."
            echo "Use USB-C SSH instead: ssh -Y jetson, then run ./isaac_ros-dev/config/run-rviz.sh on the Jetson."
            exit 1
            ;;
        --native)
            RVIZ_MODE="native"
            shift
            ;;
        --container)
            RVIZ_MODE="container"
            shift
            ;;
        --auto)
            RVIZ_MODE="auto"
            shift
            ;;
        --nav)
            RVIZ_CONFIG_MODE="nav"
            shift
            ;;
        --sensors|--sensor)
            RVIZ_CONFIG_MODE="sensors"
            shift
            ;;
        --)
            shift
            break
            ;;
        *)
            break
            ;;
    esac
done

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

if [[ "${RVIZ_MODE}" != "auto" && "${RVIZ_MODE}" != "native" && "${RVIZ_MODE}" != "container" ]]; then
    echo "ERROR: unsupported AUTONAV_RVIZ_MODE=${RVIZ_MODE}; expected 'auto', 'native', or 'container'."
    exit 1
fi

if [[ "${RVIZ_CONFIG_MODE}" != "auto" && "${RVIZ_CONFIG_MODE}" != "nav" && "${RVIZ_CONFIG_MODE}" != "sensors" ]]; then
    echo "ERROR: unsupported AUTONAV_RVIZ_CONFIG_MODE=${RVIZ_CONFIG_MODE}; expected 'auto', 'nav', or 'sensors'."
    exit 1
fi

for passthrough_var in RMW_IMPLEMENTATION ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI; do
    if [[ -n "${!passthrough_var:-}" ]]; then
        export "${passthrough_var}=${!passthrough_var}"
    fi
done

topic_exists() {
    local topic="$1"
    command -v ros2 >/dev/null 2>&1 || return 1
    timeout 3 ros2 topic list 2>/dev/null | grep -qx "${topic}"
}

select_rviz_config() {
    if [[ -n "${NAV_PATH}" ]]; then
        CONTAINER_NAV_PATH="${CONTAINER_NAV_PATH:-${NAV_PATH}}"
        return 0
    fi

    case "${RVIZ_CONFIG_MODE}" in
        nav)
            NAV_PATH="${DEFAULT_NAV_RVIZ_PATH}"
            CONTAINER_NAV_PATH="${CONTAINER_NAV_PATH:-${CONTAINER_WORKSPACE_ROOT}/src/sim/config/view_bot.rviz}"
            ;;
        sensors)
            NAV_PATH="${DEFAULT_SENSOR_RVIZ_PATH}"
            CONTAINER_NAV_PATH="${CONTAINER_NAV_PATH:-${CONTAINER_WORKSPACE_ROOT}/src/sim/config/sensors.rviz}"
            ;;
        auto)
            if topic_exists "/map"; then
                RVIZ_CONFIG_MODE="nav"
                NAV_PATH="${DEFAULT_NAV_RVIZ_PATH}"
                CONTAINER_NAV_PATH="${CONTAINER_NAV_PATH:-${CONTAINER_WORKSPACE_ROOT}/src/sim/config/view_bot.rviz}"
            else
                RVIZ_CONFIG_MODE="sensors"
                NAV_PATH="${DEFAULT_SENSOR_RVIZ_PATH}"
                CONTAINER_NAV_PATH="${CONTAINER_NAV_PATH:-${CONTAINER_WORKSPACE_ROOT}/src/sim/config/sensors.rviz}"
            fi
            ;;
    esac
}

select_rviz_config

print_env() {
    echo "AUTONAV_RVIZ_MODE=${RVIZ_MODE}"
    echo "AUTONAV_RVIZ_CONFIG_MODE=${RVIZ_CONFIG_MODE}"
    echo "AUTONAV_RVIZ_CONFIG=${NAV_PATH}"
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
    if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
        echo "WORKSPACE_OVERLAY=${WORKSPACE_ROOT}/install/setup.bash"
    fi
    for passthrough_var in RMW_IMPLEMENTATION ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI; do
        if [[ -n "${!passthrough_var:-}" ]]; then
            echo "${passthrough_var}=${!passthrough_var}"
        fi
    done
}

have_display() {
    [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]
}

have_x11_display() {
    [[ -n "${DISPLAY:-}" ]]
}

is_ssh_x11_display() {
    [[ -n "${SSH_CONNECTION:-}" && "${DISPLAY:-}" =~ ^(localhost|127\.0\.0\.1|\[::1\]|::1): ]]
}

in_container() {
    [[ -f /.dockerenv ]] || grep -qaE '/docker/|/containerd/' /proc/1/cgroup 2>/dev/null
}

display_error() {
    echo "ERROR: RViz needs an X11 display, but DISPLAY is not set."
    if in_container; then
        echo "You are inside the container. For no-Wi-Fi field use, run RViz from the Jetson host over USB-C SSH:"
        echo "  ssh -Y jetson"
        echo "  echo \$DISPLAY   # should look like localhost:10.0"
        echo "  cd AutoNav_25-26"
        echo "  ./env/docker/run-container.sh --no-attach"
        echo "  ./isaac_ros-dev/config/run-rviz.sh --container"
        echo "If you really want to run from inside the container, attach with X11/display env already passed through."
    else
        echo "Reconnect with X11 forwarding, for example: ssh -Y jetson"
    fi
}

require_container_x11() {
    if ! have_x11_display; then
        display_error
        return 2
    fi

    if [[ -n "${SSH_CONNECTION:-}" ]] && ! is_ssh_x11_display; then
        echo "ERROR: DISPLAY=${DISPLAY} does not look like an SSH-forwarded X11 display."
        echo "The Jetson is headless, so reconnect from the laptop with:"
        echo "  ssh -Y jetson"
        echo "Then confirm:"
        echo "  echo \$DISPLAY   # should look like localhost:10.0"
        return 2
    fi
}

configure_ssh_x11_gl() {
    if is_ssh_x11_display; then
        export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
        export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
    fi
}

run_native_rviz() {
    if ! command -v rviz2 >/dev/null 2>&1; then
        return 127
    fi

    if ! have_display; then
        display_error
        return 2
    fi

    echo "run-rviz.sh: launching native RViz"
    print_env
    configure_ssh_x11_gl
    rviz2 -d "${NAV_PATH}" "$@"
}

copy_xauthority_to_container() {
    local xauth_source="${XAUTHORITY:-${HOME}/.Xauthority}"
    local xauth_target="/tmp/autonav-rviz.xauthority"

    [[ -n "${DISPLAY:-}" ]] || return 0

    if [[ ! -r "${xauth_source}" ]]; then
        echo "Warning: XAUTHORITY source is not readable: ${xauth_source}" >&2
        echo "If RViz cannot open a window, reconnect with ssh -X or ssh -Y." >&2
        return 0
    fi

    docker cp "${xauth_source}" "${CONTAINER_NAME}:${xauth_target}" >/dev/null
    docker exec -u root "${CONTAINER_NAME}" chmod 644 "${xauth_target}" >/dev/null 2>&1 || true
    echo "${xauth_target}"
}

run_container_rviz() {
    local docker_args=(
        docker exec -it
        -u "${CONTAINER_USER}"
        -e "HOME=/home/${CONTAINER_USER}"
        -e "USER=${CONTAINER_USER}"
        -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
        -e "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
        -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
        -e "FASTRTPS_DEFAULT_PROFILES_FILE=${CONTAINER_WORKSPACE_ROOT}/../env/docker/fastdds_udp.xml"
        -e "FASTDDS_DEFAULT_PROFILES_FILE=${CONTAINER_WORKSPACE_ROOT}/../env/docker/fastdds_udp.xml"
        -e "QT_X11_NO_MITSHM=1"
    )
    local xauth_target

    require_container_x11 || return $?
    configure_ssh_x11_gl

    if ! command -v docker >/dev/null 2>&1; then
        echo "ERROR: docker is not available, so container RViz cannot be launched from here."
        return 1
    fi

    if ! docker ps --quiet --filter "name=^/${CONTAINER_NAME}$" 2>/dev/null | grep -q .; then
        echo "ERROR: container ${CONTAINER_NAME} is not running or Docker is not accessible."
        echo "On the Jetson, start it first: ./env/docker/run-container.sh --no-attach"
        return 1
    fi

    if [[ -n "${ROS_DISCOVERY_SERVER:-}" ]]; then
        docker_args+=(-e "ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER}")
    fi
    if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
        docker_args+=(-e "CYCLONEDDS_URI=${CYCLONEDDS_URI}")
    fi
    if [[ -n "${LIBGL_ALWAYS_SOFTWARE:-}" ]]; then
        docker_args+=(-e "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}")
    fi
    if [[ -n "${DISPLAY:-}" ]]; then
        docker_args+=(-e "DISPLAY=${DISPLAY}")
        xauth_target="$(copy_xauthority_to_container || true)"
        if [[ -n "${xauth_target:-}" ]]; then
            docker_args+=(-e "XAUTHORITY=${xauth_target}")
        fi
    fi

    echo "run-rviz.sh: launching RViz inside ${CONTAINER_NAME}"
    print_env
    docker_args+=(--workdir "${CONTAINER_WORKSPACE_ROOT}" "${CONTAINER_NAME}")
    docker_args+=(/bin/bash -lc "source /opt/ros/humble/setup.bash && if [ -f install/setup.bash ]; then source install/setup.bash; fi && exec rviz2 -d '${CONTAINER_NAV_PATH}' \"\$@\"" rviz2)
    "${docker_args[@]}" "$@"
}

if [[ "${RVIZ_MODE}" == "native" ]]; then
    if run_native_rviz "$@"; then
        exit 0
    else
        status=$?
    fi
    if [[ ${status} -eq 127 ]]; then
        echo "ERROR: rviz2 is not installed or not on PATH for native mode."
    fi
    exit "${status}"
fi

if [[ "${RVIZ_MODE}" == "container" ]]; then
    run_container_rviz "$@"
    exit $?
fi

if command -v rviz2 >/dev/null 2>&1; then
    run_native_rviz "$@" || exit $?
    exit 0
fi

if in_container; then
    echo "ERROR: rviz2 is not installed in this container, and Docker is not available from inside the container."
    exit 1
fi

echo "run-rviz.sh: native rviz2 was not found; trying container RViz."
if run_container_rviz "$@"; then
    exit 0
else
    status=$?
fi
exit "${status}"
