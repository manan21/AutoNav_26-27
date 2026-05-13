#!/bin/bash

set -e # makes script exit on command failure

# PARAMETERS
IMAGE_TAG="dev:koopa-kingdom"
CONTAINER_NAME="koopa-kingdom"
HOST_WORKDIR="$HOME/AutoNav_25-26"
CONTAINER_WORKDIR="/autonav"
ENTRYPOINT="/usr/local/bin/scripts/entrypoint.sh"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# FLAGS
NO_ATTACH=0
_FILTERED_ARGS=()
for _arg in "$@"; do
    case "$_arg" in
        --no-attach) NO_ATTACH=1 ;;
        *) _FILTERED_ARGS+=("$_arg") ;;
    esac
done
set -- "${_FILTERED_ARGS[@]}"

# DETECT PLATFORM
PLATFORM=$(uname -m)

# USERNAME
USERNAME="${USERNAME:-admin}"
CONTAINER_GUI="${AUTONAV_CONTAINER_GUI:-0}"
AUTONAV_FASTDDS_PROFILE_FILE_DEFAULT="${CONTAINER_WORKDIR}/env/docker/fastdds_udp.xml"
source "${SCRIPT_DIR}/dds-env.sh"

_detect_local_x_display() {
    local socket display

    for socket in /tmp/.X11-unix/X*; do
        [[ -S "${socket}" ]] || continue
        display=":${socket##*/X}"
        printf '%s\n' "${display}"
        return 0
    done

    return 1
}

_xauth_names_for_display() {
    local display_number="${DISPLAY#localhost:}"
    display_number="${display_number#:}"
    display_number="${display_number%%.*}"

    printf '%s\n' \
        "${DISPLAY}" \
        "$(hostname)/unix:${display_number}" \
        "unix:${display_number}" \
        "localhost:${display_number}"
}

_discover_xauth_sources() {
    local pid env_file entry key value

    printf '%s\n' \
        "${XAUTHORITY:-}" \
        "/run/user/$(id -u)/gdm/Xauthority" \
        "$HOME/.Xauthority"

    for env_file in /proc/[0-9]*/environ; do
        [[ -r "${env_file}" ]] || continue
        pid="${env_file#/proc/}"
        pid="${pid%/environ}"

        case "$(ps -p "${pid}" -o comm= 2>/dev/null || true)" in
            Xorg|Xwayland|gnome-shell|gnome-session*|gdm-session-worker)
                ;;
            *)
                continue
                ;;
        esac

        while IFS= read -r -d '' entry; do
            key="${entry%%=*}"
            value="${entry#*=}"
            if [[ "${key}" == "XAUTHORITY" && -n "${value}" ]]; then
                printf '%s\n' "${value}"
            fi
        done < "${env_file}"
    done
}

# DISPLAY FORWARDING
#
# Default to a headless container so the Jetson owns the ROS 2 stack while RViz
# runs natively on the laptop over DDS. The old X11 path is still available by
# setting AUTONAV_CONTAINER_GUI=1.
if [[ "${CONTAINER_GUI}" == "1" ]]; then
    if [[ -n "${AUTONAV_DISPLAY:-}" ]]; then
        echo "Using AUTONAV_DISPLAY=${AUTONAV_DISPLAY}."
        DISPLAY="${AUTONAV_DISPLAY}"
    elif [[ "${AUTONAV_KEEP_SSH_X11:-0}" != "1" && "${DISPLAY:-}" =~ ^localhost: && "${PLATFORM}" == "aarch64" ]]; then
        ORIGINAL_DISPLAY="${DISPLAY}"
        DISPLAY="$(_detect_local_x_display || printf ':0')"
        echo "DISPLAY=${ORIGINAL_DISPLAY} looks like SSH X11 forwarding; using DISPLAY=${DISPLAY} for Jetson hardware GL."
    elif [[ -z "${DISPLAY:-}" && "${PLATFORM}" == "aarch64" ]]; then
        DISPLAY="$(_detect_local_x_display || printf ':0')"
        echo "DISPLAY is unset; using DISPLAY=${DISPLAY} for Jetson hardware GL."
    fi

    XAUTH_FILE="/tmp/.docker-xauth-${CONTAINER_NAME}"
    XDG_RUNTIME_DIR="/tmp/runtime-${USERNAME}"
    mkdir -p "${XDG_RUNTIME_DIR}"
    chmod 700 "${XDG_RUNTIME_DIR}" 2>/dev/null || true
else
    echo "AUTONAV_CONTAINER_GUI=0; starting ${CONTAINER_NAME} headless."
    echo "Run RViz locally on the laptop with the same DDS/ROS settings."
fi

echo "DDS discovery mode: ${AUTONAV_DDS_DISCOVERY}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
if [[ "${AUTONAV_DDS_DISCOVERY}" == "server" ]]; then
    echo "ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER}"
fi

DOCKER_ARGS=()

# ENVIRONMENT VARIABLES
# ENVIRONMENT VARIABLES
DOCKER_ARGS+=("-e" "USER=${USERNAME}")
DOCKER_ARGS+=("-e" "USERNAME=${USERNAME}")
DOCKER_ARGS+=("-e" "HOST_USER_UID=$(id -u)")
DOCKER_ARGS+=("-e" "HOST_USER_GID=$(id -g)")
DOCKER_ARGS+=("-e" "WORKDIR=${CONTAINER_WORKDIR}")

for passthrough_var in ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI AUTONAV_DDS_DISCOVERY AUTONAV_DDS_DISCOVERY_PORT AUTONAV_JETSON_IP; do
    if [[ -n "${!passthrough_var:-}" ]]; then
        DOCKER_ARGS+=("-e" "${passthrough_var}=${!passthrough_var}")
    fi
done

if [[ "${CONTAINER_GUI}" == "1" ]]; then
    DOCKER_ARGS+=("-e" "DISPLAY=${DISPLAY:-:0}")
    DOCKER_ARGS+=("-e" "XAUTHORITY=${XAUTH_FILE}")
    DOCKER_ARGS+=("-e" "XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}")
    DOCKER_ARGS+=("-e" "QT_X11_NO_MITSHM=1")
fi

# BLUETOOTH AND DBUS
DOCKER_ARGS+=("-v" "/run/dbus:/run/dbus")
DOCKER_ARGS+=("-v" "/dev/input:/dev/input")
DOCKER_ARGS+=("-v" "/run/udev:/run/udev:ro")
DOCKER_ARGS+=("--network=host")

if [[ "${CONTAINER_GUI}" == "1" ]]; then
    touch "${XAUTH_FILE}"
    chmod 666 "${XAUTH_FILE}" 2>/dev/null || true
fi

_refresh_x11_auth() {
    [[ "${CONTAINER_GUI}" == "1" ]] || return 0
    [[ -n "${DISPLAY}" ]] || return 0
    local auth_source auth_entries auth_name tmp_auth

    xauth -b -f "${XAUTH_FILE}" remove "${DISPLAY}" >/dev/null 2>&1 || true
    xauth -b -f "${XAUTH_FILE}" remove "$(hostname)/unix${DISPLAY#localhost}" >/dev/null 2>&1 || true
    xauth -b -f "${XAUTH_FILE}" remove "localhost${DISPLAY#localhost}" >/dev/null 2>&1 || true

    while IFS= read -r auth_source; do
        [[ -n "${auth_source}" && -r "${auth_source}" ]] || continue
        tmp_auth="/tmp/.docker-xauth-source-${CONTAINER_NAME}-$$"
        cp "${auth_source}" "${tmp_auth}" 2>/dev/null || continue
        chmod 600 "${tmp_auth}" 2>/dev/null || true

        while IFS= read -r auth_name; do
            auth_entries="$(xauth -b -f "${tmp_auth}" nlist "${auth_name}" 2>/dev/null || true)"
            if [[ -n "${auth_entries}" ]]; then
                printf '%s\n' "${auth_entries}" \
                    | sed 's/^..../ffff/' \
                    | xauth -b -f "${XAUTH_FILE}" nmerge - >/dev/null 2>&1 && {
                        rm -f "${tmp_auth}"
                        return 0
                    }
            fi
        done < <(_xauth_names_for_display)

        rm -f "${tmp_auth}"
    done < <(_discover_xauth_sources | sort -u)

    while IFS= read -r auth_name; do
        auth_entries="$(xauth -b nlist "${auth_name}" 2>/dev/null || true)"
        if [[ -n "${auth_entries}" ]] && printf '%s\n' "${auth_entries}" \
            | sed 's/^..../ffff/' \
            | xauth -b -f "${XAUTH_FILE}" nmerge - >/dev/null 2>&1; then
            return 0
        fi
    done < <(_xauth_names_for_display)

    echo "Warning: could not copy X11 authorization for DISPLAY=${DISPLAY} into ${XAUTH_FILE}."
    echo "Local X sockets visible to this shell: $(find /tmp/.X11-unix -maxdepth 1 -type s -printf '%f ' 2>/dev/null || true)"
    echo "Checked Xauthority sources: $(_discover_xauth_sources | sort -u | tr '\n' ' ')"
    echo "If RViz reports 'Authorization required', run from the Jetson desktop session or set AUTONAV_DISPLAY."
}
_refresh_x11_auth

if [[ "${CONTAINER_GUI}" == "1" ]]; then
    DOCKER_ARGS+=("-v" "${XAUTH_FILE}:${XAUTH_FILE}:rw")
    DOCKER_ARGS+=("-v" "/tmp/.X11-unix:/tmp/.X11-unix:rw")
    DOCKER_ARGS+=("-v" "${XDG_RUNTIME_DIR}:${XDG_RUNTIME_DIR}:rw")
fi

# SSH AGENT
if [[ -n $SSH_AUTH_SOCK ]]; then
    DOCKER_ARGS+=("-v" "$SSH_AUTH_SOCK:/ssh-agent")
    DOCKER_ARGS+=("-e" "SSH_AUTH_SOCK=/ssh-agent")
fi

# JETSON SPECIFIC
if [[ $PLATFORM == "aarch64" ]]; then
    echo "Detected Jetson platform (aarch64)"
    DOCKER_ARGS+=("-v" "/usr/bin/tegrastats:/usr/bin/tegrastats")
    DOCKER_ARGS+=("-v" "/tmp/:/tmp/")
    DOCKER_ARGS+=("--pid=host")
    
    # jtop support
    if [[ $(getent group jtop) ]]; then
        DOCKER_ARGS+=("-v" "/run/jtop.sock:/run/jtop.sock:ro")
    fi
fi

# SERIAL / USB / CAMERA DEVICES
# USB
DOCKER_ARGS+=("--device=/dev/arduino_uno:/dev/arduino_uno")
DOCKER_ARGS+=("--device=/dev/roboteq:/dev/roboteq")

DOCKER_ARGS+=("-v" "/dev/serial/by-id:/dev/serial/by-id:ro")

for link in \
  /dev/serial/by-id/usb-Arduino* \
  /dev/serial/by-id/usb-RoboteQ* \
  /dev/serial/by-id/usb-Cypress*; do
  if [[ -e "$link" ]]; then
    real_dev="$(readlink -f "$link")"
    DOCKER_ARGS+=("--device=${real_dev}:${real_dev}")
  fi
done

# Jetson UART for e-stop (if you use it)
if [[ -e /dev/ttyTHS1 ]]; then
    DOCKER_ARGS+=("--device=/dev/ttyTHS1:/dev/ttyTHS1")
fi

# Ensure container user can open /dev/tty*
DIALOUT_GID=$(getent group dialout | cut -d: -f3)
if [[ -n "$DIALOUT_GID" ]]; then
    DOCKER_ARGS+=("--group-add=${DIALOUT_GID}")
fi


# MOUNTS & WORKING DIRECTORY
DOCKER_ARGS+=("-v" "${HOST_WORKDIR}:${CONTAINER_WORKDIR}")
DOCKER_ARGS+=("-v" "/etc/localtime:/etc/localtime:ro")
DOCKER_ARGS+=("--workdir=${CONTAINER_WORKDIR}/isaac_ros-dev")
DOCKER_ARGS+=("-v" "$SCRIPT_DIR/entrypoint_additions:/usr/local/bin/scripts/entrypoint_additions")
DOCKER_ARGS+=("-v" "$SCRIPT_DIR/entrypoint.sh:/usr/local/bin/scripts/entrypoint.sh")
# PERSISTENT BUILD VOLUMES
DOCKER_ARGS+=("-v" "${CONTAINER_NAME}-build:${CONTAINER_WORKDIR}/isaac_ros-dev/build")
DOCKER_ARGS+=("-v" "${CONTAINER_NAME}-install:${CONTAINER_WORKDIR}/isaac_ros-dev/install")
DOCKER_ARGS+=("-v" "${CONTAINER_NAME}-log:${CONTAINER_WORKDIR}/isaac_ros-dev/log")
# ZED settings/resources
# Ensure host dirs exist so the SDK can always download and persist
# factory calibration files (e.g. SN<serial>.conf) across container runs.
# Without the SDK-5.x path mount, ZED tries to create
# /usr/local/zed/lib/cmake/ZED/settings/ inside the (read-only) image
# and fails with "Permission denied" / "CALIBRATION FILE NOT AVAILABLE".
mkdir -p "$HOME/zed/settings" "$HOME/zed/resources"
# Legacy paths (ZED SDK < 5)
DOCKER_ARGS+=("-v" "$HOME/zed/settings:/usr/local/zed/settings")
DOCKER_ARGS+=("-v" "$HOME/zed/resources:/usr/local/zed/resources")
# ZED SDK 5.x paths (calibration + AI models e.g. neural_depth_light_5.2.model)
DOCKER_ARGS+=("-v" "$HOME/zed/settings:/usr/local/zed/lib/cmake/ZED/settings")
DOCKER_ARGS+=("-v" "$HOME/zed/resources:/usr/local/zed/lib/cmake/ZED/resources")

DOCKER_ARGS+=("--entrypoint=$ENTRYPOINT")

# GPU env
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")

# Ensure container user can open /dev/nvhost* and friends
VID_GID=$(getent group video  | cut -d: -f3)
REN_GID=$(getent group render | cut -d: -f3)
INPUT_GID=$(getent group input | cut -d: -f3)
if [[ -n "$VID_GID" ]]; then DOCKER_ARGS+=("--group-add=${VID_GID}"); fi
if [[ -n "$REN_GID" ]]; then DOCKER_ARGS+=("--group-add=${REN_GID}"); fi
if [[ -n "$INPUT_GID" ]]; then DOCKER_ARGS+=("--group-add=${INPUT_GID}"); fi

attach_shell() {
    _refresh_x11_auth

    local exec_args=(
        docker exec -i -t -u "${USERNAME}"
        -e "HOME=/home/${USERNAME}"
        -e "USER=${USERNAME}"
        -e "USERNAME=${USERNAME}"
    )

    for passthrough_var in ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI AUTONAV_DDS_DISCOVERY AUTONAV_DDS_DISCOVERY_PORT AUTONAV_JETSON_IP; do
        if [[ -n "${!passthrough_var:-}" ]]; then
            exec_args+=("-e" "${passthrough_var}=${!passthrough_var}")
        fi
    done

    if [[ "${CONTAINER_GUI}" == "1" ]]; then
        exec_args+=(
            -e "DISPLAY=${DISPLAY:-:0}"
            -e "XAUTHORITY=${XAUTH_FILE}"
            -e "XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}"
            -e "QT_X11_NO_MITSHM=1"
        )
    fi

    exec_args+=(
        --workdir "${CONTAINER_WORKDIR}/isaac_ros-dev"
        "${CONTAINER_NAME}"
    )

    if (($# == 0)); then
        "${exec_args[@]}" /bin/bash -lc \
            "source /opt/ros/humble/setup.bash && \
             if [ -f /autonav/isaac_ros-dev/install/setup.bash ]; then \
                 source /autonav/isaac_ros-dev/install/setup.bash; \
             fi; \
             if [ '${CONTAINER_GUI}' = '1' ]; then \
                 export DISPLAY='${DISPLAY:-:0}'; \
                 export XAUTHORITY='${XAUTH_FILE}'; \
                 export XDG_RUNTIME_DIR='${XDG_RUNTIME_DIR}'; \
                 export QT_X11_NO_MITSHM=1; \
             fi && \
             exec /bin/bash -i"
    else
        "${exec_args[@]}" /bin/bash -lc \
            "if [ '${CONTAINER_GUI}' = '1' ]; then \
                 export DISPLAY='${DISPLAY:-:0}'; \
                 export XAUTHORITY='${XAUTH_FILE}'; \
                 export XDG_RUNTIME_DIR='${XDG_RUNTIME_DIR}'; \
                 export QT_X11_NO_MITSHM=1; \
             fi; \
             exec /bin/bash $*"
    fi
}

wait_for_container_user() {
    local tries=30

    while ((tries > 0)); do
        if docker exec "${CONTAINER_NAME}" getent passwd "${USERNAME}" >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
        ((tries--))
    done

    echo "Timed out waiting for user ${USERNAME} to be created in ${CONTAINER_NAME}."
    echo "Container logs:"
    docker logs "${CONTAINER_NAME}" || true
    return 1
}

_container_env_value() {
    local key="$1"
    docker inspect --format '{{range .Config.Env}}{{println .}}{{end}}' "${CONTAINER_NAME}" 2>/dev/null \
        | awk -F= -v key="${key}" '$1 == key {print substr($0, length(key) + 2); exit}'
}

verify_existing_container_dds_env() {
    local key expected actual mismatch
    mismatch=0

    local dds_env_vars=(
        ROS_DOMAIN_ID
        ROS_LOCALHOST_ONLY
        RMW_IMPLEMENTATION
        ROS_DISCOVERY_SERVER
        FASTRTPS_DEFAULT_PROFILES_FILE
        FASTDDS_DEFAULT_PROFILES_FILE
        AUTONAV_DDS_DISCOVERY
        AUTONAV_DDS_DISCOVERY_PORT
    )

    if [[ -n "${AUTONAV_JETSON_IP:-}" ]]; then
        dds_env_vars+=(AUTONAV_JETSON_IP)
    fi
    if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
        dds_env_vars+=(CYCLONEDDS_URI)
    fi

    for key in "${dds_env_vars[@]}"; do
        expected="${!key:-}"
        actual="$(_container_env_value "${key}")"
        if [[ "${actual}" != "${expected}" ]]; then
            echo "DDS env mismatch for existing container ${CONTAINER_NAME}: ${key}" >&2
            echo "  expected: ${expected:-<unset>}" >&2
            echo "  actual:   ${actual:-<unset>}" >&2
            mismatch=1
        fi
    done

    if [[ "${mismatch}" == "1" ]]; then
        echo "ERROR: Existing container ${CONTAINER_NAME} was created with stale DDS settings." >&2
        echo "Recreate it so remote RViz discovery works:" >&2
        echo "  docker rm -f ${CONTAINER_NAME}" >&2
        echo "  AUTONAV_JETSON_IP=<reachable-jetson-ip> ./env/docker/run-container.sh" >&2
        return 1
    fi
}

ensure_fastdds_discovery_server() {
    [[ "${AUTONAV_DDS_DISCOVERY}" == "server" ]] || return 0

    local server_host="${AUTONAV_DISCOVERY_SERVER_HOST}"
    local server_port="${AUTONAV_DISCOVERY_SERVER_PORT}"

    if ! docker exec -u "${USERNAME}" "${CONTAINER_NAME}" /bin/bash -lc \
        "source /opt/ros/humble/setup.bash && command -v fastdds >/dev/null 2>&1"; then
        echo "ERROR: fastdds CLI is not available in ${CONTAINER_NAME} after sourcing ROS 2." >&2
        echo "Install/provide the Fast DDS CLI or use AUTONAV_DDS_DISCOVERY=simple." >&2
        return 1
    fi

    if docker exec -u "${USERNAME}" \
        -e "AUTONAV_DISCOVERY_SERVER_PORT=${server_port}" \
        "${CONTAINER_NAME}" /bin/bash -lc \
        "ps -eo args= | grep '[f]astdds discovery' | grep -q -- \"-p \${AUTONAV_DISCOVERY_SERVER_PORT}\""; then
        echo "Fast DDS discovery server already running on UDP ${server_port}."
        return 0
    fi

    echo "Starting Fast DDS discovery server on ${server_host}:${server_port}."
    docker exec -d -u "${USERNAME}" \
        -e "AUTONAV_DISCOVERY_SERVER_HOST=${server_host}" \
        -e "AUTONAV_DISCOVERY_SERVER_PORT=${server_port}" \
        "${CONTAINER_NAME}" /bin/bash -lc \
        'source /opt/ros/humble/setup.bash && exec fastdds discovery -l "${AUTONAV_DISCOVERY_SERVER_HOST}" -p "${AUTONAV_DISCOVERY_SERVER_PORT}" >/tmp/autonav_fastdds_discovery.log 2>&1'
}

# RE-USE EXISTING CONTAINER
if [ "$(docker ps -a --quiet --filter status=running --filter name=^/${CONTAINER_NAME}$)" ]; then
    echo "Container $CONTAINER_NAME is already running."
    verify_existing_container_dds_env
    wait_for_container_user
    ensure_fastdds_discovery_server
    if [[ $NO_ATTACH -eq 1 ]]; then
        echo "--no-attach set; leaving container running detached."
        exit 0
    fi
    echo "Attaching..."
    attach_shell "$@"
    exit 0
fi

# Check if container exists but is stopped
if [ "$(docker ps -a --quiet --filter status=exited --filter name=^/${CONTAINER_NAME}$)" ]; then
    echo "Container $CONTAINER_NAME exists but is stopped. Starting..."
    verify_existing_container_dds_env
    docker start "$CONTAINER_NAME" >/dev/null
    wait_for_container_user
    ensure_fastdds_discovery_server
    if [[ $NO_ATTACH -eq 1 ]]; then
        echo "--no-attach set; leaving container running detached."
        exit 0
    fi
    echo "Attaching..."
    attach_shell "$@"
    exit 0
fi

# CREATE NEW CONTAINER
echo "Starting new container: $CONTAINER_NAME"
echo "Mounting: ${HOST_WORKDIR} → ${CONTAINER_WORKDIR}"

docker run -d \
    --runtime nvidia \
    --gpus all \
    --privileged \
    --ipc host \
    --device-cgroup-rule='c 13:* rmw' \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    --device=/dev/bus/usb:/dev/bus/usb \
    "${DOCKER_ARGS[@]}" \
    --name "$CONTAINER_NAME" \
    $IMAGE_TAG \
    sleep infinity >/dev/null

wait_for_container_user
ensure_fastdds_discovery_server
if [[ $NO_ATTACH -eq 1 ]]; then
    echo "--no-attach set; leaving container running detached."
    exit 0
fi
attach_shell "$@"
