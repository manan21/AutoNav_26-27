#!/usr/bin/env bash

# Shared ROS 2 DDS defaults for the Jetson container and remote RViz.
# Callers may set AUTONAV_FASTDDS_PROFILE_FILE_DEFAULT before sourcing when
# the Fast DDS profile lives at a different path inside their runtime.

_AUTONAV_DDS_HELPER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_AUTONAV_DDS_REPO_ROOT="$(cd "${_AUTONAV_DDS_HELPER_DIR}/../.." && pwd)"

AUTONAV_DDS_DISCOVERY="${AUTONAV_DDS_DISCOVERY:-server}"
AUTONAV_DDS_DISCOVERY_PORT="${AUTONAV_DDS_DISCOVERY_PORT:-11811}"

case "${AUTONAV_DDS_DISCOVERY}" in
    server|simple)
        ;;
    *)
        echo "ERROR: AUTONAV_DDS_DISCOVERY must be 'server' or 'simple'." >&2
        return 1
        ;;
esac

if [[ ! "${AUTONAV_DDS_DISCOVERY_PORT}" =~ ^[0-9]+$ ]]; then
    echo "ERROR: AUTONAV_DDS_DISCOVERY_PORT must be a numeric UDP port." >&2
    return 1
fi

AUTONAV_FASTDDS_PROFILE_FILE_DEFAULT="${AUTONAV_FASTDDS_PROFILE_FILE_DEFAULT:-${_AUTONAV_DDS_REPO_ROOT}/env/docker/fastdds_udp.xml}"

export AUTONAV_DDS_DISCOVERY
export AUTONAV_DDS_DISCOVERY_PORT
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${AUTONAV_FASTDDS_PROFILE_FILE_DEFAULT}}"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-${FASTRTPS_DEFAULT_PROFILES_FILE}}"

if [[ "${AUTONAV_DDS_DISCOVERY}" == "server" ]]; then
    if [[ -z "${ROS_DISCOVERY_SERVER:-}" ]]; then
        if [[ -z "${AUTONAV_JETSON_IP:-}" ]]; then
            echo "ERROR: AUTONAV_JETSON_IP is required when AUTONAV_DDS_DISCOVERY=server." >&2
            echo "Set AUTONAV_JETSON_IP to the Jetson address reachable from the RViz laptop, or set ROS_DISCOVERY_SERVER directly." >&2
            return 1
        fi
        export ROS_DISCOVERY_SERVER="${AUTONAV_JETSON_IP}:${AUTONAV_DDS_DISCOVERY_PORT}"
    fi

    _AUTONAV_DISCOVERY_FIRST_SERVER="${ROS_DISCOVERY_SERVER%%;*}"
    AUTONAV_DISCOVERY_SERVER_HOST="${_AUTONAV_DISCOVERY_FIRST_SERVER%%:*}"
    AUTONAV_DISCOVERY_SERVER_PORT="${_AUTONAV_DISCOVERY_FIRST_SERVER##*:}"

    if [[ -z "${AUTONAV_DISCOVERY_SERVER_HOST}" || -z "${AUTONAV_DISCOVERY_SERVER_PORT}" || "${AUTONAV_DISCOVERY_SERVER_HOST}" == "${AUTONAV_DISCOVERY_SERVER_PORT}" ]]; then
        echo "ERROR: ROS_DISCOVERY_SERVER must look like '<host>:<port>' in server mode." >&2
        return 1
    fi

    export AUTONAV_DISCOVERY_SERVER_HOST
    export AUTONAV_DISCOVERY_SERVER_PORT
fi
