#!/bin/bash
set -e

echo "run-rviz.sh: checking display/OpenGL environment"

export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-${USER:-$(id -un)}}"
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR" 2>/dev/null || true

if [[ "${DISPLAY:-}" =~ ^localhost: ]]; then
    echo "WARNING: DISPLAY=${DISPLAY} looks like SSH X11 forwarding."
    echo "RViz needs a working GLX/OpenGL context; prefer DISPLAY=:0 on the Jetson desktop."
fi

if [[ "${AUTONAV_RVIZ_SOFTWARE:-0}" == "1" ]]; then
    echo "AUTONAV_RVIZ_SOFTWARE=1 set; using Mesa software rendering."
    export LIBGL_ALWAYS_SOFTWARE=1
fi

if command -v glxinfo >/dev/null 2>&1; then
    GLX_OUTPUT="$(glxinfo -B 2>&1)" || {
        echo "ERROR: OpenGL/GLX is not working for DISPLAY=${DISPLAY:-<unset>}."
        echo "$GLX_OUTPUT"
        echo "RViz cannot create its OGRE render window until the container display/GPU path is fixed."
        if [[ "${DISPLAY:-}" =~ ^localhost: ]]; then
            echo "SSH X11 forwarding is still failing GLX. Use the Jetson desktop display via xhost, VNC, NoMachine, or another remote desktop with OpenGL support."
        elif [[ "${AUTONAV_RVIZ_SOFTWARE:-0}" != "1" ]]; then
            echo "For diagnosis only, you can try:"
            echo "  AUTONAV_RVIZ_SOFTWARE=1 $0"
        fi
        exit 1
    }
fi

NAV_PATH="$(dirname "${BASH_SOURCE[0]}")/../src/sim/config/view_bot.rviz"

rviz2 -d "$NAV_PATH"
