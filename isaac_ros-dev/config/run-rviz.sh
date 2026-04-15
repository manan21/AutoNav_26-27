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

if command -v glxinfo >/dev/null 2>&1; then
    if ! glxinfo -B >/dev/null 2>&1; then
        echo "ERROR: OpenGL/GLX is not working for DISPLAY=${DISPLAY:-<unset>}."
        echo "RViz cannot create its OGRE render window until the container display/GPU path is fixed."
        echo "Try re-entering the container with env/docker/run-container.sh, or use:"
        echo "  LIBGL_ALWAYS_SOFTWARE=1 $0"
        exit 1
    fi
fi

NAV_PATH="$(dirname "${BASH_SOURCE[0]}")/../src/sim/config/view_bot.rviz"

rviz2 -d "$NAV_PATH"
