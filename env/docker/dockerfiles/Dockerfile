FROM autonav:koopa-kingdom

USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    gosu \
    bluez \
    bluez-tools \
    dbus \
    evtest \
    joystick \
    python3-numpy \
    python3-numpy-dev \
    python3-dev \
    ros-humble-robot-localization \
    ros-humble-zed-msgs \
    udev \
    nano \
    iproute2 \
 && rm -rf /var/lib/apt/lists/*

# Pin numpy<2 so pip doesn't upgrade past the 1.x ABI that
# the system python3-opencv / cv_bridge were compiled against.
RUN python3 -m pip install --no-cache-dir pyserial "numpy<2"
RUN NUMPY_INCLUDE="$(python3 -c 'import numpy; print(numpy.get_include())')" \
 && test -f "${NUMPY_INCLUDE}/numpy/ndarrayobject.h" \
 && ln -sfn "${NUMPY_INCLUDE}/numpy" /usr/local/include/numpy \
 && printf '#include <numpy/ndarrayobject.h>\n' \
    | cc -E $(python3-config --includes) -x c - >/dev/null

ENV CMAKE_PREFIX_PATH=/usr/local/zed \
    CPATH=/usr/local/include:/usr/local/zed/include \
    C_INCLUDE_PATH=/usr/local/include \
    CPLUS_INCLUDE_PATH=/usr/local/include:/usr/local/zed/include

RUN mkdir -p /usr/local/zed/lib/cmake/ZED && \
    ln -sfn /usr/local/zed/lib /usr/local/zed/lib/cmake/ZED/lib

RUN chmod -R a+rX /usr/local/zed || true

RUN rm -rf /autonav/isaac_ros-dev/install \
           /autonav/isaac_ros-dev/build \
           /autonav/isaac_ros-dev/log

# Create workspace directory
RUN mkdir -p /workspace

WORKDIR /workspace
