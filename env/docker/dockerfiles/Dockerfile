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
RUN python3 -c "import os, numpy; header=os.path.join(numpy.get_include(), 'numpy', 'ndarrayobject.h'); assert os.path.exists(header), header; print(header)"

ENV CMAKE_PREFIX_PATH=/usr/local/zed \
    CPATH=/usr/local/zed/include \
    CPLUS_INCLUDE_PATH=/usr/local/zed/include

RUN mkdir -p /usr/local/zed/lib/cmake/ZED && \
    ln -sfn /usr/local/zed/lib /usr/local/zed/lib/cmake/ZED/lib

RUN chmod -R a+rX /usr/local/zed || true

RUN rm -rf /autonav/isaac_ros-dev/install \
           /autonav/isaac_ros-dev/build \
           /autonav/isaac_ros-dev/log

# Create workspace directory
RUN mkdir -p /workspace

WORKDIR /workspace
