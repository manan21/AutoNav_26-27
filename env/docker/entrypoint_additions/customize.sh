#!/bin/bash

BASHRC="/home/${USERNAME}/.bashrc"
mkdir -p "/home/${USERNAME}"
touch "${BASHRC}"

if ! grep -q "bowser" "${BASHRC}"; then
  echo "PS1='${debian_chroot:+($debian_chroot)}\[\033[01;31m\]bowser\[\033[01;33m\]@\[\033[01;32m\]koopa-kingdom\[\033[00m\]:\[\033[01;33m\]\w\[\033[00m\]\$ '" >> "${BASHRC}"
fi
if ! grep -q "source /opt/ros/humble/setup.bash" "${BASHRC}"; then
  echo "source /opt/ros/humble/setup.bash" >> "${BASHRC}"
fi
if ! grep -q "source /autonav/isaac_ros-dev/install/setup.bash" "${BASHRC}"; then
  echo "source /autonav/isaac_ros-dev/install/setup.bash" >> "${BASHRC}"
fi
