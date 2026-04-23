# Remote RViz Workflow

The Jetson container now defaults to a headless ROS 2 runtime. RViz is expected
to run natively on the laptop and join the same ROS graph over DDS.

## Jetson

Start the container headless:

```bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./env/docker/run-container.sh
```

`AUTONAV_CONTAINER_GUI=0` is the default. If you need the old X11 path for a
specific debugging session, opt back in explicitly:

```bash
AUTONAV_CONTAINER_GUI=1 ./env/docker/run-container.sh
```

Inside the container, launch the normal ROS 2 stack as usual.

## Laptop

Install native ROS 2 Humble + `rviz2` on the laptop. Then run RViz with the
same DDS settings used on the Jetson:

```bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./isaac_ros-dev/config/run-rviz.sh
```

If you are forcing a middleware implementation, use the same
`RMW_IMPLEMENTATION` value on both machines.

## DDS Notes

- Both machines must be on the same reachable network or VPN.
- `ROS_DOMAIN_ID` must match on the Jetson and the laptop.
- `ROS_LOCALHOST_ONLY` must stay `0`, otherwise discovery will be limited to the
  local machine.
- If you use custom DDS discovery settings, export the same relevant variables
  before launching the container and RViz. The container launcher now forwards:
  `RMW_IMPLEMENTATION`, `ROS_DISCOVERY_SERVER`,
  `FASTDDS_DEFAULT_PROFILES_FILE`, and `CYCLONEDDS_URI`.

## Applying The Change

If an older X11-based `koopa-kingdom` container is already running, restart it
so the new environment and mounts take effect:

```bash
docker rm -f koopa-kingdom
```
