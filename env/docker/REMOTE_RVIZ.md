# Remote RViz Workflow

The Jetson container now defaults to a headless ROS 2 runtime. RViz is expected
to run natively on the laptop and join the same ROS graph over DDS.

## Jetson

Start the container headless. The launcher defaults to Fast DDS over UDP only:

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
same DDS settings used on the Jetson. The launcher uses the same default Fast
DDS UDP profile as the container launcher:

```bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./isaac_ros-dev/config/run-rviz.sh
```

## No-Wi-Fi USB-C Mode

When there is no usable Wi-Fi, run RViz on the Jetson and forward the RViz
window back to the laptop over the USB-C SSH session. This keeps ROS/DDS local
to the Jetson instead of depending on network discovery.

From the laptop:

```bash
ssh -Y jetson
cd AutoNav_25-26
./env/docker/run-container.sh --no-attach
./isaac_ros-dev/config/run-rviz.sh
```

The same launcher works in three places:

- Laptop with Wi-Fi: native laptop RViz joins the Jetson DDS graph.
- Jetson host over USB-C SSH: native Jetson RViz if `rviz2` is installed.
- Jetson host over USB-C SSH without host RViz: automatic `docker exec` into
  `koopa-kingdom` and forwards the SSH display into the container.

To force container RViz on the Jetson, run:

```bash
./isaac_ros-dev/config/run-rviz.sh --container
```

If you are forcing a middleware implementation, use the same
`RMW_IMPLEMENTATION` value on both machines.

If you use non-default DDS discovery or interface config, export the same
variables before launching RViz on the laptop as well. The RViz launcher now
passes through `ROS_DISCOVERY_SERVER`, `FASTRTPS_DEFAULT_PROFILES_FILE`,
`FASTDDS_DEFAULT_PROFILES_FILE`, and `CYCLONEDDS_URI` in addition to
`RMW_IMPLEMENTATION`.

## DDS Notes

- Both machines must be on the same reachable network or VPN.
- `ROS_DOMAIN_ID` must match on the Jetson and the laptop.
- `ROS_LOCALHOST_ONLY` must stay `0`, otherwise discovery will be limited to the
  local machine.
- By default, both launchers set `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and load
  `env/docker/fastdds_udp.xml` through both `FASTRTPS_DEFAULT_PROFILES_FILE`
  and `FASTDDS_DEFAULT_PROFILES_FILE`. This forces Fast DDS to use UDPv4 rather
  than host-local shared memory transports.
- If you use custom DDS discovery settings, export the same relevant variables
  before launching the container and RViz. The container launcher now forwards:
  `RMW_IMPLEMENTATION`, `ROS_DISCOVERY_SERVER`, `FASTRTPS_DEFAULT_PROFILES_FILE`,
  `FASTDDS_DEFAULT_PROFILES_FILE`, and `CYCLONEDDS_URI`.

## Applying The Change

If an older X11-based `koopa-kingdom` container is already running, restart it
so the new environment and mounts take effect:

```bash
docker rm -f koopa-kingdom
```
