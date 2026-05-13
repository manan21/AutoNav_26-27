# Remote RViz Workflow

The Jetson container now defaults to a headless ROS 2 runtime. RViz is expected
to run natively on the laptop and join the same ROS graph over DDS. The default
path uses a Fast DDS discovery server on the Jetson; it is not a custom RViz UDP
server.

## Jetson

Start the container headless. Set `AUTONAV_JETSON_IP` to the Jetson address that
the laptop can reach over the robot network:

```bash
AUTONAV_JETSON_IP=<reachable-jetson-ip> ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./env/docker/run-container.sh
```

In default `AUTONAV_DDS_DISCOVERY=server` mode, the launcher exports
`ROS_DISCOVERY_SERVER=<reachable-jetson-ip>:11811` into the container and starts
one `fastdds discovery` process listening on UDP port `11811`.

`AUTONAV_CONTAINER_GUI=0` is the default. If you need the old X11 path for a
specific debugging session, opt back in explicitly:

```bash
AUTONAV_JETSON_IP=<reachable-jetson-ip> AUTONAV_CONTAINER_GUI=1 ./env/docker/run-container.sh
```

Inside the container, launch the normal ROS 2 stack as usual.

## Laptop

Install native ROS 2 Humble + `rviz2` on the laptop. Then run RViz with the
same DDS settings used on the Jetson. The launcher uses the same default Fast
DDS UDP profile and discovery-server address as the container launcher:

```bash
AUTONAV_JETSON_IP=<reachable-jetson-ip> ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./isaac_ros-dev/config/run-rviz.sh
```

If you are forcing a middleware implementation, use the same
`RMW_IMPLEMENTATION` value on both machines.

If you need a non-default port, set `AUTONAV_DDS_DISCOVERY_PORT` on both
machines. If you set `ROS_DISCOVERY_SERVER` directly, it takes precedence over
`AUTONAV_JETSON_IP`.

## DDS Notes

- Both machines must be on the same reachable network or VPN.
- `ROS_DOMAIN_ID` must match on the Jetson and the laptop.
- `ROS_LOCALHOST_ONLY` must stay `0`, otherwise discovery will be limited to the
  local machine.
- By default, the launchers set `AUTONAV_DDS_DISCOVERY=server`,
  `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, and load
  `env/docker/fastdds_udp.xml` through both `FASTRTPS_DEFAULT_PROFILES_FILE`
  and `FASTDDS_DEFAULT_PROFILES_FILE`. This forces Fast DDS to use UDPv4 rather
  than host-local shared memory transports.
- `AUTONAV_DDS_DISCOVERY=simple` restores the previous non-server behavior for
  networks where ordinary DDS multicast/simple discovery already works.
- If you use custom DDS discovery settings, export the same relevant variables
  before launching the container and RViz. The container launcher forwards:
  `RMW_IMPLEMENTATION`, `ROS_DISCOVERY_SERVER`, `FASTRTPS_DEFAULT_PROFILES_FILE`,
  `FASTDDS_DEFAULT_PROFILES_FILE`, `CYCLONEDDS_URI`, `AUTONAV_DDS_DISCOVERY`,
  `AUTONAV_DDS_DISCOVERY_PORT`, and `AUTONAV_JETSON_IP`.

## Applying The Change

If an older `koopa-kingdom` container was created before the discovery-server
environment was added, recreate it so the new DDS settings take effect:

```bash
docker rm -f koopa-kingdom
```
