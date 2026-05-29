# Launch Stack

The **GUI launch panel is the stack** — whatever buttons it runs are
what we call the "full bringup," whether for a demo, a competition
run, or routine testing. The manual commands below are just what
each GUI button executes under the hood; they're useful for
isolation debugging or as a fallback if the GUI is unavailable, but
for normal operation you click the buttons.

The GUI gives you status dots, the terminal viewer, and queue pacing
(each device waits for a `[GUI_READY] <label>` sentinel on stdout
before the next one starts).

## Bringup order

The buttons appear in the launch panel in this order — clicking
**Run All** queues them in this order, and the order matters (see
the dependency notes after the table).

| # | GUI button | Status dot(s) lit | What it runs | Run All? |
|---|---|---|---|---|
| 1 | **Pre-SLAM** | Encoders + CONTROL | `./config/run-pre-slam.sh` | ✓ |
| 2 | **Camera** | Camera | `./config/run-zed.sh` | ✓ |
| 3 | **Lidar** | Lidar | `./config/run-lidar.sh` | ✓ |
| 4 | **GPS** | GPS | `./config/run-gps.sh` | ✓ |
| 5 | **PCA DETECT** | PCA DETECT | `./config/run-pca.sh` | ✓ |
| 6 | **CAMERA LINE DETECT** | CAMERA LINE DETECT | `./config/run-lines.sh` | ✓ |
| 7 | **LIDAR LINE DETECT** | LIDAR LINE DETECT | `./config/run-lidar-lines.sh` | **excluded** |
| 8 | **SLAM** | SLAM | `ros2 launch slam slam.launch.py` | ✓ |
| 9 | **NAV2** | NAV2 | `./config/run-nav2.sh` | ✓ |
| 10 | **Power PCB** | Power PCB | `./config/run-electrical.sh` | ✓ |

### Why this order

- **Pre-SLAM** brings up the encoders + control stack first so wheel
  odometry is publishing before anything that needs it (EKF, SLAM).
- **PCA DETECT comes before SLAM.**
  `slam_toolbox` itself subscribes to the raw `/scan_fullframe`, but
  `slam.launch.py` also starts the PCA PointCloud2-to-LaserScan
  converters that publish `/scan_pca_filtered` and
  `/scan_pca_filtered_clear` for Nav2's obstacle layer. Starting PCA
  before SLAM means those converter outputs are live before Nav2
  starts.
- **CAMERA LINE DETECT is the default line source.** The active
  `nav2_params_camera.yaml` profile consumes `/line_points`, publishes
  `/line_costmap`, and mirrors camera-line memory into the global costmap.
- **LIDAR LINE DETECT is opt-in.** It's an alternate / supplementary
  line-detection source running off the LiDAR. Clicking **Run All**
  skips it (`_launch_all_excluded`); click it directly when you want
  the LiDAR line stream. Pair it with `nav2_params_lidar.yaml` for the
  measured lidar-line avoidance regression or any retroreflective-tape
  test that expects RSSI line points to enter Nav2.
- **NAV2 last among nav components** because it depends on SLAM
  producing the `map→odom` TF.
- **Power PCB** can technically come up at any time; it's last only
  because nothing else depends on it.

## Pacing

The `run-*.sh` scripts emit `[GUI_READY] <Label>` **0.5 s** after
launching the underlying `ros2 launch` (a `sleep 0.5` hard-coded in
each script). The GUI flips the dot green as soon as it sees that
sentinel — so the script-side pacing is half a second.

What you actually *see* between clicking a button and the dot turning
green is dominated by ROS process startup (sensor driver init, node
graph wiring, first-message latency) — typically 2-10 s depending on
the device. **Run All** queues 9 of the 10 subsystems (LIDAR LINE
DETECT is excluded), so expect roughly **30-60 seconds** for the
panel to come up, hardware-dependent. If a device blows past its
deadline in `_ready_timeouts` (45 s for camera/lidar, 30 s for power
PCB, etc.) the GUI marks it failed but keeps the underlying process
running so you can read its logs in the terminal viewer.

## RViz

RViz runs **locally on your own Linux laptop** (not in the container)
— see the main `README.md` for the `ROS_DOMAIN_ID` setup. Once it's
open, set the base frame to `map` and add the visualizations you
need (TF, costmaps, point clouds).

You don't strictly need RViz on demo day — the GUI shows the same
costmap, path, robot pose, and GPS map that you'd otherwise be
watching in RViz.

## Going autonomous

Once the launch panel is all green:

1. **Send a goal from the GUI.** The main page has two text fields:
   - **Send Goal Args** → runs `./config/send_goal.sh` (map-frame pose)
   - **Send GPS Args** → runs `./config/send_GPS_waypoint.sh` (lat/lon)

   Type the args, click **Send** (or press Enter). The manual
   command line equivalent is `./config/send_goal.sh <args>` /
   `./config/send_GPS_waypoint.sh <args>` inside the container.

2. **Rotate the e-stop to disengage, then power-cycle the robot.**

3. **Press X on the Xbox controller** to enter autonomous mode.
   The corner badge on the GUI flips to AUTO when the robot accepts
   the toggle (the GUI mirrors `/autonomous_mode`, which control.cpp
   publishes whenever the state actually changes).

### Operator shortcut — load a goal, then engage AUTO

A common workflow is: load up a goal first, *then* press X. The GUI
will not cancel a pending goal when you flip AUTO off and back on —
the goal stays queued. This is deliberate; "load and engage" is the
intended flow.

---

## Original notes from the team (preserved)

> Ideally, you won't need to even open RVIZ, if you set the **Goal Pose** already in the code, or even better if you just use one of NAthans test scripts that should set the goal pose and bringup everything listed above. Double check with Nathan.
>
> Experimental:
>
> I wrote a script `./config/run-full-bringup.sh` that will do steps 1-5 automatically, then you only need to bringup NAV2 and rviz if necessary.
>
> Needs to be tested, but could be nice. Nathan's script is def better.
>
> — **Tamir**

*Note: "Nathan's script" is what eventually became the GUI launch panel described above. The experimental `run-full-bringup.sh` never made it into the repo — the GUI superseded it.*

---

GOOD LUCK TEAM!
