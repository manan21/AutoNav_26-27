# Demo Day Bringup

Each subsystem can be brought up **two ways** — both do the same thing:

- **GUI button** — click it in the launch panel of the AutoNav HUD.
- **Manual command** — run the script/launch directly inside the container.

The GUI is the recommended path on demo day (status dots, terminal viewer, queue pacing). The manual commands are useful when the GUI is unavailable or when you want to bring up a single subsystem in isolation for debugging.

## Bringup order

| # | GUI button | Manual command |
|---|---|---|
| 1 | **Pre-SLAM** | `./config/run-pre-slam.sh` |
| 2 | **Camera** | `./config/run-zed.sh` |
| 3 | **Lidar** | `./config/run-lidar.sh` |
| 4 | **SLAM** | `ros2 launch slam slam.launch.py` |
| 5 | **DETECT** (line + PCA grade) | `./config/run-detect.sh` |
| 6 | **NAV2** | `./config/run-nav2.sh` |
| 7 | **GPS** | `./config/run-gps.sh` |
| 8 | **Power PCB** | `./config/run-electrical.sh` |

If you go manual, run them in this order — each subsystem typically depends on the ones above it (e.g. SLAM needs `/scan_fullframe` from Lidar).

> **Heads up — pacing:** Each GUI button takes roughly **5 seconds** to flip green. The scripts deliberately wait that long before signaling ready, so the queue advances on a fixed cadence and one device's startup doesn't crowd out the next. With all 8 subsystems queued, expect ~40 seconds for the full panel to come up.

## RViz

RViz now runs **locally on your own Linux laptop** (not in the container) — see the main `README.md` for the `AUTONAV_JETSON_IP` + `ROS_DISCOVERY_SERVER` setup. Once it's open, set the base frame to `map` and add the visualizations you need (TF, costmaps, point clouds).

You don't strictly need RViz on demo day — if the **Goal Pose** is already set in code (or you're using a waypoint script), the robot can run without anyone watching RViz.

## Going autonomous

Once the launch panel is all green (or every manual command is up):

1. Send a goal — no RViz needed. Use either `./config/send_goal.sh` (map-frame pose) or `./config/send_GPS_waypoint.sh` (GPS waypoint).
2. Rotate the e-stop to disengage, then power-cycle the robot.
3. Press **X** on the Xbox controller to enter autonomous mode.

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
