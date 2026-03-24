Here is the step by step guide on what to bringup:

1. Pre-slam
    `ros2 launch bringup pre_slam.launch.py`
2. ZED
    `./config/run-zed.sh`
3. SICK
    `./config/run-lidar.sh`
4. SLAM
    `ros2 launch slam slam.launch.py`
5. Line Detection
    `./config/run-lines.sh`
6. NAV2
    `./config/run-nav2.sh`
7. RVIZ
    `./config/run-rviz.sh`

    Once in RVIZ, make sure to set the base_frame to 'map' and add the needed visual mpas, frames, etc.


Ideally, you won't need to even open RVIZ, if you set the **Goal Pose** already in the code, or even better if you just use one of NAthans test scripts that should set the goal pose and bringup everything listed above. Double check with Nathan. 

Experimental:

I wrote a script `./config/run-full-bringup.sh` that will do steps 1-5 automatically, then you only need to bringup NAV2 and rviz if necessary.

Needs to be tested, but could be nice. Nathan's script is def better.

GOOD LUCK TEAM!