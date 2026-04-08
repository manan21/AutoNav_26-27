### Manual operation of the Bowser:

1. Turn the Bowser on
2. Make sure to connect the motor controller USB, then Arduino USB
3. Connect the USB-C to the jetson and USB-A to the laptop
4. On the laptop, press `Ctrl + Alt + T` to open the terminal
5. Turn on the X-BOX joystick by pressing on the middle button (light should be white and still)
6. Now paste the following commands 1 by 1:
    * `ssh jetson`
    * `./AutoNav_25-26/env/docker/run-container.sh`
    * WAIT UNTIL THE CONTAINER BOOTS UP
    * `ros2 launch control control_dev.launch.py`
7. Wait for the node to boot up (Arduino connected, Motor Controller connected, Joystick connected)
9. You are ready to operate the robot.

#### Controls:
* Left joystick -> Left side of the robot
* Right joystick -> RIght side of the robot
* RB -> Increase speed (!!!BE CAREFUL!!!)
* LB -> Decrease speed
* A -> Starts Autonomous mode
* B -> E-Stop (Have to reboot the robot if pressed)