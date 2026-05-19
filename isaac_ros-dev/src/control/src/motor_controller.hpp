#pragma once 

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "serialib.hpp"

class MotorController {
private:
    int stepSize = 10;
    std::pair<int, int> right_turn_speeds = {-10, -10};
    std::pair<int, int> left_turn_speeds = {10, 10};
    // Initial manual-mode "gear" = 10 so full-stick joystick deflection
    // matches the autonomous 0.25 m/s smoother cap on level ground.
    // Math: stepSize * speed = 10 * 10 = 100 per-mille = 10% RoboteQ
    // throttle, which calibrates to ~0.25 m/s (autonomous uses
    // linear_move * 40 * stepSize = 0.25 * 400 = 100 same way).
    // With this baseline, joystick + tilt-block calibration of Phase D
    // grade compensation shows multiplier directly as wheel-speed
    // delta from the level-ground baseline. Bumpers still step ±1.
    int speed = 10;
    std::string comPort;
    int prevLeftEncoderCount = 0;
    int prevRightEncoderCount = 0;
    int temp = 0;

public:
    // Constructor
    MotorController();
    serialib motorSerial;
    // configuration
    char configure(const char * port);

    // Moveeeeeee
    void forward();
    void backward();
    void turnLeft();
    void turnRight();
    void move(float right_speed, float left_speed);
    void stop();
    void shutdown();

    // Get and set
    void setStepSize(int size);
    int getStepSize();
    void setSpeed(int s);
    int getSpeed();
    int  getLeftEncoderCount();
    int getRightEncoderCount();
    int getLeftRPM();
    int getRightRPM();
};