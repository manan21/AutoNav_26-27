
#include "xbox.hpp"

Xbox::Xbox() 
  : b_button_state(false), x_button_state(false), y_button_state(false),
    right_bumper_state(false), left_bumper_state(false),
    left_stick_x_pos(0.0f), left_stick_y_pos(0.0f), 
    right_stick_x_pos(0.0f), right_stick_y_pos(0.0f){

}

void Xbox::set_b(bool state){
  b_button_state = state;
}

void Xbox::set_x(bool state){
  x_button_state = state;
}

void Xbox::set_y(bool state){
  y_button_state = state;
}

void Xbox::set_right_bumper(bool state){
  right_bumper_state = state;
}

void Xbox::set_left_bumper(bool state){
  left_bumper_state = state;
}

void Xbox::set_left_stick_x(float pos) {
  left_stick_x_pos = pos;
}

void Xbox::set_left_stick_y(float pos) {
  left_stick_y_pos = pos;
}

void Xbox::set_right_stick_x(float pos) {
  right_stick_x_pos = pos;
}

void Xbox::set_right_stick_y(float pos) {
  right_stick_y_pos = pos;
}

void Xbox::adjust_joysticks(){
  if(left_stick_y_pos < 0.1 && left_stick_y_pos > -0.1){
    left_stick_y_pos = 0;
  }

  if(right_stick_y_pos < 0.1 && right_stick_y_pos > -0.1){
    right_stick_y_pos = 0;
  }

  if(left_stick_x_pos < 0.1 && left_stick_x_pos > -0.1){
    left_stick_x_pos = 0;
  }

  if(right_stick_x_pos < 0.1 && right_stick_x_pos > -0.1){
    right_stick_x_pos = 0;
  }
}

Xbox::CommandData Xbox::calculateCommand(){
  CommandData cmd;
  cmd.left_motor_speed = 0.0f;
  cmd.right_motor_speed = 0.0f;

  // B button still preempts everything — it triggers motors.shutdown()
  // which closes the serial port, so we don't want stick values applied.
  if(b_button_state){
    cmd.cmd = STOP;
    return cmd;
  }

  // Compute the stick-derived motor speeds independently of which
  // (non-STOP) command flag we end up returning. Previously a held
  // bumper short-circuited this block and zeroed the speeds, which
  // meant the wheels couldn't respond to the stick while the user
  // was simultaneously ramping the speed setpoint. Now stick motion
  // and bumper speed-change are independent inputs.
  adjust_joysticks();
  const bool sticks_zero =
      (left_stick_y_pos == 0 && right_stick_y_pos == 0);

  if(!sticks_zero){
    if(TANKDRIVE){
      cmd.left_motor_speed = left_stick_y_pos;
      cmd.right_motor_speed = right_stick_y_pos;
    } else {
      //fucking trig
      float magnitude = sqrt((left_stick_x_pos * left_stick_x_pos) + (left_stick_y_pos * left_stick_y_pos));
      float angle = atan2(left_stick_y_pos, left_stick_x_pos);
      cmd.left_motor_speed = magnitude * sin(angle + M_PI / 4);  // Adjust for robot orientation
      cmd.right_motor_speed = magnitude * cos(angle + M_PI / 4); // Adjust for robot orientation
    }
  }

  // The cmd flag tells apply_manual_command() what side-effect action
  // to take (speed bump, etc.). The motor speeds above are always
  // present, so a bumper-held stick-deflected case both bumps speed
  // AND drives the wheels.
  if(left_bumper_state){
    cmd.cmd = SPEED_UP;
  } else if(right_bumper_state){
    cmd.cmd = SPEED_DOWN;
  } else if(sticks_zero){
    cmd.cmd = NONE;
  } else {
    cmd.cmd = MOVE;
  }
  return cmd;
}

bool Xbox::switchMode(){
  if(x_button_state){
    return true;
  }
  return false;
}

