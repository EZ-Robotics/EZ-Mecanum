/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"
#include "util/util.hpp"

void auto_task() {
  while (true) {
    // Autonomous PID
    switch (mode) {
      case DRIVE:
        drive_pid_task();
        break;
      case TURN:
        turn_pid_task();
        break;
      case TO_POINT:
        point_to_point();
        break;
      default:
        break;
    }

    AUTO_RAN = mode != DISABLE ? true : false;

    pros::delay(DELAY_TIME);
  }
}
pros::Task autoTask(auto_task);

void drive_pid_task() {
  // Compute PID
  leftPID.compute(get_left());
  rightPID.compute(get_right());
  headingPID.compute(get_angle());

  // Clip output power
  int l_drive_out = clip_num(leftPID.output, 110, -110);
  int r_drive_out = clip_num(rightPID.output, 110, -110);
  int h_out = clip_num(headingPID.output, 127, -127);

  // Set motors
  set_left(l_drive_out + h_out);
  set_right(r_drive_out - h_out);
}

void turn_pid_task() {
  // Compute PID
  turnPID.compute(get_angle());

  // Clip outpout power
  int turn_out = clip_num(turnPID.output, 110, -110);

  // Set motors
  set_left(turn_out);
  set_right(-turn_out);
}

void point_to_point() {
  // Compute PID
  xPID.compute(current.x);
  yPID.compute(current.y);
  aPID.compute(get_angle());

  // Vector math
  double angle = to_rad(get_angle());
  double raw_x_power = (xPID.output * cos(angle)) - (yPID.output * sin(angle));
  double raw_y_power = (yPID.output * cos(angle)) + (xPID.output * sin(angle));
  double raw_a_power = aPID.output;

  // Set output powers
  int x_output = raw_x_power;
  int y_output = raw_y_power;
  int a_output = clip_num(raw_a_power, 60, -60);
  int max_xy = 110;

  // Vector scaling 
  if (fabs(raw_x_power) > max_xy || fabs(raw_y_power) > max_xy) {
    if (fabs(raw_x_power) > fabs(raw_y_power)) {
      double scale = max_xy / fabs(raw_x_power);
      x_output = clip_num(raw_x_power, max_xy, -max_xy);
      y_output = raw_y_power * scale;
    } else {
      double scale = max_xy / fabs(raw_y_power);
      x_output = raw_x_power * scale;
      y_output = clip_num(raw_y_power, max_xy, -max_xy);
    }
  } else {
    x_output = raw_x_power;
    y_output = raw_y_power;
  }

  // Set motors
  raw_set_drive(x_output, y_output, a_output);
}