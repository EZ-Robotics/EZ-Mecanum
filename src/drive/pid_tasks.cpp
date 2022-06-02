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
  // Comute turn PID and find shortest path to angle
  turnPID.set_target(relative_angle_to_point(target.theta));
  turnPID.compute(0);

  // Clip outpout power
  int turn_out = clip_num(turnPID.output, 110, -110);

  // Set motors
  set_left(turn_out);
  set_right(-turn_out);
}

void point_to_point() {
  // Set angle target
  // if distance to target is less then some value and fast move is on
  //   set angle to target.theta
  // else
  //   set angle to face point

  // Compute PID
  xPID.compute(current.x);
  yPID.compute(current.y);

  // Comute angle PID and find shortest path to angle
  aPID.set_target(relative_angle_to_point(absolute_angle_to_point(target.x, target.y)));
  aPID.compute(0);

  // Vector math
  double angle = to_rad(get_angle());
  double x_raw_output = (xPID.output * cos(angle)) - (yPID.output * sin(angle));
  double y_raw_output = (yPID.output * cos(angle)) + (xPID.output * sin(angle));
  double a_raw_output = aPID.output;

  // Set output powers
  int x_output = x_raw_output;
  int y_output = y_raw_output;
  int a_output = clip_num(a_raw_output, 60, -60);
  int max_xy = 110;

  // Vector scaling
  if (fabs(x_raw_output) > max_xy || fabs(y_raw_output) > max_xy) {
    if (fabs(x_raw_output) > fabs(y_raw_output)) {
      double scale = max_xy / fabs(x_raw_output);
      x_output = clip_num(x_raw_output, max_xy, -max_xy);
      y_output = y_raw_output * scale;
    } else {
      double scale = max_xy / fabs(y_raw_output);
      x_output = x_raw_output * scale;
      y_output = clip_num(y_raw_output, max_xy, -max_xy);
    }
  }

  // Set motors
  raw_set_drive(x_output, y_output, a_output);
}
