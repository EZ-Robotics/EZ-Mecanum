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
      case PURE_PURSUIT:
        pure_pursuit();
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
  headingPID.compute(imu.get_rotation());

  // Clip output power
  int l_drive_out = clip_num(leftPID.output, max_xy, -max_xy);
  int r_drive_out = clip_num(rightPID.output, max_xy, -max_xy);
  int h_out = clip_num(headingPID.output, 127, -127);

  // Set motors
  set_left(l_drive_out + h_out);
  set_right(r_drive_out - h_out);
}

void turn_pid_task() {
  // Comute turn PID and find shortest path to angle
  //aPID.compute(imu.get_rotation());
  aPID.compute(imu.get_rotation());

  // Clip outpout power
  int turn_out = clip_num(aPID.output, max_a, -max_a);

  // Set motors
  set_left(turn_out);
  set_right(-turn_out);
}

void point_to_point() {
  // Add for direction
  int add = current_turn_type == FAST_MOVE_REV || current_turn_type == LOOK_AT_TARGET_REV ? 180 : 0;

  // Set angle target
  switch (current_turn_type) {
    // Looks at target until final distance then goes to final angle
    case FAST_MOVE_FWD:
    case FAST_MOVE_REV:
      // if (fabs(distance_to_point(target, current)) < clip_num(relative_angle * (TURN_FAST_MOVE / 90.0), 9.0, -9.0)) {
      if (fabs(distance_to_point(target, current)) < TURN_FAST_MOVE) {
        a_target = target.theta;
      } else {
        a_target = absolute_angle_to_point(target, current) + add;
      }
      break;
    // Looks at target the entire motion
    case LOOK_AT_TARGET_FWD:
    case LOOK_AT_TARGET_REV:
      if (fabs(distance_to_point(target, current)) > STOP_UPDATING_ANGLE) {
        a_target = absolute_angle_to_point(target, current) + add;
      }
      break;
    // Holds angle the entire motion
    case HOLD_ANGLE:
      a_target = target.theta;
    default:
      break;
  }

  // Comute angle PID and find shortest path to angle
  aPID.set_target(relative_angle_to_point(a_target));
  aPID.compute(0);

  // Compute PID
  xPID.compute(current.x);
  yPID.compute(current.y);

  // Vector math
  double angle = to_rad(get_angle());
  double x_raw_output = (xPID.output * cos(angle)) - (yPID.output * sin(angle)) * VECTOR_SCALING;
  double y_raw_output = (yPID.output * cos(angle)) + (xPID.output * sin(angle));
  double a_raw_output = aPID.output;

  // Set output powers
  int x_output = x_raw_output;
  int y_output = y_raw_output;
  int a_output = clip_num(a_raw_output, max_a, -max_a);

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

  printf("a: %f  x: %f  y: %f      x(%f, %f)\n", aPID.target, xPID.error, yPID.error, target.x, current.x);

  // Set motors
  raw_set_drive(x_output, y_output, a_output);
}

void pure_pursuit() {
  raw_move_odom(movements[pp_index]);

  if (fabs(distance_to_point(movements[pp_index].target, current)) < LOOK_AHEAD) {
    pp_index++;
    if (pp_index >= movements.size()) {
      pp_index = movements.size() - 1;
    }
  }

  point_to_point();
}