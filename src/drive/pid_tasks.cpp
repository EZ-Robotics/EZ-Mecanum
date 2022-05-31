/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

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
      default:
        break;
    }

    AUTO_RAN = mode != DISABLE ? true : false;

    pros::delay(DELAY_TIME);
  }
}
pros::Task autoTask(auto_task);

void drive_pid_task() {
  leftPID.compute(get_left());
  rightPID.compute(get_right());
  headingPID.compute(get_angle());

  int l_drive_out = clip_num(leftPID.output, 110, -110);
  int r_drive_out = clip_num(rightPID.output, 110, -110);
  int h_out = clip_num(headingPID.output, 127, -127);

  set_left(l_drive_out + h_out);
  set_right(r_drive_out - h_out);
}

void turn_pid_task() {
  turnPID.compute(get_angle());

  int turn_out = clip_num(turnPID.output, 110, -110);

  set_left(turn_out);
  set_right(-turn_out);
}