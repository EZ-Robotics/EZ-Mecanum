/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

void reset_pid_targets() {
  headingPID.set_target(0);
  leftPID.set_target(0);
  rightPID.set_target(0);
  forwardPID.set_target(0);
  backwardPID.set_target(0);
  turnPID.set_target(0);
}

void set_drive_pid(double target, int speed, bool slew_on, bool toggle_heading) {
  // Print targets
  printf("Drive Started... Target Value: %f ", target);
  if (slew_on) printf(" with slew");
  printf("\n");

  bool is_backwards = false;

  reset_trackers();

  // Figure out if going forward or backward
  if (target < 0) {
    auto consts = backwardPID.get_constants();
    leftPID.set_constants(consts.kp, consts.ki, consts.kd, consts.start_i);
    rightPID.set_constants(consts.kp, consts.ki, consts.kd, consts.start_i);
    is_backwards = true;
  } else {
    auto consts = forwardPID.get_constants();
    leftPID.set_constants(consts.kp, consts.ki, consts.kd, consts.start_i);
    rightPID.set_constants(consts.kp, consts.ki, consts.kd, consts.start_i);
    is_backwards = false;
  }

  // Set targets
  leftPID.set_target(target);
  rightPID.set_target(target);

  mode = DRIVE;
}

void set_turn_pid(double target, int speed) {
  // Print targets
  printf("Turn Started... Target Value: %f ", target);
  printf("\n");

  headingPID.set_target(target);
  turnPID.set_target(target);

  mode = TURN;
}