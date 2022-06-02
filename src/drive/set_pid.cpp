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

  xPID.set_target(0);
  yPID.set_target(0);
  aPID.set_target(0);
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

void set_turn_pid(double itarget, int speed) {
  // Print targets
  printf("Turn Started... Target Value: %f ", itarget);
  printf("\n");

  headingPID.set_target(itarget);
  turnPID.set_target(itarget);
  target.theta = itarget;

  mode = TO_POINT;
}

void move_to_point(pose itarget) {
  fast_move = false;

  target = itarget;

  xPID.set_target(target.x);
  yPID.set_target(target.y);
  aPID.set_target(target.theta);

  mode = TO_POINT;
}

void fast_to_point(pose itarget, direction idir) {
  fast_move = true;
  target = itarget;
  dir = idir;

  xPID.set_target(target.x);
  yPID.set_target(target.y);

  mode = TO_POINT;
}
