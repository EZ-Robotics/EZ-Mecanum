/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

void set_pid_defaults() {
  // These are used for odometry motions
  xPID.set_exit_condition(300, 1, 625, 3, 750, 750);
  yPID.set_exit_condition(300, 1, 625, 3, 750, 750);
  aPID.set_exit_condition(100, 3, 500, 7, 500, 500);
  xPID.set_constants(22.5, 0, 250, 0);
  yPID.set_constants(22.5, 0, 250, 0);
  aPID.set_constants(5, 0, 35, 0);

  // These are for ez-template style drive pid and can be ignored for odom
  auto consts = xPID.get_constants();
  leftPID.set_constants(consts.kp, consts.ki, consts.kd, consts.start_i);
  rightPID.set_constants(consts.kp, consts.ki, consts.kd, consts.start_i);
  leftPID.set_exit_condition(300, 1, 625, 3, 750, 750);
  rightPID.set_exit_condition(300, 1, 625, 3, 750, 750);
  headingPID.set_constants(11, 0, 20, 0);
}