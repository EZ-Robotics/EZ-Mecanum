/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

#include "drive/set_pid.hpp"
#include "util/util.hpp"

// This occurs as soon as the program starts.
void initialize() {
  pros::delay(300);

  imu.reset();
  pros::delay(2000);
  master.rumble(".");

  reset_trackers();
  reset_odom();

  set_pid_defaults();

  pros::lcd::initialize();
  pros::lcd::set_background_color(255, 110, 199);  // ez pink
}

// Runs while the robot is in disabled at on a competition.
void disabled() {}

// Runs after initialize(), and before autonomous when connected to a competition.
void competition_initialize() {}

// Runs the user autonomous code.
void autonomous() {
  reset_trackers();
  reset_odom();
  reset_pid_targets();
  drive_brake(MOTOR_BRAKE_HOLD);

  smooth_pure_pursuit(
      {{{-6, 24, 50}, LOOK_AT_TARGET_FWD},
       {{36, 24, 50}, HOLD_ANGLE, 60},
       {{36, 18, 50}, HOLD_ANGLE},
       {{0, 0, 0}, FAST_MOVE_REV}},
      0.1, 0.005, 0.0001);

  pp_wait_until(2);
  set_intake(127);
  pp_wait_until(3);
  set_intake(0);

  wait_drive();
}

// Runs the operator control code.
void opcontrol() {
  // Drive brake, this is preference
  drive_brake(MOTOR_BRAKE_HOLD);

  while (true) {
    flywheel_opcontrol();
    joystick_control();
    indexer_opcontrol();
    intake_opcontrol();

    pros::delay(DELAY_TIME);
  }
}