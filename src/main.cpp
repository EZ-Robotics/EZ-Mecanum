/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

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

  injected_pure_pursuit({
      {{0, 24, 90}, FAST_MOVE_FWD},
      {{24, 24, 180}, FAST_MOVE_FWD},
      {{24, 0, 270}, FAST_MOVE_FWD},
      {{0, 0, 0}, FAST_MOVE_FWD},
  });
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

    pros::delay(100);
  }
}