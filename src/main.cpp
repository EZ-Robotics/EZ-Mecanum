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

  ez::as::auton_selector.add_autons({
      Auton("Normal PID Example\n\nNormal drive PID", normal_pid_example),
      Auton("Odom Normal PID\n\n'Normal' PID but it's actually odom", odom_normal_example),
      Auton("Move to Point Hold Angle\n\nDrive diagonally and hold an angle", odom_hold_angle_example),
      Auton("Move to Point Fast\n\nDrive diagonally but face target", odom_fast_move_example),
      Auton("Pp Square\n\nPure Pursuit through a square", pp_example),
      Auton("Injected Pp Square\n\nPure Pursuit through a square, \nbut injected.", injected_pp_example),
      Auton("Smooth Pp Square\n\nPure Pursuit through a square, \nbut smooth.", smooth_pp_example),
      Auton("Smooth Pp Square Wait Until\n\nPure Pursuit through a square,\nbut smooth,\nbut run the intake.", wait_until_pp),
  });

  ez::as::initialize();
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

  ez::as::auton_selector.call_selected_auton();

  /*
  smooth_pure_pursuit(
      {{{-6, 24, 50}, LOOK_AT_TARGET_FWD},
       {{36, 36, 50}, HOLD_ANGLE, 60},
       {{36, 30, 50}, HOLD_ANGLE},
       {{0, 0, 0}, FAST_MOVE_REV}},
      0.1, 0.005, 0.0001);

  pp_wait_until(2);
  set_intake(127);
  pp_wait_until(3);
  set_intake(0);

  wait_drive();
  */
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