/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

#include "autons.hpp"
#include "drive/exit_conditions.hpp"
#include "drive/purepursuit_math.hpp"
#include "drive/set_pid.hpp"
#include "drive/tracking.hpp"
#include "util/util.hpp"

// This occurs as soon as the program starts.
void initialize() {
  pros::delay(300);

  trackingTask.suspend();

  imu.reset();
  pros::delay(2000);
  master.rumble(".");

  reset_trackers();
  reset_odom();

  set_pid_defaults();

  ez::as::auton_selector.add_autons({
      Auton("Full Field Test Path\n\nSmooth PP testing", smooth_pp_testing),
      Auton("Normal PID Example\n\nNormal drive PID", normal_pid_example),
      Auton("Odom Normal PID\n\n'Normal' PID but it's actually odom", odom_normal_example),
      Auton("Move to Point Hold Angle\n\nDrive diagonally and hold an angle", odom_hold_angle_example),
      Auton("Move to Point Fast\n\nDrive diagonally but face target", odom_fast_move_example),
      Auton("Pp Square\n\nPure Pursuit through a square", pp_example),
      Auton("Injected Pp Square\n\nPure Pursuit through a square, \nbut injected.", injected_pp_example),
      Auton("Smooth Pp Square\n\nPure Pursuit through a square, \nbut smooth.", smooth_pp_example),
      Auton("Smooth Pp Square Wait Until\n\nPure Pursuit through a square,\nbut smooth,\nbut run the intake.", wait_until_pp),
  });
  /*
    std::vector<odom> path =
        smooth_path(inject_points(
                        {{{15, 10, 45}, HOLD_ANGLE, 40},
                         {{26, 21, 45}, HOLD_ANGLE, 20},
                         {{30, 25, 45}, HOLD_ANGLE, 20}}),
                    0.3, 0.005, 0.0001);
    print_path_for_python(path);
    */

  ez::as::initialize();
}

// Runs while the robot is in disabled at on a competition.
void disabled() { trackingTask.suspend(); }

// Runs after initialize(), and before autonomous when connected to a competition.
void competition_initialize() { trackingTask.suspend(); }

// Runs the user autonomous code.
void autonomous() {
  reset_trackers();
  reset_odom();
  reset_pid_targets();
  drive_brake(MOTOR_BRAKE_HOLD);
  trackingTask.resume();

  targetRPM = 3000;

  smooth_pure_pursuit(
      {{{15, 9, 45}, HOLD_ANGLE},
       {{20, 8, 45}, HOLD_ANGLE, 40},
       {{25, 13, 45}, HOLD_ANGLE, 20},
       {{40, 28, 45}, HOLD_ANGLE, 20},
       {{36, 12, 157}, HOLD_ANGLE}});

  set_intake(127);

  wait_drive();

  while (getRPM() <= targetRPM || indexer_on) pros::delay(10);
  fire_indexer();
  while (getRPM() <= targetRPM || indexer_on) pros::delay(10);
  fire_indexer();
  while (getRPM() <= targetRPM || indexer_on) pros::delay(10);
  fire_indexer();

  /*
  set_intake(-127);
  wait_drive();
  set_intake(127);
  pros::delay(500);
  */

  /*
  smooth_pure_pursuit(
      {{{18, 12, 90}, HOLD_ANGLE},
       {{36, 12, 90}, HOLD_ANGLE},
       {{36, 12, 160}, FAST_MOVE_FWD}});

  wait_drive();
  */

  // ez::as::auton_selector.call_selected_auton();
  /*
  smooth_pure_pursuit(
      {{{0, 24, 0}, LOOK_AT_TARGET_FWD},
       {{24, 24, 0}, LOOK_AT_TARGET_FWD},
       {{24, 48, 0}, FAST_MOVE_FWD}});
       */

  /*
    smooth_pure_pursuit(
      {{{24, 40, 0}, LOOK_AT_TARGET_FWD},
       {{24, 48, 0}, FAST_MOVE_FWD}});
       */

  /*
  smooth_pure_pursuit(
      {{{-6, 24, 50}, LOOK_AT_TARGET_FWD},
       {{36, 36, 50}, HOLD_ANGLE},
       {{36, 30, 50}, HOLD_ANGLE},
       {{0, 0, 0}, FAST_MOVE_REV}},
      KINDA_SMOOTH);

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

  trackingTask.resume();

  while (true) {
    flywheel_opcontrol();
    joystick_control();
    // lucas_joystick_control();
    indexer_opcontrol();
    intake_opcontrol();

    pros::delay(DELAY_TIME);
  }
}