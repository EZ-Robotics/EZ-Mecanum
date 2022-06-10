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

  imu.set_data_rate(5);
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
                      {{{15, 9, 45}, HOLD_ANGLE},
                       {{20, 8, 45}, HOLD_ANGLE, 40},
                       {{25, 13, 45}, HOLD_ANGLE, 20},
                       {{40, 28, 45}, HOLD_ANGLE, 20},
                       {{36, 12, 157}, HOLD_ANGLE}}),
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
  set_angle(0);
  trackingTask.resume();

  printf("(%f, %f, %f)\n", current.x, current.y, current.theta);

  // pure_pursuit({{{0, 24, 0}, HOLD_ANGLE}, {{24, 24, 0}, HOLD_ANGLE}, {{24, 0, 0}, HOLD_ANGLE}, {{0, 0, 0}, HOLD_ANGLE}});
  // imu_pid(360*10);

  // double width = 3.75 * 12;
  // double length = 5.5 * 12;
  double width = 3 * 12;
  double length = 4 * 12;
  double h_length = length / 2.0;
  int xy_speed = 60;
  int a_speed = 80;
  turn_types turn = HOLD_ANGLE;

  std::vector<odom> path =
      {{{width, 0, 0}, turn, xy_speed, a_speed},
       {{width, h_length, 0}, turn, xy_speed, a_speed},
       {{0, h_length, 0}, turn, xy_speed, a_speed},
       {{0, length, 0}, turn, xy_speed, a_speed},
       {{width, length, 0}, turn, xy_speed, a_speed},
       {{width, h_length, 0}, turn, xy_speed, a_speed},
       {{0, h_length, 0}, turn, xy_speed, a_speed},
       {{0, 0, 0}, turn, xy_speed, a_speed}};

  std::vector<odom> path2 =
      {{{0, length, 0}, turn, xy_speed, a_speed},
       {{width, length, 90}, turn, xy_speed, a_speed},
       {{width, h_length, 180}, turn, xy_speed, a_speed},
       {{0, h_length, -90}, turn, xy_speed, a_speed},
       {{0, 0, 180}, turn, xy_speed, a_speed}};

  std::vector<odom> path3 =
      {{{0, length, 0}, HOLD_ANGLE, xy_speed, a_speed},
       {{width, length, 0}, HOLD_ANGLE, xy_speed, a_speed},
       {{width, h_length, 0}, HOLD_ANGLE, xy_speed, a_speed},
       {{0, h_length, 0}, HOLD_ANGLE, xy_speed, a_speed},
       {{0, 0, 0}, HOLD_ANGLE, xy_speed, a_speed}};
   smooth_pure_pursuit(path3);
  //move_to_point({{0, 36, 0}, turn});

  /*
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