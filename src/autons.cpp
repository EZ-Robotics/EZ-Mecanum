/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "drive/exit_conditions.hpp"
#include "drive/set_pid.hpp"
#include "main.h"
#include "util/util.hpp"

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

// With all of the drive motions, speed is defaulted to what's in setup.hpp

// This is normal PID, no odometry, just like EZ-Template
void normal_pid_example() {
  drive_pid(24, MAX_XY);
  wait_drive();

  imu_pid(45, TURN_SPEED);
  wait_drive();

  imu_pid(-45);
  wait_drive();

  imu_pid(0);
  wait_drive();

  drive_pid(-24);
  wait_drive();
}

// This IS odometry, but it has wrappers to look like normal EZ-Template
void odom_normal_example() {
  relative_move_to_point(24, MAX_XY);
  wait_drive();

  odom_turn(45, TURN_SPEED);
  wait_drive();

  odom_turn(-45);
  wait_drive();

  odom_turn(0);
  wait_drive();

  relative_move_to_point(-24);
  wait_drive();
}

// Move to point + hold angle
void odom_hold_angle_example() {
  // First paramter is pose.  x, y and angle
  // Second is turn type, this will be exampled lower down, but HOLD_ANGLE will turn to the desired angle right away
  // Third is max XY speed (this is defaulted)
  // Fourth is max A (turn) speed (this is defaulted)
  move_to_point({{24, 24, -45}, HOLD_ANGLE, MAX_XY, MAX_A});
  wait_drive();

  move_to_point({{0, 0, 0}, HOLD_ANGLE});
  wait_drive();
}

// Move to point + FAST_MOVE
void odom_fast_move_example() {
  // Other turn types are FAST_MOVE_FWD and FAST_MOVE_REV, they work the same except for direction.
  // They try to drive FWD or REV during the majority of the motion, and at the end the robot will turn to final angle.
  move_to_point({{24, 24, 45}, FAST_MOVE_FWD});
  wait_drive();

  move_to_point({{0, 0, 0}, FAST_MOVE_REV});
  wait_drive();
}

// Pure pursuit
void pp_example() {
  // This accepts the same parameters as above, except you can change them per point.
  pure_pursuit(
      {{{0, 24, 0}, HOLD_ANGLE},
       {{24, 24, 0}, HOLD_ANGLE},
       {{24, 0, 0}, HOLD_ANGLE},
       {{0, 0, 0}, HOLD_ANGLE}});
  wait_drive();
}

// Inejcted pure pursuit
void injected_pp_example() {
  // Exact same parameters as above, except points are injected so this reacts differently when pushed.
  injected_pure_pursuit(
      {{{0, 24, 0}, HOLD_ANGLE},
       {{24, 24, 0}, HOLD_ANGLE},
       {{24, 0, 0}, HOLD_ANGLE},
       {{0, 0, 0}, HOLD_ANGLE}});
  wait_drive();
}

// Smooth pure pursuit
void smooth_pp_example() {
  // This can accept a few more parameters to change how smooth the path is, but uh, its weird and the
  // defaults should be fine.  just message me if the defaults arent good and ill tell you how to tune it
  smooth_pure_pursuit(
      {{{0, 24, 0}, HOLD_ANGLE},
       {{24, 24, 0}, HOLD_ANGLE},
       {{24, 0, 0}, HOLD_ANGLE},
       {{0, 0, 0}, HOLD_ANGLE}});
  wait_drive();
}

// Smooth pure pursuit
void wait_until_pp() {
  // This can accept a few more parameters to change how smooth the path is, but uh, its weird and the
  // defaults should be fine.  just message me if the defaults arent good and ill tell you how to tune it
  smooth_pure_pursuit(
      {{{0, 24, 0}, HOLD_ANGLE},
       {{24, 24, 0}, HOLD_ANGLE},
       {{24, 0, 0}, HOLD_ANGLE},
       {{0, 0, 0}, HOLD_ANGLE}});

  // This waits until the second motion has started to spin the intake, then shuts it off once 3 starts
  // This works the same for normal pp and injected
  pp_wait_until(2);
  set_intake(127);
  pp_wait_until(3);
  set_intake(0);

  wait_drive();
}

void smooth_pp_testing() {
  smooth_pure_pursuit(
      {{{0, 24, 0}, LOOK_AT_TARGET_FWD},
       {{-12, 48, 0}, LOOK_AT_TARGET_FWD},
       {{0, 72, 0}, LOOK_AT_TARGET_FWD},
       {{12, 84, 0}, LOOK_AT_TARGET_FWD},
       {{36, 84, 0}, LOOK_AT_TARGET_FWD},
       {{48, 72, 0}, LOOK_AT_TARGET_FWD},
       {{0, 0, 90}, FAST_MOVE_FWD}});
  wait_drive();
}