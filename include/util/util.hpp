/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once
#include <string.h>

#include "api.h"
#include "setup.hpp"

inline pros::Controller master(pros::E_CONTROLLER_MASTER);

/**
 * Enum for drive types.
 */
enum e_mode { DISABLE = 0,
              DISABLED = 0,
              SWING = 1,
              TURN = 2,
              DRIVE = 3,
              TO_POINT = 4,
              PURE_PURSUIT = 5 };

/**
 * Enum for exit_condition outputs.
 */
enum exit_output { RUNNING = 1,
                   SMALL_EXIT = 2,
                   BIG_EXIT = 3,
                   VELOCITY_EXIT = 4,
                   mA_EXIT = 5,
                   ERROR_NO_CONSTANTS = 6 };

/**
 * Enum for direction
 */
enum edirection { FWD = 0,
                  FORWARD = 0,
                  F = 0,
                  FORWARDS = 0,
                  REV = 1,
                  REVERSED = 1,
                  BACKWARD = 1,
                  BACKWARDS = 1,
                  R = 1,
                  B = 1 };

/**
 * Enum for turn types
 */
enum turn_types { FAST_MOVE = 0,
                  LOOK_AT_TARGET = 1,
                  HOLD_ANGLE = 2 };

/**
 * Struct for coordinates
 */
typedef struct pose {
  double x;
  double y;
  double theta;
} pose;

/**
 * Struct for odom movements
 */
typedef struct odom {
  pose target;
  turn_types turn_type;
  int max_xy_speed = MAX_XY;
  int max_turn_speed = MAX_A;
  edirection dir = FWD;
} odom;

/**
 * Outputs string for exit_condition enum.
 */
std::string exit_to_string(exit_output input);

const int DELAY_TIME = 10;

inline int mode = DISABLE;
inline bool AUTO_RAN = false;

/**
 * Returns input restricted to min-max threshold
 */
double clip_num(double input, double max, double min);

/**
 * Constrains the angle to 180 to -180
 */
double wrap_angle(double theta);

/**
 * Outputs absolute angle to point
 */
double absolute_angle_to_point(double x_target, double y_target);

/**
 * Outputs relative angle to point (error)
 */
double relative_angle_to_point(double angle);

/**
 * Outputs distance to point (hypot)
 */
double distance_to_point(double x_target, double y_target);

/**
 * Returns 1 if input is positive and -1 if input is negative
 */
int sgn(double input);

/**
 * Convert radians to degrees
 */
double to_deg(double input);

/**
 * Convert degrees to radians
 */
double to_rad(double input);