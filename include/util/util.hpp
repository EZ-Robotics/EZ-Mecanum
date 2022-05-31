/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once
#include <string.h>

#include "api.h"

inline pros::Controller master(pros::E_CONTROLLER_MASTER);

/**
 * Enum for drive types.
 */
enum e_mode { DISABLE = 0,
              DISABLED = 0,
              SWING = 1,
              TURN = 2,
              DRIVE = 3 };

/**
 * Enum for PID::exit_condition outputs.
 */
enum exit_output { RUNNING = 1,
                   SMALL_EXIT = 2,
                   BIG_EXIT = 3,
                   VELOCITY_EXIT = 4,
                   mA_EXIT = 5,
                   ERROR_NO_CONSTANTS = 6 };

/**
 * Struct for coordinates
 */
typedef struct pose {
  double x;
  double y;
  double theta;
} pose;

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