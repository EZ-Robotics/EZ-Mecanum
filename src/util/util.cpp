/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

bool AUTON_RAN = true;

double clip_num(double input, double max, double min) {
  if (input > max)
    return max;
  else if (input < min)
    return min;
  return input;
}

int sgn(double input) {
  if (input > 0)
    return 1;
  else if (input < 0)
    return -1;
  return 0;
}

double to_deg(double input) { return input * (180 / M_PI); }
double to_rad(double input) { return input * (M_PI / 180); }

double wrap_angle(double theta) {
  while (theta > 180) theta -= 360;
  while (theta < -180) theta += 360;
  return theta;
}

// Finds error in shortest angle to point
double absolute_angle_to_point(double x_target, double y_target) {
  // Difference in target to current (legs of triangle)
  double x_error = x_target - current.x;
  double y_error = y_target - current.y;

  // Displacement of error
  double error = to_deg(atan2(x_error, y_error));
  return error;
}

double relative_angle_to_point(double angle) {
  return wrap_angle(angle - get_angle());
}

// Find shortest distance to point
double distance_to_point(double x_target, double y_target) {
  // Difference in target to current (legs of triangle)
  double x_error = (x_target - current.x);
  double y_error = (y_target - current.y);

  // Hypotenuse of triangle 
  double distance = hypot(x_error, y_error);

  return distance;
}

std::string exit_to_string(exit_output input) {
  switch ((int)input) {
    case RUNNING:
      return "Running";
    case SMALL_EXIT:
      return "Small";
    case BIG_EXIT:
      return "Big";
    case VELOCITY_EXIT:
      return "Velocity";
    case mA_EXIT:
      return "mA";
    case ERROR_NO_CONSTANTS:
      return "Error: Exit condition constants not set!";
    default:
      return "Error: Out of bounds!";
  }

  return "Error: Out of bounds!";
}