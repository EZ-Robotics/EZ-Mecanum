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