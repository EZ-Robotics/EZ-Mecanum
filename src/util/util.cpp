/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "util.hpp"

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
double absolute_angle_to_point(pose itarget, pose icurrent) {
  // Difference in target to current (legs of triangle)
  double x_error = itarget.x - icurrent.x;
  double y_error = itarget.y - icurrent.y;

  // Displacement of error
  double error = to_deg(atan2(x_error, y_error));
  return error;
}

double relative_angle_to_point(double angle) {
  return wrap_angle(angle - get_angle());
}

// Find shortest distance to point
double distance_to_point(pose itarget, pose icurrent) {
  // Difference in target to current (legs of triangle)
  double x_error = (itarget.x - icurrent.x);
  double y_error = (itarget.y - icurrent.y);

  // Hypotenuse of triangle
  double distance = hypot(x_error, y_error);

  return distance;
}

pose vector_off_point(double added, pose icurrent) {
  double x_error = sin(to_rad(icurrent.theta)) * added;
  double y_error = cos(to_rad(icurrent.theta)) * added;

  pose output;
  output.x = x_error + icurrent.x;
  output.y = y_error + icurrent.y;
  output.theta = icurrent.theta;
  return output;
}

// Inject point based on https://www.chiefdelphi.com/t/paper-implementation-of-the-adaptive-pure-pursuit-controller/166552
std::vector<odom> inject_points(std::vector<odom> imovements) {
  // Create new vector that includes the starting point
  std::vector<odom> input = imovements;
  input.insert(input.begin(), {{{target.x, target.x, imovements[0].target.theta}, imovements[0].turn_type, imovements[0].max_xy_speed, imovements[0].max_turn_speed}});

  std::vector<odom> output;  // Output vector
  int output_index = -1;     // Keeps track of current index

  // This for loop runs for how many points there are minus one because there is one less vector then points
  for (int i = 0; i < input.size() - 1; i++) {
    // Figure out how many points fit in the vector
    int num_of_points_that_fit = (distance_to_point(input[i + 1].target, input[i].target)) / SPACING;

    // Add parent point
    output.push_back({input[i].target});
    output_index++;

    // Add the injected points
    for (int j = 0; j < num_of_points_that_fit; j++) {
      // Calculate the new point with known information: hypot and angle
      double angle_to_point = absolute_angle_to_point(input[i + 1].target, input[i].target);
      pose new_point = vector_off_point(SPACING, {output[output_index].target.x, output[output_index].target.y, angle_to_point});

      // Make sure the robot is looking at next point
      turn_types turn;
      if ((input[i + 1].turn_type == FAST_MOVE_REV || input[i + 1].turn_type == FAST_MOVE_FWD) && fabs(distance_to_point(input[i + 1].target, new_point)) > TURN_FAST_MOVE - LOOK_AHEAD) {
        turn = input[i + 1].turn_type == FAST_MOVE_REV ? LOOK_AT_TARGET_REV : LOOK_AT_TARGET_FWD;
      } else {
        turn = input[i + 1].turn_type;
      }

      // Push new point to vector
      output.push_back({{new_point.x, new_point.y, input[i + 1].target.theta},
                        turn,
                        input[i + 1].max_xy_speed,
                        input[i + 1].max_turn_speed});
      output_index++;
    }
    // Make sure the final point is there
    output.push_back(input[i + 1]);
    output_index++;
  }

  // Return final vector
  return output;
}

// Print exit conditions
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

// Print turn types
std::string turn_types_to_string(turn_types input) {
  switch ((int)input) {
    case FAST_MOVE_REV:
      return "Fast Move Rev";
    case FAST_MOVE_FWD:
      return "Fast Move FWD";
    case LOOK_AT_TARGET_FWD:
      return "Look At Target FWD";
    case LOOK_AT_TARGET_REV:
      return "Look At Target REV";
    case HOLD_ANGLE:
      return "Hold Angle";
    default:
      return "Error: Out of bounds!";
  }

  return "Error: Out of bounds!";
}