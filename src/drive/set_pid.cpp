/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "set_pid.hpp"

#include "main.h"
#include "util/util.hpp"

void reset_pid_targets() {
  headingPID.set_target(0);
  leftPID.set_target(0);
  rightPID.set_target(0);

  xPID.set_target(0);
  yPID.set_target(0);
  aPID.set_target(0);
}

void drive_pid(double target, int speed) {
  // Print targets
  printf("Drive Started... Target Distance: %f \n", target);
  reset_trackers();

  // Max speed
  max_xy = abs(speed);

  // Set targets
  leftPID.set_target(target);
  rightPID.set_target(target);

  mode = DRIVE;
}

// Set turn PID, for external use
void imu_pid(double itarget, int speed) {
  // Print targets
  printf("IMU Turn Started... Target Degree: %f ", itarget);
  printf("\n");

  max_a = abs(speed);
  headingPID.set_target(itarget);
  aPID.set_target(itarget);  // this should get deleted at some point

  mode = TURN;
}

// For internal use
void raw_move_odom(odom imovement) {
  current_turn_type = imovement.turn_type;
  target = imovement.target;
  max_xy = abs(imovement.max_xy_speed);
  max_a = abs(imovement.max_turn_speed);
  xPID.set_target(target.x);
  yPID.set_target(target.y);
}

// Set turn PID, for external use
void odom_turn(double itarget, int speed) {
  // Print targets
  printf("Odom Turn Started... Target Coordinates: (%f, %f, %f) \n", target.x, target.y, itarget);

  headingPID.set_target(itarget);

  // Run raw odom
  raw_move_odom({{target.x, target.y, itarget}, HOLD_ANGLE, MAX_XY, speed});

  // Run point_to_point()
  mode = TO_POINT;
}

// Relative odom
void relative_move_to_point(double distance, int speed) {
  // Calculate x,y based on distance (hypot)
  pose output = vector_off_point(distance, target);

  // Print targets
  printf("Relative Odom Motion Started... Target Coordinates: (%f, %f, %f) \n", output.x, output.y, output.theta);

  // Run raw odom
  raw_move_odom({output, HOLD_ANGLE, speed, MAX_A});

  // Run point_to_point()
  mode = TO_POINT;
}

// Move point to point, for external use
void move_to_point(odom imovement) {
  // Print targets
  printf("Odom Motion Started... Target Coordinates: (%f, %f, %f) \n", imovement.target.x, imovement.target.y, imovement.target.theta);

  // Set new targets
  raw_move_odom(imovement);

  // Run point_to_point()
  mode = TO_POINT;
}

// Pure pursuit, for external use
void pure_pursuit(std::vector<odom> imovements) {
  // Print targets
  printf("Pure Pursuit Motion Started... Target Coordinates: \n");
  for (int i = 0; i < imovements.size(); i++) {
    std::string turn = turn_types_to_string(imovements[i].turn_type);
    std::cout << "Point " << i << ": (" << imovements[i].target.x << ", " << imovements[i].target.y << ", " << imovements[i].target.theta << ")  Turn: " << turn << "\n";

  }

  // Reset indexes and previous movements
  movements.clear();
  injected_pp_index.clear();
  pp_index = 0;

  // This is used for pp_wait_until()
  for (int i = 0; i < imovements.size(); i++) {
    injected_pp_index.push_back(i);
  }

  // Set new targets
  movements = imovements;

  // Run pure_pursuit()
  mode = PURE_PURSUIT;
}

// Pure pursuit, for external use
void injected_pure_pursuit(std::vector<odom> imovements) {
  // Print targets
  printf("Point Injected Pure Pursuit Motion Started... Target Coordinates: \n");
  for (int i = 0; i < imovements.size(); i++) {
    std::string turn = turn_types_to_string(imovements[i].turn_type);
    std::cout << "Point " << i << ": (" << imovements[i].target.x << ", " << imovements[i].target.y << ", " << imovements[i].target.theta << ")  Turn: " << turn << "\n";

  }

  // Reset indexes and previous movements
  movements.clear();
  injected_pp_index.clear();
  pp_index = 0;

  // Set new targets
  movements = inject_points({imovements});

  /*
  // Print subpoints
  printf("Subpoints\n");
  for (int i = 0; i < movements.size(); i++) {
    std::string turn = turn_types_to_string(movements[i].turn_type);
    std::cout << "Point " << i << ": (" << movements[i].target.x << ", " << movements[i].target.y << ", " << movements[i].target.theta << ")  Turn: " << turn << "\n";
  }
  */

  // Run pure_pursuit()
  mode = PURE_PURSUIT;
}

// Pure pursuit, for external use
void smooth_pure_pursuit(std::vector<odom> imovements, double weight_smooth, double weight_data, double tolerance) {
  // Print targets
  printf("Point Injected Pure Pursuit Motion Started... Target Coordinates: \n");
  for (int i = 0; i < imovements.size(); i++) {
    std::string turn = turn_types_to_string(imovements[i].turn_type);
    std::cout << "Point " << i << ": (" << imovements[i].target.x << ", " << imovements[i].target.y << ", " << imovements[i].target.theta << ")  Turn: " << turn << "\n";

  }

  // Reset indexes and previous movements
  movements.clear();
  injected_pp_index.clear();
  pp_index = 0;

  // Set new targets
  movements = smooth_path(inject_points({imovements}), weight_smooth, weight_data, tolerance);

  /*
  // Print subpoints
  printf("Subpoints\n");
  for (int i = 0; i < movements.size(); i++) {
    std::string turn = turn_types_to_string(movements[i].turn_type);
    std::cout << "Point " << i << ": (" << movements[i].target.x << ", " << movements[i].target.y << ", " << movements[i].target.theta << ")  Turn: " << turn << "\n";
  }
  */

  // Run pure_pursuit()
  mode = PURE_PURSUIT;
}