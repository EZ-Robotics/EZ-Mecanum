/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "set_pid.hpp"

#include "main.h"

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
  printf("Drive Started... Target Value: %f \n", target);
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
  printf("IMU Turn Started... Target Value: %f ", itarget);
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
  dir = imovement.dir;
  max_xy = abs(imovement.max_xy_speed);
  max_a = abs(imovement.max_turn_speed);
  xPID.set_target(target.x);
  yPID.set_target(target.y);
}

// Set turn PID, for external use
void turn_pid(double itarget, int speed) {
  // Print targets
  printf("Odom Turn Started... Target Value: %f ", itarget);
  printf("\n");

  headingPID.set_target(itarget);
  raw_move_odom({{target.x, target.y, itarget}, HOLD_ANGLE, MAX_XY, speed, FWD});

  mode = TO_POINT;
}

// Move point to point, for external use
void move_to_point(odom imovement) {
  // Print targets
  printf("Odom Motion Started... Target Value: (%f, %f, %f) \n", imovement.target.x, imovement.target.y, imovement.target.theta);

  // Set new targets
  raw_move_odom(imovement);

  // Run point_to_point()
  mode = TO_POINT;
}

// Pure pursuit, for external use
void pure_pursuit(std::vector<odom> imovements) {
  // Print targets
  printf("Pure Pursuit Motion Started... Target Value: \n");
  for (int i = 0; i < imovements.size(); i++) {
    printf(" Point %i: (%f, %f, %f)\n", i + 1, imovements[i].target.x, imovements[i].target.y, imovements[i].target.theta);
  }

  // Reset indexes and previous movements
  movements.clear();
  pp_index = 0;

  // Set new targets
  movements = imovements;

  // Run pure_pursuit()
  mode = PURE_PURSUIT;
}