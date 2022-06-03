/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "util/pid.hpp"

inline double a_target = 0;
inline int pp_index = 0;
inline int max_xy = MAX_XY;
inline int max_a = MAX_A;
inline turn_types current_turn_type = HOLD_ANGLE;
inline std::vector<odom> movements;

inline PID headingPID(1);

inline PID leftPID(0);
inline PID rightPID(0);

inline PID yPID(22.5, 0, 250, 0);
inline PID xPID(22.5, 0, 250, 0);
inline PID aPID(5, 0, 35, 0);

void reset_pid_targets();
void raw_move_odom(odom imovement);

void drive_pid(double target, int speed = MAX_XY);
void imu_pid(double itarget, int speed = TURN_SPEED);

void odom_turn(double itarget, int speed = TURN_SPEED);
void relative_move_to_point(double distance, int speed = MAX_XY);
void move_to_point(odom imovement);
void pure_pursuit(std::vector<odom> imovements);
void injected_pure_pursuit(std::vector<odom> imovements);