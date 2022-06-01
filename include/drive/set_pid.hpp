/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "util/pid.hpp"

inline PID headingPID(1);
inline PID forwardPID(3);
inline PID backwardPID(3);
inline PID turnPID(2);

inline PID leftPID(0);
inline PID rightPID(0);

inline PID yPID(10);
inline PID xPID(10);
inline PID aPID(5);

void reset_pid_targets();

void set_drive_pid(double target, int speed, bool slew_on = false, bool toggle_heading = false);
void set_turn_pid(double target, int speed);
void move_to_point(pose itarget);
void fast_to_point(pose itarget);