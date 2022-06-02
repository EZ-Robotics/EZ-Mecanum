/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "util/pid.hpp"

inline bool fast_move = false;
inline bool only_look_at_point = false;
inline direction dir = FWD;

inline PID headingPID(1);
inline PID forwardPID(3);
inline PID backwardPID(3);
inline PID turnPID(2);

inline PID leftPID(0);
inline PID rightPID(0);

inline PID yPID(22.5, 0, 250, 0);
inline PID xPID(22.5, 0, 250, 0);
inline PID aPID(5, 0, 35, 0);

void reset_pid_targets();

void set_drive_pid(double target, int speed, bool slew_on = false, bool toggle_heading = false);
void set_turn_pid(double itarget, int speed);
void move_to_point(pose itarget);
void fast_to_point(pose itarget, direction idir);