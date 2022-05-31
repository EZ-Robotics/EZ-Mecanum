#pragma once 

#include "util/pid.hpp"

inline PID headingPID(1);
inline PID forwardPID(3);
inline PID backwardPID(3);
inline PID turnPID(2);

inline PID leftPID(0);
inline PID rightPID(0);

void reset_pid_targets();

void set_drive_pid(double target, int speed, bool slew_on = false, bool toggle_heading = false);
void set_turn_pid(double target, int speed);