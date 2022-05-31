#pragma once

#include "api.h"

void set_drive(int forward, int strafe, int turn);
void drive_brake(pros::motor_brake_mode_e_t input);
int deadzone(int input);

void joystick_control();