#pragma once

#include "api.h"

double get_center();
double get_left();
double get_right();
double get_angle();
void reset_trackers();

void set_drive(int forward, int strafe, int turn);
void drive_brake(pros::motor_brake_mode_e_t input);
int deadzone(int input);

void joystick_control();

void set_left(int input);
void set_right(int input);