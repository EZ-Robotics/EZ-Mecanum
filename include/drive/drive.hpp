/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "api.h"

long get_raw_center();
long get_raw_left();
long get_raw_right();

double get_center();
double get_left();
double get_right();

double get_angle();
void reset_trackers();
void set_angle(double input);

void raw_set_drive(int x, int y, int a);
void set_drive(int x, int y, int a);
void drive_brake(pros::motor_brake_mode_e_t input);
int deadzone(int input);

void joystick_control();
void lucas_joystick_control();

void set_left(int input);
void set_right(int input);
void tank_control();