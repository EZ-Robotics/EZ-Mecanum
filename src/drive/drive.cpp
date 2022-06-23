/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

long get_raw_center() { return center_tracker.get_value(); }
long get_raw_left() { return left_tracker.get_value(); }
long get_raw_right() { return right_tracker.get_value(); }

// Returns inches
double get_center() { return get_raw_center() / TICK_PER_INCH; }
double get_left() { return get_raw_left() / TICK_PER_INCH; }
double get_right() { return get_raw_right() / TICK_PER_INCH; }
double get_angle() { return current.theta; }
// double get_angle() { return imu.get_rotation(); }

void reset_trackers() {
  center_tracker.reset();
  left_tracker.reset();
  right_tracker.reset();
}

void set_angle(double input) {
  headingPID.set_target(input);
  angle_rad = to_rad(input);
  angle_rad = to_rad(input);
  imu.set_rotation(input);
  target.theta = input;
  current.theta = input;
}

// For internal use only
void raw_set_drive(int x, int y, int a) {
  int fl = x + y + a;
  int bl = -x + y + a;
  int fr = -x + y - a;
  int br = x + y - a;

  l1_front = fl;
  l2_front = fl;
  l1_back = bl;
  l2_back = bl;
  r1_front = fr;
  r2_front = fr;
  r1_back = br;
  r2_back = br;
}

// For internal use only
void lucas_set_drive(int x, int y, int a) {
  mode = DISABLED;

  int fl = x + y + a;
  int bl = -x + y + a;
  int fr = -x + y - a;
  int br = x + y - a;

  l1_front = br;
  l2_front = br;
  l1_back = fr;
  l2_back = fr;
  r1_front = bl;
  r2_front = bl;
  r1_back = fl;
  r2_back = fl;
}

// For external use only
void set_drive(int x, int y, int a) {
  raw_set_drive(x, y, a);
}

void set_left(int input) {
  l1_front = input;
  l2_front = input;
  l1_back = input;
  l2_back = input;
}

void set_right(int input) {
  r1_front = input;
  r2_front = input;
  r1_back = input;
  r2_back = input;
}

// Brake drive
void drive_brake(pros::motor_brake_mode_e_t input) {
  l1_front.set_brake_mode(input);
  l2_front.set_brake_mode(input);
  r1_front.set_brake_mode(input);
  r2_front.set_brake_mode(input);
  l1_back.set_brake_mode(input);
  l2_back.set_brake_mode(input);
  r1_back.set_brake_mode(input);
  r2_back.set_brake_mode(input);
}

// Joystick deadzone
int deadzone(int input) {
  if (abs(input) > 5)
    return input;
  return 0;
}

// Input curve based on pilons
double inputcurve(int x) {
  double e = 2.718;
  double t = 2.1;
  return (powf(e, -(t / 10)) + powf(e, ((fabs(x) - 127) / 10)) * (1 - powf(e, -(t / 10)))) * x;
}

// Opcontrol
void joystick_control() {
  int y = deadzone(master.get_analog(ANALOG_LEFT_Y));
  int x = deadzone(master.get_analog(ANALOG_LEFT_X));
  int a = inputcurve(deadzone(master.get_analog(ANALOG_RIGHT_X)));

  set_drive(x, y, a);
}

// Opcontrol
void lucas_joystick_control() {
  int y = deadzone(master.get_analog(ANALOG_LEFT_Y));
  int x = deadzone(master.get_analog(ANALOG_LEFT_X));
  int a = inputcurve(deadzone(master.get_analog(ANALOG_RIGHT_X)));

  lucas_set_drive(-x, -y, -a);
}

void tank_control() {
  set_left(deadzone(master.get_analog(ANALOG_LEFT_Y)));
  set_right(deadzone(master.get_analog(ANALOG_RIGHT_Y)));
  mode = DISABLED;

}