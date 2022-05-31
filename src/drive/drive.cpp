#include "main.h"

// Returns inches
double get_center() { return center_tracker.get_value() / (TICK_PER_REV / (WHEEL_DIA * M_PI)); }
double get_left() { return left_tracker.get_value() / (TICK_PER_REV / (WHEEL_DIA * 3.1415)); }
double get_right() { return right_tracker.get_value() / (TICK_PER_REV / (WHEEL_DIA * 3.1415)); }
double get_angle() { return imu.get_rotation(); }

void reset_trackers() {
  center_tracker.reset();
  left_tracker.reset();
  right_tracker.reset();
}

// For internal use only
void raw_set_drive(int forward, int strafe, int turn) {
  int fl = forward + turn + strafe;
  int bl = forward + turn - strafe;
  int fr = forward - turn - strafe;
  int br = forward - turn + strafe;

  l1_front = fl;
  l2_front = fl;
  l1_back = bl;
  l2_back = bl;
  r1_front = fr;
  r2_front = fr;
  r1_back = br;
  r2_back = br;
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

// For external use only
void set_drive(int forward, int strafe, int turn) {
  mode = DISABLED;
  raw_set_drive(forward, strafe, turn);
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
  return (powf(e, -(t / 10)) + powf(e, ((abs(x) - 127) / 10)) * (1 - powf(e, -(t / 10)))) * x;
}

// Opcontrol
void joystick_control() {
  int forward = master.get_analog(ANALOG_LEFT_Y);
  int strafe = master.get_analog(ANALOG_LEFT_X);
  int turn = inputcurve(master.get_analog(ANALOG_RIGHT_X));

  set_drive(forward, strafe, turn);
}