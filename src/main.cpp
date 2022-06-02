#include "main.h"

// This occurs as soon as the program starts.
void initialize() {
  pros::delay(300);

  imu.reset();
  pros::delay(2000);
  master.rumble(".");

  reset_trackers();
  reset_odom();

  pros::lcd::initialize();
  pros::lcd::set_background_color(255, 110, 199);  // ez pink
}

// Runs while the robot is in disabled at on a competition.
void disabled() {}

// Runs after initialize(), and before autonomous when connected to a competition.
void competition_initialize() {}

// Runs the user autonomous code.
void autonomous() {
  reset_trackers();
  reset_odom();
  reset_pid_targets();
  imu.set_heading(0);
  drive_brake(MOTOR_BRAKE_HOLD);

  fast_to_point({0, 36, -22.5}, FWD);
  pros::delay(2000);

  move_to_point({0, 0, 22.5});
  pros::delay(2000);

  // move_to_point({0, 0, 0});

  // set_turn_pid(90, 110);
}

// Runs the operator control code.
void opcontrol() {
  // Drive brake, this is preference
  drive_brake(MOTOR_BRAKE_BRAKE);

  while (true) {
    flywheel_opcontrol();
    joystick_control();
    indexer_opcontrol();
    intake_opcontrol();

    pros::delay(DELAY_TIME);
  }
}