#include "main.h"

// This occurs as soon as the program starts.
void initialize() {
  pros::lcd::initialize();
}

// Runs while the robot is in disabled at on a competition.
void disabled() {}

// Runs after initialize(), and before autonomous when connected to a competition.
void competition_initialize() {}

// Runs the user autonomous code.
void autonomous() {
  drive_brake(MOTOR_BRAKE_HOLD);

  // . . .
}

// Runs the operator control code.
void opcontrol() {
  // Drive preference
  drive_brake(MOTOR_BRAKE_BRAKE);

  while (true) {

    flywheel_opcontrol();
    joystick_control();
    indexer_opcontrol();
    intake_opcontrol();

    pros::delay(DELAY_TIME);
  }
}
