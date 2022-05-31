#include "main.h"

// This occurs as soon as the program starts.
void initialize() {
  pros::delay(300);

  imu.reset();
  pros::delay(2000);
  master.rumble(".");

  reset_trackers();

  pros::lcd::initialize();
  pros::lcd::set_background_color(255, 110, 199);
}

// Runs while the robot is in disabled at on a competition.
void disabled() {}

// Runs after initialize(), and before autonomous when connected to a competition.
void competition_initialize() {}

// Runs the user autonomous code.
void autonomous() {
  reset_trackers();
  reset_pid_targets();
  imu.set_heading(0);
  drive_brake(MOTOR_BRAKE_HOLD);


  set_turn_pid(90, 110);
  pros::delay(1000);
}

// Runs the operator control code.
void opcontrol() {
  // Drive brake, this is preference
  drive_brake(MOTOR_BRAKE_BRAKE);

  while (true) {
    // printf("C: %.1f  L: %.1f  R: %.1f  \n", get_center(), get_left(), get_right());
    printf("x: %.1f  y: %.1f  theta: %.1f \n", current.x, current.y, current.theta);
    // printf("theta: %f    anle: %f \n", current.theta, get_angle());

    flywheel_opcontrol();
    joystick_control();
    indexer_opcontrol();
    intake_opcontrol();

    pros::delay(DELAY_TIME);
  }
}
