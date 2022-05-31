/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

void set_flywheel(int input) { flywheel = input; }

double getRPM() { return flywheel.get_actual_velocity() * 5; }

void flywheel_control() {
  const double kP = 3.0;
  const double maxVoltage = 12000;
  const double maxRPM = 3200;                    // Geared for 3000RPM (600 5:1), but this is the measured max RPM
  const double kv = 0.98 * maxVoltage / maxRPM;  // Multiplied by a scalar to adjust for the real world
  double deadband = 100;                         // "Acceptable" velocity range to switch to hold power + p control

  while (true) {
    double rpmError = targetRPM - getRPM();
    double holdPower = targetRPM * kv;
    double output = 0;

    // Coast down
    if (targetRPM == 0) {
      pros::lcd::set_text(2, "Flywheel off!         ");
      output = 0;
    }

    // Running too slow (< target - deadband), apply full power
    else if (rpmError > deadband) {
      pros::lcd::set_text(2, "Flywheel Too Slow!      ");
      output = maxVoltage;
    }

    // Running too fast (> target + deadband), apply half of hold power
    else if (rpmError < -deadband) {
      pros::lcd::set_text(2, "Flywheel Too Fast!      ");
      output = holdPower * 0.5;
    }

    // Running within deadband, switch to hold power + p contorl
    else {
      output = holdPower + kP * rpmError;
      pros::lcd::set_text(2, "At Velocity!          ");
    }

    flywheel.move_voltage(output);

    std::string str_rpm = std::to_string(targetRPM);
    std::string str_cur = std::to_string((int)getRPM());
    pros::lcd::set_text(0, "Target Vel: " + str_rpm);
    pros::lcd::set_text(1, "Current Vel: " + str_cur);

    pros::delay(10);
  }
}
pros::Task flywheelControl(flywheel_control);

void flywheel_opcontrol() {
  if (master.get_digital_new_press(B_FLYWHEEL_MAX))
    targetRPM = targetRPM != 0 ? 0 : 3230;
  else if (master.get_digital_new_press(B_FLYWHEEL_SLOW))
    targetRPM = targetRPM != 0 ? 0 : 3000;
}