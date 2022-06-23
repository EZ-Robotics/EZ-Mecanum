/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

void set_flywheel(int input) {
  flywheel.move_voltage(input);
  flywheel2.move_voltage(input);
}
double getRPM() { return flywheel.get_actual_velocity() * 5; }

void flywheel_control() {
  const double kP = 3.5;
  const double maxVoltage = 12000;
  const double maxRPM = 3150;                    // Geared for 3000RPM (600 5:1), but this is the measured max RPM
  const double kv = 0.99 * maxVoltage / maxRPM;  // Multiplied by a scalar to adjust for the real world
  double deadband = 100;                         // "Acceptable" velocity range to switch to hold power + p control

  while (true) {
    double rpmError = targetRPM - getRPM();
    double holdPower = targetRPM * kv;
    double output = 0;

    // Coast down
    if (targetRPM == 0)
      output = 0;

    // Running too slow (< target - deadband), apply full power
    else if (rpmError > deadband)
      output = maxVoltage;

    // Running too fast (> target + deadband), apply half of hold power
    else if (rpmError < -deadband)
      output = holdPower * 0.5;

    // Running within deadband, switch to hold power + p contorl
    else
      output = holdPower + kP * rpmError;

    set_flywheel(output);
    // flywheel.move_voltage(holdPower);

    pros::delay(DELAY_TIME);
  }
}
pros::Task flywheelControl(flywheel_control);

void flywheel_opcontrol() {
  if (master.get_digital_new_press(B_FLYWHEEL_MAX))
    targetRPM = targetRPM != 0 ? 0 : 3000;
  else if (master.get_digital_new_press(B_FLYWHEEL_SLOW))
    targetRPM = targetRPM != 0 ? 0 : 2800;
}