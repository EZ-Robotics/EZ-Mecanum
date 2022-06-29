/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

void set_flywheel(int input) {
  flywheel = input;
}
double getRPM() { return flywheel.get_actual_velocity() * 3; }

void flywheel_control() {
  // const double max[2] = {3300, 127};
  // const double min[2] = {2850, 110};
  const double max[2] = {1930, 127};
  const double min[2] = {966, 63};
  const double m = (max[1] - min[1]) / (max[0] - min[0]);
  const double b = max[1] - (m * max[0]);

  const double thresh = 100;
  double output = 0;

  // PID flywheelPID(0, 0.005, 0, thresh);
  PID flywheelPID(0, 0.0008, 0, thresh);
  flywheelPID.reset_i_sgn = false;

  while (true) {
    // Calculate theoretical power to hold rpm
    double hold_power = (targetRPM * m) + b;

    // If the target is 0 do nothing
    if (targetRPM == 0) {
      output = 0;
    }

    // If flywheel is slower then target rpm, go max speed
    else if (getRPM() <= targetRPM - thresh) {
      output = 127;
    }

    // When flywheel is faster then target rpm, go 0
    else if (getRPM() >= targetRPM + thresh) {
      output = hold_power;
    }

    // When flywheel is within deadband, run I controller
    else {
      flywheelPID.set_target(targetRPM);
      flywheelPID.compute(getRPM());
      output = hold_power + flywheelPID.output;
    }

    output = clip_num(output, 127, 0);
    set_flywheel(output);

    // printf("%f + %f     rpM: %f, speed: %i\n", flywheelPID.output, hold_power, getRPM(), (int)output);
    // printf("%f\n", getRPM());

    pros::delay(DELAY_TIME);
  }
}
pros::Task flywheelControl(flywheel_control);

void flywheel_opcontrol() {
  if (master.get_digital_new_press(B_FLYWHEEL_MAX))
    targetRPM = targetRPM != 0 ? 0 : 1800;
  else if (master.get_digital_new_press(B_FLYWHEEL_SLOW))
    targetRPM = targetRPM != 0 ? 0 : 1550;
}
