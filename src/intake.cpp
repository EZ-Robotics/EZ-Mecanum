/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

void set_intake(int input) { intake = input; }

void intake_opcontrol() {
        if (master.get_digital(B_INTAKE_IN)) {
      set_intake(127);
    } else if (master.get_digital(B_INTAKE_OUT)) {
      set_intake(-127);
    } else {
      set_intake(0);
    }
}