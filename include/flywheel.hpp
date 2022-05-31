/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once 

inline double targetRPM = 0;

void set_flywheel(int input);
double getRPM();

void flywheel_opcontrol();