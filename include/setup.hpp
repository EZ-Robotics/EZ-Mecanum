/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "api.h"
#include "pros/adi.hpp"

/**
 * Ports
 */
/*
// Flywheel Front
inline pros::Motor l1_front(19, true);
inline pros::Motor l2_front(20);
inline pros::Motor r1_front(12, true);
inline pros::Motor r2_front(14);
inline pros::Motor l1_back(17, true);
inline pros::Motor l2_back(18);
inline pros::Motor r1_back(15, true);
inline pros::Motor r2_back(16);
*/
// Intake Front
///*
inline pros::Motor l1_front(16, true);
inline pros::Motor l2_front(15);
inline pros::Motor r1_back(20, true);
inline pros::Motor r2_back(19);
inline pros::Motor r1_front(18, true);
inline pros::Motor r2_front(17);
inline pros::Motor l1_back(14, true);
inline pros::Motor l2_back(12);
inline pros::Imu imu(7);
//*/
inline pros::Motor flywheel(4, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);  // this makes rpm in degrees
inline pros::Motor intake(3);
inline pros::ADIDigitalOut indexerPiston('A');
inline pros::ADIEncoder center_tracker('C', 'D');
inline pros::ADIEncoder left_tracker('E', 'F');
inline pros::ADIEncoder right_tracker('G', 'H');

/**
 * Tracking Wheel Constants
 */
// Wheel size and encoder
inline const int TICK_PER_REV = 8096;
inline const double WHEEL_DIA = 2.0;

// Tracking wheel offsets
inline const double WIDTH = 6.87;  // biger means angle will grow
inline const double CENTER_OFFSET = 3.0;

// ignore these unless the left/right tracker aren't mounted symetrically
inline const double RIGHT_OFFSET = WIDTH / 2.0;
inline const double LEFT_OFFSET = WIDTH / 2.0;

/**
 * Controls
 */

#define B_INDEXER pros::E_CONTROLLER_DIGITAL_DOWN
#define B_INTAKE_IN pros::E_CONTROLLER_DIGITAL_L1
#define B_INTAKE_OUT pros::E_CONTROLLER_DIGITAL_L2
#define B_FLYWHEEL_MAX pros::E_CONTROLLER_DIGITAL_R1
#define B_FLYWHEEL_SLOW pros::E_CONTROLLER_DIGITAL_R2