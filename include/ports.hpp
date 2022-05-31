#pragma once

#include "api.h"

inline pros::Motor l1_front(19, true);
inline pros::Motor l2_front(20);
inline pros::Motor r1_front(12, true);
inline pros::Motor r2_front(14);
inline pros::Motor l1_back(17, true);
inline pros::Motor l2_back(18);
inline pros::Motor r1_back(15, true);
inline pros::Motor r2_back(16);

inline pros::Motor flywheel(4, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
inline pros::Motor intake(3);

inline pros::ADIDigitalOut indexerPiston('A');