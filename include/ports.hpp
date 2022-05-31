#pragma once

#include "api.h"
#include "pros/adi.hpp"

inline pros::IMU imu(7);

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
inline pros::Motor l1_front(16, true);
inline pros::Motor l2_front(15);
inline pros::Motor r1_back(20, true);
inline pros::Motor r2_back(19);
inline pros::Motor r1_front(18, true);
inline pros::Motor r2_front(17);
inline pros::Motor l1_back(14, true);
inline pros::Motor l2_back(12);

inline pros::Motor flywheel(4, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);  // this makes rpm in degrees
inline pros::Motor intake(3);

inline pros::ADIDigitalOut indexerPiston('A');

inline pros::ADIEncoder center_tracker('C', 'D');
inline pros::ADIEncoder left_tracker('E', 'F');
inline pros::ADIEncoder right_tracker('G', 'H');

inline const int TICK_PER_REV = 8000;
inline const double WHEEL_DIA = 2;