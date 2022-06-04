/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "util/util.hpp"

/**
 * Constrains the angle to 180 to -180
 */
double wrap_angle(double theta);

/**
 * Outputs absolute angle to point
 */
double absolute_angle_to_point(pose itarget, pose icurrent);

/**
 * Outputs relative angle to point (error)
 */
double relative_angle_to_point(double angle);

/**
 * Outputs distance to point (hypot)
 */
double distance_to_point(pose itarget, pose icurrent);

/**
 * Outputs x,y from a pose
 */
pose vector_off_point(double added, pose icurrent);

/**
 * Injects points in a list
 */
std::vector<odom> inject_points(std::vector<odom> imovements);

/**
 * Smooths an input path
 */
std::vector<odom> smooth_path(std::vector<odom> ipath, double weight_smooth, double weight_data, double tolerance);