//
// Created by garen_lee on 2024/1/26.
/**
 ******************************************************************************
 * @file           : utils.h
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/26
 ******************************************************************************
 */
//

#ifndef UTILS_H
#define UTILS_H
#pragma once

#include "parking_planning/apa/common/math_helper.h"

PLANNING_NAMESPACE_START

class Utils {
  public:
    static void rotate2d(double lx, double ly, double theta, double ox, double oy, double &gx, double &gy);

    static double normalize_angle(const double angle);

    static std::pair<double, double> cartesian_to_polar(double x, double y);

    static Pose2d tf2d(const Pose2d &local_frame, const Pose2d &pose);

    static Vec2d tf2d_inv(const Pose2d &local_frame, const Vec2d &point_local);
};

class PathPlanUtils {
  public:
    static PathPoint EvaluateByS(const std::vector<PathPoint> &path, const double s);
};

PLANNING_NAMESPACE_END

#endif // UTILS_H
