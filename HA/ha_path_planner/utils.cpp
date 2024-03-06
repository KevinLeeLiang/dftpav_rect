//
// Created by garen_lee on 2024/1/26.
/**
 ******************************************************************************
 * @file           : utils.cpp.cc
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/26
 ******************************************************************************
 */
//

#include "utils.h"
#include "include/math/linear_interpolation.h"

PLANNING_NAMESPACE_START

void Utils::rotate2d(double lx, double ly, double theta, double ox, double oy, double &gx, double &gy) {
    double cos_a = cos(theta);
    double sin_a = sin(theta);
    gx = ox + lx * cos_a - ly * sin_a;
    gy = oy + lx * sin_a + ly * cos_a;
}

double Utils::normalize_angle(const double angle) {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

std::pair<double, double> Utils::cartesian_to_polar(double x, double y) {
    double r = std::sqrt(x * x + y * y);
    double theta = std::atan2(y, x);
    return std::make_pair(r, theta);
}

Pose2d Utils::tf2d(const Pose2d &local_frame, const Pose2d &pose) {

    Pose2d Point_local;
    double x, y;
    rotate2d(pose.x() - local_frame.x(), pose.y() - local_frame.y(), -local_frame.theta(), 0.0, 0.0, x, y);
    Point_local = Pose2d(x, y, normalize_angle(pose.theta() - local_frame.theta()));
    return Point_local;
}

Vec2d Utils::tf2d_inv(const Pose2d &local_frame, const Vec2d &point_local) {
    double x_global, y_global;
    rotate2d(point_local.x(), point_local.y(), local_frame.theta(), local_frame.x(), local_frame.y(), x_global,
             y_global);
    return Vec2d(x_global, y_global);
}



PathPoint PathPlanUtils::EvaluateByS(const std::vector<PathPoint> &path, const double s) {
    auto func = [](const PathPoint &tp, const double path_s) { return tp.s() < path_s; };
    auto it_lower = std::lower_bound(path.begin(), path.end(), s, func);

    if (it_lower == path.begin()) {
        return path.front();
    }
    if (it_lower == path.end()) {
        return path.back();
    }

    return haomo::hidelivery::math::InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, s);
}
PLANNING_NAMESPACE_END