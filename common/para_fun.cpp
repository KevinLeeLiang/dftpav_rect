#include "para_func.h"
#include "math/math_utils.h"
#include "parking_planning/math/double/double.h"
#include "parking_planning/math/util.h"

PLANNING_NAMESPACE_START

bool ParaFunc::mutable_dubins_path(const Pose2d &start_loc, const Pose2d &end_loc,
                                   const bool is_first_straight,
                                   std::array<double, 5> &heuristic_result) {
    const double eps = 1e-6;
    double arc_angle = std::fabs(end_loc.theta() - start_loc.theta());
    double k = std::tan(start_loc.theta());
    double b = start_loc.y() - k * start_loc.x();
    if (!is_first_straight) {
        bool relative_pos = k * end_loc.x() + b - end_loc.y() > 0 ? true : false;
        if (relative_pos) {
            MLOG(PARKING_PLANNING, INFO) << "relative_pos error";
            return false;
        }
    }
    double distance_target_to_line = std::fabs(k * end_loc.x() - end_loc.y() + b) / std::sqrt(1.0 + std::pow(k, 2.0));
    double circle_radius = distance_target_to_line / (1.0 - std::cos(arc_angle));
    if (circle_radius + eps < VehicleConfigHelper::GetConfig().vehicle_param().minturningradius()) {
        MLOG(PARKING_PLANNING, DEBUG) << "dubins_path radius small";
        return false;
    }
    double chord_length = 2.0 * circle_radius * std::sin(arc_angle / 2.0);
    double chord_angle =
        is_first_straight ? (start_loc.theta() + arc_angle / 2.0) : (end_loc.theta() + arc_angle / 2.0);
    double chord_direction = is_first_straight ? 1.0 : -1.0;
    double middle_loc_x = end_loc.x() - chord_direction * chord_length * std::cos(chord_angle);
    double middle_loc_y = end_loc.y() - chord_direction * chord_length * std::sin(chord_angle);
    double middle_loc_heading = start_loc.theta();
    if (is_first_straight && (middle_loc_x + eps < start_loc.x() || middle_loc_y + eps < start_loc.y())) {
        return false;
    } else if (!is_first_straight && ((middle_loc_x > start_loc.x() + eps) || (middle_loc_y > start_loc.y() + eps))) {
        return false;
    }
    heuristic_result[0] = circle_radius;
    heuristic_result[1] = arc_angle;
    heuristic_result[2] = middle_loc_x;
    heuristic_result[3] = middle_loc_y;
    heuristic_result[4] = middle_loc_heading;
    return true;
}

bool ParaFunc::calculate_heuristic_path(const Pose2d &start_loc, const Pose2d &target_loc, PathSegments &heuristic_result) {
    std::array<double, 5> data_vec;
    if (mutable_dubins_path(start_loc, target_loc, true, data_vec)) { // first straight, then arc
        Pose2d middle_loc(data_vec[2], data_vec[3], data_vec[4]);
        PathSegment path_segment1;
        path_segment1.path_type = PathType::STRAIGHT;
        path_segment1.start_pose = start_loc;
        path_segment1.target_pose = middle_loc;
        path_segment1.length = std::hypot((start_loc.x() - middle_loc.x()), (start_loc.y() - middle_loc.y()));
        path_segment1.gear = GearBoxInfoPb::GEAR_DRIVE;

        Vec2d arc_center = MathHelper::center(middle_loc, data_vec[0], false);
        PathSegment path_segment2 =
            make_circle_pathsegment(middle_loc, target_loc, arc_center, data_vec[0], data_vec[1], false, false);
        heuristic_result = {path_segment1, path_segment2};
        return true;
    } else if (mutable_dubins_path(target_loc, start_loc, false, data_vec)) { // first arc, then straight
        Pose2d middle_loc(data_vec[2], data_vec[3], data_vec[4]);
        Vec2d arc_center = MathHelper::center(start_loc, data_vec[0], false);
        PathSegment path_segment1 =
            make_circle_pathsegment(start_loc, middle_loc, arc_center, data_vec[0], data_vec[1], false, false);

        PathSegment path_segment2;
        path_segment2.path_type = PathType::STRAIGHT;
        path_segment2.start_pose = middle_loc;
        path_segment2.target_pose = target_loc;
        path_segment2.length = std::hypot((target_loc.x() - middle_loc.x()), (target_loc.y() - middle_loc.y()));
        path_segment2.gear = GearBoxInfoPb::GEAR_DRIVE;
        heuristic_result = {path_segment1, path_segment2};
        return true;
    }
    return false;
}

bool ParaFunc::calculate_csc_path(const Pose2d &start_pose, const Pose2d &target_pose,
                                  const double &min_turning_radius, const bool is_forward,
                                  PathSegments &csc_result) {
    MLOG(PARKING_PLANNING, DEBUG) << "compute_csc_path";
    Pose2d target_to_start = start_pose.inverse() * target_pose;
    MLOG(PARKING_PLANNING, DEBUG) << "target_to_start " << target_to_start.x() << " " << target_to_start.y() << " "
                                 << target_to_start.theta();
    double u, t, v;
    bool is_right = false;
    bool LSR_result = false;
    if (is_forward) {
        if (MathHelper::LpSpRp(target_to_start.x() / min_turning_radius, target_to_start.y() / min_turning_radius,
                   target_to_start.theta(), t, u, v)) {
            LSR_result = true;
            is_right = false;

        } else if (MathHelper::LpSpRp(target_to_start.x() / min_turning_radius, -target_to_start.y() / min_turning_radius,
                          -target_to_start.theta(), t, u, v)) {
            LSR_result = true;
            is_right = true;
        }
    } else {
        if (MathHelper::LpSpRp(-target_to_start.x() / min_turning_radius, -target_to_start.y() / min_turning_radius,
                   target_to_start.theta(), t, u, v)) {
            LSR_result = true;
            is_right = true;
        } else if (MathHelper::LpSpRp(-target_to_start.x() / min_turning_radius, target_to_start.y() / min_turning_radius,
                          -target_to_start.theta(), t, u, v)) {
            LSR_result = true;
            is_right = false;
        }
    }

    if (LSR_result) {
        double is_right_sign = is_right ? -1.0 : 1.0;
        double is_forward_sign = is_forward ? 1.0 : -1.0;
        double sign = is_right_sign * is_forward_sign;

        MLOG(PARKING_PLANNING, DEBUG) << "LSR success is_right " << is_right;
        Vec2d center_first = MathHelper::center(start_pose, min_turning_radius, is_right);
        double theta_first = sign * t;
        Pose2d mid_pose1 = MathHelper::get_target_loc(start_pose, center_first, min_turning_radius, theta_first);
        PathSegment path_segment1 = make_circle_pathsegment(start_pose, mid_pose1, center_first, min_turning_radius,
                                                            theta_first, is_right, !is_forward);
        double straight_len = u * min_turning_radius;
        if (is_forward) {
        } else {
        }
        Pose2d mid_2_1 = Pose2d(is_forward_sign * straight_len, 0, 0);
        Pose2d mid_pose2 = mid_pose1 * mid_2_1;
        PathSegment path_segment2;
        path_segment2.path_type = PathType::STRAIGHT;
        path_segment2.start_pose = mid_pose1;
        path_segment2.target_pose = mid_pose2;
        path_segment2.length = straight_len;
        path_segment2.gear = is_forward ? GearBoxInfoPb::GEAR_DRIVE : GearBoxInfoPb::GEAR_REVERSE;
        Vec2d center_second = MathHelper::center(mid_pose2, min_turning_radius, !is_right);
        double theta_second = -sign * v;
        Pose2d mid_pose3 = MathHelper::get_target_loc(mid_pose2, center_second, min_turning_radius, theta_second);
        PathSegment path_segment3 = make_circle_pathsegment(mid_pose2, mid_pose3, center_second, min_turning_radius,
                                                            theta_second, !is_right, !is_forward);

        PathSegments segments{path_segment1, path_segment2, path_segment3};
        csc_result = segments;
        return true;
    }
    MLOG(PARKING_PLANNING, DEBUG) << "LSR fail ";
    return false;
}

PathSegment ParaFunc::reverse_path_segment(const PathSegment &path_segment) {
    PathSegment new_path_segment = path_segment;
    auto target_pose = path_segment.target_pose;
    new_path_segment.target_pose = path_segment.start_pose;
    new_path_segment.start_pose = target_pose;
    new_path_segment.arc_theta = -path_segment.arc_theta;
    if (path_segment.gear == GearBoxInfoPb::GEAR_DRIVE || GearBoxInfoPb::GEAR_REVERSE) {
        new_path_segment.gear =
            (path_segment.gear == GearBoxInfoPb::GEAR_DRIVE) ? GearBoxInfoPb::GEAR_REVERSE : GearBoxInfoPb::GEAR_DRIVE;
    }
    return new_path_segment;
}

PathSegment ParaFunc::make_straight_pathsegment(const Pose2d &start_pose, const Pose2d &end_pose,
                                                                   const double &length, bool is_reverse) {
    PathSegment path_segment;
    path_segment.path_type = PathType::STRAIGHT;
    path_segment.start_pose = start_pose;
    path_segment.target_pose = end_pose;
    path_segment.length = length;
    path_segment.gear = GearBoxInfoPb::GEAR_DRIVE;
    if (is_reverse) {
        path_segment.gear = GearBoxInfoPb::GEAR_REVERSE;
    }

    return path_segment;
}

PathSegment ParaFunc::make_circle_pathsegment(const Pose2d &start_pose, const Pose2d &end_pose,
                                              const Vec2d &arc_center, const double &radius,
                                              const double &theta, bool is_right, bool is_reverse) {
    PathSegment path_segment;
    path_segment.path_type = PathType::CIRCLE;
    path_segment.start_pose = start_pose;
    path_segment.target_pose = end_pose;
    path_segment.arc_center = arc_center;
    path_segment.arc_theta = theta;
    path_segment.length = std::abs(theta) * radius;
    path_segment.is_right = is_right;
    path_segment.arc_radius = radius;
    path_segment.gear = GearBoxInfoPb::GEAR_DRIVE;
    if (is_reverse) {
        path_segment.gear = GearBoxInfoPb::GEAR_REVERSE;
    }

    return path_segment;
}

bool ParaFunc::check_pose(const Pose2d &pose, const std::string &topic) {
    if (check_scalar(pose.x(), "pose_x") && check_scalar(pose.y(), "pose_x") &&
        check_scalar(pose.theta(), "pose_theta")) {
        return true;
    }
    MLOG(PARKING_PLANNING, INFO) << "check pose: " << topic << "failed";
    return false;
}

bool ParaFunc::check_vec(const Vec2d &vec, const std::string &topic) {
    if (check_scalar(vec.x(), "vec_x") && check_scalar(vec.y(), "vec_y")) {
        return true;
    }
    MLOG(PARKING_PLANNING, INFO) << "check vec: " << topic << "failed";
    return false;
}

bool ParaFunc::check_gear(const GearBoxInfoPb::GearNum &gear) {
    if (gear == GearBoxInfoPb::GEAR_REVERSE || gear == GearBoxInfoPb::GEAR_DRIVE) {
        return true;
    }
    MLOG(PARKING_PLANNING, INFO) << "check gear failed";
    return false;
}

bool ParaFunc::check_scalar(const double &data, const std::string &topic) {
    if (std::isnan(data) || std::isinf(data)) {
        MLOG(PARKING_PLANNING, INFO) << "check scalar" << topic << " failed";
        return false;
    }

    return true;
}

bool ParaFunc::check_path_segment(const PathSegment &path_segment) {
    bool success = true;
    success &= check_pose(path_segment.start_pose, "start_pose");
    success &= check_pose(path_segment.target_pose, "target_pose");
    success &= check_gear(path_segment.gear);
    success &= check_scalar(path_segment.length, "length");
    if (path_segment.path_type == PathType::CIRCLE) {
        success &= check_vec(path_segment.arc_center, "arc_center");
        success &= check_scalar(path_segment.arc_theta, "arc_theta");
        success &= check_scalar(path_segment.arc_radius, "arc_radius");
    }
    const double eps = 1e-6;
    if ((path_segment.length + eps) < 0.1) {
        success = false;
    } 
    return success;
}

bool ParaFunc::check_path_segments(const std::vector<PathSegment> &path_segments) {
    if (path_segments.empty()) {
        return false;
    }
    for (const auto &path_seg : path_segments) {
        if (!check_path_segment(path_seg)) {
            return false;
        }
    }

    return true;
}


PLANNING_NAMESPACE_END
