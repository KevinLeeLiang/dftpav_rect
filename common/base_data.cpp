#include "base_data.h"
#include "math_helper.h"

PLANNING_NAMESPACE_START

std::string path_type_by_string(const PathType &path_type) {
    std::string type_name;
    switch (path_type) {
    case PathType::STRAIGHT: {
        type_name = "STRAIGHT";
        break;
    }
    case PathType::CIRCLE: {
        type_name = "CIRCLE";
        break;
    }
    case PathType::SMOOTH: {
        type_name = "SMOOTH";
        break;
    }
    default: {
        type_name = "UNDEFINED";
        break;
    }
    }

    return type_name;
}

void Path::merge(const std::vector<PathPoint> &path_points) { real_path.push_back(path_points); }

void Path::merge_points(const std::vector<PathPoint> &path_points) {
    if (real_path.empty()) {
        real_path.push_back(path_points);
    } else {
        for (const auto &path_point : path_points) {
            real_path.back().push_back(path_point);
        }
    }
}

Path::Ptr Path::mirror_injection() const {
    auto mirror_path = std::make_shared<Path>();
    *mirror_path = *this;
    mirror_path->start_pose = mirror_path->start_pose.mirror_injection();
    mirror_path->target_pose = mirror_path->target_pose.mirror_injection();
    for (auto &path_segment : mirror_path->path_segments) {
        path_segment.start_pose = path_segment.start_pose.mirror_injection();
        path_segment.target_pose = path_segment.target_pose.mirror_injection();
        path_segment.arc_center = Vec2d(-path_segment.arc_center.x(), path_segment.arc_center.y());
    }

    for (auto &path_point_vec : mirror_path->real_path) {
        for (auto &path_point : path_point_vec) {
            double x = -path_point.x();
            double theta = Rot2::normalize_angle(M_PI - path_point.theta());
            path_point.set_x(x);
            path_point.set_theta(theta);
        }
    }

    return mirror_path;
}

void Path::merge(const PathSegments &other_path_segments) {
    for (const auto &path_segment : other_path_segments) {
        target_pose = path_segment.target_pose;
        total_length += path_segment.length;
        if (!path_segments.empty()) {
            if (path_segments.back().gear != path_segment.gear) {
                gear_switch_count += 1;
                path_segments.push_back(path_segment);
            } else {
                if (path_segments.back().path_points.empty()) {
                    if (path_segments.back().path_type == PathType::STRAIGHT) {
                        path_segments.back().path_points = MathHelper::straight_sample(path_segments.back());
                    } else if (path_segments.back().path_type == PathType::CIRCLE) {
                        path_segments.back().path_points = MathHelper::arc_sample(path_segments.back());
                    }
                }
                path_segments.back().path_type = PathType::SMOOTH;
                path_segments.back().target_pose = path_segment.target_pose;
                path_segments.back().length += path_segment.length;
                std::vector<PathPoint> curr_points;
                if (path_segment.path_type == PathType::STRAIGHT) {
                    curr_points = MathHelper::straight_sample(path_segment);
                } else if (path_segment.path_type == PathType::CIRCLE) {
                    curr_points = MathHelper::arc_sample(path_segment);
                }
                double s_offset = path_segments.back().path_points.back().s();
                std::for_each(curr_points.begin(), curr_points.end(),
                              [s_offset](auto &pt) { pt.set_s(pt.s() + s_offset); });
                path_segments.back().path_points.insert(path_segments.back().path_points.end(), curr_points.begin(),
                                                        curr_points.end());
            }
        } else {
            start_pose = path_segment.start_pose;
            path_segments.push_back(path_segment);
        }
    }
}

void Path::print() const {
    std::stringstream ss;
    ss << "************************************************" << std::endl;
    if (success) {
        ss << "success!" << std::endl
           << "START: " << start_pose << std::endl
           << "END:   " << target_pose << std::endl
           << "TOTAL_LENGTH: " << total_length << std::endl
           << "GEAR_SWITCH_CNT: " << gear_switch_count << std::endl;
    }
    ss << "msg: " << msg << std::endl;
    for (const auto &path_segment : path_segments) {
        ss << "***********************" << std::endl;
        ss << path_type_by_string(path_segment.path_type) << std::endl
           << "start: " << path_segment.start_pose << std::endl
           << "end:   " << path_segment.target_pose << std::endl
           << "length: " << path_segment.length << std::endl;
        if (path_segment.arc_radius > 0.0) {
            if (path_segment.is_right) {
                ss << "right rotate" << std::endl;
            } else {
                ss << "left rotate" << std::endl;
            }
            ss << "center: [" << path_segment.arc_center.x() << ", " << path_segment.arc_center.y() << "] " << std::endl
               << "radius: " << path_segment.arc_radius << std::endl
               << "theta:   " << path_segment.arc_theta / M_PI * 180.0 << std::endl;
        }
        if (path_segment.gear == GearBoxInfoPb::GEAR_DRIVE) {
            ss << "GEAR_DRIVE" << std::endl;
        } else {
            ss << "GEAR_REVERSE" << std::endl;
        }
    }
    MLOG(PARKING_PLANNING, INFO) << ss.str();
}

Path::Path() { clear(); }

void Path::clear() {
    success = false;
    roi_success = false;
    start_pose = Pose2d();
    target_pose = Pose2d();
    total_length = 0.0;
    gear_switch_count = 0;
    curr_idx = 0;
    path_segments.clear();
    real_path.clear();
    msg.clear();
}

PLANNING_NAMESPACE_END
