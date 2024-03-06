#include "math_helper.h"
#include "math/math_utils.h"
#include "parking_planning/math/double/double.h"
#include "parking_planning/math/util.h"

PLANNING_NAMESPACE_START

PathSegment MathHelper::compute_max_rotate_angle(const PlanEnvironment::ConstPtr &park_env_ptr,
                                                 const PathSegment &path_segment, bool is_turning_left,
                                                 VehicleModel &vehicle_model, std::shared_ptr<Vec2d> related_point,
                                                 std::shared_ptr<LineSegment2d> related_line) {
    vehicle_model.update_pose(path_segment.start_pose);
    auto corners = vehicle_model.cut_corners();
    double max_angle = compute_max_rotate_angle(
        park_env_ptr, path_segment.start_pose, corners, path_segment.arc_radius, path_segment.arc_center,
        is_turning_left, path_segment.gear == GearBoxInfoPb::GEAR_DRIVE, vehicle_model, related_point, related_line);
    MLOG(PARKING_PLANNING, INFO) << "max theta: " << max_angle << ", theta: " << path_segment.arc_theta;
    if (std::abs(max_angle) < std::abs(path_segment.arc_theta)) {
        PathSegment new_path_segment = path_segment;
        new_path_segment.arc_theta = max_angle;
        new_path_segment.target_pose =
            get_target_loc(path_segment.start_pose, path_segment.arc_center, path_segment.arc_radius, max_angle);
        new_path_segment.length = max_angle * path_segment.arc_radius;
        return new_path_segment;
    }
    return path_segment;
}
PathSegment MathHelper::compute_max_straight_length(const PlanEnvironment::ConstPtr &park_env_ptr,
                                                    const PathSegment &path_segment, VehicleModel &vehicle_model,
                                                    std::shared_ptr<Vec2d> related_point,
                                                    std::shared_ptr<LineSegment2d> related_line) {
    double length = compute_max_straight_length(park_env_ptr, path_segment.start_pose, path_segment.target_pose,
                                                path_segment.gear == GearBoxInfoPb::GEAR_DRIVE, vehicle_model,
                                                related_point, related_line);
    MLOG(PARKING_PLANNING, INFO) << "max length: " << length << ", length: " << path_segment.length;
    if (std::abs(length) < std::abs(path_segment.length)) {
        PathSegment new_path_segment = path_segment;

        double traj_orientation = 1.0;
        if (path_segment.gear == GearBoxInfoPb::GEAR_REVERSE) {
            traj_orientation = -1.0;
        }
        double delta_x = traj_orientation * std::abs(length) * path_segment.start_pose.rotation().cos_theta();
        double delta_y = traj_orientation * std::abs(length) * path_segment.start_pose.rotation().sin_theta();

        new_path_segment.target_pose = Pose2d(path_segment.start_pose.x() + delta_x,
                                              path_segment.start_pose.y() + delta_y, path_segment.start_pose.theta());

        new_path_segment.length = length;
        return new_path_segment;
    }

    return path_segment;
}

double MathHelper::compute_max_rotate_angle(const PlanEnvironment::ConstPtr &park_env_ptr, const Pose2d &start_pose,
                                            const std::vector<Vec2d> &vehicle_corners, const double &radius,
                                            const Vec2d &center, bool is_turning_left, bool is_forward,
                                            VehicleModel &vehicle_model, std::shared_ptr<Vec2d> related_point,
                                            std::shared_ptr<LineSegment2d> related_line) {

    const auto &obstacle_points = park_env_ptr->obstacle_points;

    const auto &obstacle_lines = park_env_ptr->obstacle_edges;
    //  obstacle_lines.push_back(park_env_ptr->slot.left_edge());
    //  obstacle_lines.push_back(park_env_ptr->slot.right_edge());
    //  obstacle_lines.push_back(park_env_ptr->slot.rear_edge());
    double safety_thresh = 0.0;
    if (park_env_ptr->slot.park_type() == ParkType::VERTICAL || park_env_ptr->slot.park_type() == ParkType::OBLIQUE) {
        safety_thresh = std::max(1e-6, FLAGS_apa_vert_vehicle_safety_thresh / radius);
    } else {
        safety_thresh = std::max(
            1e-6,
            (FLAGS_apa_para_vehicle_lon_inflation - FLAGS_apa_para_vehicle_lat_inflation * std::sqrt(2)) / radius);
    }
    return compute_max_rotate_angle(start_pose, vehicle_corners, obstacle_points, obstacle_lines, center,
                                    is_turning_left, is_forward, safety_thresh, related_point, related_line);
}

double MathHelper::compute_max_straight_length(const PlanEnvironment::ConstPtr &park_env_ptr, const Pose2d &start_pose,
                                               const Pose2d &end_pose, bool is_forward, VehicleModel &vehicle_model,
                                               std::shared_ptr<Vec2d> related_point,
                                               std::shared_ptr<LineSegment2d> related_line) {
    const auto &obstacle_points = park_env_ptr->obstacle_points;
    const auto &obstacle_lines = park_env_ptr->obstacle_edges;

    double safety_thresh = 0.0;
    if (park_env_ptr->slot.park_type() == ParkType::VERTICAL || park_env_ptr->slot.park_type() == ParkType::OBLIQUE) {
        safety_thresh = FLAGS_apa_vert_vehicle_safety_thresh;
    } else {
        safety_thresh = FLAGS_apa_para_vehicle_lon_inflation - FLAGS_apa_para_vehicle_lat_inflation;
    }

    vehicle_model.update_pose(start_pose);
    std::vector<Vec2d> vehicle_corners = vehicle_model.corners();
    double straight_length = (start_pose.translation() - end_pose.translation()).Length();
    return compute_max_straight_length(start_pose, vehicle_corners, obstacle_points, obstacle_lines, straight_length,
                                       is_forward, safety_thresh, related_point, related_line);
}

std::vector<PathPoint> MathHelper::straight_sample(const PathSegment &path_segment) {
    std::vector<PathPoint> result;
    double length = path_segment.length;

    double traj_orientation = 1.0;
    if (path_segment.gear == GearBoxInfoPb::GEAR_REVERSE) {
        traj_orientation = -1.0;
    }

    double delta_l = FLAGS_apa_output_trajectory_length_resolution;
    double discrete_point_num = std::fabs(std::ceil(length / delta_l)) + 1;
    if (discrete_point_num < 2) {
        MLOG(PARKING_PLANNING, INFO) << "path points num is less than 2";
        discrete_point_num = 2;
    }

    for (size_t i = 0; i < discrete_point_num; ++i) {
        double s = (i * delta_l) > length ? length : (i * delta_l);
        s *= traj_orientation;
        double delta_x = s * path_segment.start_pose.rotation().cos_theta();
        double delta_y = s * path_segment.start_pose.rotation().sin_theta();

        PathPoint path_point;
        path_point.set_x(path_segment.start_pose.x() + delta_x);
        path_point.set_y(path_segment.start_pose.y() + delta_y);
        path_point.set_theta(path_segment.start_pose.theta());
        path_point.set_kappa(0.0);
        path_point.set_s(std::fabs(s));
        result.push_back(std::move(path_point));
    }
    return result;
}

std::vector<PathPoint> MathHelper::arc_sample(const PathSegment &path_segment) {
    std::vector<PathPoint> result;
    double traj_theta = haomo::hidelivery::math::NormalizeAngle(path_segment.arc_theta);
    double curvature_sign = (traj_theta > 0.0) ? 1.0 : -1.0;

    if (path_segment.gear == GearBoxInfoPb::GEAR_REVERSE) {
        curvature_sign *= -1.0;
    }

    double theta = std::atan2((path_segment.start_pose.y() - path_segment.arc_center.y()),
                              (path_segment.start_pose.x() - path_segment.arc_center.x()));
    double trajectory_angle_resolution = FLAGS_apa_output_trajectory_length_resolution / path_segment.arc_radius;
    double delta_angle = traj_theta > 0 ? trajectory_angle_resolution : -trajectory_angle_resolution;
    int num_points = std::ceil(traj_theta / delta_angle) + 1;
    if (num_points < 2) {
        MLOG(PARKING_PLANNING, INFO) << "path points num is less than 2";
        num_points = 2;
    }
    for (int i = 0; i < num_points; ++i) {
        double delta_theta = (std::fabs(i * delta_angle) > std::fabs(traj_theta)) ? traj_theta : (i * delta_angle);

        double s = std::fabs(delta_theta * path_segment.arc_radius);
        double temp_theta = theta + delta_theta;
        double point_theta = haomo::hidelivery::math::NormalizeAngle(path_segment.start_pose.theta() + delta_theta);

        PathPoint path_point;
        path_point.set_x(path_segment.arc_center.x() + path_segment.arc_radius * cos(temp_theta));
        path_point.set_y(path_segment.arc_center.y() + path_segment.arc_radius * sin(temp_theta));
        path_point.set_theta(point_theta);
        path_point.set_kappa(curvature_sign * 1.0 / path_segment.arc_radius);
        path_point.set_s(s);
        result.push_back(std::move(path_point));
    }

    return result;
}

void parse_related_obstacle(std::shared_ptr<Vec2d> related_point, std::shared_ptr<LineSegment2d> related_line,
                            const std::vector<size_t> &od_indices, const std::vector<bool> &od_type,
                            const std::vector<LineSegment2d> &obstacle_lines, const std::vector<Vec2d> &obstacle_points,
                            size_t i) {
    if (related_line && related_point) {
        auto od_idx = od_indices[i];
        if (od_type[i]) {

            *related_line = obstacle_lines[od_idx];
            *related_point = Vec2d(1e8, 1e8);
        } else {
            *related_line = LineSegment2d(Vec2d(1e8, 1e8), Vec2d(1e8, 1e8));
            *related_point = obstacle_points[od_idx];
        }
    }
}

double MathHelper::compute_max_rotate_angle(const Pose2d &start_pose, const std::vector<Vec2d> &vehicle_corners,
                                            const std::vector<Vec2d> &obstacle_points,
                                            const std::vector<LineSegment2d> &obstacle_lines, const Vec2d &arc_center,
                                            const bool &is_turning_left, const bool &is_forward,
                                            const double &safety_thresh, std::shared_ptr<Vec2d> related_point,
                                            std::shared_ptr<LineSegment2d> related_line) {

    Polygon2d polygon = Polygon2d(vehicle_corners);
    for (auto point : obstacle_points) {
        if (polygon.IsPointIn(point) || polygon.IsPointOnBoundary(point)) {
            if (related_point != nullptr) {
                *related_point = point;
            }
            return 0.0;
        }
    }
    for (auto line : obstacle_lines) {
        if (polygon.HasOverlap(line)) {
            return 0.0;
        }
    }
    int steer_direction = is_turning_left ? 1 : -1;
    int gear_direction = is_forward ? 1 : -1;
    int is_counter_clockwise = steer_direction * gear_direction;
    double steer_central_angle = is_counter_clockwise * M_PI;
    std::vector<double> theta_vector;
    std::vector<bool> od_type;
    std::vector<size_t> od_indices;
    for (const auto &vehicle_corner : vehicle_corners) {
        for (size_t i = 0; i < obstacle_lines.size(); ++i) {
            const auto &obstacle_line = obstacle_lines[i];
            double radius = (vehicle_corner - arc_center).Length();
            auto intersection_pairs = compute_line_circle_intersection(arc_center, radius, obstacle_line);
            for (const auto &intersection_pair : intersection_pairs) {
                if (intersection_pair.first >= 0 && intersection_pair.first <= 1.0) {
                    double theta = compute_rotate_angle(vehicle_corner, intersection_pair.second, arc_center);
                    theta_vector.push_back(theta);
                    od_type.push_back(true);
                    od_indices.push_back(i);
                }
            }
        }
    }
    std::vector<LineSegment2d> vehicle_edges;
    if (!vehicle_corners.empty()) {
        for (size_t i = 0; i < vehicle_corners.size(); ++i) {
            vehicle_edges.emplace_back(vehicle_corners.at(i), vehicle_corners.at((i + 1) % (vehicle_corners.size())));
        }
    }
    for (size_t i = 0; i < obstacle_points.size(); ++i) {
        const auto &obstacle_point = obstacle_points[i];
        for (const auto &vehicle_edge : vehicle_edges) {
            double radius = (obstacle_point - arc_center).Length();
            auto intersection_pairs = compute_line_circle_intersection(arc_center, radius, vehicle_edge);
            for (const auto &intersection_pair : intersection_pairs) {
                if (intersection_pair.first >= 0 && intersection_pair.first <= 1.0) {
                    double theta = -compute_rotate_angle(obstacle_point, intersection_pair.second, arc_center);
                    theta_vector.push_back(theta);
                    od_type.push_back(false);
                    od_indices.push_back(i);
                }
            }
        }
    }

    for (size_t i = 0; i < theta_vector.size(); ++i) {
        const auto &max_theta_to_obs = theta_vector[i];
        if (is_counter_clockwise * max_theta_to_obs < 0) {
        } else if (std::abs(max_theta_to_obs) <= safety_thresh || std::isnan(max_theta_to_obs)) {
            steer_central_angle = 0.0;
            if (std::abs(max_theta_to_obs) <= safety_thresh) {
                parse_related_obstacle(related_point, related_line, od_indices, od_type, obstacle_lines,
                                       obstacle_points, i);
            }
        } else if (is_counter_clockwise > 0) {
            double angle_minus_thresh = max_theta_to_obs - safety_thresh;
            if (angle_minus_thresh < steer_central_angle) {
                steer_central_angle = angle_minus_thresh;
                parse_related_obstacle(related_point, related_line, od_indices, od_type, obstacle_lines,
                                       obstacle_points, i);
            }
        } else {
            double angle_minus_thresh = max_theta_to_obs + safety_thresh;
            if (angle_minus_thresh > steer_central_angle) {
                steer_central_angle = angle_minus_thresh;
                parse_related_obstacle(related_point, related_line, od_indices, od_type, obstacle_lines,
                                       obstacle_points, i);
            }
        }
    }

    return steer_central_angle;
}

double MathHelper::compute_max_straight_length(const Pose2d &start_pose, const std::vector<Vec2d> &vehicle_corners,
                                               const std::vector<Vec2d> &obstacle_points,
                                               const std::vector<LineSegment2d> &obstacle_lines,
                                               const double &straight_length, const bool &is_forward,
                                               const double &safety_thresh, std::shared_ptr<Vec2d> related_point,
                                               std::shared_ptr<LineSegment2d> related_line) {

    Polygon2d polygon = Polygon2d(vehicle_corners);
    for (auto point : obstacle_points) {
        if (polygon.IsPointIn(point) || polygon.IsPointOnBoundary(point)) {
            if (related_point) {
                *related_point = point;
            }
            return 0.0;
        }
    }
    for (auto line : obstacle_lines) {
        if (polygon.HasOverlap(line)) {
            return 0.0;
        }
    }

    double safe_straight_length = straight_length;
    std::vector<Vec2d> points;
    double tmp_distance = 0.0;
    LineSegment2d line;
    if (is_forward) {
        line = LineSegment2d{vehicle_corners[3], vehicle_corners[2]};
        tmp_distance = straight_length + safety_thresh;

    } else {
        line = LineSegment2d{vehicle_corners[0], vehicle_corners[1]};
        tmp_distance = -(straight_length + safety_thresh);
    }

    Pose2d p1{start_pose.rotation(), line.start()};
    Pose2d p2{start_pose.rotation(), line.end()};
    Pose2d end_to_start{Rot2::identity(), Vec2d(tmp_distance, 0)};

    if (is_forward) {
        points.push_back(vehicle_corners[0]);
        points.push_back(vehicle_corners[1]);
    } else {
        points.push_back(vehicle_corners[3]);
        points.push_back(vehicle_corners[2]);
    }
    points.push_back((p2 * end_to_start).translation());
    points.push_back((p1 * end_to_start).translation());

    Polygon2d ego_polygon(points);
    for (const auto &line_segment : obstacle_lines) {
        if (ego_polygon.HasOverlap(line_segment)) {
            Vec2d first;
            Vec2d last;
            bool res = ego_polygon.GetOverlap(line_segment, &first, &last);
            if (res) {
                double first_dis = line.DistanceTo(first);
                double second_dis = line.DistanceTo(last);
                double min_distance = std::min(first_dis, second_dis) - safety_thresh;
                // safe_straight_length = std::min(safe_straight_length, min_distance);
                if (min_distance < safe_straight_length) {
                    if (related_line && related_point) {
                        *related_line = line_segment;
                        *related_point = Vec2d(1e8, 1e8);
                    }
                    safe_straight_length = min_distance;
                }
            }
        }
    }
    for (const auto &point : obstacle_points) {
        if (ego_polygon.IsPointIn(point)) {
            double dis = line.DistanceTo(point) - safety_thresh;
            if (dis < safe_straight_length) {
                if (related_line && related_point) {
                    *related_line = LineSegment2d(Vec2d(1e8, 1e8), Vec2d(1e8, 1e8));
                    *related_point = point;
                }
                safe_straight_length = dis;
            }
        }
    }

    return safe_straight_length;
}

double MathHelper::compute_max_straight_length_with_cut(
    const Pose2d &start_pose, const std::vector<Vec2d> &vehicle_corners, const std::vector<Vec2d> &obstacle_points,
    const std::vector<LineSegment2d> &obstacle_lines, const double &straight_length, const bool &is_forward,
    const double &safety_thresh, std::shared_ptr<Vec2d> related_point, std::shared_ptr<LineSegment2d> related_line) {
    double safe_straight_length = straight_length;
    std::vector<Vec2d> points;
    double tmp_distance = 0.0;
    double gap_1 = 0.0, gap_2 = 0.0, gap_3 = 0.0, veh_len = 0.0;
    LineSegment2d line, l2, l3;
    LineSegment2d l4{vehicle_corners[0], vehicle_corners[7]};
    Pose2d p1, p2, p3, p4;
    if (is_forward) {
        line = LineSegment2d{vehicle_corners[1], vehicle_corners[2]};
        tmp_distance = straight_length + safety_thresh;

        p1 = {start_pose.rotation(), vehicle_corners[4]};
        p2 = {start_pose.rotation(), vehicle_corners[5]};
        p3 = {start_pose.rotation(), vehicle_corners[6]};
        p4 = {start_pose.rotation(), vehicle_corners[7]};
        l2 = LineSegment2d{vehicle_corners[7], vehicle_corners[6]};
        l3 = LineSegment2d{vehicle_corners[4], vehicle_corners[5]};
        gap_1 = std::fabs(l4.ProductOntoUnit(vehicle_corners[6]));
        gap_2 = std::fabs(l4.ProductOntoUnit(vehicle_corners[5]));
        gap_3 = std::fabs(l4.ProductOntoUnit(vehicle_corners[4]));
        veh_len = std::fabs(line.ProductOntoUnit(vehicle_corners[5]));
    } else {
        line = LineSegment2d{vehicle_corners[5], vehicle_corners[6]};
        tmp_distance = -(straight_length + safety_thresh);

        p1 = {start_pose.rotation(), vehicle_corners[3]};
        p2 = {start_pose.rotation(), vehicle_corners[2]};
        p3 = {start_pose.rotation(), vehicle_corners[1]};
        p4 = {start_pose.rotation(), vehicle_corners[0]};
        l2 = LineSegment2d{vehicle_corners[0], vehicle_corners[1]};
        l3 = LineSegment2d{vehicle_corners[3], vehicle_corners[2]};
        gap_1 = std::fabs(l4.ProductOntoUnit(vehicle_corners[1]));
        gap_2 = std::fabs(l4.ProductOntoUnit(vehicle_corners[2]));
        gap_3 = std::fabs(l4.ProductOntoUnit(vehicle_corners[3]));
        veh_len = std::fabs(line.ProductOntoUnit(vehicle_corners[1]));
    }
    Pose2d end_to_start{Rot2::identity(), Vec2d(tmp_distance, 0)};

    if (is_forward) {
        points.push_back(vehicle_corners[0]);
        points.push_back(vehicle_corners[1]);
        points.push_back(vehicle_corners[2]);
        points.push_back(vehicle_corners[3]);
    } else {
        points.push_back(vehicle_corners[7]);
        points.push_back(vehicle_corners[6]);
        points.push_back(vehicle_corners[5]);
        points.push_back(vehicle_corners[4]);
    }
    points.push_back((p1 * end_to_start).translation());
    points.push_back((p2 * end_to_start).translation());
    points.push_back((p3 * end_to_start).translation());
    points.push_back((p4 * end_to_start).translation());

    Polygon2d ego_polygon(points);
    for (const auto &line_segment : obstacle_lines) {
        if (ego_polygon.HasOverlap(line_segment)) {
            Vec2d first;
            Vec2d last;
            bool res = ego_polygon.GetOverlap(line_segment, &first, &last);
            if (res) {
                double first_dis = 0.0;
                double gap = std::fabs(l4.ProductOntoUnit(first));
                if (gap < gap_1) {
                    Vec2d gen_pt = first + (1.0 - gap / gap_1) * l2.length() * l2.unit_direction();
                    first_dis = std::max(std::fabs(line.ProductOntoUnit(gen_pt)) - veh_len, 0.0);
                } else if (gap <= gap_2) {
                    first_dis = std::fabs(line.ProductOntoUnit(first));
                } else {
                    Vec2d gen_pt = first + ((gap - gap_2) / (gap_3 - gap_2)) * l3.length() * l3.unit_direction();
                    first_dis = std::max(std::fabs(line.ProductOntoUnit(gen_pt)) - veh_len, 0.0);
                }

                double second_dis = 0.0;
                gap = std::fabs(l4.ProductOntoUnit(last));
                if (gap < gap_1) {
                    Vec2d gen_pt = last + (1.0 - gap / gap_1) * l2.length() * l2.unit_direction();
                    second_dis = std::max(std::fabs(line.ProductOntoUnit(gen_pt)) - veh_len, 0.0);
                } else if (gap <= gap_2) {
                    second_dis = std::fabs(line.ProductOntoUnit(last));
                } else {
                    Vec2d gen_pt = last + ((gap - gap_2) / (gap_3 - gap_2)) * l3.length() * l3.unit_direction();
                    second_dis = std::max(std::fabs(line.ProductOntoUnit(gen_pt)) - veh_len, 0.0);
                }

                double min_distance = std::min(first_dis, second_dis) - safety_thresh;
                // safe_straight_length = std::min(safe_straight_length, min_distance);
                if (min_distance < safe_straight_length) {
                    if (related_line && related_point) {
                        *related_line = line_segment;
                        *related_point = Vec2d(1e8, 1e8);
                    }
                    safe_straight_length = min_distance;
                }
            }
        }
    }
    for (const auto &point : obstacle_points) {
        if (ego_polygon.IsPointIn(point)) {
            double dis = 0.0;
            double gap = std::fabs(l4.ProductOntoUnit(point));
            if (gap < gap_1) {
                Vec2d gen_pt = point + (1.0 - gap / gap_1) * l2.length() * l2.unit_direction();
                dis = std::max(std::fabs(line.ProductOntoUnit(gen_pt)) - veh_len, 0.0) - safety_thresh;
            } else if (gap <= gap_2) {
                dis = std::fabs(line.ProductOntoUnit(point)) - safety_thresh;
            } else {
                Vec2d gen_pt = point + ((gap - gap_2) / (gap_3 - gap_2)) * l3.length() * l3.unit_direction();
                dis = std::max(std::fabs(line.ProductOntoUnit(gen_pt)) - veh_len, 0.0) - safety_thresh;
            }
            if (dis < safe_straight_length) {
                if (related_line && related_point) {
                    *related_line = LineSegment2d(Vec2d(1e8, 1e8), Vec2d(1e8, 1e8));
                    *related_point = point;
                }
                safe_straight_length = dis;
            }
        }
    }

    return std::max(safe_straight_length, 0.0);
}

std::vector<std::pair<double, Vec2d>>
MathHelper::compute_line_circle_intersection(const Vec2d &arc_center, double radius, const LineSegment2d &line) {
    std::vector<std::pair<double, Vec2d>> result;

    double dist_line = line.DistanceTo(arc_center);

    if (radius < dist_line) {
        return result;
    }

    double x1 = line.start().x(), y1 = line.start().y();
    double x2 = line.end().x(), y2 = line.end().y();

    double A = std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2);
    double B = 2 * (x2 - arc_center.x()) * (x1 - x2) + 2 * (y2 - arc_center.y()) * (y1 - y2);
    double C = std::pow(x2 - arc_center.x(), 2) + std::pow(y2 - arc_center.y(), 2) - std::pow(radius, 2);

    double lambda_1 = (-B + sqrt(std::pow(B, 2) - 4 * A * C)) / (2 * A);
    double lambda_2 = (-B - sqrt(std::pow(B, 2) - 4 * A * C)) / (2 * A);

    Vec2d intersection_point_1(lambda_1 * x1 + (1 - lambda_1) * x2, lambda_1 * y1 + (1 - lambda_1) * y2);

    result.emplace_back(lambda_1, intersection_point_1);

    if (std::abs(lambda_1 - lambda_2) > 1e-3) {
        Vec2d intersection_point_2(lambda_2 * x1 + (1 - lambda_2) * x2, lambda_2 * y1 + (1 - lambda_2) * y2);

        result.emplace_back(lambda_2, intersection_point_2);
    }

    return result;
}

double MathHelper::compute_rotate_angle(const Vec2d &corner, const Vec2d &intersection_point, const Vec2d &arc_center) {
    Vec2d corner_vec = corner - arc_center;
    Vec2d intersection_vec = intersection_point - arc_center;
    int clockwise = corner_vec.CrossProd(intersection_vec) > 0 ? 1 : -1;
    double cos_value = std::max(
        -1.0,
        std::min(corner_vec.InnerProd(intersection_vec) / (corner_vec.Length() * intersection_vec.Length()), 1.0));
    double theta = clockwise * acos(cos_value);

    return theta;
}

Vec2d MathHelper::center(const Pose2d &start_pose, double radius, bool is_right) {
    double x = 0.0, y = 0.0;
    if (is_right) {
        x = start_pose.x() + radius * sin(start_pose.theta());
        y = start_pose.y() - radius * cos(start_pose.theta());
    } else {
        x = start_pose.x() - radius * sin(start_pose.theta());
        y = start_pose.y() + radius * cos(start_pose.theta());
    }
    return Vec2d(x, y);
}

bool MathHelper::parallel_able_park(const Pose2d &start_pose, const Pose2d &target_pose, const double &aim_R,
                                    const std::string &park_state, const double &safty_front_distance_buffer,
                                    const double &safty_rear_distance_buffer) {
    VehicleModel vehicle_model(0.0);
    vehicle_model.update_pose(start_pose);
    bool is_right = true;
    if (park_state == "Parallel_RotateDown") {
        if ((start_pose.y() < target_pose.y() && start_pose.theta() >= 0) ||
            (start_pose.y() > target_pose.y() && start_pose.theta() <= 0)) {
            MLOG(PARKING_PLANNING, DEBUG) << "vehicle_loc is not corrcet";
            return false;
        }
        is_right = start_pose.y() > target_pose.y() ? false : true;
    } else if (park_state == "Parallel_RotateUp") {
        if ((start_pose.y() > target_pose.y() && start_pose.theta() >= 0) ||
            (start_pose.y() < target_pose.y() && start_pose.theta() <= 0)) {
            MLOG(PARKING_PLANNING, DEBUG) << "vehicle_loc is not corrcet";
            return false;
        }
        is_right = start_pose.y() > target_pose.y() ? false : true;
    }
    double theta_rotate = 0 - start_pose.theta();
    double delta_y = fabs(target_pose.y() - start_pose.y());
    double vehicle_rear_circle = delta_y / (1.0 - std::cos(theta_rotate));
    MLOG(PARKING_PLANNING, DEBUG) << "park vehicle_rear_circle: " << vehicle_rear_circle << " aim_R: " << aim_R;
    const double eps = 1e-6;
    if (vehicle_rear_circle + eps < aim_R) {
        MLOG(PARKING_PLANNING, DEBUG) << "vehicle_rear_circle too small";
        return false;
    }
    if (vehicle_rear_circle > 100) {
        MLOG(PARKING_PLANNING, DEBUG) << "vehicle_rear_circle too big";
        return false;
    }
    Vec2d Center = center(start_pose, vehicle_rear_circle, is_right);
    Pose2d end_loc = get_target_loc(start_pose, Center, vehicle_rear_circle, theta_rotate);
    VehicleModel end_vehicle_model(0.0);
    end_vehicle_model.update_pose(end_loc);
    if ((end_vehicle_model.rear_left().x() < safty_rear_distance_buffer) ||
        (end_vehicle_model.front_left().x() > safty_front_distance_buffer)) {
        MLOG(PARKING_PLANNING, DEBUG) << "cross park bound";
        MLOG(PARKING_PLANNING, DEBUG) << "end_corner[0].x: " << end_vehicle_model.rear_left().x()
                                      << " end_corner[3].x: " << end_vehicle_model.front_left().x();
        return false;
    }
    return true;
}

bool MathHelper::able_park(const Pose2d &start_pose, const Pose2d &target_pose, const double &aim_R,
                           const double &safty_bottom_distance_buffer) {
    Pose2d start_to_target = target_pose.inverse() * start_pose;
    double delta = fabs(start_to_target.y());
    double theta = fabs(start_to_target.theta());
    double radius = delta / (1 - cos(theta));

    // 判断车是否超出车位中轴线
    if (start_to_target.y() > 0) {
        return false;
    }

    // 判断是否大于最小转弯半径
    if (radius <= aim_R) {
        MLOG(PARKING_PLANNING, DEBUG) << "radius: " << radius << ", aim_R: " << aim_R;
        return false;
    }

    //判断车终点车尾是否超出安全距离
    auto circle_center = center(start_to_target, radius, true);
    if (std::isnan(circle_center.x()) || circle_center.x() < safty_bottom_distance_buffer) {
        return false;
    }
    return true;
}

bool MathHelper::ready_park(const Pose2d &start_pose, const Pose2d &target_pose) {
    Pose2d start_to_target = target_pose.inverse() * start_pose;
    double error_x = fabs(start_to_target.y());
    double error_heading = fabs(start_to_target.theta());

    MLOG(PARKING_PLANNING, INFO) << "planning:ready_park error_x: " << error_x << " "
                                 << FLAGS_apa_vert_plan_x_tolerance;
    MLOG(PARKING_PLANNING, INFO) << "planning:ready_park error_yaw: " << error_heading << " "
                                 << FLAGS_apa_vert_plan_heading_tolerance;
    if (error_x < FLAGS_apa_vert_plan_x_tolerance && error_heading < FLAGS_apa_vert_plan_heading_tolerance) {
        return true;
    }
    return false;
}

bool MathHelper::parallel_ready_park(const Pose2d &start_pose, const Pose2d &target_pose) {
    double error_y = fabs(start_pose.y() - target_pose.y());
    double error_heading = fabs(start_pose.theta() - target_pose.theta());

    double parallel_park_y_tolerance = FLAGS_apa_para_park_y_tolerance;
    double parallel_park_error_heading = FLAGS_apa_vert_target_heading_tolerance;
    MLOG(PARKING_PLANNING, DEBUG) << "planning:ready_park error_x: " << error_y << " " << parallel_park_y_tolerance;
    MLOG(PARKING_PLANNING, DEBUG) << "planning:ready_park error_yaw: " << error_heading << " "
                                  << parallel_park_error_heading;
    if (error_y < parallel_park_y_tolerance && error_heading < parallel_park_error_heading) {
        return true;
    }
    return false;
}

double MathHelper::dis_point_to_line(const Vec2d &point_a, const Vec2d &point_b, const Vec2d &point_p) {
    using ::haomo::parking::planning::Double;
    double k_line = 0.0;
    double b_line = 0.0;
    double dis = 0.0;
    if (Double::compare(point_a.x(), point_b.x()) == 0) {
        dis = fabs(point_p.x() - point_a.x());
    } else {
        k_line = (point_a.y() - point_b.y()) / (point_a.x() - point_b.x());
        b_line = point_a.y() - k_line * point_a.x();
        dis = fabs(k_line * point_p.x() - point_p.y() + b_line) / sqrt(1 + k_line * k_line);
    }
    return dis;
}

Pose2d MathHelper::get_target_loc(const Pose2d &vehicle_state, const Vec2d &center_of_circle,
                                  const double &circle_radius, const double &theta) {
    Pose2d vehicle_loc;
    double current_theta =
        atan2((vehicle_state.y() - center_of_circle.y()), (vehicle_state.x() - center_of_circle.x()));
    double end_theta = current_theta + theta;                             // 确定终止点与圆心夹角
    double end_x = center_of_circle.x() + circle_radius * cos(end_theta); // 确定终止点x坐标
    double end_y = center_of_circle.y() + circle_radius * sin(end_theta); // 确定终止点y坐标
    double end_heading = vehicle_state.theta() + theta;

    return Pose2d(end_x, end_y, end_heading);
}

LineSegment2d MathHelper::transform_line(const LineSegment2d &line, const Eigen::Isometry3d &T_world_local,
                                         bool mirror) {
    Eigen::Vector3d start(line.start().x(), line.start().y(), 0.0);
    Eigen::Vector3d end(line.end().x(), line.end().y(), 0.0);
    if (mirror) {
        start.x() = -start.x();
        end.x() = -end.x();
    }

    Eigen::Vector3d world_start = T_world_local * start;
    Eigen::Vector3d world_end = T_world_local * end;

    return LineSegment2d(Vec2d(world_start.x(), world_start.y()), Vec2d(world_end.x(), world_end.y()));
}

PathPoint MathHelper::transform_path_point(const PathPoint &path_point, const Eigen::Isometry3d &T_world_local,
                                           bool mirror) {
    Pose2d pose_local(path_point.x(), path_point.y(), path_point.theta());
    if (mirror) {
        pose_local = pose_local.mirror_injection();
    }
    Eigen::Isometry3d pose_world_iso = T_world_local * pose_local.to_iso3d();
    auto theta_world = Rot2::extract_yaw_from_rotation_matrix(pose_world_iso.linear());
    PathPoint path_point_world;
    path_point_world.CopyFrom(path_point);
    path_point_world.set_x(pose_world_iso.translation().x());
    path_point_world.set_y(pose_world_iso.translation().y());
    path_point_world.set_z(pose_world_iso.translation().z());
    path_point_world.set_theta(theta_world);

    return path_point_world;
}

std::vector<TrajectoryPoint> MathHelper::transform_trajectory(const std::vector<TrajectoryPoint> &trajectory_local,
                                                              const Eigen::Isometry3d &T_world_local, bool mirror) {
    std::vector<TrajectoryPoint> result;
    for (const auto &trajectory_point_local : trajectory_local) {
        auto path_point_world = transform_path_point(trajectory_point_local.path_point(), T_world_local, mirror);
        TrajectoryPoint trajectory_point_world;
        trajectory_point_world.CopyFrom(trajectory_point_local);
        trajectory_point_world.mutable_path_point()->CopyFrom(path_point_world);
        result.push_back(trajectory_point_world);
    }

    return result;
}

std::vector<std::vector<PathPoint>>
MathHelper::transform_path_points(const std::vector<std::vector<PathPoint>> &path_points_vec,
                                  const Eigen::Isometry3d &T_world_local, bool mirror) {
    std::vector<std::vector<PathPoint>> result;
    for (const auto &path_points : path_points_vec) {
        result.emplace_back();
        for (const auto &path_point : path_points) {
            auto path_point_world = transform_path_point(path_point, T_world_local, mirror);
            result.back().push_back(path_point_world);
        }
    }

    return result;
}

Pose2d MathHelper::transform_pose(const Pose2d &pose_local, const Eigen::Isometry3d &T, bool mirror) {
    Pose2d pose = pose_local;
    if (mirror) {
        pose = pose_local.mirror_injection();
    }
    Eigen::Isometry3d pose_world_iso = T * pose.to_iso3d();
    return Pose2d::from_iso3d(pose_world_iso);
}

Vec2d MathHelper::transform_vec2d(const Vec2d &point_local, const Eigen::Isometry3d &T, bool mirror) {
    Eigen::Vector3d point_3d(point_local.x(), point_local.y(), 0);
    if (mirror) {
        point_3d.x() = -point_3d.x();
    }
    Eigen::Vector3d point_3d_world = T * point_3d;
    return Vec2d(point_3d_world.x(), point_3d_world.y());
}

PathSegments MathHelper::srs_path(const Pose2d &start_pose, const Pose2d &target_pose,
                                  const double &min_turning_radius) {
    auto target_pose_proj = start_pose.inverse() * target_pose;
    /*
     * s0 = [x0, y0] = [0,  0, 0]
     * s1 = [x1, y1] = [l1, 0, 0]
     * s2 = [x2, y2] = [x1 - R  * sin(theta),   R * cos(theta) - R, theta]
     * s3 = [x3, y3] = [x2 + l2 * cos(theta), y2 + l2 * sin(theta), theta]
     * x3 = l1 - R  * sin(theta) + l2 * cos(theta)
     * y3 = R * cos(theta) + l2 * sin(theta) - R
     */

    const double theta = target_pose_proj.theta();
    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);
    const double R_sin_theta = min_turning_radius * sin_theta;
    const double R_cos_theta = min_turning_radius * cos_theta;

    if (sin_theta < 1e-2) {
        return {};
    }

    const double x3 = target_pose_proj.x();
    const double y3 = target_pose_proj.y();

    const double l2 = (y3 - R_cos_theta + min_turning_radius) / sin_theta;
    const double l1 = x3 + R_sin_theta - l2 * cos_theta;
    MLOG(PARKING_PLANNING, DEBUG) << "l1: " << l1;
    MLOG(PARKING_PLANNING, DEBUG) << "l2: " << l2;

    const double x1 = l1;

    const double x2 = x1 - R_sin_theta;
    const double y2 = R_cos_theta - min_turning_radius;

    Pose2d s1(x1, 0, 0);
    Pose2d s2(x2, y2, target_pose_proj.theta());

    Pose2d s1_real = start_pose * s1;
    Pose2d s2_real = start_pose * s2;

    PathSegment p1;
    p1.path_type = PathType::STRAIGHT;
    p1.start_pose = start_pose;
    p1.target_pose = s1_real;
    p1.length = std::abs(l1);
    p1.gear = compute_gear(start_pose, s1_real);

    PathSegment p2;
    p2.path_type = PathType::CIRCLE;
    p2.start_pose = s1_real;
    p2.length = std::abs(target_pose_proj.theta()) * min_turning_radius;
    p2.arc_radius = min_turning_radius;
    p2.arc_center = center(s1_real, min_turning_radius, true);
    p2.arc_theta = target_pose_proj.theta();
    p2.target_pose = s2_real;
    p2.is_right = true;
    p2.gear = compute_gear(s1_real, s2_real);

    PathSegment p3;
    p3.path_type = PathType::STRAIGHT;
    p3.start_pose = s2_real;
    p3.target_pose = target_pose;
    p3.length = std::abs(l2);
    p3.gear = compute_gear(s2_real, p3.target_pose);

    return {p1, p2, p3};
}

GearBoxInfoPb::GearNum MathHelper::compute_gear(const Pose2d &start_pose, const Pose2d &target_pose) {
    Pose2d diff2 = start_pose.inverse() * target_pose;
    GearBoxInfoPb::GearNum gear = GearBoxInfoPb::GEAR_DRIVE;
    if (diff2.x() < 0) {
        gear = GearBoxInfoPb::GEAR_REVERSE;
    }

    return gear;
}

double MathHelper::compute_path_cost(Path::Ptr path) {
    double cost = 10 * path->gear_switch_count + path->total_length;
    MLOG(PARKING_PLANNING, INFO) << "gear_switch_count " << path->gear_switch_count;
    MLOG(PARKING_PLANNING, INFO) << "total_length " << path->total_length;
    MLOG(PARKING_PLANNING, INFO) << "cost " << cost;
    return cost;
}

bool MathHelper::path_segments_check_collision(const PathSegments &path_segments,
                                               const std::vector<Vec2d> &obstacle_points,
                                               const std::vector<LineSegment2d> &obstacle_lines,
                                               VehicleModel &vehicle_model, std::shared_ptr<Vec2d> related_point,
                                               std::shared_ptr<LineSegment2d> related_line, const double &safe_thresh) {
    return path_segments_check_collision(path_segments, obstacle_points, obstacle_lines, vehicle_model, vehicle_model,
                                         related_point, related_line, safe_thresh);
}

bool MathHelper::path_segments_check_collision(const PathSegments &path_segments,
                                               const std::vector<Vec2d> &obstacle_points,
                                               const std::vector<LineSegment2d> &obstacle_lines,
                                               VehicleModel &vehicle_model_drive, VehicleModel &vehicle_model_reverse,
                                               std::shared_ptr<Vec2d> related_point,
                                               std::shared_ptr<LineSegment2d> related_line, const double &safe_thresh) {
    for (auto &path_segment : path_segments) {
        bool is_forward = path_segment.gear == GearBoxInfoPb::GEAR_DRIVE;
        auto vehicle_model = is_forward ? vehicle_model_drive : vehicle_model_reverse;
        vehicle_model.update_pose(path_segment.start_pose);
        auto vehicle_corners =
            (path_segment.path_type == PathType::STRAIGHT) ? vehicle_model.corners() : vehicle_model.cut_corners();
        if (path_segment.path_type == PathType::STRAIGHT) {
            double safety_thresh = safe_thresh;
            auto free_length = std::abs(MathHelper::compute_max_straight_length(
                path_segment.start_pose, vehicle_corners, obstacle_points, obstacle_lines, path_segment.length,
                is_forward, safety_thresh, related_point, related_line));
            if (free_length < std::abs(path_segment.length)) {
                return true;
            }
        } else {
            auto center_to_start = path_segment.start_pose.transform_to(path_segment.arc_center);
            bool is_turning_left = center_to_start.y() > 0 ? true : false;
            double safety_thresh = safe_thresh / path_segment.arc_radius;
            double rotate_theta = std::abs(MathHelper::compute_max_rotate_angle(
                path_segment.start_pose, vehicle_corners, obstacle_points, obstacle_lines, path_segment.arc_center,
                is_turning_left, is_forward, safety_thresh, related_point, related_line));
            if (rotate_theta < std::abs(path_segment.arc_theta)) {
                return true;
            }
        }
    }
    return false;
}

bool MathHelper::LpSpRp(const double x, const double y, const double phi, double &t, double &u, double &v) {
    const double eps = 1e-6;
    std::pair<double, double> polar =
        haomo::hidelivery::math::Cartesian2Polar(x + std::sin(phi), y - 1.0 - std::cos(phi));
    double u1 = polar.first * polar.first;
    double t1 = polar.second;
    MLOG(PARKING_PLANNING, DEBUG) << "u1: " << u1;
    if (u1 >= 4.0) {
        double theta = 0.0;
        u = std::sqrt(u1 - 4.0);
        theta = std::atan2(2.0, u);
        t = haomo::hidelivery::math::NormalizeAngle(t1 + theta);
        v = haomo::hidelivery::math::NormalizeAngle(t - phi);
        MLOG(PARKING_PLANNING, DEBUG) << "t: " << t << " v: " << v;
        return t >= 0.0 + eps && v >= 0.0 + eps;
    }
    return false;
}

/**
 * @brief       - pts点容器拟合出一条y=n直线(平行于x轴)
 * @param pts   - {std::vector<Vec2d>&} 待拟合点
 * @return      - {double} 被拟合出来的一条y=n直线
 */
double MathHelper::yline_fit(const std::vector<Vec2d> &pts) {
    std::vector<double> y_list;
    double y_sum = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        y_list.push_back(pts.at(i).y());
        y_sum = y_sum + pts.at(i).y();
    }
    double delta = 10.;
    double y_res = 0.0;
    double y_res_last = 10;
    size_t y_cnt = y_list.size();
    while (true) {
        if (delta < 0.1 || y_cnt < 2)
            break;
        double dy_cnt = (double)(y_cnt)*1.0;
        y_res = y_sum / dy_cnt;
        size_t iter = 0;
        double y_mask = 0.;
        size_t i_mask = 0;
        while (true) {
            if (iter == y_list.size())
                break;
            auto y_tmp = fabs(y_list[iter] - y_res);
            if (y_tmp > y_mask) {
                y_mask = y_tmp;
                i_mask = iter;
            }
            iter++;
        }
        delta = fabs(y_res_last - y_res);
        y_res_last = y_res;
        y_sum = y_sum - y_list.at(i_mask);
        y_cnt--;
        y_list.erase(y_list.begin() + i_mask);
    }
    return y_res;
}

/**
 * @brief       - 已知斜率a的直线,进行最小而乘拟合,返回截距
 * @param pts   - {const std::vector<Vec2d> &pts} 待拟合直线
 * @param a     - {const double &} 待拟合直线斜率
 * @return      - {double} 返回已知斜率a的直线拟合后的截距
 */
double MathHelper::b_line_fit(const std::vector<Vec2d> &pts, const double &a) {
    std::vector<Vec2d> ps;
    double ysum = 0, xsum = 0;
    double b_res = 0;
    double b_res_last = 10;
    for (size_t i = 0; i < pts.size(); i++) {
        ps.push_back(pts.at(i));
        ysum = ysum + pts.at(i).y();
        xsum = xsum + pts.at(i).x();
    }
    double p_cnt = (double)(pts.size()) * 1.0;
    double delta = 10.;
    while (true) {
        if (delta < 0.1 || p_cnt < 2)
            break;
        b_res = (ysum - a * xsum) / p_cnt;
        size_t iter = 0;
        double d_mask = 0.; // 点到直线的最大距离
        size_t i_mask = 0;  // 点到直线最大距离的索引号
        while (true) {
            if (iter == ps.size())
                break;
            auto d_tmp = fabs(a * ps[iter].x() - ps[iter].y() + b_res);
            if (d_tmp > d_mask) {
                d_mask = d_tmp;
                i_mask = iter;
            }
            iter++;
        }
        delta = fabs(b_res_last - b_res);
        b_res_last = b_res;
        ysum = ysum - ps.at(i_mask).y();
        xsum = xsum - ps.at(i_mask).x();
        p_cnt--;
        ps.erase(ps.begin() + i_mask);
    }
    return b_res;
}


bool MathHelper::check_collision(const VehicleModel &veh_model, const std::vector<Vec2d> &obs_pts,
                                 const std::vector<LineSegment2d> &obs_lines) {

    auto polygon = veh_model.polygon();
    for (const auto &pt : obs_pts) {
        if (polygon.IsPointIn(pt)) {
            return true;
        }
    }
    for (const auto &line : obs_lines) {
        if (polygon.DistanceTo(line) == 0)
            return true;
    }
    return false;
}

PLANNING_NAMESPACE_END
