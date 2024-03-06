//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : obstacle_line.cpp.cc
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//

#include "obstacle_line.h"
PLANNING_NAMESPACE_START

double SBPObstacleLine::get_cost(const SearchNode::Ptr &node, VehicleModel &vehicle_model) { return 0; }

bool SBPObstacleLine::check_collision(const SearchNode::Ptr &node, VehicleModel &vehicle_model) {
    vehicle_model.update_pose(Pose2d(node->x, node->y, node->theta));
    bool is_collision = false;

    // todo(JSQ): 这里要考虑当前位置和上一个位置和line是否碰撞
    //   const SearchNode::Ptr &previous_node = node->previous;
    //   if (previous_node &&
    //       footprint_model->check_trace_overlap(
    //           Pose2D(previous_node->x, previous_node->y, previous_node->theta),
    //           Pose2D(node->x, node->y, node->theta), lines_)) {
    //     return true;
    //   }

    // MLOG(PARKING_PLANNING, INFO) <<"SBPObstacleLine lines_ size "<< lines_.size();
    // MLOG(PARKING_PLANNING, INFO) <<"x "<< node->x;
    // MLOG(PARKING_PLANNING, INFO) <<"y "<< node->y;
    // MLOG(PARKING_PLANNING, INFO) <<"theta "<< node->theta;

    for (auto &l : lines_) {

        if (vehicle_model.polygon().HasOverlap(l)) {
            is_collision = true;
            break;
        }
    }

    return is_collision;
}

double SBPObstacleLine::get_distance(const Vec2d &point) {
    double min_dist = std::numeric_limits<double>::infinity();
    for (const LineSegment2d &line : lines_) {
        min_dist = std::min(min_dist, line.DistanceTo(point));
    }
    return min_dist;
}

std::vector<Vec2d> SBPObstacleLine::get_nearest_points(const LineSegment2d &ego_centerline) {
    std::vector<Vec2d> nearest_pts{};
    for (auto line : lines_) {
        nearest_pts.push_back(get_single_nearest_point(line, ego_centerline));
    }
    return nearest_pts;
}

SBPObstacleLine::SBPObstacleLine(const std::vector<ObstacleLineTmp> &lines)
    : lines_(lines) {}

SBPObstacleLine::~SBPObstacleLine() {}

PLANNING_NAMESPACE_END