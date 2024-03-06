//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : obstacle_point.cpp.cc
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//

#include "obstacle_point.h"
PLANNING_NAMESPACE_START

double SBPObstaclePoint::get_cost(const SearchNode::Ptr &node, VehicleModel &vehicle_model) { return 0; }

bool SBPObstaclePoint::check_collision(const SearchNode::Ptr &node, VehicleModel &vehicle_model) {
    vehicle_model.update_pose(Pose2d(node->x, node->y, node->theta));
    bool is_collision = false;
    for (auto &p : points_) {
        if (vehicle_model.polygon().IsPointIn(p)) {
            //        MLOG(PARKING_PLANNING, ERROR) << "collision p x," << p.x() << ",y," << p.y();
            is_collision = true;
            break;
        }
    }
    return is_collision;
}

double SBPObstaclePoint::get_distance(const Vec2d &point) {
    double min_dist = std::numeric_limits<double>::infinity();
    for (const Vec2d &pt : points_) {
        min_dist = std::min(min_dist, pt.DistanceTo(point));
    }
    return min_dist;
}

std::vector<Vec2d> SBPObstaclePoint::get_nearest_points(const LineSegment2d &ego_centerline) { return points_; }

SBPObstaclePoint::SBPObstaclePoint(const std::vector<Vec2d> &points)
    : points_(points) {}

SBPObstaclePoint::~SBPObstaclePoint() {}

SBPObstaclePointKDTree::SBPObstaclePointKDTree(const std::vector<Vec2d> &points)
    : SBPObstaclePoint(points) {
    // 构建kdtree输入数据
    Kdtree::KdNodeVector nodes;
    for (size_t i = 0; i < points.size(); ++i) {
        std::vector<double> point(2);
        point[0] = points.at(i).x();
        point[1] = points.at(i).y();
        nodes.push_back(Kdtree::KdNode(point));
    }

    // 构建kdtree障碍物点
    this->kdtree_pts_ptr_ = std::make_shared<Kdtree::KdTree>(&nodes);
}

bool SBPObstaclePointKDTree::check_collision(const SearchNode::Ptr &node, VehicleModel &vehicle_model) {
    float dis2c = hypot(vehicle_model.front_left().x() - vehicle_model.rear_right().x(),
                        vehicle_model.front_left().y() - vehicle_model.rear_right().y());
    vehicle_model.update_pose(Pose2d(node->x, node->y, node->theta));
    Kdtree::KdNodeVector obs_nodes;
    bool is_collision = false;
    std::vector<double> test_node = {node->x, node->y};
    this->kdtree_pts_ptr_->range_nearest_neighbors(test_node, dis2c, &obs_nodes);
    for (auto &nodetmp : obs_nodes) {
        Vec2d p = Vec2d(nodetmp.point[0], nodetmp.point[1]);
        //    MLOG(PARKING_PLANNING, ERROR) << "nodetmp, p.x," << p.x() << ",p.y," << p.y();
        if (vehicle_model.polygon().IsPointIn(p)) {
            is_collision = true;
            break;
        }
    }
    return is_collision;
}

PLANNING_NAMESPACE_END