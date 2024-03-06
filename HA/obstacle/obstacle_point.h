//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : obstacle_point.h
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//

#ifndef OBSTACLE_POINT_H
#define OBSTACLE_POINT_H

#pragma once
#include "obstacle_interface.h"
#include "parking_planning/apa/common/vehicle_model.h"
#include "kdtree/kdtree.h"

PLANNING_NAMESPACE_START

class SBPObstaclePoint : public SBPObstacleInterface {
  protected:
    std::vector<Vec2d> points_;

  public:
    SBPObstaclePoint(const std::vector<Vec2d> &points);
    ~SBPObstaclePoint();

    double get_cost(const SearchNode::Ptr &node, VehicleModel &vehicle_model) override;

    bool check_collision(const SearchNode::Ptr &node, VehicleModel &vehicle_model) override;

    double get_distance(const Vec2d &point) override;

    std::vector<Vec2d> get_nearest_points(const LineSegment2d &ego_centerline) override;

    SBPObstacleType get_type() const override { return SBPObstacleType::Point; }
};

class SBPObstaclePointKDTree : public SBPObstaclePoint {
  private:
    Kdtree::KdTree::Ptr kdtree_pts_ptr_; // kdtree
  public:
    SBPObstaclePointKDTree(const std::vector<Vec2d> &points);
    ~SBPObstaclePointKDTree(){};
    bool check_collision(const SearchNode::Ptr &node, VehicleModel &vehicle_model);
};

PLANNING_NAMESPACE_END

#endif // OBSTACLE_POINT_H
