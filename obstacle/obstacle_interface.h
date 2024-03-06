//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : obstacle_interface.h
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//

#ifndef OBSTACLE_INTERFACE_H
#define OBSTACLE_INTERFACE_H

#pragma once

#include "apa/path_plan/HA/ha_path_planner/sbp_node.h"
#include "parking_planning/apa/common/planning_typedef.h"
#include "geometry.h"
#include "parking_planning/apa/common/vehicle_model.h"

using namespace HA;
enum class SBPObstacleType {
    Box,
    Grid,
    Line,
    MapLine,
    Point,
};

class SBPObstacleInterface {
  public:
    DEFINE_SHARDED_PTR(SBPObstacleInterface)

    virtual double get_cost(const SearchNode::Ptr &node, VehicleModel &vehicle_model) = 0;

    virtual bool check_collision(const SearchNode::Ptr &node, VehicleModel &vehicle_model) = 0;

    virtual double get_distance(const Vec2d &point) = 0;

    virtual std::vector<Vec2d> get_nearest_points(const LineSegment2d &ego_centerline) = 0;

    virtual SBPObstacleType get_type() const = 0;

    static Vec2d get_single_nearest_point(const LineSegment2d &obs_line, const LineSegment2d &ego_centerline);
};


#endif // OBSTACLE_INTERFACE_H
