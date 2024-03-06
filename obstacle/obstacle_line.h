//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : obstacle_line.h
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//

#ifndef OBSTACLE_LINE_H
#define OBSTACLE_LINE_H

#pragma once
#include "obstacle_interface.h"
#include <limits>

PLANNING_NAMESPACE_START

class SBPObstacleLine : public SBPObstacleInterface {
  private:
    std::vector<ObstacleLineTmp> lines_;

  public:
    explicit SBPObstacleLine(const std::vector<ObstacleLineTmp> &lines);

    ~SBPObstacleLine();

    double get_cost(const SearchNode::Ptr &node, VehicleModel &vehicle_model) override;

    bool check_collision(const SearchNode::Ptr &node, VehicleModel &vehicle_model) override;
    double get_distance(const Vec2d &point) override;
    std::vector<Vec2d> get_nearest_points(const LineSegment2d &ego_centerline) override;

    SBPObstacleType get_type() const override { return SBPObstacleType::Line; }
};

PLANNING_NAMESPACE_END

#endif // OBSTACLE_LINE_H
