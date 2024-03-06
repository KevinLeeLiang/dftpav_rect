//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : gird_map.h
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//

#ifndef GIRD_MAP_H
#define GIRD_MAP_H

#include <vector>
#include "grid_collision_detection.h"
#include "../obstacle/obstacle_interface.h"

PLANNING_NAMESPACE_START

class GridMap : public GridCollisionDetection {
  public:
    GridMap();
    ~GridMap();
    void BuildGridMap(const std::vector<Vec2d> &obs, const Box2d &map_bound);

  public:
    std::vector<std::vector<int>> grid_map;

    // bool **bin_map;
    uint64_t map_length_;
    uint64_t map_width_;
};

PLANNING_NAMESPACE_END
#endif // GIRD_MAP_H
