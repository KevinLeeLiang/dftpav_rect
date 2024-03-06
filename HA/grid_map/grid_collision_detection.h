//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : grid_collision_detection.h
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//

#ifndef GRID_COLLISION_DETECTION_H
#define GRID_COLLISION_DETECTION_H

#include "constant.h"
// #include "lookup.h"
#include "singleton_lookup.h"
#include <cstring>
#include <vector>

PLANNING_NAMESPACE_START

class GridCollisionDetection {
  public:
    GridCollisionDetection()
        : x_min_(std::numeric_limits<double>::max())
        , y_min_(std::numeric_limits<double>::max())
        , width_(0)
        , length_(0) {}

    GridCollisionDetection(int width, int length)
        : x_min_(std::numeric_limits<double>::max())
        , y_min_(std::numeric_limits<double>::max())
        , width_(width)
        , length_(length) {}

    bool configurationTest(double x, double y, double t, bool test = false);

    void updateGrid(const std::vector<std::vector<int>> &map);

    void setGridSize(int width, int length);

    void setGridOri(const double x_min, const double y_min);

    void clear();

    void updateLookupTable();

    std::vector<std::vector<int>> grid;
    std::vector<int> gridmap;

  private:
    Constants::config collisionLookuptable[Constants::headings * Constants::positions];

    double x_min_ = std::numeric_limits<double>::max();
    double y_min_ = std::numeric_limits<double>::max();
    int width_ = 0;
    int length_ = 0;
};

PLANNING_NAMESPACE_END
#endif // GRID_COLLISION_DETECTION_H
