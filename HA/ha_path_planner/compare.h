//
// Created by garen_lee on 2024/1/26.
/**
 ******************************************************************************
 * @file           : compare.h
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/26
 ******************************************************************************
 */
//

#ifndef COMPARE_H
#define COMPARE_H

#include "sbp_node.h"

#include "parking_planning/apa/common/pose2d.h"
#include "../obstacle/obstacle_interface.h"
#include <memory>
#include <queue>
#include <string>
#include <vector>

PLANNING_NAMESPACE_START

class Compare {
  public:
    Compare();

    Compare(double xy_grid_resolution, double grid_Dijkstra_xy_resolution);

    void loadFrom(const std::vector<SBPObstacleInterface::Ptr> &obstacles, const double x_bound,
                  const double y_bound) const;

    bool operator()(const SearchNode::Ptr &s1,
                    const SearchNode::Ptr &s2) const; // search

    static float non_holonomic_without_obs(const SearchNode::Ptr &src); // search

    float holonomic_with_obs(const SearchNode::Ptr &src) const; // search

    void run_dijkstra(const double x_bound, const double y_bound) const;

  public:
    static SearchNode target;
    static std::vector<std::vector<char>> grid_obs_map;
    static std::vector<std::vector<float>> shortest_2d;
    static int DX_;
    static int DY_;
    static Pose2d cmp_frame_pose_;

  private:
    float xy_grid_resolution_;
    float grid_Dijkstra_xy_resolution_;
    float holonomic_with_obs_heuristic_;
    float non_holonomic_without_obs_heuristic_;
};

PLANNING_NAMESPACE_END

#endif // COMPARE_H
