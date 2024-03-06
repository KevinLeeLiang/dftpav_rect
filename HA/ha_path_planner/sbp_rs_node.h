//
// Created by garen_lee on 2024/1/29.
/**
 ******************************************************************************
 * @file           : sbp_rs_node.h
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/29
 ******************************************************************************
 */
//

#ifndef SBP_RS_NODE_H
#define SBP_RS_NODE_H
#pragma once
#include "../grid_map/grid_map.h"
#include "../obstacle/obstacle_interface.h"
#include "parking_planning/apa/common/vehicle_model.h"
#include "reeds_shepp_path.h"
#include "sbp_node.h"

PLANNING_NAMESPACE_START
class SBPRSPath {
  public:
    SBPRSPath();

    SBPRSPath(const SearchNode::Ptr &current_node, const ReedSheppPath::Ptr &reeds_shepp_path);

    ~SBPRSPath() = default;

    void update(const SearchNode::Ptr &current_node, const ReedSheppPath::Ptr &reeds_shepp_path);

    std::vector<SearchNode::Ptr> get_nodes() const { return nodes_; }

    double get_cost(const std::vector<SBPObstacleInterface::Ptr> &obs_ptrs, VehicleModel &vehicle_model);

    bool check_collision(const double step_size, const std::vector<SBPObstacleInterface::Ptr> &obs_ptrs,
                         VehicleModel &vehicle_model);
    bool check_collision(const double step_size, const std::shared_ptr<GridMap> &grid_map_ptr);

  private:
    bool check_single_node_collision(const SearchNode::Ptr &current_state,
                                     const std::vector<SBPObstacleInterface::Ptr> &obs_ptrs,
                                     VehicleModel &vehicle_model);

    void calc_traj_cost() const;

    void calc_obstacle_cost(const std::vector<SBPObstacleInterface::Ptr> &obs_ptrs, VehicleModel &vehicle_model) const;

  private:
    ReedSheppPath::Ptr reeds_shepp_path_;
    std::vector<SearchNode::Ptr> nodes_;
    std::vector<SearchNode::Ptr> nodes_vec_;

    int max_nodes_num_ = 100;
};
PLANNING_NAMESPACE_END

#endif // SBP_RS_NODE_H
