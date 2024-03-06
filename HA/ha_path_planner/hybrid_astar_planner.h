//
// Created by garen_lee on 2024/1/26.
/**
 ******************************************************************************
 * @file           : hybrid_astar_planner.h
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/26
 ******************************************************************************
 */
//

#ifndef HYBRID_ASTAR_PLANNER_H
#define HYBRID_ASTAR_PLANNER_H

#pragma once

#include <fstream>
#include <vector>

#include "compare.h"
#include "../grid_map/grid_map.h"
#include "../obstacle/obstacle_interface.h"
#include "sbp_node.h"
#include "sbp_rs_node.h"
#include "reeds_shepp_path.h"
#include "search_planner.h"

PLANNING_NAMESPACE_START

class HybridAStarPlanner : public SearchBasedPlanner{
  public:
    DEFINE_SHARDED_PTR(HybridAStarPlanner)

    HybridAStarPlanner(const size_t max_node_num);

    HybridAStarPlanner(const Pose2d &goal_pose, double goal_v, const Box2d &map_boundary,
                       const std::vector<LineSegment2d> &map = std::vector<LineSegment2d>());

    void setPlanStartTime(uint64_t time) { this->start_time_ = time; }

    void buildGridMap(const std::vector<LineSegment2d> &obs_lines, const std::vector<Vec2d> &obs_pts,
                      const Box2d &map_boundary);

    //<--------------------- Interface ------------------------>>//
    void update(const Pose2d &goal_pose, double goal_v, const Box2d &map_boundary,
                const std::vector<LineSegment2d> &map = std::vector<LineSegment2d>()) override;

    bool Plan(const ParkEnvironment::ConstPtr &park_env_ptr, Path::Ptr &path);

    bool plan(const std::vector<SBPObstacleInterface::Ptr> &obs_ptrs) override;

    void set_start_pose(const Pose2d &start_pose, double start_vel = 0) override;

    void set_target_pose(const Pose2d &target_pose) override;

    // for visualziation
    std::vector<Pose2d> get_search_points() override;

    std::vector<Pose2d> get_search_path() const {return this->searchPath_;}

    SBPResult get_result() override;

    void SetResult(SBPResult *ha_res, Path::Ptr &path);

    void clear_result() {
        this->result_.clear();
        this->searchPoints_.clear();
        this->searchPath_.clear();
    }

  private:
    bool check_collision_real(const SearchNode::Ptr &current_state);

    double get_obstacle_cost(const SearchNode::Ptr &node, VehicleModel &vehicle_model) const;

    static void combine_trajectory(SBPResult *result, const SearchNode::Ptr &current);

    std::vector<SearchNode::Ptr> get_next_states(const SearchNode::Ptr &current);

    bool check_collision(const SearchNode::Ptr &current_state);

    bool check_collision(const SearchNode::Ptr &current_state, const bool test);

    bool CheckCollisionInGridMap(const SearchNode::Ptr &current_state);

    bool CheckInBound(const SearchNode::Ptr &current_state);

    bool analytic_expansion(const SearchNode::Ptr &current_node,
                            const std::vector<SBPObstacleInterface::Ptr> &obs_ptrs);

    bool analytic_expansion(const SearchNode::Ptr &current_node, const SearchNode::Ptr &end_node,
                            const std::vector<SBPObstacleInterface::Ptr> &obs_ptrs);

    bool is_node_out_of_range(const SearchNode::Ptr &current_node);

    double calc_circular_radius(const SearchNode::Ptr &node_a, const SearchNode::Ptr &node_b);


  private:
    std::vector<std::shared_ptr<HybridAStarNode>> nodes_vec_;
    unsigned int nodes_size_;
    uint64_t start_time_;

    std::vector<double> wheel_base_offset_options_;

    // This value is used to set a threshold for distance of obstacle to car
    double safe_diff_distance_ = 3;

    SBPRSPath sbp_rspath_;
    std::shared_ptr<ReedSheppPath> reeds_shepp_path_;

    std::vector<SBPObstacleInterface::Ptr> obs_ptrs_;
    std::shared_ptr<GridMap> grid_map_ptr_;
    SBPObstacleInterface::Ptr obs_which_collision_;

    VehicleModel vehicle_model_;
    ParkType park_type_;
    double x_left_boun_ = 0.0;
    double x_right_boun_ = 0.0;
    double y_bot_boun_ = 0.0;
    double y_up_boun_ = 8.;

    // TODO: test
    size_t trick_dis_rev_ = 0;
    Box2d map_boundary_;
    std::vector<LineSegment2d> map_;
    bool is_reach_midway_ = false;
    int32_t max_zigzag_allowd_;
};

PLANNING_NAMESPACE_END

#endif // HYBRID_ASTAR_PLANNER_H
