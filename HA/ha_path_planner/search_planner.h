//
// Created by garen_lee on 2024/1/29.
/**
 ******************************************************************************
 * @file           : search_planner.h
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/29
 ******************************************************************************
 */
//

#ifndef SEARCH_PLANNER_H
#define SEARCH_PLANNER_H


#pragma once

#include "reeds_shepp_path.h"
#include "parking_planning/apa/path_plan/vertical_slot/arc_based_planner.h"
#include "parking_planning/apa/path_plan/HA/obstacle/obstacle_interface.h"
#include <fstream>
#include <iostream>
#include <vector>

PLANNING_NAMESPACE_START
namespace HA {
enum SBPStatus : int {
    SUCCESS = 0,
    INFEASIBLE = 1, // Infeasible
    TIMEOUT = 2,    // TimeOut
    EXCEPTION = 3,  // Exeception
};
};
struct SBPResult {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> phi;
    std::vector<double> steer;
    std::vector<double> v;
    std::vector<double> a;
    std::vector<double> accumulated_s;
    std::vector<double> wheel_base_offset;
    size_t num_segments;
    size_t iteration_times = 1000000;
    std::string debug_string;
    SBPStatus status;

    void emplace_back(double x_arg, double y_arg, double phi_arg, double steer_arg = 0, double v_arg = 0,
                      double a_arg = 0, double s_arg = 0, double wheel_base_offset_arg = 0) {
        x.push_back(x_arg);
        y.push_back(y_arg);
        phi.push_back(phi_arg);
        steer.push_back(steer_arg);
        v.push_back(v_arg);
        a.push_back(a_arg);
        accumulated_s.push_back(s_arg);
        wheel_base_offset.push_back(wheel_base_offset_arg);
    }

    void clear() {
        x.clear();
        y.clear();
        phi.clear();
        steer.clear();
        v.clear();
        a.clear();
        accumulated_s.clear();
        wheel_base_offset.clear();
    }
};

inline SBPResult operator+(const SBPResult &lhs, const SBPResult &rhs) {
    SBPResult stitched_result(lhs);
    std::copy(rhs.x.begin(), rhs.x.end(), std::back_inserter(stitched_result.x));
    std::copy(rhs.y.begin(), rhs.y.end(), std::back_inserter(stitched_result.y));
    std::copy(rhs.phi.begin(), rhs.phi.end(), std::back_inserter(stitched_result.phi));
    std::copy(rhs.v.begin(), rhs.v.end(), std::back_inserter(stitched_result.v));
    std::copy(rhs.a.begin(), rhs.a.end(), std::back_inserter(stitched_result.a));
    std::copy(rhs.steer.begin(), rhs.steer.end(), std::back_inserter(stitched_result.steer));
    std::copy(rhs.wheel_base_offset.begin(), rhs.wheel_base_offset.end(),
              std::back_inserter(stitched_result.wheel_base_offset));

    std::vector<double> accumulated_s_biased = rhs.accumulated_s;
    double s_bias = 0;
    if (!stitched_result.accumulated_s.empty()) {
        s_bias = stitched_result.accumulated_s.back();
    }
    for (size_t i = 0; i < accumulated_s_biased.size(); ++i) {
        accumulated_s_biased[i] += s_bias;
    }
    std::copy(accumulated_s_biased.begin(), accumulated_s_biased.end(),
              std::back_inserter(stitched_result.accumulated_s));
    return stitched_result;
}

/**
 * @class SearchBasedPlannerInterface
 * @brief This abstract class defines an interface for Search Based planners
 */
class SearchBasedPlanner {
  public:
    DEFINE_SHARDED_PTR(SearchBasedPlanner)

    /**
     * @brief Update and re-init SearchBasedPlanner.
     * @param goal_pose target pose
     * @param goal_v target velocity
     * @param map_boundary planning boundary
     * @param map obstacles in the current environment
     * @return no param
     */
    virtual void update(const Pose2d &goal_pose, double goal_v, const Box2d &map_boundary,
                        const std::vector<LineSegment2d> &map = std::vector<LineSegment2d>()) = 0;

    /**
     * @brief Plan a trajectory based on an initial reference plan.
     * @param obstacles in the current environment
     * @return \c true if planning was successful, \c false otherwise
     */
    virtual bool plan(const std::vector<SBPObstacleInterface::Ptr> &obs_ptrs) = 0;

    /**
     * @brief set ther planning start of the planner.
     * @param the starting pose
     */
    virtual void set_start_pose(const Pose2d &start_pose, double start_vel = 0) = 0;

    /**
     * @brief set the planning target of the planner.
     * @param the starting pose
     */
    virtual void set_target_pose(const Pose2d &start_pose) = 0;

    /**
     * @brief get the planned trajectory
     * @param no param
     */
    virtual SBPResult get_result() = 0;

    /**
     * @brief for visualization, get all the searchpoints
     * @param no param
     */
    virtual std::vector<Pose2d> get_search_points() = 0;

  public:
    std::unique_ptr<ReedShepp> reed_shepp_generator_;
    ReedSheppPath reeds_shepp_to_end_;

  protected:
    double x_bound_;
    double y_bound_;
    double xy_grid_resolution_;
    double phi_grid_resolution_;
    double step_size_;
    double max_delta_angle_;
    double wheel_base_;
    double front_edge_to_rear_;
    double back_edge_to_rear_;
    double right_edge_to_center_;
    double left_edge_to_center_;
    double delta_t_;
    int next_node_num_;

    std::shared_ptr<SearchNode> start_node_;
    std::shared_ptr<SearchNode> end_node_;

    double partition_init_v_;
    double partition_init_a_;

    clock_t clock_start_;

    int verbose;
    int display_points;
    static const int NO_PRINT = 0;
    static const int BI_PRINT = 1;
    static const int FORWARD_PRINT = 2;
    static const int BACKWARD_PRINT = 3;
    static const int SHOW_CHANGE_NODES = 0;
    static const int SHOW_PATH = 0;

    static const int VERBOSE_DEBUG = 1;
    static const int BATCH_RESULTS = 2;

    // for visualization
    std::vector<Pose2d> searchPoints_;
    std::vector<Pose2d> searchPath_;
    std::vector<std::vector<float>> searchEdges_;
    unsigned long iter;

    SBPResult result_;

    // to support any coordinate system.
    Pose2d local_frame_pose_;
};

PLANNING_NAMESPACE_END


#endif // SEARCH_PLANNER_H
