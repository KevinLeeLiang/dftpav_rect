//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : ha_planning_config.h
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//

#ifndef HA_PLANNING_CONFIG_H
#define HA_PLANNING_CONFIG_H
#include "parking_planning/apa/common/planning_macro.h"

PLANNING_NAMESPACE_START

struct HybridAStarParam {
    bool use_t_line = true;
    int max_zigzag_allowd = 100;

    bool enable_delta_cost = true;
    bool enable_analytic_expansion = false;
    double max_analytic_expansion_length = -1.0;
    double analytic_expansion_end_size_threshold = 0.1;
    double force_analytic_expansion_end_direction = 0;
    double xy_grid_resolution = 0.3;
//    double phi_grid_resolution = 0.017 * 6;
    double phi_grid_resolution = 0.017 * 6;
    double grid_dijkstra_xy_resolution = 1.0;
    double pixel_resolution = 0.03;

    double traj_forward_penalty = 1.0;
    double traj_back_penalty = 1.0;
    double traj_gear_switch_penalty = 5.0;
    double traj_steer_penalty = 0.0002;
    double traj_steer_change_penalty = 0.01;
    double traj_sides_diff_penalty = 0.05;

    double step_size = 0.5;
    int next_node_num = 7;
    double step_direction = 0;

    double delta_t = 1;
    double holonomic_with_obs_heuristic = 0.2;
    double non_holonomic_without_obs_heuristic = 0.8;

    int verbose = 1;
    int display_points = 2;

    int planning_core = 0;
    int max_iter_base = 10000;
    int max_iter_max = 10000;

    int footprint_model = 0;
    int footprint_model_precise = 0;
};

struct CarParam {
    double lon_inflation_min = 0.04;
    double lon_inflation_max = 0.0;
    double lat_inflation_min = 0.05;
    double lat_inflation_max = 0.0;
    double inflation_rearview_mirror = 0.1;
    double shrink_ratio_for_lines = 0.8;
    double shrink_ratio_for_lines_min = 0.1;
    double max_acceleration = 2.0;
    double min_acceleration = 3.0;
    double max_steer_angle = 445.0;
    double max_steer_angle_rate = 400.0;
    double max_steer_angle_rear = 44.4;
    double max_steer_angle_rate_rear = 38.48;
    bool enable_multiple_steer_modes = false;

    double light_to_front_edge = 0.52;
    double bumper_length = 0.89;
    double front_edge_to_mirror = 1.8;

    double brake_distance_buffer = 0.1;
};

class HAPlanningConfig {
  public:
    DEFINE_SINGLETON(HAPlanningConfig)

    bool init();

    // const VehicleParameter& vehicle_parameter() const;

    // const StrategyParam& strategy_param() const;

    const CarParam &car_param() const;

    // const ParkingSlotParam& parking_slot_param() const;

    const HybridAStarParam &hybrid_astar_param() const;

    double center_to_geometry_center() const;

    double max_delta_angle() const;

    void set_traj_gear_switch_penalty(double val) { _hybrid_astar_param.traj_gear_switch_penalty = val; };

    // Box2d getBox(double x, double y, double theta);

  private:
    // PlanningConfig() = default;

    HAPlanningConfig();
    // ParkingPlanningParam planning_config_;

    // VehicleParameter     vehicle_parameters_;
    HybridAStarParam _hybrid_astar_param;
    CarParam _car_param;
};

PLANNING_NAMESPACE_END

#endif // HA_PLANNING_CONFIG_H
