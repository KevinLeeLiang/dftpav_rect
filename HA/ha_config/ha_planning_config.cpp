//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : ha_planning_config.cpp.cc
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//

#include "ha_planning_config.h"
#include "parking_planning/apa/common/parking_info.h"

PLANNING_NAMESPACE_START
HAPlanningConfig::HAPlanningConfig() {init();}
bool HAPlanningConfig::init() {
    this->_hybrid_astar_param.xy_grid_resolution = FLAGS_apa_ha_xy_grid_resolution;
    this->_hybrid_astar_param.phi_grid_resolution = FLAGS_apa_ha_phi_grid_resolution;
    this->_hybrid_astar_param.holonomic_with_obs_heuristic = FLAGS_apa_ha_holonomic_with_obs_heuristic;
    this->_hybrid_astar_param.non_holonomic_without_obs_heuristic = FLAGS_apa_ha_non_holonomic_without_obs_heuristic;
    this->_hybrid_astar_param.step_size = FLAGS_apa_ha_step_size;
    this->_hybrid_astar_param.max_iter_base = FLAGS_apa_ha_search_max_iter;
    this->_hybrid_astar_param.max_iter_max = FLAGS_apa_ha_search_max_iter;
    return true;
}

const HybridAStarParam &HAPlanningConfig::hybrid_astar_param() const { return _hybrid_astar_param; }

const CarParam &HAPlanningConfig::car_param() const { return _car_param; }

double HAPlanningConfig::center_to_geometry_center() const {
    double overalllength = VehicleConfigHelper::GetConfig().vehicle_param().overalllength();
    //   double overallwidth =
    //       VehicleConfigHelper::GetConfig().vehicle_param().overallwidth();
    double center_to_geometry_center =
        VehicleConfigHelper::GetConfig().vehicle_param().front_edge_to_center() - overalllength / 2;
    return center_to_geometry_center;
}

double HAPlanningConfig::max_delta_angle() const { // 前轮转角最大值没有明确的数值在配置文件中
    //    double maxsteeringangle = VehicleConfigHelper::GetConfig().vehicle_param().maxsteeringangle();
    //    double steer_ratio = VehicleConfigHelper::GetConfig().vehicle_param().steer_ratio();
    //     double max_delta_angle = maxsteeringangle/M_PI*180.0 / steer_ratio;
    //     MLOG(PARKING_PLANNING, INFO) << "maxsteeringangle " << maxsteeringangle;
    // MLOG(PARKING_PLANNING, INFO) << "steer_ratio " << steer_ratio;
    // MLOG(PARKING_PLANNING, INFO) << "max_delta_angle " << max_delta_angle;
    //    double max_delta_angle =434.0 / steer_ratio;
    return 26.11;
}

PLANNING_NAMESPACE_END