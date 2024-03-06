//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : sbp_node.cpp.cc
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//

#include "sbp_node.h"
#include <limits>
#include <stdexcept>

#include "apa/path_plan/HA/ha_config/ha_planning_config.h"
#include "parking_planning/common/planning_gflags.h"

PLANNING_NAMESPACE_START

SearchNode::SearchNode() {
    this->gx = 1 / FLAGS_apa_ha_xy_grid_resolution;
    this->gy = 1 / FLAGS_apa_ha_xy_grid_resolution;
    this->gtheta = 1 / FLAGS_apa_ha_phi_grid_resolution;
    compute_string_index();
}

void SearchNode::compute_string_index() {
    index = 0;
    boost::hash_combine(index, gx);
    boost::hash_combine(index, gy);
    boost::hash_combine(index, gtheta);
}

void SearchNode::set_traj_cost() {}

// void SearchNode::set_node(double x, double y, double theta, double vel,
//                           double delta, double wheel_base_offset) {}

void SearchNode::set_node(double x, double y, double theta, double vel, double delta, double kappa,
                          double wheel_base_offset) {}

HybridAStarNode::HybridAStarNode() {
    this->x = -1;
    this->y = -1;
    this->theta = -1;
}

HybridAStarNode::HybridAStarNode(double x, double y, double theta, double vel, double delta, double kappa,
                                 double wheel_base_offset) {
    this->x = x;
    this->y = y;
    this->theta = theta;
    this->gx = x / FLAGS_apa_ha_xy_grid_resolution;
    this->gy = y / FLAGS_apa_ha_xy_grid_resolution;
    this->gtheta = theta / FLAGS_apa_ha_phi_grid_resolution;

    this->delta = delta;
    this->vel = vel;
    this->trajcost = 0;
    wheel_base_offset_ = wheel_base_offset;
    this->kappa = kappa;
    compute_string_index();
}

void HybridAStarNode::set_node(double x, double y, double theta, double vel, double delta, double wheel_base_offset) {
    this->x = x;
    this->y = y;
    this->theta = theta;

    this->gx = x / FLAGS_apa_ha_xy_grid_resolution;
    this->gy = y / FLAGS_apa_ha_xy_grid_resolution;
    this->gtheta = theta / FLAGS_apa_ha_phi_grid_resolution;

    this->delta = delta;
    this->vel = vel;
    this->trajcost = 0;
    wheel_base_offset_ = wheel_base_offset;
    compute_string_index();
}

void HybridAStarNode::set_node(double x, double y, double theta, double vel, double delta, double kappa,
                               double wheel_base_offset) {
    this->x = x;
    this->y = y;
    this->theta = theta;

    this->gx = x / FLAGS_apa_ha_xy_grid_resolution;
    this->gy = y / FLAGS_apa_ha_xy_grid_resolution;
    this->gtheta = theta / FLAGS_apa_ha_phi_grid_resolution;

    this->delta = delta;
    this->vel = vel;
    this->trajcost = 0;
    wheel_base_offset_ = wheel_base_offset;
    this->kappa = kappa;
    compute_string_index();
}

void HybridAStarNode::set_traj_cost() {
    const double &step_size = std::fabs(vel);
    if (!previous) {
        trajcost = 0;
        return;
    }

    trajcost = previous->trajcost;
    if (previous->vel * vel < 0) {
        trajcost += FLAGS_apa_ha_traj_gear_switch_penalty;
    } else {
        trajcost += std::abs(delta) * FLAGS_apa_ha_traj_steer_penalty;
        trajcost += std::abs((delta - previous->delta)) * FLAGS_apa_ha_traj_steer_change_penalty;
    }

    if (vel > 0.0) {
        trajcost += FLAGS_apa_ha_traj_forward_penalty * step_size;
    } else {
        trajcost += FLAGS_apa_ha_traj_back_penalty * step_size;
    }
    trajcost += std::min(std::abs(wheel_base_offset_ - previous->wheel_base_offset_), 1.0) * 1.0;

    trajcost += FLAGS_apa_ha_traj_sides_diff_penalty * side_diff;
}

PLANNING_NAMESPACE_END
