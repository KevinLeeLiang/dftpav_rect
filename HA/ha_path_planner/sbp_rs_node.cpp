//
// Created by garen_lee on 2024/1/29.
/**
 ******************************************************************************
 * @file           : sbp_rs_node.cpp.cc
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/29
 ******************************************************************************
 */
//

#include "sbp_rs_node.h"
#include <iostream>
#include <memory>

#include "../ha_config/ha_planning_config.h"
#include "parking_planning/apa/common/planning_typedef.h"
#include "utils.h"

PLANNING_NAMESPACE_START

SBPRSPath::SBPRSPath() {
    reeds_shepp_path_ = ReedSheppPath::Ptr(nullptr);
    for (int i = 0; i < max_nodes_num_; i++) {
        nodes_vec_.emplace_back(std::make_shared<HybridAStarNode>());
    }
}

SBPRSPath::SBPRSPath(const SearchNode::Ptr &current_node, const ReedSheppPath::Ptr &reeds_shepp_path) {
    int difference = std::max(max_nodes_num_, (int)reeds_shepp_path->x.size()) - nodes_vec_.size();
    if (difference > 0) {
        for (int i = 0; i < difference; i++) {
            nodes_vec_.emplace_back(std::make_shared<HybridAStarNode>());
        }
    }
    nodes_.clear();
    reeds_shepp_path_ = reeds_shepp_path;
    nodes_.emplace_back(current_node);

    int gear;
    double x, y, phi, vel, kap;
    // std::shared_ptr<HybridAStarNode> node;
    double rs_equevalent_delta = 0;
    double max_delta_angle_ = HAPlanningConfig::instance()->max_delta_angle();

    for (size_t i = 1; i < reeds_shepp_path_->x.size(); ++i) {
        x = reeds_shepp_path_->x[i];
        y = reeds_shepp_path_->y[i];
        phi = reeds_shepp_path_->phi[i];
        kap = reeds_shepp_path_->kappa[i];
        auto prev_node = nodes_[i - 1];
        Vec2d prev_heading_vec(std::cos(prev_node->theta), std::sin(prev_node->theta));
        Vec2d prev_node_vec(prev_node->x, prev_node->y);
        Vec2d node_vec(x, y);
        gear = prev_heading_vec.InnerProd(node_vec - prev_node_vec) > 0 ? 1 : -1;
        vel = hypot(x - prev_node->x, y - prev_node->y) * gear;

        double sin_diff_phi = std::sin(phi - prev_node->theta) * gear;
        if (sin_diff_phi > 1e-6) {
            rs_equevalent_delta = max_delta_angle_;
        } else if (sin_diff_phi < -1e-6) {
            rs_equevalent_delta = -max_delta_angle_;
        } else {
            rs_equevalent_delta = 0;
        }

        nodes_vec_[i - 1]->set_node(x, y, phi, vel, rs_equevalent_delta, kap);
        nodes_vec_[i - 1]->previous = nodes_[i - 1];
        nodes_.emplace_back(nodes_vec_[i - 1]);
    }
}

void SBPRSPath::update(const SearchNode::Ptr &current_node, const ReedSheppPath::Ptr &reeds_shepp_path) {
    int difference = std::max(max_nodes_num_, (int)reeds_shepp_path->x.size()) - nodes_vec_.size();
    if (difference > 0) {
        MLOG(PARKING_PLANNING, INFO) << "增加 RS nodes_vec";
        for (int i = 0; i < difference; i++) {
            nodes_vec_.emplace_back(std::make_shared<HybridAStarNode>());
        }
    }
    nodes_.clear();
    reeds_shepp_path_ = reeds_shepp_path;
    nodes_.emplace_back(current_node);

    int gear;
    double x, y, phi, vel, kap;
    double rs_equevalent_delta = 0;
    double max_delta_angle_ = HAPlanningConfig::instance()->max_delta_angle();

    for (size_t i = 1; i < reeds_shepp_path_->x.size(); ++i) {
        x = reeds_shepp_path_->x[i];
        y = reeds_shepp_path_->y[i];
        phi = reeds_shepp_path_->phi[i];
        kap = reeds_shepp_path_->kappa[i];
        auto prev_node = nodes_[i - 1];
        Vec2d prev_heading_vec(std::cos(prev_node->theta), std::sin(prev_node->theta));
        Vec2d prev_node_vec(prev_node->x, prev_node->y);
        Vec2d node_vec(x, y);
        gear = prev_heading_vec.InnerProd(node_vec - prev_node_vec) > 0 ? 1 : -1;
        vel = hypot(x - prev_node->x, y - prev_node->y) * gear;

        double sin_diff_phi = std::sin(phi - prev_node->theta) * gear;
        if (sin_diff_phi > 1e-6) {
            rs_equevalent_delta = max_delta_angle_;
        } else if (sin_diff_phi < -1e-6) {
            rs_equevalent_delta = -max_delta_angle_;
        } else {
            rs_equevalent_delta = 0;
        }
        nodes_vec_[i - 1]->set_node(x, y, phi, vel, rs_equevalent_delta, kap);
        nodes_vec_[i - 1]->previous = nodes_[i - 1];
        nodes_.emplace_back(nodes_vec_[i - 1]);
    }
}

void SBPRSPath::calc_traj_cost() const {
    for (size_t i = 1; i < nodes_.size(); ++i) {
        auto curr_node = nodes_[i];
        curr_node->set_traj_cost();
    }
}

void SBPRSPath::calc_obstacle_cost(const std::vector<SBPObstacleInterface::Ptr> &obs_ptrs,
                                   VehicleModel &vehicle_model) const {
    double single_node_obs_cost = 0;
    for (const auto &node : nodes_) {
        single_node_obs_cost = 0;
        for (const auto &obs_ptr : obs_ptrs) {
            single_node_obs_cost += obs_ptr->get_cost(node, vehicle_model);
        }
        node->obs_cost = node->previous->obs_cost + single_node_obs_cost;
    }
}

double SBPRSPath::get_cost(const std::vector<SBPObstacleInterface::Ptr> &obs_ptrs, VehicleModel &vehicle_model) {
    if (nodes_.empty()) {
        return 0.0;
    }
    calc_traj_cost();
    calc_obstacle_cost(obs_ptrs, vehicle_model);
    return nodes_.back()->trajcost + nodes_.back()->obs_cost - nodes_.front()->trajcost - nodes_.front()->obs_cost;
}

bool SBPRSPath::check_collision(const double step_size, const std::vector<SBPObstacleInterface::Ptr> &obs_ptrs,
                                VehicleModel &vehicle_model) {
    if (nodes_.empty()) {
        return false;
    }
    if (nodes_.size() < 2) {
        for (const auto &obs_ptr : obs_ptrs) {
            if (obs_ptr->check_collision(nodes_.back(), vehicle_model)) {
                return true;
            }
        }
        return false;
    }
    for (size_t i = 1; i + 1 < nodes_.size(); i++) {
        if (check_single_node_collision(nodes_[i], obs_ptrs, vehicle_model)) {
            return true;
        }
    }

    auto prev_node = nodes_.back();
    double last_segment_length =
        std::hypot(prev_node->previous->x - prev_node->x, prev_node->previous->y - prev_node->y);
    if (prev_node->previous->vel * prev_node->vel < 0) {
        if (last_segment_length < step_size) {
            // std::cerr << "HybridAstar::last_segment_length too short to follow"
            //           << std::endl;
            return true;
        }
    }

    // force direction
    if (prev_node->vel * HAPlanningConfig::instance()->hybrid_astar_param().force_analytic_expansion_end_direction < 0) {
        return true;
    }

    double theta1 = prev_node->previous->theta;
    double theta2 = prev_node->theta;
    double deltas_x = prev_node->x - prev_node->previous->x;
    double deltas_y = prev_node->y - prev_node->previous->y;
    last_segment_length = std::hypot(deltas_x, deltas_y);

    // double average_heading =
    //         std::atan2((std::cos(theta1) + std::cos(theta2)) / 2,
    //                    (std::sin(theta1) + std::sin(theta2)) / 2);
    double arc_heading = std::atan2(deltas_y, deltas_x);
    double holonomic_error = Utils::normalize_angle(Utils::normalize_angle(theta1 - arc_heading) -
                                                    Utils::normalize_angle(theta2 - arc_heading));
    double holonomic_error_tolerance = 0.02 * std::max(last_segment_length, step_size); // rad
    if (holonomic_error > holonomic_error_tolerance) {
        return true;
    }
    double turn_radius = last_segment_length / (2 * sin(Utils::normalize_angle(theta2 - theta1) / 2.0));
    if (std::abs(turn_radius) < VehicleConfigHelper::Instance()->GetConfig().vehicle_param().minturningradius()) {
        return true;
    }

    return false;
}

bool SBPRSPath::check_single_node_collision(const SearchNode::Ptr &current_state,
                                            const std::vector<SBPObstacleInterface::Ptr> &obs_ptrs,
                                            VehicleModel &vehicle_model) {
    int gear = (current_state->vel < 0.0) ? -1 : 1;
    const SearchNode &pos = *current_state;

    if (pos.previous != nullptr) {
        const SearchNode &previous_pos = *pos.previous;
        int previous_gear = (previous_pos.vel < 0.0) ? -1 : 1;
        if (previous_gear != gear && previous_pos.previous) {
            double x_previous_safe = previous_pos.x + previous_gear *
                                                      HAPlanningConfig::instance()->car_param().lon_inflation_max *
                                                      cos(previous_pos.theta);
            double y_previous_safe = previous_pos.y + previous_gear *
                                                      HAPlanningConfig::instance()->car_param().lon_inflation_max *
                                                      sin(previous_pos.theta);
            SearchNode::Ptr tmp_check_node =
                std::make_shared<HybridAStarNode>(x_previous_safe, y_previous_safe, previous_pos.theta);
            for (const auto &obs_ptr : obs_ptrs) {
                if (obs_ptr->check_collision(tmp_check_node, vehicle_model)) {
                    return true;
                }
            }
        }
    }

    for (const auto &obs_ptr : obs_ptrs) {
        if (obs_ptr->check_collision(current_state, vehicle_model)) {
            return true;
        }
    }

    return false;
}

bool SBPRSPath::check_collision(const double step_size, const std::shared_ptr<GridMap> &grid_map_ptr) {
    if (nodes_.empty()) {
        return false;
    }
    if (nodes_.size() < 2) {

        if (grid_map_ptr->configurationTest(nodes_.back()->x, nodes_.back()->y, nodes_.back()->theta)) {
            return true;
        }
        return false;
    }
    for (size_t i = 1; i + 1 < nodes_.size(); i++) {
        if (grid_map_ptr->configurationTest(nodes_[i]->x, nodes_[i]->y, nodes_[i]->theta)) {
            return true;
        }
    }

    auto prev_node = nodes_.back();
    double last_segment_length =
        std::hypot(prev_node->previous->x - prev_node->x, prev_node->previous->y - prev_node->y);
    if (prev_node->previous->vel * prev_node->vel < 0) {
        if (last_segment_length < step_size) {
            // std::cerr << "HybridAstar::last_segment_length too short to follow"
            //           << std::endl;
            return true;
        }
    }

    // force direction
    if (prev_node->vel * HAPlanningConfig::instance()->hybrid_astar_param().force_analytic_expansion_end_direction < 0) {
        return true;
    }

    double theta1 = prev_node->previous->theta;
    double theta2 = prev_node->theta;
    double deltas_x = prev_node->x - prev_node->previous->x;
    double deltas_y = prev_node->y - prev_node->previous->y;
    last_segment_length = std::hypot(deltas_x, deltas_y);

    // double average_heading =
    //         std::atan2((std::cos(theta1) + std::cos(theta2)) / 2,
    //                    (std::sin(theta1) + std::sin(theta2)) / 2);
    double arc_heading = std::atan2(deltas_y, deltas_x);
    double holonomic_error = Utils::normalize_angle(Utils::normalize_angle(theta1 - arc_heading) -
                                                    Utils::normalize_angle(theta2 - arc_heading));
    double holonomic_error_tolerance = 0.02 * std::max(last_segment_length, step_size); // rad
    if (holonomic_error > holonomic_error_tolerance) {
        return true;
    }
    double turn_radius = last_segment_length / (2 * sin(Utils::normalize_angle(theta2 - theta1) / 2.0));
    if (std::abs(turn_radius) < VehicleConfigHelper::Instance()->GetConfig().vehicle_param().minturningradius()) {
        return true;
    }

    return false;
}

PLANNING_NAMESPACE_END
