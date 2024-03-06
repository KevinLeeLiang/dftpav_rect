//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : sbp_node.h
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//

#ifndef SBP_NODE_H
#define SBP_NODE_H

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <boost/functional/hash.hpp>

#include "parking_planning/apa/common/planning_typedef.h"
#include "parking_planning/common/planning_gflags.h"

PLANNING_NAMESPACE_START

class SearchNode {
  public:
    DEFINE_SHARDED_PTR(SearchNode)

    SearchNode();
    virtual ~SearchNode() = default;

    size_t get_index() const { return index; };

    virtual void compute_string_index();

    virtual void set_traj_cost();

    // virtual void set_node(double x, double y, double theta, double vel = 0,
    //                       double delta = 0, double wheel_base_offset = 0);

    virtual void set_node(double x, double y, double theta, double vel = 0, double delta = 0, double kappa = 0,
                          double wheel_base_offset = 0);

    friend bool operator==(const SearchNode::Ptr &node1, const SearchNode::Ptr &node2) {
        if (fabs(node1->x - node2->x) < FLAGS_apa_ha_xy_grid_resolution &&
            fabs(node1->y - node2->y) < FLAGS_apa_ha_xy_grid_resolution &&
            fabs(node1->theta - node2->theta) < FLAGS_apa_ha_phi_grid_resolution) {
            return true;
        }else{
            return false;
        }
    }

  public:
    double x = -1.0;
    double y = -1.0;
    double theta = -1.0;

    // gx, gy and gtheta are co-ordinates in the 80X80 grid
    int gx;
    int gy;
    int gtheta;

    // for running dijkstra
    int dx = 0;
    int dy = 0;

    double delta = -1.0;
    double vel = -1.0;
    double cost2d = 0.0;
    double side_diff = 0.0;
    double trajcost = 0.0;
    double obs_cost = 0.0;
    double heuristic_cost_ = 0.0;
    double wheel_base_offset_ = 0.0;

    int zigzags = 0;

    size_t index = 1;

    std::shared_ptr<SearchNode> previous = nullptr;

    // d*Lite use
    int num = 0;
    bool open = true;

    double g_value = 0.0;
    double rhs_value = 0.0;

    double change = 0.0;
    double velocity = 0.0;

    double kappa = -1.0;

    std::shared_ptr<SearchNode> next = nullptr;

    // box
    std::shared_ptr<Box2d> box;
    std::shared_ptr<Box2d> box_real;

    std::pair<double, double> key;
};

class HybridAStarNode : public SearchNode {
  public:
    HybridAStarNode();

    // HybridAStarNode(double x, double y, double theta, double vel = 0,
    //                 double delta = 0, double wheel_base_offset = 0);

    HybridAStarNode(double x, double y, double theta, double vel = 0, double delta = 0, double kappa = 0,
                    double wheel_base_offset = 0);

    virtual void set_node(double x, double y, double theta, double vel = 0, double delta = 0,
                          double wheel_base_offset = 0);

    virtual void set_node(double x, double y, double theta, double vel = 0, double delta = 0, double kappa = 0,
                          double wheel_base_offset = 0);

    virtual void set_traj_cost();
};

PLANNING_NAMESPACE_END

#endif // SBP_NODE_H
