//
// Created by garen_lee on 2024/1/26.
/**
 ******************************************************************************
 * @file           : compare.cpp.cc
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/26
 ******************************************************************************
 */
//

#include "compare.h"
#include <iostream>
#include <string.h>
#include "../ha_config/ha_planning_config.h"
#include "utils.h"

PLANNING_NAMESPACE_START

SearchNode Compare::target;
Pose2d Compare::cmp_frame_pose_;
// float **Compare::shortest_2d = nullptr;
// int **Compare::grid_obs_map = nullptr;
std::vector<std::vector<char>> Compare::grid_obs_map;
std::vector<std::vector<float>> Compare::shortest_2d;
int Compare::DX_ = 0;
int Compare::DY_ = 0;

typedef bool (*compare2dSignature)(SearchNode, SearchNode);

Compare::Compare()
    : xy_grid_resolution_(static_cast<float>(HAPlanningConfig::instance()->hybrid_astar_param().xy_grid_resolution))
    , grid_Dijkstra_xy_resolution_(
        static_cast<float>(HAPlanningConfig::instance()->hybrid_astar_param().grid_dijkstra_xy_resolution))
    , holonomic_with_obs_heuristic_(
        static_cast<float>(HAPlanningConfig::instance()->hybrid_astar_param().holonomic_with_obs_heuristic))
    , non_holonomic_without_obs_heuristic_(
        static_cast<float>(HAPlanningConfig::instance()->hybrid_astar_param().non_holonomic_without_obs_heuristic)) {}

Compare::Compare(double xy_grid_resolution, double grid_Dijkstra_xy_resolution)
    : xy_grid_resolution_(static_cast<float>(xy_grid_resolution))
    , grid_Dijkstra_xy_resolution_(static_cast<float>(grid_Dijkstra_xy_resolution))
    , holonomic_with_obs_heuristic_(
        static_cast<float>(HAPlanningConfig::instance()->hybrid_astar_param().holonomic_with_obs_heuristic))
    , non_holonomic_without_obs_heuristic_(
        static_cast<float>(HAPlanningConfig::instance()->hybrid_astar_param().non_holonomic_without_obs_heuristic)) {}

void Compare::loadFrom(const std::vector<SBPObstacleInterface::Ptr> &obstacles, const double x_bound,
                       const double y_bound) const {
    DX_ = std::max(1, int(ceil(x_bound / grid_Dijkstra_xy_resolution_)));
    DY_ = std::max(1, int(ceil(y_bound / grid_Dijkstra_xy_resolution_)));

    MLOG(PARKING_PLANNING, INFO) << "DX_ " << DX_;
    MLOG(PARKING_PLANNING, INFO) << "DY_ " << DY_;
    grid_obs_map.resize(DX_);
    for (int i = 0; i < DX_; i++) {
        grid_obs_map[i].resize(DY_);
        for (int j = 0; j < DY_; j++) {
            grid_obs_map[i][j] = 0;
            for (const auto &obstacle : obstacles) {
                Vec2d temp_grid = Utils::tf2d_inv(cmp_frame_pose_, Vec2d((i + 0.5) * grid_Dijkstra_xy_resolution_,
                                                                         (j + 0.5) * grid_Dijkstra_xy_resolution_));
                if (obstacle->get_distance(temp_grid) < sqrt(0.5) * grid_Dijkstra_xy_resolution_) {
                    grid_obs_map[i][j] = 1;
                    break;
                }
            }
        }
    }
}

bool Compare::operator()(const SearchNode::Ptr &s1, const SearchNode::Ptr &s2) const {
    // TODO: utilize State->cost3d as second heuristic
    return (s1->trajcost + s1->obs_cost + holonomic_with_obs_heuristic_ * holonomic_with_obs(s1) +
            non_holonomic_without_obs_heuristic_ * non_holonomic_without_obs(s1)) >
           (s2->trajcost + s2->obs_cost + holonomic_with_obs_heuristic_ * holonomic_with_obs(s2) +
            non_holonomic_without_obs_heuristic_ * non_holonomic_without_obs(s2));
}

bool compare2d(SearchNode a, SearchNode b) {
    return a.cost2d > b.cost2d; // simple dijkstra
}

// currently uses dijkstra's algorithm in x-y space
float Compare::holonomic_with_obs(const SearchNode::Ptr &src) const {
    Pose2d temp_src = Utils::tf2d(cmp_frame_pose_, Pose2d(src->x, src->y, src->theta));
    if ((size_t)(temp_src.x() / grid_Dijkstra_xy_resolution_) >= shortest_2d.size())
        return 10000;
    if ((size_t)(temp_src.y() / grid_Dijkstra_xy_resolution_) >=
        shortest_2d.at((size_t)(temp_src.x() / grid_Dijkstra_xy_resolution_)).size())
        return 10000;
    return shortest_2d[(int)(temp_src.x() / grid_Dijkstra_xy_resolution_)]
    [(int)(temp_src.y() / grid_Dijkstra_xy_resolution_)];
    // return shortest_2d[(int)(src->x /
    // grid_Dijkstra_xy_resolution_)][(int)(src->y /
    // grid_Dijkstra_xy_resolution_)];
}

void Compare::run_dijkstra(const double x_bound, const double y_bound) const {
    SearchNode src = Compare::target;

    Pose2d temp_tar(target.x, target.y, target.theta);
    temp_tar = Utils::tf2d(cmp_frame_pose_, temp_tar);
    HybridAStarNode temp_end(temp_tar.x(), temp_tar.y(), temp_tar.theta(), 0, 0, 0, 0);
    target.dx = temp_end.gx * xy_grid_resolution_ / grid_Dijkstra_xy_resolution_;
    target.dy = temp_end.gy * xy_grid_resolution_ / grid_Dijkstra_xy_resolution_;
    src.dx = target.dx; // src.gx * xy_grid_resolution_ / grid_Dijkstra_xy_resolution_;
    src.dy = target.dy; // src.gy * xy_grid_resolution_ / grid_Dijkstra_xy_resolution_;

    std::priority_queue<SearchNode, std::vector<SearchNode>, compare2dSignature> frontier(&compare2d);

    int vis[DX_][DY_];

    // delete[] shortest_2d;
    // shortest_2d = new float *[DX_];
    shortest_2d.resize(DX_);
    for (int i = 0; i < DX_; i++) {
        // shortest_2d[i] = new float[DY_];
        shortest_2d[i].resize(DY_);
        for (int j = 0; j < DY_; j++) {
            shortest_2d[i][j] = 0;
        }
    }

    memset(vis, 0, sizeof(int) * DX_ * DY_);

    for (int i = 0; i < DX_; i++) {
        for (int j = 0; j < DY_; j++) {
            shortest_2d[i][j] = 10000;
        }
    }

    shortest_2d[src.dx][src.dy] = 0;

    frontier.push(src);
    // int count = 0;
    while (!frontier.empty()) {
        SearchNode current = frontier.top();
        frontier.pop();

        int x = current.dx;
        int y = current.dy;

        if (vis[x][y]) {
            continue;
        }

        vis[x][y] = 1;

        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                if (x + i < 0 || x + i >= DX_ || y + j < 0 || y + j >= DY_)
                    continue;
                if ((i == 0 && j == 0))
                    continue;

                if (shortest_2d[x + i][y + j] > shortest_2d[x][y] + sqrt(i * i + j * j) ||
                    grid_obs_map[x + i][y + j] != 0) {
                    if (grid_obs_map[x + i][y + j] == 0) // no obstacle
                    {
                        shortest_2d[x + i][y + j] =
                            shortest_2d[x][y] + sqrt(i * i + j * j) * grid_Dijkstra_xy_resolution_;
                    }
                    SearchNode tempstate;
                    tempstate.dx = current.dx + i;
                    tempstate.dy = current.dy + j;
                    tempstate.cost2d = shortest_2d[x + i][y + j];
                    frontier.push(tempstate);
                }
            }
        }
    }
}

float Compare::non_holonomic_without_obs(const SearchNode::Ptr &src) { return src->heuristic_cost_; }

PLANNING_NAMESPACE_END