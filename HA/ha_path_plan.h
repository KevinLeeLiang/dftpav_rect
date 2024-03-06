//
// Created by garen_lee on 2024/1/23.
/**
 ******************************************************************************
 * @file           : ha_path_plan.h
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/23
 ******************************************************************************
 */
//

#ifndef HA_PATH_PLAN_H
#define HA_PATH_PLAN_H

#pragma once

#include "parking_planning/apa/common/planning_info.h"
#include "apa/path_plan/HA/ha_path_planner/sbp_node.h"
#include "apa/path_plan/HA/ha_path_planner/hybrid_astar_planner.h"
#include "parking_planning/apa/path_plan/HA/obstacle/obstacle_line.h"
#include "parking_planning/apa/path_plan/HA/obstacle/obstacle_point.h"

PLANNING_NAMESPACE_START
class HAPathPlan {
  public:
    DEFINE_SHARDED_PTR(HAPathPlan)
    HAPathPlan(const size_t max_node_num);
    ~HAPathPlan() = default;
    void Plan(const PlanningInfo::Ptr &planning_info, Path::Ptr &path);

  private:
    HybridAStarPlanner::Ptr ha_planner_;
    std::vector<SBPObstacleInterface::Ptr> obs_ptrs_;

    Box2d map_boundary_;
    bool is_rev_;

  private:
    void buildGridMap(const std::vector<LineSegment2d> &obs_lines, const std::vector<Vec2d> &obs_pts,
                      const Box2d &map_boundary);

    void SetResult(SBPResult *ha_res, Path::Ptr &path);
    void samplePath(const PlanningInfo::Ptr &planning_info, Path::Ptr &path);
    std::vector<PathPoint> samplePath(PlanningInfo::Ptr planning_info, const PathSegment &path_segment);

};
PLANNING_NAMESPACE_END
#endif // HA_PATH_PLAN_H
