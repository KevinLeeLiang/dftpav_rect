//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : obstacle_interface.cpp.c
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//
#include "obstacle_interface.h"

PLANNING_NAMESPACE_START

Vec2d SBPObstacleInterface::get_single_nearest_point(const LineSegment2d &obs_line,
                                                     const LineSegment2d &ego_centerline) {
    if (obs_line.HasIntersect(ego_centerline)) {
        return Vec2d(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
    }

    // project obs_line onto ego_centerline
    std::vector<double> dis(4, std::numeric_limits<double>::infinity());
    dis[0] = ego_centerline.DistanceTo(obs_line.start());
    dis[1] = ego_centerline.DistanceTo(obs_line.end());

    // project ego_centerline onto obs_line
    Vec2d *nearest_pt_2 = new Vec2d();
    Vec2d *nearest_pt_3 = new Vec2d();
    dis[2] = obs_line.DistanceTo(ego_centerline.start(), nearest_pt_2);
    dis[3] = obs_line.DistanceTo(ego_centerline.end(), nearest_pt_3);

    int min_index = min_element(dis.begin(), dis.end()) - dis.begin();

    switch (min_index) {
    case 0:
        return obs_line.start();
    case 1:
        return obs_line.end();
    case 2:
        return *nearest_pt_2;
    case 3:
        return *nearest_pt_3;
    default:
        return Vec2d(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
    }
}

PLANNING_NAMESPACE_END
