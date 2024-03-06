#ifndef PLANNING_PARA_FUNC_H
#define PLANNING_PARA_FUNC_H

#include "math_helper.h"
#include <array>
PLANNING_NAMESPACE_START

class ParaFunc {
  public:
    static bool mutable_dubins_path(const Pose2d &start_loc, const Pose2d &end_loc, const bool is_first_straight,
                                    std::array<double, 5> &heuristic_result);

    static bool calculate_heuristic_path(const Pose2d &start_loc, const Pose2d &target_loc,
                                         PathSegments &heuristic_result);

    static bool calculate_csc_path(const Pose2d &start_pose, const Pose2d &target_pose,
                                   const double &min_turning_radius, const bool is_forward, PathSegments &csc_result);

    static PathSegment reverse_path_segment(const PathSegment &path_segment);

    static PathSegment make_straight_pathsegment(const Pose2d &start_pose, const Pose2d &end_pose, const double &length,
                                                 bool is_reverse);

    static PathSegment make_circle_pathsegment(const Pose2d &start_pose, const Pose2d &end_pose,
                                               const Vec2d &arc_center, const double &radius, const double &theta,
                                               bool is_right, bool is_reverse);

    static bool check_path_segments(const std::vector<PathSegment> &path_segments);

    static bool check_path_segment(const PathSegment &path_segment);

    static bool check_pose(const Pose2d &pose, const std::string &topic);

    static bool check_vec(const Vec2d &vec, const std::string &topic);

    static bool check_gear(const GearBoxInfoPb::GearNum &gear);

    static bool check_scalar(const double &data, const std::string &topic);
};

PLANNING_NAMESPACE_END

#endif // PLANNING_PARA_FUNC_H
