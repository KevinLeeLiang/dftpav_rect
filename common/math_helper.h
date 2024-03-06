#ifndef PLANNING_MATH_HELPER_H
#define PLANNING_MATH_HELPER_H

#include "parking_info.h"
#include "planning_info.h"
#include "vehicle_model.h"
#include <boost/optional.hpp>

PLANNING_NAMESPACE_START

class MathHelper {
  public:
    static PathSegment compute_max_rotate_angle(const PlanEnvironment::ConstPtr &park_env_ptr,
                                                const PathSegment &path_segment, bool is_turning_left,
                                                VehicleModel &vehicle_model,
                                                std::shared_ptr<Vec2d> related_point = nullptr,
                                                std::shared_ptr<LineSegment2d> related_line = nullptr);

    static PathSegment compute_max_straight_length(const PlanEnvironment::ConstPtr &park_env_ptr,
                                                   const PathSegment &path_segment, VehicleModel &vehicle_model,
                                                   std::shared_ptr<Vec2d> related_point = nullptr,
                                                   std::shared_ptr<LineSegment2d> related_line = nullptr);

    static double compute_max_rotate_angle(const PlanEnvironment::ConstPtr &park_env_ptr, const Pose2d &start_pose,
                                           const std::vector<Vec2d> &vehicle_corners, const double &radius,
                                           const Vec2d &center, bool is_turning_left, bool is_forward,
                                           VehicleModel &vehicle_model, std::shared_ptr<Vec2d> related_point = nullptr,
                                           std::shared_ptr<LineSegment2d> related_line = nullptr);

    static double compute_max_straight_length(const PlanEnvironment::ConstPtr &park_env_ptr, const Pose2d &start_pose,
                                              const Pose2d &end_pose, bool is_forward, VehicleModel &vehicle_model,
                                              std::shared_ptr<Vec2d> related_point = nullptr,
                                              std::shared_ptr<LineSegment2d> related_line = nullptr);

    static double compute_max_rotate_angle(const Pose2d &start_pose, const std::vector<Vec2d> &vehicle_corners,
                                           const std::vector<Vec2d> &obstacle_points,
                                           const std::vector<LineSegment2d> &obstacle_lines, const Vec2d &arc_center,
                                           const bool &is_turning_left, const bool &is_forward,
                                           const double &safety_thresh, std::shared_ptr<Vec2d> related_point = nullptr,
                                           std::shared_ptr<LineSegment2d> related_line = nullptr);

    static double compute_max_straight_length(const Pose2d &start_pose, const std::vector<Vec2d> &vehicle_corners,
                                              const std::vector<Vec2d> &obstacle_points,
                                              const std::vector<LineSegment2d> &obstacle_lines,
                                              const double &straight_length, const bool &is_forward,
                                              const double &safety_thresh,
                                              std::shared_ptr<Vec2d> related_point = nullptr,
                                              std::shared_ptr<LineSegment2d> related_line = nullptr);

    static double compute_max_straight_length_with_cut(
        const Pose2d &start_pose, const std::vector<Vec2d> &vehicle_corners, const std::vector<Vec2d> &obstacle_points,
        const std::vector<LineSegment2d> &obstacle_lines, const double &straight_length, const bool &is_forward,
        const double &safety_thresh, std::shared_ptr<Vec2d> related_point = nullptr,
        std::shared_ptr<LineSegment2d> related_line = nullptr);

    static std::vector<std::pair<double, Vec2d>>
    compute_line_circle_intersection(const Vec2d &arc_center, double radius, const LineSegment2d &line);

    static double compute_rotate_angle(const Vec2d &corner, const Vec2d &intersection_point, const Vec2d &arc_center);

    static std::vector<PathPoint> straight_sample(const PathSegment &path_segment);

    static std::vector<PathPoint> arc_sample(const PathSegment &path_segment);

    static Vec2d center(const Pose2d &start_pose, double radius, bool is_right);

    static bool able_park(const Pose2d &start_pose, const Pose2d &target_pose, const double &aim_R,
                          const double &safty_bottom_distance_buffer);

    static bool parallel_able_park(const Pose2d &start_pose, const Pose2d &target_pose, const double &aim_R,
                                   const std::string &park_state, const double &safty_front_distance_buffer,
                                   const double &safty_rear_distance_buffer);

    static bool ready_park(const Pose2d &start_pose, const Pose2d &target_pose);

    static bool parallel_ready_park(const Pose2d &start_pose, const Pose2d &target_pose);

    static double dis_point_to_line(const Vec2d &point_a, const Vec2d &point_b, const Vec2d &point_p);

    static Pose2d get_target_loc(const Pose2d &vehicle_state, const Vec2d &center_of_circle,
                                 const double &circle_radius, const double &theta);

    static PathPoint transform_path_point(const PathPoint &path_point, const Eigen::Isometry3d &T_world_local,
                                          bool mirror);

    static LineSegment2d transform_line(const LineSegment2d &line, const Eigen::Isometry3d &T_world_local, bool mirror);

    static std::vector<TrajectoryPoint> transform_trajectory(const std::vector<TrajectoryPoint> &trajectory_local,
                                                             const Eigen::Isometry3d &T_world_local, bool mirror);

    static std::vector<std::vector<PathPoint>>
    transform_path_points(const std::vector<std::vector<PathPoint>> &trajectory_local,
                          const Eigen::Isometry3d &T_world_local, bool mirror);

    static Pose2d transform_pose(const Pose2d &pose_local, const Eigen::Isometry3d &T, bool mirror);

    static Vec2d transform_vec2d(const Vec2d &point_local, const Eigen::Isometry3d &T, bool mirror);

    static PathSegments srs_path(const Pose2d &start_pose, const Pose2d &target_pose, const double &min_turning_radius);

    static GearBoxInfoPb::GearNum compute_gear(const Pose2d &start_pose, const Pose2d &target_pose);

    static double compute_path_cost(Path::Ptr path);

    static bool path_segments_check_collision(const PathSegments &path_segments,
                                              const std::vector<Vec2d> &obstacle_points,
                                              const std::vector<LineSegment2d> &obstacle_lines,
                                              VehicleModel &vehicle_model, std::shared_ptr<Vec2d> related_point,
                                              std::shared_ptr<LineSegment2d> related_line, const double &safe_thresh);

    static bool path_segments_check_collision(const PathSegments &path_segments,
                                              const std::vector<Vec2d> &obstacle_points,
                                              const std::vector<LineSegment2d> &obstacle_lines,
                                              VehicleModel &vehicle_model_drive, VehicleModel &vehicle_model_reverse,
                                              std::shared_ptr<Vec2d> related_point,
                                              std::shared_ptr<LineSegment2d> related_line, const double &safe_thresh);

    static bool LpSpRp(const double x, const double y, const double phi, double &t, double &u, double &v);

    static double yline_fit(const std::vector<Vec2d> &pts);

    static double b_line_fit(const std::vector<Vec2d> &pts, const double &a);

    static bool check_collision(const VehicleModel &veh_model, const std::vector<Vec2d> &obs_pts,
                                const std::vector<LineSegment2d> &obs_lines);
};

PLANNING_NAMESPACE_END

#endif // PLANNING_MATH_HELPER_H
