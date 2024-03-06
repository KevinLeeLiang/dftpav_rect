#pragma once

#include "parking_planning/apa/common/pose2d.h"

PLANNING_NAMESPACE_START

using haomo::hidelivery::planning::PathPoint;

struct TShapeArea {
    double road_width;
    LineSegment2d upper_line;
    LineSegment2d slot_left_edge;
    LineSegment2d slot_right_edge;
    LineSegment2d slot_bottom_edge;

    LineSegment2d road_left_edge;
    LineSegment2d road_right_edge;

    Vec2d c_point;
};

enum class PathType {
    UNDEFINED = 0,
    STRAIGHT = 1,
    CIRCLE = 2,
    SMOOTH = 3,
};

struct PathSegment {
    PathType path_type = PathType::UNDEFINED;
    Pose2d start_pose = Pose2d();
    Pose2d target_pose = Pose2d();
    Vec2d arc_center = Vec2d();
    double arc_radius = 0.0;
    double arc_theta = 0.0;
    double length = 0.0;
    bool is_right = false;
    std::vector<PathPoint> path_points;
    GearBoxInfoPb::GearNum gear = GearBoxInfoPb::GEAR_NEUTRAL;
};

typedef std::vector<PathSegment> PathSegments;

class Path {
  public:
    DEFINE_SHARDED_PTR(Path)
    explicit Path();
    bool success = false;
    bool roi_success = false;
    Pose2d start_pose = Pose2d();
    Pose2d target_pose = Pose2d();
    double total_length = 0.0;
    size_t gear_switch_count = 0;
    size_t curr_idx = 0;
    PathSegments path_segments;
    std::string msg;

    std::vector<std::vector<PathPoint>> real_path;

    void clear();
    void print() const;
    void merge(const PathSegments &other_path_segments);
    void merge(const std::vector<PathPoint> &path_points);
    void merge_points(const std::vector<PathPoint> &path_points);
    Path::Ptr mirror_injection() const;
};

PLANNING_NAMESPACE_END
