#pragma once

#include "parking_planning/apa/common/base_data.h"
#include "parking_planning/apa/common/pose2d.h"
#include "parking_planning/apa/common/parking_slot.h"
#include "parking_planning/apa/common/obstacle.h"
#include "proto/apa_decision.pb.h"

PLANNING_NAMESPACE_START

// using haomo::hidelivery::planning::ParkingDebug;

// 车位系数据
struct PlanEnvironment {
    DEFINE_SHARDED_PTR(PlanEnvironment)

    ParkingSlot slot;
    std::vector<Vec2d> obstacle_points;
    std::vector<LineSegment2d> obstacle_edges;

    Pose2d start_pose;
    Pose2d target_pose;

    bool is_candidate_valid = false;
    std::vector<Pose2d> candidate_pose;

    std::vector<double> target_area; // 泊出的目标区域（分布：x_min、x_max、y_min、y_max、theta_min、theta_max）

    TShapeArea t_shape;

    bool right_park = true;

    ObstacleInfo obstacle_info;
};

struct PathPlanDebug {
    DEFINE_SHARDED_PTR(PathPlanDebug)

    std::string planner_method;
    std::string failure_reason;
    std::vector<Vec2d> obstacle_points; // 路径规划失败时，影响的障碍物点
    std::vector<Vec2d> obstacle_edges;  // 路径规划失败时，影响的障碍物线
    std::vector<Vec2d> vertical_planner_search_points;
};

class PlanningInfo {
  public:
    DEFINE_SHARDED_PTR(PlanningInfo)

    PlanningInfo() = default;

    virtual ~PlanningInfo() = default;

    PlanEnvironment::ConstPtr plan_env() const { return plan_env_; }

    PlanEnvironment::Ptr &mutable_plan_env() { return plan_env_; }

    const GearBoxInfoPb::GearNum &chassis_gear() const { return chassis_gear_; }

    void set_chassis_gear(const GearBoxInfoPb::GearNum chassis_gear) { chassis_gear_ = chassis_gear; }

    Path::ConstPtr global_path_info() const { return global_path_info_; }

    Path::Ptr &mutable_global_path_info() { return global_path_info_; }

    ParkingTask parking_task() const { return parking_task_; }

    void set_parking_task(const ParkingTask &parking_task) { parking_task_ = parking_task; }

    PathPlanType path_plan_type() const { return path_plan_type_; }

    void set_path_plan_type(const PathPlanType &path_plan_type) { path_plan_type_ = path_plan_type; }

    const std::vector<PathPoint> &heuristic_area() const { return heuristic_area_; }

    void set_heuristic_area(const std::vector<PathPoint> &heuristic_area) { heuristic_area_ = heuristic_area; }

    PathPlanDebug::ConstPtr path_plan_debug() const { return path_plan_debug_; }

    PathPlanDebug::Ptr &mutable_path_plan_debug() { return path_plan_debug_; }

    void clear();

    void clear_all();

    void set_perception_stamp(uint64_t perception_stamp) { perception_stamp_ = perception_stamp; }

    const uint64_t get_perception_stamp() const { return perception_stamp_; }

    void set_is_select_candidate(const bool is_select_candidate) { is_select_candidate_ = is_select_candidate; }

    const bool get_is_select_candidate() const { return is_select_candidate_; }

  private:
    PlanEnvironment::Ptr plan_env_ = std::make_shared<PlanEnvironment>();
    GearBoxInfoPb::GearNum chassis_gear_;

    Path::Ptr global_path_info_ = std::make_shared<Path>();
    ParkingTask parking_task_;
    PathPlanType path_plan_type_ = PathPlanType::UNKNOWN_TYPE;

    std::vector<PathPoint> heuristic_area_;
    PathPlanDebug::Ptr path_plan_debug_ = std::make_shared<PathPlanDebug>();
    uint64_t perception_stamp_;

    bool is_select_candidate_ = false;
};

PLANNING_NAMESPACE_END
