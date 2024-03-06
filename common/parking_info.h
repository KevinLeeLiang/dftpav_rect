#pragma once

#include "parking_planning/apa/common/base_data.h"
#include "parking_planning/apa/common/pose2d.h"
#include "parking_planning/apa/common/parking_slot.h"
#include "parking_planning/apa/common/obstacle.h"
#include "planning_debug_data.pb.h"
#include "proto/apa_decision.pb.h"

PLANNING_NAMESPACE_START

using haomo::hidelivery::planning::AutoParkingInfo;
// using haomo::hidelivery::planning::ParkingDebug;
using haomo::hidelivery::planning::State;

struct ParkEnvironment {
    DEFINE_SHARDED_PTR(ParkEnvironment)

    ParkingTask parking_task;

    // slot coordinate
    ParkingSlot slot;
    std::vector<Vec2d> obstacle_points;
    std::vector<Vec2d> obstacle_points_for_decider;
    std::vector<Vec2d> removed_obstacle_points;

    std::vector<LineSegment2d> obstacle_edges;
    std::vector<LineSegment2d> obstacle_edges_for_decider;

    Eigen::Isometry3d T_world_body = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_body_slot = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_world_slot = Eigen::Isometry3d::Identity();

    Pose2d start_pose;
    Pose2d target_pose;              // 泊入的目标位姿
    std::vector<double> target_area; // 泊出的目标区域（分布：x_min、x_max、y_min、y_max、theta_min、theta_max）

    TShapeArea t_shape;

    Box2d map_boundary;

    bool right_park = true;

    ObstacleInfo obstacle_info;

    ParkingSlot slot_world; // target slot in world frame
    ParkingSlot slot_body;  // target slot in body frame

    std::vector<ObstacleLine::Ptr> raw_obstacle_line_slot_vector;

    std::vector<ObstacleLine::Ptr> obstacle_line_body_vector;
    std::vector<ObstacleLine::Ptr> obstacle_line_world_vector;

    std::vector<ObstacleLine::Ptr> uss_obstacle_body_vector;
    std::vector<ObstacleLine::Ptr> uss_obstacle_world_vector;

    std::vector<ObstacleBox::Ptr> obstacle_box_body_vector;
    std::vector<ObstacleBox::Ptr> obstacle_box_world_vector;

    std::vector<ObstacleBox::Ptr> dynamic_obstacle_box_body_vector;
    std::vector<ObstacleBox::Ptr> dynamic_obstacle_box_world_vector;
};

struct ParkOutput {
    DEFINE_SHARDED_PTR(ParkOutput)

    AutoParkingInfo apa_info;
    State::ParkingResult parking_state;
    std::string obs_state;
    std::string replan_reason;
    std::vector<PathPoint> global_path_output;
    ParkingDebug debug_info;
    std::vector<Vec2d> vertical_planner_search_points;
};

struct ObstacleDeciderResult {
    DEFINE_SHARDED_PTR(ObstacleDeciderResult)
    ObsState obs_state = ObsState::UNDEFINE;
    std::vector<std::vector<Vec2d>> relevant_obs;

    // if is brake, export the following fields:
    bool is_coll = false;
    bool is_static = true;
    double max_free_length = 0;
    Vec2d obs_point;

    bool is_adjust_valid = false;
    Pose2d adjusted_pose;
};

struct ReplanData {
    DEFINE_SHARDED_PTR(ReplanData)
    bool replan_trigger = false;
    bool replan_for_try = false;
    ReplanCmd replan_cmd;
    bool is_force_replan = false;
};

struct PathReqData {
    DEFINE_SHARDED_PTR(PathReqData)
    bool path_req_trigger = true;
    PathRequest decision_proto;
    PathReqStatus status = PathReqStatus::NO_NEED;
    bool is_select_candidate = false;
};

struct PipelineData {
    DEFINE_SHARDED_PTR(PipelineData)

    bool is_following_completed = false;
    bool is_reverse_release = false;
    size_t traj_id = 0;
    size_t reverse_release_id = 0;
    double remain_len = 0.0;
    bool is_escape = false;
    double escape_len = 0.0;
};

class ParkingInfo {
  public:
    DEFINE_SHARDED_PTR(ParkingInfo)

    ParkingInfo() = default;

    virtual ~ParkingInfo() = default;

    ParkEnvironment::ConstPtr park_env() const { return park_env_; }

    ParkEnvironment::Ptr &mutable_park_env() { return park_env_; }

    ParkEnvironment::ConstPtr park_env_backup() const { return park_env_backup_; }

    ParkEnvironment::Ptr &mutable_park_env_backup() { return park_env_backup_; }

    void set_park_env(ParkEnvironment::Ptr park_env) { park_env_ = park_env; }

    const GearBoxInfoPb::GearNum &chassis_gear() const { return chassis_gear_; }

    void set_chassis_gear(const GearBoxInfoPb::GearNum chassis_gear) { chassis_gear_ = chassis_gear; }

    const GearBoxInfoPb::GearNum &planned_gear() const { return planned_gear_; }

    void set_planned_gear(const GearBoxInfoPb::GearNum planned_gear) { planned_gear_ = planned_gear; }

    const double &vehicle_speed() const { return vehicle_speed_; }

    void set_vehicle_speed(const double vehicle_speed) { vehicle_speed_ = vehicle_speed; }

    const double &vehicle_acc() const { return vehicle_acc_; }

    void set_vehicle_acc(const double vehicle_acc) { vehicle_acc_ = vehicle_acc; }

    Path::ConstPtr global_path_info() const { return global_path_info_; }

    Path::Ptr &mutable_global_path_info() { return global_path_info_; }

    Path::ConstPtr current_path_info() const { return current_path_info_; }

    Path::Ptr &mutable_current_path_info() { return current_path_info_; }

    const std::vector<std::vector<PathPoint>> &raw_global_path() const { return raw_global_path_; }

    void set_raw_global_path(const std::vector<std::vector<PathPoint>> &raw_global_path) {
        raw_global_path_ = raw_global_path;
    }

    const std::vector<std::vector<PathPoint>> &global_path() const { return global_path_; }

    void set_global_path(const std::vector<std::vector<PathPoint>> &global_path) { global_path_ = global_path; }

    const std::vector<PathPoint> &published_path() const { return published_path_; }

    void set_published_path(const std::vector<PathPoint> &published_path) { published_path_ = published_path; }

    const std::vector<TrajectoryPoint> &published_trajcetory() const { return published_trajcetory_; }

    void set_published_trajcetory(const std::vector<TrajectoryPoint> &published_trajcetory) {
        published_trajcetory_ = published_trajcetory;
    }

    ObstacleDeciderResult::ConstPtr obstacle_decider_result() const { return obstacle_decider_result_; }

    ObstacleDeciderResult::Ptr &mutable_obstacle_decider_result() { return obstacle_decider_result_; }

    ReplanData::ConstPtr replan_data() const { return replan_data_; }

    ReplanData::Ptr &mutable_replan_data() { return replan_data_; }

    PathReqData::ConstPtr path_req() const { return path_req_; }

    PathReqData::Ptr &mutable_path_req() { return path_req_; }

    PipelineData::ConstPtr pipeline_data() const { return pipeline_data_; }

    PipelineData::Ptr &mutable_pipeline_data() { return pipeline_data_; }

    const std::vector<PathPoint> &heuristic_area() const { return heuristic_area_; }

    void set_heuristic_area(const std::vector<PathPoint> &heuristic_area) { heuristic_area_ = heuristic_area; }

    ParkOutput::ConstPtr park_output() const { return park_output_; }

    ParkOutput::Ptr &mutable_park_output() { return park_output_; }

    void clear();

    void clear_all();

    void set_perception_stamp(uint64_t perception_stamp) { perception_stamp_ = perception_stamp; }

    const uint64_t get_perception_stamp() const { return perception_stamp_; }

    const bool is_active() const { return is_active_; }

    void set_is_active(const bool is_active) { is_active_ = is_active; }

  private:
    ParkEnvironment::Ptr park_env_ = std::make_shared<ParkEnvironment>();
    ParkEnvironment::Ptr park_env_backup_ = std::make_shared<ParkEnvironment>();
    GearBoxInfoPb::GearNum chassis_gear_;
    GearBoxInfoPb::GearNum planned_gear_;
    double vehicle_speed_ = 0.0;
    double vehicle_acc_ = 0.0;

    Path::Ptr global_path_info_ = std::make_shared<Path>();
    Path::Ptr current_path_info_ = std::make_shared<Path>();

    std::vector<std::vector<PathPoint>> raw_global_path_; // 未平滑的全局路径
    std::vector<std::vector<PathPoint>> global_path_;     // 输出的全局路径
    std::vector<PathPoint> published_path_;
    std::vector<TrajectoryPoint> published_trajcetory_;

    ObstacleDeciderResult::Ptr obstacle_decider_result_ = std::make_shared<ObstacleDeciderResult>();

    ReplanData::Ptr replan_data_ = std::make_shared<ReplanData>();
    PathReqData::Ptr path_req_ = std::make_shared<PathReqData>();
    PipelineData::Ptr pipeline_data_ = std::make_shared<PipelineData>();
    std::vector<PathPoint> heuristic_area_;
    ParkOutput::Ptr park_output_ = std::make_shared<ParkOutput>();

    uint64_t perception_stamp_;
    bool is_active_ = false;
};

PLANNING_NAMESPACE_END
