#ifndef PLANNING_PARK_ENV_CONSTRUCTOR_H
#define PLANNING_PARK_ENV_CONSTRUCTOR_H

#include "parking_slot.h"
#include "obstacle.h"
#include "parking_info.h"
#include "apa/decision/target_pose_decider.h"

PLANNING_NAMESPACE_START

class ParkEnvConstructor {
  public:
    ParkEnvConstructor() = default;

    virtual ~ParkEnvConstructor() = default;

    ParkEnvironment::Ptr update(const haomo::hidelivery::perception::PerceptionView &perception,
                                const haomo::hidelivery::perception::PerceptionView &fusion_uss,
                                const haomo::hidelivery::perception::PerceptionView &fusion_object,
                                const haomo::hidelivery::localization::Pose &pose, const bool is_reset,
                                const bool is_replan);

    void update(const haomo::hidelivery::perception::PerceptionView &perception,
                const haomo::hidelivery::perception::PerceptionView &fusion_uss,
                const haomo::hidelivery::perception::PerceptionView &fusion_object,
                const haomo::hidelivery::localization::Pose &pose, const bool is_reset, const bool is_replan,
                ParkEnvironment::Ptr park_env);

    void reset();

    bool valid() const;

  private:
    Box2d generate_map_bound(const ParkingSlot &slot, const Pose2d &start_pose) const;

    bool check_parking_slot(const ParkingSlot &parking_slot) const;

  private:
    ParkEnvironment::Ptr last_park_env_ = nullptr;
    double front_edge_park_depth_ = 0.1;
    SlotCoordinateType slot_coord_type_ = SlotCoordinateType::Front_Left;
    bool valid_ = false;
    ParkingSlotManager slot_manager_;
    ObstacleManager obstacle_manager_;
    std::shared_ptr<TargetPoseDecider> targ_pose_decider_ = std::make_shared<TargetPoseDecider>();

    // debug data
    std::vector<Vec2d> removed_obstacles_points_;
};

PLANNING_NAMESPACE_END

#endif // PLANNING_PARK_ENV_CONSTRUCTOR_H
