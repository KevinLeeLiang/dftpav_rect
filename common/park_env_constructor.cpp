#include "park_env_constructor.h"
#include "vehicle_model.h"

PLANNING_NAMESPACE_START

ParkEnvironment::Ptr ParkEnvConstructor::update(const haomo::hidelivery::perception::PerceptionView &perception,
                                                const haomo::hidelivery::perception::PerceptionView &fusion_uss,
                                                const haomo::hidelivery::perception::PerceptionView &fusion_object,
                                                const haomo::hidelivery::localization::Pose &pose, const bool is_reset,
                                                const bool is_replan) {
    auto park_env = std::make_shared<ParkEnvironment>();
    update(perception, fusion_uss, fusion_object, pose, is_reset, is_replan, park_env);
    return park_env;
}

void ParkEnvConstructor::update(const haomo::hidelivery::perception::PerceptionView &perception,
                                const haomo::hidelivery::perception::PerceptionView &fusion_uss,
                                const haomo::hidelivery::perception::PerceptionView &fusion_object,
                                const haomo::hidelivery::localization::Pose &pose, const bool is_reset,
                                const bool is_replan, ParkEnvironment::Ptr park_env) {
    if (is_reset) {
        slot_manager_.reset();
    }

    if (pose.status() != haomo::hidelivery::localization::Pose::OK) {
        MLOG(PARKING_PLANNING, REPORT) << "E_Planning_Parking_Input_Error"
                                       << "|"
                                       << " ego pose data error";
        // MLOG(PARKING_PLANNING, ERROR) << "ego pose data error";
        valid_ = false;
        return;
    }

    park_env->T_world_body.linear() = Eigen::Quaterniond(pose.odom_attitude().qw(), pose.odom_attitude().qx(),
                                                         pose.odom_attitude().qy(), pose.odom_attitude().qz())
                                          .matrix();
    park_env->T_world_body.translation() =
        Eigen::Vector3d(pose.odom_position().x(), pose.odom_position().y(), pose.odom_position().z());

    if (park_env->parking_task.park_mode() == ParkingTask::PARK_IN) {
        if (!slot_manager_.update(perception, park_env->T_world_body, slot_coord_type_)) {
            MLOG(PARKING_PLANNING, REPORT) << "E_Planning_Parking_Input_Error"
                                           << "|"
                                           << " slot data error";
            // MLOG(PARKING_PLANNING, ERROR) << "slot data error";
            valid_ = false;
            return;
        }
    } else if (park_env->parking_task.park_mode() == ParkingTask::PARK_OUT) {
        if (!slot_manager_.update_for_out(perception, park_env->T_world_body, slot_coord_type_)) {
            MLOG(PARKING_PLANNING, REPORT) << "E_Planning_Parking_Input_Error"
                                           << "|"
                                           << " slot data error";
            // MLOG(PARKING_PLANNING, ERROR) << "slot data error";
            valid_ = false;
            return;
        }
    } else {
        MLOG(PARKING_PLANNING, INFO) << "park mode is not suport!";
        valid_ = false;
        return;
    }

    if (is_reset) {
        park_env->T_world_slot = slot_manager_.get_slot_world_transform();
        park_env->T_body_slot = slot_manager_.get_slot_body_transform();
        park_env->right_park = slot_manager_.right_park();
        park_env->slot = *slot_manager_.get_slot(CoordinateType::Slot);
    } else {
        // T_world_slot 和 right_park 保持不变
        park_env->T_body_slot = park_env->T_world_body.inverse() * park_env->T_world_slot;
        park_env->slot = *slot_manager_.get_slot(CoordinateType::Slot);
        if (!park_env->right_park) {
            park_env->slot.mirror_injection();
        }
        Eigen::Isometry3d T_fixed_curr = park_env->T_world_slot.inverse() * slot_manager_.get_slot_world_transform();
        park_env->slot = *(park_env->slot.transform(T_fixed_curr));
        if (!park_env->right_park) {
            park_env->slot.mirror_injection();
        }
    }

    if (!check_parking_slot(park_env->slot)) {
        if (last_park_env_ != nullptr) {
            MLOG(PARKING_PLANNING, INFO) << "Parking slot size has changed, and use last frame data";
            park_env = last_park_env_;
            valid_ = true;
        } else {
            MLOG(PARKING_PLANNING, REPORT) << "E_Planning_Parking_Input_Error"
                                           << "|"
                                           << " Parking slot size is error";
            // MLOG(PARKING_PLANNING, ERROR) << "Parking slot size is error";
            valid_ = false;
        }
        return;
    }
    if (last_park_env_ != nullptr && last_park_env_->right_park != park_env->right_park) {
        if (park_env->right_park) {
            MLOG(PARKING_PLANNING, DEBUG) << "left park change to right park";
        } else {
            MLOG(PARKING_PLANNING, DEBUG) << "right park change to left park";
        }
    }

    obstacle_manager_.update(perception, fusion_uss, fusion_object, park_env->T_world_body, park_env->T_body_slot,
                             park_env->right_park);

    Eigen::Isometry3d T_slot_body = park_env->T_body_slot.inverse();
    park_env->start_pose = Pose2d::from_iso3d(T_slot_body);
    if (!park_env->right_park) {
        park_env->start_pose = park_env->start_pose.mirror_injection();
    }

    //障碍物点粗删及收集
    park_env->map_boundary = generate_map_bound(park_env->slot, park_env->start_pose);
    bool is_construct_t_shape = is_reset || is_replan;
    obstacle_manager_.collect_data_and_construct_T(park_env, is_construct_t_shape);

    if (park_env->parking_task.park_mode() == ParkingTask::PARK_IN) {
        park_env->target_pose = targ_pose_decider_->calcuate_target_pose(park_env);
    } else {
        park_env->target_area = targ_pose_decider_->calcuate_target_area(park_env);
    }

    park_env->slot_world = *slot_manager_.get_slot(CoordinateType::World);
    park_env->slot_body = *slot_manager_.get_slot(CoordinateType::Body);

    park_env->obstacle_line_world_vector = obstacle_manager_.get_obstacle_lines(CoordinateType::World);
    park_env->obstacle_line_body_vector = obstacle_manager_.get_obstacle_lines(CoordinateType::Body);

    park_env->uss_obstacle_body_vector = obstacle_manager_.get_uss_obstacle(CoordinateType::Body);
    park_env->uss_obstacle_world_vector = obstacle_manager_.get_uss_obstacle(CoordinateType::World);

    park_env->obstacle_box_body_vector = obstacle_manager_.get_static_obstacle_box(CoordinateType::Body);
    park_env->obstacle_box_world_vector = obstacle_manager_.get_static_obstacle_box(CoordinateType::World);

    park_env->dynamic_obstacle_box_body_vector = obstacle_manager_.get_dynamic_obstacle_box(CoordinateType::Body);
    park_env->dynamic_obstacle_box_world_vector = obstacle_manager_.get_dynamic_obstacle_box(CoordinateType::World);

    valid_ = true;
    last_park_env_ = park_env;
    MLOG(PARKING_PLANNING, INFO) << "T_world_body: " << park_env->T_world_body.translation().transpose();
    MLOG(PARKING_PLANNING, INFO) << "T_world_slot: " << park_env->T_world_slot.translation().transpose();
    MLOG(PARKING_PLANNING, INFO) << "T_body_slot: " << park_env->T_body_slot.translation().transpose();
    MLOG(PARKING_PLANNING, INFO) << "start pose: " << park_env->start_pose;
    MLOG(PARKING_PLANNING, INFO) << "target pose: " << park_env->target_pose;
}

bool ParkEnvConstructor::valid() const { return valid_; }

void ParkEnvConstructor::reset() {
    last_park_env_ = nullptr;
    slot_manager_.reset();
    obstacle_manager_.reset();
}

Box2d ParkEnvConstructor::generate_map_bound(const ParkingSlot &slot, const Pose2d &start_pose) const {
    Pose2d T_sfl_sc(slot.center().x(), slot.center().y(), slot.heading());
    Pose2d T_sc_sfl = T_sfl_sc.inverse();

    ParkingSlot slot_center = T_sc_sfl * slot;
    Pose2d start_pose_center = T_sc_sfl * start_pose;

    const double u_turn_envelope_radius =
        std::hypot(VehicleConfigHelper::Instance()->GetConfig().vehicle_param().minturningradius() +
                       VehicleConfigHelper::Instance()->GetConfig().vehicle_param().overallwidth() / 2,
                   VehicleConfigHelper::Instance()->GetConfig().vehicle_param().front_edge_to_center());
    const double x_buffer_near_lot = 1.0;
    const double y_buffer_near_lot =
        VehicleConfigHelper::Instance()->GetConfig().vehicle_param().minturningradius() / 2;

    Box2d box_lot_with_buffer = slot_center.box();
    box_lot_with_buffer.Shift(u_turn_envelope_radius / 2 *
                              Vec2d(box_lot_with_buffer.cos_heading(), box_lot_with_buffer.sin_heading()));
    box_lot_with_buffer.LongitudinalExtend(u_turn_envelope_radius + 2 * x_buffer_near_lot);
    box_lot_with_buffer.LateralExtend(y_buffer_near_lot * 2);
    AABox2d map_aabox_local = box_lot_with_buffer.GetAABox();

    double nearby_buffer = 2.2*u_turn_envelope_radius;
    map_aabox_local.MergeFrom(Vec2d(start_pose_center.x() + nearby_buffer, start_pose_center.y() + nearby_buffer));
    map_aabox_local.MergeFrom(Vec2d(start_pose_center.x() - nearby_buffer, start_pose_center.y() - nearby_buffer));
    Vec2d virtual_gear_switch_point_local(
        VehicleConfigHelper::Instance()->GetConfig().vehicle_param().minturningradius(),
        -VehicleConfigHelper::Instance()->GetConfig().vehicle_param().minturningradius());
    if (start_pose_center.rotation().sin_theta() > 0) {
        virtual_gear_switch_point_local =
            Vec2d(VehicleConfigHelper::Instance()->GetConfig().vehicle_param().minturningradius(),
                  VehicleConfigHelper::Instance()->GetConfig().vehicle_param().minturningradius());
    }
    map_aabox_local.MergeFrom(virtual_gear_switch_point_local + Vec2d(nearby_buffer, nearby_buffer));
    map_aabox_local.MergeFrom(virtual_gear_switch_point_local + Vec2d(-nearby_buffer, -nearby_buffer));
    // map_aabox_local.MergeFrom(Vec2d(target_state_local_.path_point.x +
    // nearby_buffer, target_state_local_.path_point.y + nearby_buffer));
    // map_aabox_local.MergeFrom(Vec2d(target_state_local_.path_point.x -
    // nearby_buffer, target_state_local_.path_point.y - nearby_buffer));

    Box2d local_map_box(map_aabox_local);

    Box2d map_box = T_sfl_sc * local_map_box;
    return map_box;
}

bool ParkEnvConstructor::check_parking_slot(const ParkingSlot &parking_slot) const {
    if (parking_slot.park_type() == ParkType::VERTICAL || parking_slot.park_type() == ParkType::OBLIQUE) {
        return parking_slot.length() > parking_slot.width();
    } else if (parking_slot.park_type() == ParkType::PARALLEL) {
        return parking_slot.length() < parking_slot.width();
    }
    return false;
}

PLANNING_NAMESPACE_END