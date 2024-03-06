#ifndef PLANNING_PARKING_SLOT_H
#define PLANNING_PARKING_SLOT_H

#include "pose2d.h"
#include "parking_planning/common/planning_gflags.h"
#include "proto/apa_decision.pb.h"

PLANNING_NAMESPACE_START

using haomo::hidelivery::perception::Limter3D;
using haomo::hidelivery::perception::ParkingSlot3D;
using haomo::hidelivery::perception::ParkingSlotPoint3D;
using haomo::hidelivery::perception::PerceptionView;

class Limiter {
  public:
    explicit Limiter(const Limter3D &limiter_proto);

    explicit Limiter(const Eigen::Vector3d &left = Eigen::Vector3d::Zero(),
                     const Eigen::Vector3d &right = Eigen::Vector3d::Zero(), bool available = false);

    void mirror_injection();

    Vec2d left() const;

    Vec2d right() const;

    Eigen::Vector3d left_3d() const;

    Eigen::Vector3d right_3d() const;

    LineSegment2d line() const;

    std::vector<Vec2d> points() const;

    double max_x() const;

    double min_x() const;

    double max_y() const;

    double min_y() const;

    bool available() const;

    friend Limiter operator*(const Pose2d &pose, const Limiter &limiter_local) {
        if (limiter_local.available()) {
            Eigen::Isometry3d se3 = pose.to_iso3d();
            return se3 * limiter_local;
        }
        return Limiter();
    }

    friend Limiter operator*(const Eigen::Isometry3d &se3, const Limiter &limiter_local) {
        if (limiter_local.available()) {
            Eigen::Vector3d left_global = se3 * limiter_local.left_3d();
            Eigen::Vector3d right_global = se3 * limiter_local.right_3d();

            return Limiter(left_global, right_global, true);
        }
        return Limiter();
    }

  private:
    void init();

    bool available_ = false;
    Eigen::Vector3d left_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d right_ = Eigen::Vector3d::Zero();

    LineSegment2d line_;
    Vec2d left_2d_;
    Vec2d right_2d_;
    double min_x_;
    double max_x_;

    double min_y_;
    double max_y_;
};

class ParkingSlot {
  public:
    DEFINE_SHARDED_PTR(ParkingSlot)

    explicit ParkingSlot(const ParkingSlot3D &slot_proto);

    explicit ParkingSlot(const ParkingSpace &decision_slot_proto);

    explicit ParkingSlot() = default;

    void switch_left_right();

    void mirror_injection();

    void complete_slot();

    static Ptr make(const ParkingSlot3D &slot_proto);

    static Ptr make(const ParkingSpace &decision_slot_proto);

    static Ptr make();

    bool has_limiter() const;

    const Limiter &limiter() const;

    Box2d box() const;

    // rear_left, rear_right, front_right, front left,
    std::vector<Vec2d> points() const;

    LineSegment2d left_edge() const;

    LineSegment2d right_edge() const;

    LineSegment2d front_edge() const;

    LineSegment2d rear_edge() const;

    LineSegment2d center_line() const;

    Vec2d front_left() const;

    Vec2d front_right() const;

    Vec2d rear_left() const;

    Vec2d rear_right() const;

    Vec2d front_mid() const;

    Vec2d rear_mid() const;

    Vec2d center() const;

    double length() const;

    double width() const;

    double heading() const;

    double slot_angle() const;

    Eigen::Vector3d front_left_3d() const;

    Eigen::Vector3d front_right_3d() const;

    Eigen::Vector3d rear_left_3d() const;

    Eigen::Vector3d rear_right_3d() const;

    ParkType park_type() const { return park_type_; }

    Vec2d calculate_c_point(const std::vector<Vec2d> &obstacle_points) const;

    /*
     * @brief if slot in world frame, this function return slot to world transform
     *        (transform slot point into world frame)
     *        else if slot in body frame, this function return slot to body transform
     *        (transform slot point into body frame)
     *        the slot coordinate center is define by SlotCoordinateType
     */
    Eigen::Isometry3d get_slot_transform(SlotCoordinateType type = SlotCoordinateType::Front_Left,
                                         const bool is_right_park = true) const;

    Ptr transform(const Pose2d &pose) const;

    Ptr transform(const Eigen::Isometry3d &se3) const;

    friend ParkingSlot operator*(const Pose2d &pose, const ParkingSlot &slot_local) {
        Eigen::Isometry3d se3 = pose.to_iso3d();
        return se3 * slot_local;
    }

    friend ParkingSlot operator*(const Eigen::Isometry3d &se3, const ParkingSlot &slot_local) {
        ParkingSlot slot_global = slot_local;
        slot_global.front_left_ = se3 * slot_local.front_left_;
        slot_global.front_right_ = se3 * slot_local.front_right_;
        slot_global.rear_left_ = se3 * slot_local.rear_left_;
        slot_global.rear_right_ = se3 * slot_local.rear_right_;
        if (slot_local.has_limiter()) {
            slot_global.limiter_ = se3 * slot_local.limiter_;
        }
        slot_global.init();
        return slot_global;
    }

  private:
    void init();

    void update_info();

    Eigen::Vector3d from_proto(const ParkingSlotPoint3D &pt_proto);

    Eigen::Vector3d front_left_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d front_right_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d rear_left_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d rear_right_ = Eigen::Vector3d::Zero();
    Limiter limiter_;
    ParkType park_type_ = ParkType::UNDEFINE;

    // processed data
    Eigen::Vector3d front_mid_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d rear_mid_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d center_ = Eigen::Vector3d::Zero();

    Vec2d front_left_2d_;
    Vec2d front_right_2d_;
    Vec2d rear_left_2d_;
    Vec2d rear_right_2d_;

    Vec2d front_mid_2d_;
    Vec2d rear_mid_2d_;
    Vec2d center_2d_;

    double length_;
    double width_;
    double heading_;
    double slot_angle_;

    LineSegment2d front_edge_;
    LineSegment2d rear_edge_;
    LineSegment2d left_edge_;
    LineSegment2d right_edge_;
    LineSegment2d center_line_;
    Box2d box_;

    bool is_init_ = false;
};

class ParkingSlotManager {
  public:
    explicit ParkingSlotManager();

    ParkingSlot::Ptr get_slot(CoordinateType coord = CoordinateType::Slot) const;

    bool update(const PerceptionView &perception_view, const Eigen::Isometry3d &T_world_body,
                SlotCoordinateType slot_coord_type = SlotCoordinateType::Front_Left);

    bool update_for_out(const PerceptionView &perception_view, const Eigen::Isometry3d &T_world_body,
                        SlotCoordinateType slot_coord_type = SlotCoordinateType::Front_Left);

    void create_park_slot_by_vehicle(const PerceptionView &perception_view, const Eigen::Isometry3d &T_world_body,
                                     ParkingSlot3D &parking_space);

    void create_park_slot_by_perception(const PerceptionView &perception_view, const Eigen::Isometry3d &T_world_body,
                                        ParkingSlot3D &parking_space);

    void reset();

    void lock_right_park_status_at_init();

    void unlock_right_park_status_at_init();

    bool right_park() const;

    bool valid() const;

    Eigen::Isometry3d get_slot_world_transform() const;

    Eigen::Isometry3d get_slot_body_transform() const;

  private:
    void update_right_park_status();

    ParkingSlot::Ptr slot_world_ = nullptr;
    ParkingSlot::Ptr slot_body_ = nullptr;
    ParkingSlot::Ptr slot_sl_ = nullptr;

    bool is_right_park_ = true;
    bool is_valid_ = false;
    bool lock_right_park_status_ = true;
    bool is_first_out_ = true;

    Eigen::Isometry3d T_body_slot_;
    Eigen::Isometry3d T_world_slot_;
};

PLANNING_NAMESPACE_END

#endif // PLANNING_PARKING_SLOT_H
