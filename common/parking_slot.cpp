#include "parking_slot.h"
#include "parking_info.h"
#include "vehicle_model.h"

PLANNING_NAMESPACE_START

using haomo::hidelivery::perception::ParkingOutDirection;
using haomo::hidelivery::perception::ParkingSlotType;

Limiter::Limiter(const Limter3D &limiter_proto) {
    if (limiter_proto.available()) {
        left_ = Eigen::Vector3d(limiter_proto.left().x(), limiter_proto.left().y(), limiter_proto.left().z());
        right_ = Eigen::Vector3d(limiter_proto.right().x(), limiter_proto.right().y(), limiter_proto.right().z());
        available_ = true;
        init();
    }
}

void Limiter::mirror_injection() {
    left_.x() = -left_.x();
    right_.x() = -right_.x();
    init();
}

Limiter::Limiter(const Eigen::Vector3d &left, const Eigen::Vector3d &right, bool available) {
    left_ = left;
    right_ = right;
    available_ = available;
    init();
}

void Limiter::init() {
    left_2d_ = Vec2d(left_.x(), left_.y());
    right_2d_ = Vec2d(right_.x(), right_.y());
    line_ = LineSegment2d(left_2d_, right_2d_);
    min_x_ = std::min(left_2d_.x(), right_2d_.x());
    max_x_ = std::max(left_2d_.x(), right_2d_.x());
    min_y_ = std::min(left_2d_.y(), right_2d_.y());
    max_y_ = std::max(left_2d_.y(), right_2d_.y());
}

Vec2d Limiter::left() const { return left_2d_; }

Vec2d Limiter::right() const { return right_2d_; }

Eigen::Vector3d Limiter::left_3d() const { return left_; }

Eigen::Vector3d Limiter::right_3d() const { return right_; }

LineSegment2d Limiter::line() const { return line_; }

std::vector<Vec2d> Limiter::points() const { return std::vector<Vec2d>{line_.start(), line_.end()}; }

double Limiter::max_x() const { return max_x_; }

double Limiter::min_x() const { return min_x_; }

double Limiter::max_y() const { return max_y_; }

double Limiter::min_y() const { return min_y_; }

bool Limiter::available() const { return available_; }

ParkingSlot::ParkingSlot(const ParkingSlot3D &slot_proto) {
    front_left_ = from_proto(slot_proto.front_left());
    front_right_ = from_proto(slot_proto.front_right());
    rear_left_ = from_proto(slot_proto.rear_left());
    rear_right_ = from_proto(slot_proto.rear_right());
    if (slot_proto.type() == ParkingSlotType::PERPENDICULAR) {
        park_type_ = VERTICAL;
    } else if (slot_proto.type() == ParkingSlotType::PARALLEL) {
        park_type_ = PARALLEL;
    } else if (slot_proto.type() == ParkingSlotType::SLANTED) {
        park_type_ = OBLIQUE;
    }

    if (slot_proto.has_limter()) {
        limiter_ = Limiter(slot_proto.limter());
    } else {
        MLOG(PARKING_PLANNING, INFO) << "no limter!";
    }

    init();
}

ParkingSlot::ParkingSlot(const ParkingSpace &decision_slot_proto) {
    front_left_ = Eigen::Vector3d(decision_slot_proto.front_left().x(), decision_slot_proto.front_left().y(), 0.0);
    front_right_ = Eigen::Vector3d(decision_slot_proto.front_right().x(), decision_slot_proto.front_right().y(), 0.0);
    rear_left_ = Eigen::Vector3d(decision_slot_proto.rear_left().x(), decision_slot_proto.rear_left().y(), 0.0);
    rear_right_ = Eigen::Vector3d(decision_slot_proto.rear_right().x(), decision_slot_proto.rear_right().y(), 0.0);
    if (decision_slot_proto.slot_type() == ParkingSpace::VERTICAL) {
        park_type_ = VERTICAL;
    } else if (decision_slot_proto.slot_type() == ParkingSpace::PARALLEL) {
        park_type_ = PARALLEL;
    } else if (decision_slot_proto.slot_type() == ParkingSpace::OBLIQUE) {
        park_type_ = OBLIQUE;
    }

    if (decision_slot_proto.is_with_stopper()) {
        Eigen::Vector3d left(decision_slot_proto.stopper().left().x(), decision_slot_proto.stopper().left().y(), 0.0);
        Eigen::Vector3d right(decision_slot_proto.stopper().right().x(), decision_slot_proto.stopper().right().y(),
                              0.0);
        limiter_ = Limiter(left, right, true);
    } else {
        MLOG(PARKING_PLANNING, INFO) << "no limter!";
    }
    init();
}

ParkingSlot::Ptr ParkingSlot::make(const ParkingSlot3D &slot_proto) {
    auto ptr = std::make_shared<ParkingSlot>(slot_proto);

    return ptr;
}

ParkingSlot::Ptr ParkingSlot::make(const ParkingSpace &decision_slot_proto) {
    auto ptr = std::make_shared<ParkingSlot>(decision_slot_proto);

    return ptr;
}

ParkingSlot::Ptr ParkingSlot::make() {
    auto ptr = std::make_shared<ParkingSlot>();

    return ptr;
}
bool ParkingSlot::has_limiter() const { return limiter_.available(); }
const Limiter &ParkingSlot::limiter() const { return limiter_; }

Box2d ParkingSlot::box() const { return box_; }

std::vector<Vec2d> ParkingSlot::points() const { return {rear_left(), rear_right(), front_right(), front_left()}; }

LineSegment2d ParkingSlot::left_edge() const { return left_edge_; }

LineSegment2d ParkingSlot::right_edge() const { return right_edge_; }

LineSegment2d ParkingSlot::front_edge() const { return front_edge_; }

LineSegment2d ParkingSlot::rear_edge() const { return rear_edge_; }

LineSegment2d ParkingSlot::center_line() const { return center_line_; }

Vec2d ParkingSlot::front_left() const { return front_left_2d_; }

Vec2d ParkingSlot::front_right() const { return front_right_2d_; }

Vec2d ParkingSlot::rear_left() const { return rear_left_2d_; }

Vec2d ParkingSlot::rear_right() const { return rear_right_2d_; }

Vec2d ParkingSlot::front_mid() const { return front_mid_2d_; }

Vec2d ParkingSlot::rear_mid() const { return rear_mid_2d_; }

Vec2d ParkingSlot::center() const { return center_2d_; }

double ParkingSlot::length() const { return length_; }

double ParkingSlot::width() const { return width_; }

double ParkingSlot::heading() const { return heading_; }

double ParkingSlot::slot_angle() const { return slot_angle_; }

Eigen::Vector3d ParkingSlot::front_left_3d() const { return front_left_; }

Eigen::Vector3d ParkingSlot::front_right_3d() const { return front_right_; }

Eigen::Vector3d ParkingSlot::rear_left_3d() const { return rear_left_; }

Eigen::Vector3d ParkingSlot::rear_right_3d() const { return rear_right_; }

Eigen::Isometry3d ParkingSlot::get_slot_transform(SlotCoordinateType type, const bool is_right_park) const {
    //这个函数主要用于将车身系的车位转换到车位系，默认用车位左上角点作为车位系原点。
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    switch (type) {
    case SlotCoordinateType::Front_Left: {
        transform.translation() = front_left_;
        break;
    }

    case SlotCoordinateType::Front_Center: {
        transform.translation() = front_mid_;
        break;
    }

    case SlotCoordinateType::Rear_Center: {
        transform.translation() = rear_mid_;
        break;
    }

    case SlotCoordinateType::Rear_Left: {
        transform.translation() = rear_left_;
        break;
    }

    case SlotCoordinateType::Slot_Center: {
        transform.translation() = center_;
        break;
    }
    default: {
    }
    }
    MLOG(PARKING_PLANNING, INFO) << " heading() " << heading();
    MLOG(PARKING_PLANNING, INFO) << " slot_angle() " << slot_angle();
    if (park_type_ == ParkType::OBLIQUE) {
        double slot_angle;
        MLOG(PARKING_PLANNING, INFO) << " is_right_park " << is_right_park;
        if (is_right_park) {
            slot_angle = std::atan2(front_right_.y() - front_left_.y(), front_right_.x() - front_left_.x());
        } else {
            //因为如果是左车位，会交换车位左右角点
            slot_angle = std::atan2(front_left_.y() - front_right_.y(), front_left_.x() - front_right_.x());
        }
        MLOG(PARKING_PLANNING, INFO) << "slot_angle " << slot_angle;
        transform.linear() = Eigen::AngleAxisd(slot_angle, Eigen::Vector3d::UnitZ()).matrix();
    } else {
        transform.linear() = Eigen::AngleAxisd(heading() - M_PI_2, Eigen::Vector3d::UnitZ()).matrix();
    }

    return transform;
}

void ParkingSlot::init() {
    front_mid_ = 0.5 * (front_left_ + front_right_);
    rear_mid_ = 0.5 * (rear_left_ + rear_right_);
    center_ = 0.5 * (front_mid_ + rear_mid_);

    front_left_2d_ = Vec2d(front_left_.x(), front_left_.y());
    front_right_2d_ = Vec2d(front_right_.x(), front_right_.y());
    rear_left_2d_ = Vec2d(rear_left_.x(), rear_left_.y());
    rear_right_2d_ = Vec2d(rear_right_.x(), rear_right_.y());

    update_info();
    is_init_ = true;
}

void ParkingSlot::complete_slot() {
    if (park_type_ == ParkType::OBLIQUE) {
        front_left_2d_ = Vec2d(front_left_.x(), front_left_.y());
        front_right_2d_ = Vec2d(front_right_.x(), front_right_.y());
        rear_left_2d_ = Vec2d(rear_left_.x(), rear_left_.y());
        rear_right_2d_ = Vec2d(rear_right_.x(), rear_right_.y());
    } else {
        double park_A_y = length_;
        double park_B_x = width_ * sin(slot_angle_);

        rear_left_2d_ = Vec2d(0.0, -park_A_y);

        rear_right_2d_ = Vec2d(park_B_x, -park_A_y);

        double park_D_y = width_ * cos(slot_angle_);
        front_right_2d_ = Vec2d(park_B_x, park_D_y);
        front_left_2d_ = Vec2d(0.0, park_D_y);
    }

    update_info();
    is_init_ = true;
}
void ParkingSlot::switch_left_right() {
    Eigen::Vector3d tmp = front_right_;
    front_right_ = front_left_;
    front_left_ = tmp;

    tmp = rear_right_;
    rear_right_ = rear_left_;
    rear_left_ = tmp;
    if (!is_init_) {
        init();
    } else {
        auto tmp = front_right_2d_;
        front_right_2d_ = front_left_2d_;
        front_left_2d_ = tmp;

        tmp = rear_right_2d_;
        rear_right_2d_ = rear_left_2d_;
        rear_left_2d_ = tmp;
        update_info();
    }
}
void ParkingSlot::mirror_injection() {
    front_left_.x() = -front_left_.x();
    front_right_.x() = -front_right_.x();
    rear_left_.x() = -rear_left_.x();
    rear_right_.x() = -rear_right_.x();
    limiter_.mirror_injection();
    if (!is_init_) {
        init();
    } else {
        front_mid_ = 0.5 * (front_left_ + front_right_);
        rear_mid_ = 0.5 * (rear_left_ + rear_right_);
        center_ = 0.5 * (front_mid_ + rear_mid_);

        front_left_2d_ = Vec2d(-front_left_2d_.x(), front_left_2d_.y());
        front_right_2d_ = Vec2d(-front_right_2d_.x(), front_right_2d_.y());
        rear_left_2d_ = Vec2d(-rear_left_2d_.x(), rear_left_2d_.y());
        rear_right_2d_ = Vec2d(-rear_right_2d_.x(), rear_right_2d_.y());
        update_info();
    }
}

void ParkingSlot::update_info() {
    front_mid_2d_ = 0.5 * (front_left_2d_ + front_right_2d_);
    rear_mid_2d_ = 0.5 * (rear_left_2d_ + rear_right_2d_);
    center_2d_ = 0.5 * (front_mid_2d_ + rear_mid_2d_);

    front_edge_ = LineSegment2d(front_left_2d_, front_right_2d_);
    rear_edge_ = LineSegment2d(rear_left_2d_, rear_right_2d_);
    left_edge_ = LineSegment2d(rear_left_2d_, front_left_2d_);
    right_edge_ = LineSegment2d(rear_right_2d_, front_right_2d_);

    center_line_ = LineSegment2d(rear_mid_2d_, front_mid_2d_);

    length_ = front_left_2d_.DistanceTo(rear_left_2d_);
    width_ = front_left_2d_.DistanceTo(front_right_2d_);
    heading_ = Rot2::normalize_angle(std::atan2(-rear_mid_.y() + front_mid_.y(), -rear_mid_.x() + front_mid_.x()));
    double theta_AD = atan2((front_left_.y() - rear_left_.y()), (front_left_.x() - rear_left_.x()));
    double theta_DC = atan2((front_right_.y() - front_left_.y()), (front_right_.x() - front_left_.x()));
    slot_angle_ = Rot2::normalize_angle(theta_AD - theta_DC);
    if (park_type_ == ParkType::PARALLEL) {
        // todo
    }

    box_ = Box2d(center_2d_, heading_, length_, width_ * std::sin(slot_angle_));
}

Eigen::Vector3d ParkingSlot::from_proto(const ParkingSlotPoint3D &pt_proto) {
    return Eigen::Vector3d(pt_proto.point().x(), pt_proto.point().y(), pt_proto.point().z());
}

ParkingSlot::Ptr ParkingSlot::transform(const Pose2d &pose) const {
    Eigen::Isometry3d se3 = pose.to_iso3d();
    return transform(se3);
}

ParkingSlot::Ptr ParkingSlot::transform(const Eigen::Isometry3d &se3) const {
    auto ptr = make();
    ptr->front_left_ = se3 * front_left_;
    ptr->front_right_ = se3 * front_right_;
    ptr->rear_left_ = se3 * rear_left_;
    ptr->rear_right_ = se3 * rear_right_;
    if (has_limiter()) {
        ptr->limiter_ = se3 * limiter_;
    }
    if (!is_init_) {
        ptr->init();
    } else {
        ptr->front_mid_ = 0.5 * (ptr->front_left_ + ptr->front_right_);
        ptr->rear_mid_ = 0.5 * (ptr->rear_left_ + ptr->rear_right_);
        ptr->center_ = 0.5 * (ptr->front_mid_ + ptr->rear_mid_);
        ptr->front_left_2d_ = Vec2d(ptr->front_left_.x(), ptr->front_left_.y());
        ptr->front_right_2d_ = Vec2d(ptr->front_right_.x(), ptr->front_right_.y());
        ptr->rear_left_2d_ = Vec2d(ptr->rear_left_.x(), ptr->rear_left_.y());
        ptr->rear_right_2d_ = Vec2d(ptr->rear_right_.x(), ptr->rear_right_.y());
        ptr->update_info();
    }
    ptr->park_type_ = this->park_type_;
    return ptr;
}

Vec2d ParkingSlot::calculate_c_point(const std::vector<Vec2d> &obstacle_points) const {
    double parkC_corner_distance = FLAGS_apa_vert_corner_distance;
    double park_width = rear_right().x() - rear_left().x();
    for (auto obstacle_point : obstacle_points) {
        // 障碍物坐标和C角点的相对距离
        double delta_x = obstacle_point.x() - rear_right().x();
        double delta_y = obstacle_point.y() - front_right().y();
        if (delta_x < 1.0 && //@chenghe tmp change
            delta_x > 0 && obstacle_point.y() > rear_left().y() &&
            obstacle_point.y() < front_right().y() + FLAGS_apa_vert_wall_case_consider_dis_top_y) {
            parkC_corner_distance = std::max(FLAGS_apa_vert_park_c_corner_buffer + delta_y, parkC_corner_distance);
        }
    }
    Vec2d c_point = front_right();
    if (parkC_corner_distance > FLAGS_apa_vert_corner_distance) {
        c_point.set_y(front_right().y() + parkC_corner_distance);
    } else {
        c_point.set_x(park_width / 2 + (VehicleConfigHelper::GetConfig().vehicle_param().overallwidth()) / 2 +
                      FLAGS_apa_vert_rotate_down_target_x);
        c_point.set_y(front_right().y() - (VehicleConfigHelper::GetConfig().vehicle_param().front_edge_to_center()) +
                      FLAGS_apa_vert_rotate_down_target_y);
    }
    return c_point;
}

void ParkingSlotManager::lock_right_park_status_at_init() { lock_right_park_status_ = true; }

void ParkingSlotManager::unlock_right_park_status_at_init() { lock_right_park_status_ = false; }

ParkingSlotManager::ParkingSlotManager() { reset(); }

void ParkingSlotManager::reset() {
    slot_world_ = ParkingSlot::make();
    slot_body_ = ParkingSlot::make();
    slot_sl_ = ParkingSlot::make();

    is_right_park_ = true;
    is_valid_ = false;
}

bool ParkingSlotManager::right_park() const { return is_right_park_; }

bool ParkingSlotManager::valid() const { return is_valid_; }

ParkingSlot::Ptr ParkingSlotManager::get_slot(CoordinateType coord) const {
    switch (coord) {
    case CoordinateType::World: {
        return slot_world_;
    }

    case CoordinateType::Body: {
        return slot_body_;
    }

    case CoordinateType::Slot: {
        return slot_sl_;
    }

    default: {
        return nullptr;
    }
    }
}

Eigen::Isometry3d ParkingSlotManager::get_slot_world_transform() const { return T_world_slot_; }

Eigen::Isometry3d ParkingSlotManager::get_slot_body_transform() const { return T_body_slot_; }

bool ParkingSlotManager::update(const PerceptionView &perception_view, const Eigen::Isometry3d &T_world_body,
                                SlotCoordinateType slot_coord_type) {
    if (!perception_view.has_parking_slots() || perception_view.parking_slots().global_parking_slots_size() < 1) {
        MLOG(PARKING_PLANNING, ERROR) << "no slot!";
        is_valid_ = false;
        return false;
    }
    Eigen::Isometry3d T_body_world = T_world_body.inverse();
    slot_world_ = ParkingSlot::make(perception_view.parking_slots().global_parking_slots(0));
    slot_body_ = slot_world_->transform(T_body_world);
    //    Pose2d start_pose = Pose2d::from_iso3d(T_body_slot_.inverse());
    //    Vec2d init_vec(start_pose.rotation().cos_theta(),
    //                   start_pose.rotation().sin_theta());
    //    Eigen::Vector3d rear_mid_slot = T_body_slot_.inverse() * (0.5 * slot_body_->rear_right_3d() + 0.5 *
    //    slot_body_->rear_right_3d()); Vec2d i2t_vec(rear_mid_slot.x() - start_pose.x(),
    //                  rear_mid_slot.y() - start_pose.y());
    //    if(init_vec.CrossProd(i2t_vec) > 0)
    //    {
    //        is_right_park_ = false;
    //    }
    if (!is_valid_ || !lock_right_park_status_) {
        update_right_park_status();
    }
    if (!is_right_park_) {
        slot_body_->switch_left_right();
    }
    T_body_slot_ = slot_body_->get_slot_transform(slot_coord_type, is_right_park_);

    Eigen::Isometry3d T_slot_body = T_body_slot_.inverse();
    slot_sl_ = slot_body_->transform(T_slot_body);
    if (!is_right_park_) {
        slot_sl_->mirror_injection();
    }
    slot_sl_->complete_slot();

    T_world_slot_ = T_world_body * T_body_slot_;
    is_valid_ = true;

    return true;
}

void ParkingSlotManager::update_right_park_status() {
    // todo 泊入过程中左库右库来回跳
    double body_angle = (slot_body_->slot_angle() - M_PI / 2) - slot_body_->heading();
    MLOG(PARKING_PLANNING, INFO) << "body_angle: " << body_angle;
    if (body_angle > 0) {
        is_right_park_ = false;
    } else {
        is_right_park_ = true;
    }
    MLOG(PARKING_PLANNING, INFO) << "is_right_park_: " << is_right_park_;
}

bool ParkingSlotManager::update_for_out(const PerceptionView &perception_view, const Eigen::Isometry3d &T_world_body,
                                        SlotCoordinateType slot_coord_type) {
    if (!is_first_out_) {
        return true;
    }
    Eigen::Isometry3d T_body_world = T_world_body.inverse();
    ParkingSlot3D parking_space;
    if (perception_view.has_parking_slots() && perception_view.parking_slots().global_parking_slots_size() >= 1) {
        create_park_slot_by_perception(perception_view, T_world_body, parking_space);
    } else {
        MLOG(PARKING_PLANNING, INFO) << "create_park_slot_by_vehicle";
        create_park_slot_by_vehicle(perception_view, T_world_body, parking_space);
    }
    slot_world_ = ParkingSlot::make(parking_space);
    slot_body_ = slot_world_->transform(T_body_world);
    if (!is_valid_ || !lock_right_park_status_) {
        update_right_park_status();
    }
    if (!is_right_park_) {
        slot_body_->switch_left_right();
    }
    T_body_slot_ = slot_body_->get_slot_transform(slot_coord_type);
    Eigen::Isometry3d T_slot_body = T_body_slot_.inverse();
    slot_sl_ = slot_body_->transform(T_slot_body);
    if (!is_right_park_) {
        slot_sl_->mirror_injection();
    }
    slot_sl_->complete_slot();

    T_world_slot_ = T_world_body * T_body_slot_;
    is_valid_ = true;
    is_first_out_ = false;

    return true;
}

void ParkingSlotManager::create_park_slot_by_vehicle(const PerceptionView &perception_view,
                                                     const Eigen::Isometry3d &T_world_body,
                                                     ParkingSlot3D &parking_space) {
    VehicleModel vehicle_model = VehicleModel(0.0, 0.1, 0.0, 0.0, 0.0, 0.0);
    Pose2d world_start_pose = Pose2d::from_iso3d(T_world_body);
    vehicle_model.update_pose(world_start_pose);
    ParkingSlotPoint3D front_left, front_right, rear_left, rear_right;
    if (perception_view.park_out_direction().selected() == ParkingOutDirection::PARALLEL_LEFT) {
        front_left.mutable_point()->set_x(vehicle_model.corners()[0].x());
        front_left.mutable_point()->set_y(vehicle_model.corners()[0].y());
        front_left.mutable_point()->set_z(T_world_body.translation().z());

        rear_left.mutable_point()->set_x(vehicle_model.corners()[1].x());
        rear_left.mutable_point()->set_y(vehicle_model.corners()[1].y());
        rear_left.mutable_point()->set_z(T_world_body.translation().z());

        rear_right.mutable_point()->set_x(vehicle_model.corners()[2].x());
        rear_right.mutable_point()->set_y(vehicle_model.corners()[2].y());
        rear_right.mutable_point()->set_z(T_world_body.translation().z());

        front_right.mutable_point()->set_x(vehicle_model.corners()[3].x());
        front_right.mutable_point()->set_y(vehicle_model.corners()[3].y());
        front_right.mutable_point()->set_z(T_world_body.translation().z());

        parking_space.set_type(ParkingSlotType::PARALLEL);
    } else if (perception_view.park_out_direction().selected() == ParkingOutDirection::PARALLEL_RIGHT) {
        front_left.mutable_point()->set_x(vehicle_model.corners()[2].x());
        front_left.mutable_point()->set_y(vehicle_model.corners()[2].y());
        front_left.mutable_point()->set_z(T_world_body.translation().z());

        rear_left.mutable_point()->set_x(vehicle_model.corners()[3].x());
        rear_left.mutable_point()->set_y(vehicle_model.corners()[3].y());
        rear_left.mutable_point()->set_z(T_world_body.translation().z());

        rear_right.mutable_point()->set_x(vehicle_model.corners()[0].x());
        rear_right.mutable_point()->set_y(vehicle_model.corners()[0].y());
        rear_right.mutable_point()->set_z(T_world_body.translation().z());

        front_right.mutable_point()->set_x(vehicle_model.corners()[1].x());
        front_right.mutable_point()->set_y(vehicle_model.corners()[1].y());
        front_right.mutable_point()->set_z(T_world_body.translation().z());

        parking_space.set_type(ParkingSlotType::PARALLEL);
    } else {
        const auto &vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
        front_left.mutable_point()->set_x(0.0);
        front_left.mutable_point()->set_y(0.0);
        front_left.mutable_point()->set_z(0.0);
        front_right.mutable_point()->set_x(vehicle_param.overallwidth());
        front_right.mutable_point()->set_y(0.0);
        front_right.mutable_point()->set_z(0.0);
        rear_left.mutable_point()->set_x(0.0);
        rear_left.mutable_point()->set_y(-vehicle_param.overalllength());
        rear_left.mutable_point()->set_z(0.0);
        rear_right.mutable_point()->set_x(vehicle_param.overallwidth());
        rear_right.mutable_point()->set_y(-vehicle_param.overalllength());
        rear_right.mutable_point()->set_z(0.0);
        parking_space.set_type(ParkingSlotType::PERPENDICULAR);
    }
    parking_space.mutable_front_left()->CopyFrom(front_left);
    parking_space.mutable_front_right()->CopyFrom(front_right);
    parking_space.mutable_rear_left()->CopyFrom(rear_left);
    parking_space.mutable_rear_right()->CopyFrom(rear_right);
}

void ParkingSlotManager::create_park_slot_by_perception(const PerceptionView &perception_view,
                                                        const Eigen::Isometry3d &T_world_body,
                                                        ParkingSlot3D &parking_space) {
    ParkingSlot3D real_park_space = perception_view.parking_slots().global_parking_slots(0);
    ParkingSlot::Ptr real_slot_world = ParkingSlot::make(real_park_space);
    Eigen::Isometry3d T_body_world = T_world_body.inverse();
    ParkingSlot::Ptr slot_body = real_slot_world->transform(T_body_world);
    auto angle = std::abs(std::atan((slot_body->front_left().y() - slot_body->rear_left().y()) /
                                    (slot_body->front_left().x() - slot_body->rear_left().x())));
    MLOG(PARKING_PLANNING, INFO) << "angle: " << angle;
    bool is_right_park = true;
    ParkingSlotPoint3D front_left, front_right, rear_left, rear_right;
    if (angle < 0.15 && slot_body->front_left().x() < 3.0) {
        // 车在库内的时候，如果更新状态就设置成右车位
        // 当前左右车位是固定的，这里处理主要是为了重规划的测试集
        is_right_park = true;
    } else {
        if (slot_body->rear_left().y() > slot_body->front_left().y()) { // left park, A.y>D.y
            is_right_park = false;
        } else if (slot_body->rear_left().y() < slot_body->front_left().y()) {
            is_right_park = true;
        } else {
            if (slot_body->rear_left().y() > 0) {
                is_right_park = false;
            } else {
                is_right_park = true;
            }
        }
    }
    MLOG(PARKING_PLANNING, INFO) << "test is_right_park: " << is_right_park;
    if ((is_right_park && perception_view.park_out_direction().selected() == ParkingOutDirection::PARALLEL_LEFT) ||
        (!is_right_park && perception_view.park_out_direction().selected() == ParkingOutDirection::PARALLEL_RIGHT)) {
        MLOG(PARKING_PLANNING, INFO) << "park space right";
        front_left.mutable_point()->set_x(real_park_space.front_left().point().x());
        front_left.mutable_point()->set_y(real_park_space.front_left().point().y());
        front_left.mutable_point()->set_z(real_park_space.front_left().point().z());

        rear_left.mutable_point()->set_x(real_park_space.rear_left().point().x());
        rear_left.mutable_point()->set_y(real_park_space.rear_left().point().y());
        rear_left.mutable_point()->set_z(real_park_space.rear_left().point().z());

        rear_right.mutable_point()->set_x(real_park_space.rear_right().point().x());
        rear_right.mutable_point()->set_y(real_park_space.rear_right().point().y());
        rear_right.mutable_point()->set_z(real_park_space.rear_right().point().z());

        front_right.mutable_point()->set_x(real_park_space.front_right().point().x());
        front_right.mutable_point()->set_y(real_park_space.front_right().point().y());
        front_right.mutable_point()->set_z(real_park_space.front_right().point().z());
    } else {
        front_left.mutable_point()->set_x(real_park_space.rear_right().point().x());
        front_left.mutable_point()->set_y(real_park_space.rear_right().point().y());
        front_left.mutable_point()->set_z(real_park_space.rear_right().point().z());

        rear_left.mutable_point()->set_x(real_park_space.front_right().point().x());
        rear_left.mutable_point()->set_y(real_park_space.front_right().point().y());
        rear_left.mutable_point()->set_z(real_park_space.front_right().point().z());

        rear_right.mutable_point()->set_x(real_park_space.front_left().point().x());
        rear_right.mutable_point()->set_y(real_park_space.front_left().point().y());
        rear_right.mutable_point()->set_z(real_park_space.front_left().point().z());

        front_right.mutable_point()->set_x(real_park_space.rear_left().point().x());
        front_right.mutable_point()->set_y(real_park_space.rear_left().point().y());
        front_right.mutable_point()->set_z(real_park_space.rear_left().point().z());
    }

    if (real_park_space.type() == ParkingSlotType::PARALLEL) {
        parking_space.set_type(ParkingSlotType::PARALLEL);
    } else {
        parking_space.set_type(ParkingSlotType::PERPENDICULAR);
    }
    parking_space.mutable_front_left()->CopyFrom(front_left);
    parking_space.mutable_front_right()->CopyFrom(front_right);
    parking_space.mutable_rear_left()->CopyFrom(rear_left);
    parking_space.mutable_rear_right()->CopyFrom(rear_right);
}

PLANNING_NAMESPACE_END