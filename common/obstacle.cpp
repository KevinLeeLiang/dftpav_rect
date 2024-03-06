#include "obstacle.h"
#include "parking_info.h"
#include "vehicle_model.h"
#include "parking_planning/apa/common/math_helper.h"

PLANNING_NAMESPACE_START

ObstacleBox::Ptr ObstacleBox::make(const haomo::hidelivery::perception::Obstacle &obstacle, double flat_thresh) {
    return std::make_shared<ObstacleBox>(obstacle, flat_thresh);
}

ObstacleBox::Ptr ObstacleBox::make() { return std::make_shared<ObstacleBox>(); }

void ObstacleBox::mirror_injection() {
    center_.x() = -center_.x();
    theta_ = M_PI - theta_;
    init();
}

ObstacleBox::ObstacleBox(const haomo::hidelivery::perception::Obstacle &obstacle, double flat_thresh) {
    // todo use odom
    center_ = Eigen::Vector3d(obstacle.obstacle_pos().x(), obstacle.obstacle_pos().y(), obstacle.obstacle_pos().z());

    size_ = Eigen::Vector3d(obstacle.obstacle_geo().x() + 2 * flat_thresh,
                            obstacle.obstacle_geo().y() + 2 * flat_thresh, obstacle.obstacle_geo().z());

    theta_ = obstacle.theta();

    init();
}

void ObstacleBox::init() {
    corners_.clear();
    if (FLAGS_apa_obstacle_cut_angle) {
        const double x_cut = FLAGS_apa_obstacle_box_x_cut;
        const double y_cut = FLAGS_apa_obstacle_box_y_cut;
        corners_.resize(6);
        Eigen::Matrix3d rot = Eigen::AngleAxisd(theta_, Eigen::Vector3d::UnitZ()).matrix();
        corners_[0] = center_ + rot * Eigen::Vector3d(+0.5 * size_.x() - x_cut, -0.5 * size_.y(), 0);
        corners_[1] = center_ + rot * Eigen::Vector3d(+0.5 * size_.x(), -(0.5 * size_.y() - y_cut), 0);
        corners_[2] = center_ + rot * Eigen::Vector3d(+0.5 * size_.x(), +(0.5 * size_.y() - y_cut), 0);
        corners_[3] = center_ + rot * Eigen::Vector3d(+0.5 * size_.x() - x_cut, +0.5 * size_.y(), 0);
        corners_[4] = center_ + 0.5 * rot * Eigen::Vector3d(-size_.x(), +size_.y(), 0);
        corners_[5] = center_ + 0.5 * rot * Eigen::Vector3d(-size_.x(), -size_.y(), 0);
    } else {
        corners_.resize(4);
        Eigen::Matrix3d rot = Eigen::AngleAxisd(theta_, Eigen::Vector3d::UnitZ()).matrix();
        corners_[0] = center_ + 0.5 * rot * Eigen::Vector3d(+size_.x(), -size_.y(), 0);
        corners_[1] = center_ + 0.5 * rot * Eigen::Vector3d(+size_.x(), +size_.y(), 0);
        corners_[2] = center_ + 0.5 * rot * Eigen::Vector3d(-size_.x(), +size_.y(), 0);
        corners_[3] = center_ + 0.5 * rot * Eigen::Vector3d(-size_.x(), -size_.y(), 0);
    }

    corners_2d_.clear();
    for (const auto &corner : corners_) {
        corners_2d_.push_back(Vec2d(corner.x(), corner.y()));
    }
    edges_.clear();
    for (size_t i = 0; i < corners_2d_.size(); ++i) {
        edges_.emplace_back(corners_2d_.at(i), corners_2d_.at((i + 1) % (corners_2d_.size())));
    }
    box_ = Box2d(Vec2d(center_.x(), center_.y()), theta_, size_.x(), size_.y());
}

const std::vector<Vec2d> &ObstacleBox::corner() const { return corners_2d_; }

const std::vector<LineSegment2d> &ObstacleBox::edges() const { return edges_; }

Box2d ObstacleBox::get_box() const { return box_; }

ObstacleBox::Ptr ObstacleBox::transform(const Pose2d &pose) const {
    Eigen::Isometry3d se3 = pose.to_iso3d();
    return transform(se3);
}

ObstacleBox::Ptr ObstacleBox::transform(const Eigen::Isometry3d &se3) const {
    auto ptr = make();
    ptr->center_ = se3 * center_;
    ptr->size_ = size_;

    Eigen::Matrix3d new_rotation = se3.linear() * Eigen::AngleAxisd(theta_, Eigen::Vector3d::UnitZ()).matrix();
    ptr->theta_ = Rot2::extract_yaw_from_rotation_matrix(new_rotation);
    ptr->init();

    return ptr;
}

ObstacleLine::ObstacleLine(const ObstacleLine3D &odl_3d) {
    points_origin_.reserve(odl_3d.world_points_size());
    for (int i = 0; i < odl_3d.world_points_size(); ++i) {
        points_origin_.push_back(Eigen::Vector3d(odl_3d.world_points(i).point().x(), odl_3d.world_points(i).point().y(),
                                                 odl_3d.world_points(i).point().z()));
    }
    type_ = (enum ObstacleLineTypeEnum)(static_cast<int>(odl_3d.type()));
    func_ = odl_3d.function_type();
}

ObstacleLine::Ptr ObstacleLine::make(const ObstacleLine3D &odl_3d) {
    auto ptr = std::make_shared<ObstacleLine>(odl_3d);
    return ptr;
}

ObstacleLine::Ptr ObstacleLine::make() {
    auto ptr = std::make_shared<ObstacleLine>();
    return ptr;
}

void ObstacleLine::mirror_injection() {
    for (size_t i = 0; i < points_origin_.size(); ++i) {
        points_origin_[i].x() = -points_origin_[i].x();
    }
}

void ObstacleLine::init(double interpolate_thresh) {
    if (interpolate_thresh < 0.0) {
        points_2d_interpolated_.clear();
        for (size_t i = 0; i < points_origin_.size(); ++i) {
            points_2d_interpolated_.push_back(Vec2d(points_origin_[i].x(), points_origin_[i].y()));
        }
        is_init_ = true;
        return;
    }
    points_2d_interpolated_.clear();
    for (size_t i = 0; i < (points_origin_.size() - 1); ++i) {
        // interpolate point todo
        Eigen::Vector3d dir = points_origin_[i + 1] - points_origin_[i];
        double distance = dir.norm();

        if (distance > interpolate_thresh) {
            dir.normalize();
            int size = static_cast<int>(std::floor(distance / interpolate_thresh));
            double step = distance / size;
            for (int j = 0; j < size; ++j) {
                Eigen::Vector3d pt = points_origin_[i] + j * step * dir;
                points_2d_interpolated_.push_back(Vec2d(pt.x(), pt.y()));
            }
        } else {
            points_2d_interpolated_.push_back(Vec2d(points_origin_[i].x(), points_origin_[i].y()));
        }
    }
    points_2d_interpolated_.push_back(Vec2d(points_origin_.back().x(), points_origin_.back().y()));
    is_init_ = true;
}

const std::vector<Vec2d> &ObstacleLine::points() const { return points_2d_interpolated_; }

const std::vector<Eigen::Vector3d> &ObstacleLine::origin_points() const { return points_origin_; }

bool ObstacleLine::inited() const { return is_init_; }

ObstacleLine::Ptr ObstacleLine::transform(const Eigen::Isometry3d &se3) const {
    auto ptr = make();
    ptr->points_origin_.reserve(points_origin_.size());
    for (size_t i = 0; i < points_origin_.size(); ++i) {
        Eigen::Vector3d point = se3 * points_origin_[i];
        ptr->points_origin_.push_back(point);
    }

    return ptr;
}

ObstacleLine::Ptr ObstacleLine::transform(const Pose2d &pose) const {
    Eigen::Isometry3d se3 = pose.to_iso3d();
    return transform(se3);
}

ObstacleManager::ObstacleManager(double interpolate_thresh) { interpolate_thresh_ = interpolate_thresh; }

std::vector<ObstacleLine::Ptr> ObstacleManager::get_obstacle_lines(CoordinateType coord) const {
    switch (coord) {
    case CoordinateType::World: {
        return obstacle_line_ptr_vector_world_frame_;
    }

    case CoordinateType::Body: {
        return obstacle_line_ptr_vector_body_frame_;
    }

    case CoordinateType::Slot: {
        return obstacle_line_ptr_vector_slot_frame_;
    }

    default: {
        return {};
    }
    }
}
std::vector<ObstacleLine::Ptr> ObstacleManager::get_uss_obstacle(CoordinateType coord) const {
    switch (coord) {
    case CoordinateType::World: {
        return uss_obstacle_ptr_vector_world_frame_;
    }

    case CoordinateType::Body: {
        return uss_obstacle_ptr_vector_body_frame_;
    }

    case CoordinateType::Slot: {
        return uss_obstacle_ptr_vector_slot_frame_;
    }

    default: {
        return {};
    }
    }
}

std::vector<ObstacleBox::Ptr> ObstacleManager::get_static_obstacle_box(CoordinateType coord) const {
    switch (coord) {
    case CoordinateType::World: {
        return static_obstacle_ptr_vector_world_frame_;
    }

    case CoordinateType::Body: {
        return static_obstacle_ptr_vector_body_frame_;
    }

    case CoordinateType::Slot: {
        return static_obstacle_ptr_vector_slot_frame_;
    }

    default: {
        return {};
    }
    }
}

std::vector<ObstacleBox::Ptr> ObstacleManager::get_dynamic_obstacle_box(CoordinateType coord) const {
    switch (coord) {
    case CoordinateType::World: {
        return dynamic_obstacle_ptr_vector_world_frame_;
    }

    case CoordinateType::Body: {
        return dynamic_obstacle_ptr_vector_body_frame_;
    }

    case CoordinateType::Slot: {
        return dynamic_obstacle_ptr_vector_slot_frame_;
    }

    default: {
        return {};
    }
    }
}

void ObstacleManager::update(const PerceptionView &perception_view, const PerceptionView &fusion_uss,
                             const PerceptionView &fusion_object, const Eigen::Isometry3d &T_wb,
                             const Eigen::Isometry3d &T_body_slot, bool right_park) {
    Eigen::Isometry3d T_bw = T_wb.inverse();
    Eigen::Isometry3d T_slot_body = T_body_slot.inverse();
    reset();
    for (int i = 0; i < perception_view.parking_obstacle_lines().obstacle_lines_size(); ++i) {
        auto odl_world = ObstacleLine::make(perception_view.parking_obstacle_lines().obstacle_lines(i));
        auto odl_body = odl_world->transform(T_bw);
        auto odl_slot = odl_body->transform(T_slot_body);
        if (!right_park) {
            odl_slot->mirror_injection();
        }
        odl_slot->init(interpolate_thresh_);
        odl_slot->set_type(odl_world->type());
        obstacle_line_ptr_vector_world_frame_.push_back(odl_world);
        obstacle_line_ptr_vector_body_frame_.push_back(odl_body);
        obstacle_line_ptr_vector_slot_frame_.push_back(odl_slot);
    }

    for (int i = 0; i < fusion_uss.parking_obstacle_lines().obstacle_lines_size(); ++i) {
        auto od_point_world = ObstacleLine::make(fusion_uss.parking_obstacle_lines().obstacle_lines(i));
        auto od_point_body = od_point_world->transform(T_bw);
        auto od_point_slot = od_point_body->transform(T_slot_body);
        if (!right_park) {
            od_point_slot->mirror_injection();
        }
        od_point_slot->init(-1);
        od_point_slot->set_func(od_point_world->func());
        uss_obstacle_ptr_vector_world_frame_.push_back(od_point_world);
        uss_obstacle_ptr_vector_body_frame_.push_back(od_point_body);
        uss_obstacle_ptr_vector_slot_frame_.push_back(od_point_slot);
    }

    for (int i = 0; i < fusion_object.obstacles_size(); ++i) {
        auto obstacle_world = ObstacleBox::make(fusion_object.obstacles(i));
        auto obstacle_body = obstacle_world->transform(T_bw);
        auto obstacle_slot = obstacle_body->transform(T_slot_body);
        if (!right_park) {
            obstacle_slot->mirror_injection();
        }
        // if (fusion_object.obstacles(i).movestatus() != Obstacle::MOVING) {
        static_obstacle_ptr_vector_world_frame_.push_back(obstacle_world);
        static_obstacle_ptr_vector_body_frame_.push_back(obstacle_body);
        static_obstacle_ptr_vector_slot_frame_.push_back(obstacle_slot);
        // } else {
        // dynamic_obstacle_ptr_vector_world_frame_.push_back(obstacle_world);
        // dynamic_obstacle_ptr_vector_body_frame_.push_back(obstacle_body);
        // dynamic_obstacle_ptr_vector_slot_frame_.push_back(obstacle_slot);

        // double vx = 0.0, vy = 0.0;
        // if (fusion_object.obstacles(i).has_odom_obs_vel()) {
        //     vx = fusion_object.obstacles(i).odom_obs_vel().x();
        //     vy = fusion_object.obstacles(i).odom_obs_vel().y();
        // } else {
        //     double linear_v = 1.0;
        //     vx = linear_v * std::cos(fusion_object.obstacles(i).odom_obs_theta());
        //     vy = linear_v * std::sin(fusion_object.obstacles(i).odom_obs_theta());
        // }
        // double max_prediction_duration = 5.0;
        // double kTimePiece = 1.0;
        // for (double pred_t = kTimePiece; pred_t <= max_prediction_duration; pred_t+= kTimePiece) {
        //     Obstacle pred_obstacle;
        //     pred_obstacle.CopyFrom(fusion_object.obstacles(i));
        //     pred_obstacle.mutable_odom_obs_pos()->set_x(fusion_object.obstacles(i).odom_obs_pos().x() + vx * pred_t);
        //     pred_obstacle.mutable_odom_obs_pos()->set_y(fusion_object.obstacles(i).odom_obs_pos().y() + vy * pred_t);
        //     auto pred_obstacle_world = ObstacleBox::make(pred_obstacle);
        //     auto pred_obstacle_body  = pred_obstacle_world->transform(T_bw);
        //     auto pred_obstacle_slot  = pred_obstacle_body->transform(T_slot_body);
        //     if (!right_park) {
        //         pred_obstacle_slot->mirror_injection();
        //     }
        //     dynamic_obstacle_ptr_vector_world_frame_.push_back(pred_obstacle_world);
        //     dynamic_obstacle_ptr_vector_body_frame_.push_back(pred_obstacle_body);
        //     dynamic_obstacle_ptr_vector_slot_frame_.push_back(pred_obstacle_slot);
        // }
        // }
    }
    valid_ = true;
}

void ObstacleManager::reset() {
    obstacle_line_ptr_vector_world_frame_.clear();
    obstacle_line_ptr_vector_body_frame_.clear();
    obstacle_line_ptr_vector_slot_frame_.clear();

    uss_obstacle_ptr_vector_world_frame_.clear();
    uss_obstacle_ptr_vector_body_frame_.clear();
    uss_obstacle_ptr_vector_slot_frame_.clear();

    static_obstacle_ptr_vector_world_frame_.clear();
    static_obstacle_ptr_vector_body_frame_.clear();
    static_obstacle_ptr_vector_slot_frame_.clear();

    dynamic_obstacle_ptr_vector_world_frame_.clear();
    dynamic_obstacle_ptr_vector_body_frame_.clear();
    dynamic_obstacle_ptr_vector_slot_frame_.clear();

    valid_ = false;
}

bool ObstacleManager::valid() const { return valid_; }

void ObstacleManager::cal_parallel_yline_bottom_uss(std::shared_ptr<ParkEnvironment> park_env) {
    park_env->obstacle_info.bottom_obstacle_line_type = this->bottom_obstacle_line_type_;
    auto slot = &(park_env->slot);
    auto slot_x_angle = std::atan((slot->front_right().y() - slot->front_left().y()) /
                                  (slot->front_right().x() - slot->front_left().x()));
    // 获取底边的接地线,uss点和车位底边
    Vec2d bottom_center = (slot->rear_left() + slot->rear_right()) / 2; // 车位底边中心点
    // 底边扩展区域, 车位底边中心点为中心
    double low_y = 1;
    Box2d bottom_line_area =
        Box2d(Vec2d(bottom_center.x(), bottom_center.y()), slot_x_angle, slot->width() - 1.0, low_y);
    // 确定底边类型是墙体还是路沿
    std::unordered_map<ObstacleLineTypeEnum, int> obs_line_type_cnt;
    auto obs_l = obstacle_line_ptr_vector_slot_frame_;
    for (size_t i = 0; i < obs_l.size(); ++i) {
        for (size_t j = 0; j < obs_l[i]->points().size(); ++j) {
            const Vec2d &obstacle_point = obs_l[i]->points()[j];
            // 障碍物点是否在底边区域内
            if (bottom_line_area.IsPointIn(obstacle_point) || bottom_line_area.IsPointOnBoundary(obstacle_point)) {
                obs_line_type_cnt[obs_l[i]->type()]++;
            }
        }
    }
    std::vector<ObstacleLineTypeEnum> obs_line_type_cnt_vector;
    for (auto obs_line : obs_line_type_cnt) {
        obs_line_type_cnt_vector.emplace_back(obs_line.first);
    }
    MLOG(PARKING_PLANNING, INFO) << "park_env->obstacle_info.bottom_obstacle_line_type,"
                                 << park_env->obstacle_info.bottom_obstacle_line_type;
    if (!obs_line_type_cnt_vector.empty()) {
        sort(obs_line_type_cnt_vector.begin(), obs_line_type_cnt_vector.end(),
             [&](const ObstacleLineTypeEnum &a, const ObstacleLineTypeEnum &b) -> bool {
                 return obs_line_type_cnt[a] > obs_line_type_cnt[b];
             });
        park_env->obstacle_info.bottom_obstacle_line_type = get_obstacle_line_type(obs_line_type_cnt_vector[0]);
    }
    this->bottom_obstacle_line_type_ = park_env->obstacle_info.bottom_obstacle_line_type;

    // 区分墙体,柱子,路沿,首先判断是墙体还是路沿,还是都不是,然后找出是否有柱子
    auto obs_uss = uss_obstacle_ptr_vector_slot_frame_;
    // 底边area内的uss点容器
    std::vector<Vec2d> uss_in_bottom_line_area;

    if (obs_uss.size() < 2 && fabs(this->y_line_bottom_uss_ - park_env->slot.rear_left().y()) < 0.3) {
        MLOG(PARKING_PLANNING, INFO) << "uss less and y_line_bottom_uss right";
        return;
    }

    for (size_t i = 0; i < obs_uss.size(); i++) {
        auto uss = obs_uss.at(i);
        for (size_t j = 0; j < uss->points().size(); ++j) {
            auto p = uss->points().at(j);
            if (bottom_line_area.IsPointIn(p) || bottom_line_area.IsPointOnBoundary(p)) { // uss 点在这个区域内
                uss_in_bottom_line_area.emplace_back(p);
            }
        }
    }
    // 底边拟合
    Pose2d loc = park_env->start_pose;
    if (loc.y() > park_env->slot.front_left().y() + 0.5 && loc.theta() < 10 * M_PI / 360.) {
        y_line_bottom_uss_ = MathHelper::yline_fit(uss_in_bottom_line_area);
    }
    if (y_line_bottom_uss_ > -0.3) {
        this->y_line_bottom_uss_ = park_env->slot.rear_left().y();
    }
}

void ObstacleManager::collect_data_and_construct_T(std::shared_ptr<ParkEnvironment> park_env,
                                                   const bool &is_construct_t_shape) {
    park_env->obstacle_points.clear();
    park_env->obstacle_points_for_decider.clear();
    park_env->removed_obstacle_points.clear();
    park_env->obstacle_edges.clear();
    park_env->obstacle_edges_for_decider.clear();
    park_env->obstacle_info = ObstacleInfo();
    park_env->raw_obstacle_line_slot_vector.clear();

    // 对侧的墙和柱子的膨胀
    if (is_construct_t_shape) {
        // 保存一份原始接地线点用于可视化debug
        auto raw_obstacle_line = ObstacleLine::make();
        for (size_t i = 0; i < obstacle_line_ptr_vector_slot_frame_.size(); ++i) {
            for (size_t j = 0; j < obstacle_line_ptr_vector_slot_frame_[i]->points().size(); ++j) {
                raw_obstacle_line->mutable_points().push_back(obstacle_line_ptr_vector_slot_frame_[i]->points()[j]);
            }
        }
        park_env->raw_obstacle_line_slot_vector.push_back(raw_obstacle_line);

        obstacle_line_ptr_vector_slot_frame_ =
            expansion_obstacle_by_zone(obstacle_line_ptr_vector_slot_frame_, park_env->start_pose);
    }

    if (park_env->slot.park_type() == ParkType::VERTICAL || park_env->slot.park_type() == ParkType::OBLIQUE) {
        collect_data_and_construct_T_vert(park_env);
    } else if (park_env->slot.park_type() == ParkType::PARALLEL) {
        collect_data_and_construct_T_para(park_env, is_construct_t_shape);
    }

    for (size_t i = 0; i < uss_obstacle_ptr_vector_slot_frame_.size(); ++i) {
        if (uss_obstacle_ptr_vector_slot_frame_[i]->func() != haomo::hidelivery::perception::COLLISION_AVOIDANCE) {
            continue;
        }
        for (size_t j = 0; j < uss_obstacle_ptr_vector_slot_frame_[i]->points().size(); ++j) {
            park_env->obstacle_points_for_decider.push_back(uss_obstacle_ptr_vector_slot_frame_[i]->points()[j]);
        }
    }
}

void ObstacleManager::collect_data_and_construct_T_vert(std::shared_ptr<ParkEnvironment> park_env) {
    auto slot_angle = std::atan((park_env->slot.front_right().y() - park_env->slot.front_left().y()) /
                                (park_env->slot.front_right().x() - park_env->slot.front_left().x()));
    Vec2d slot_x_direction = park_env->slot.front_right() - park_env->slot.front_left();
    slot_x_direction.Normalize();
    Vec2d slot_y_direction = park_env->slot.front_left() - park_env->slot.rear_left();
    slot_y_direction.Normalize();
    Vec2d slot_vertical_direction = Vec2d(-slot_x_direction.y(), slot_x_direction.x());
    slot_vertical_direction.Normalize();
    Vec2d slot_horizontal_direction = Vec2d(slot_y_direction.y(), -slot_y_direction.x());
    slot_horizontal_direction.Normalize();

    MLOG(PARKING_PLANNING, INFO) << "slot_x_direction " << slot_x_direction.x() << " " << slot_x_direction.y();
    MLOG(PARKING_PLANNING, INFO) << "slot_y_direction " << slot_y_direction.x() << " " << slot_y_direction.y();

    MLOG(PARKING_PLANNING, INFO) << "slot_angle: " << slot_angle;

    const double default_slot_width = std::min(2.5, park_env->slot.width());
    const double default_upper_line_y = FLAGS_apa_vert_default_road_width + 3;
    Box2d upper_area = Box2d::CreateAABox(Vec2d(park_env->slot.front_left().x() - 1.0,
                                                park_env->slot.front_left().y() + FLAGS_apa_vert_default_road_width -
                                                    FLAGS_apa_park_env_upper_box_height),
                                          Vec2d(park_env->slot.front_left().x() + default_slot_width * 4,
                                                park_env->slot.front_left().y() + default_upper_line_y));

    const auto &slot_box = park_env->slot.box();

    std::vector<Vec2d> tmp_obstacle_points;
    std::vector<Vec2d> tmp_obstacle_points_for_decider;
    std::vector<Vec2d> tmp_obstacle_edge_points;
    std::vector<Vec2d> tmp_dynamic_obstacle_edge_points;
    std::vector<LineSegment2d> tmp_obstacle_lines;
    std::vector<LineSegment2d> tmp_dynamic_obstacle_lines;

    // 因为当前USS, 接地线会入侵车位，构造一个虚拟膨胀车位删除车位内的USS点。
    Box2d virtual_slot_box = park_env->slot.box();
    double longitudinal_extend = FLAGS_apa_uss_filter_box_longitudinal_extend;
    virtual_slot_box.Shift(Vec2d(0, longitudinal_extend / 2));
    virtual_slot_box.LongitudinalExtend(longitudinal_extend);

    // 因为当前接地线会入侵车位和规划的终态，构造一个虚拟膨胀车位删除车位一点范围内的接地线。
    Box2d remove_obstacle_line_box = park_env->slot.box();
    remove_obstacle_line_box.LongitudinalExtend(1.0);

    // TODO: 临时借用之前的参数，删除车位指定区域的USS
    // double lateral_extend = std::max(
    //     0.0, FLAGS_apa_uss_filter_box_width - virtual_slot_box.width());
    double lateral_extend = (FLAGS_apa_uss_filter_box_width - 1.0) * virtual_slot_box.width();
    virtual_slot_box.LateralExtend(lateral_extend);

    double confidence_distance = FLAGS_apa_uss_confidence_distance;
    VehicleModel vehicle_model(0.0);
    vehicle_model.update_pose(park_env->start_pose);
    auto current_car_polygon = vehicle_model.polygon();

    // double slot_width = park_env->slot.width();
    double box_length = 5.5; // 比车位长度要小一点
    double box_width = 1.0;  // 检测区域宽度2m
    MLOG(PARKING_PLANNING, INFO) << "slot angle," << park_env->slot.slot_angle() << ",slot heading,"
                                 << park_env->slot.heading();
    Vec2d slot_center = park_env->slot.center();
    // double vehicle_width = VehicleConfigHelper::GetConfig().vehicle_param().overallwidth();
    Box2d lane_area(Vec2d(park_env->slot.front_left().x() + 4, park_env->slot.front_left().y() + 5.5), slot_angle, 5.0,
                    4.0);
    Box2d left_area(Vec2d(slot_center.x() - park_env->slot.width() / 2 -
                              (box_width / 2 - 0.3) / std::sin(park_env->slot.slot_angle()),
                          slot_center.y()),
                    park_env->slot.heading() - M_PI / 2, box_width, box_length);
    left_area.Shift(Vec2d(1.5 * std::cos(park_env->slot.slot_angle()), 1.5 * std::sin(park_env->slot.slot_angle())));
    for (auto p : left_area.GetAllCorners()) {
        MLOG(PARKING_PLANNING, INFO) << "left_area_corners " << p.x() << " " << p.y();
    }
    Box2d right_area(Vec2d(slot_center.x() + park_env->slot.width() / 2 +
                               (box_width / 2 - 0.3) / std::sin(park_env->slot.slot_angle()),
                           slot_center.y()),
                     park_env->slot.heading() - M_PI / 2, box_width, box_length);
    right_area.Shift(Vec2d(1.5 * std::cos(park_env->slot.slot_angle()), 1.5 * std::sin(park_env->slot.slot_angle())));

    // 底边,确定底边类型,对于墙面路沿左特别处理
    Box2d bottom_area(park_env->slot.rear_mid(),
                      std::atan((park_env->slot.rear_right().y() - park_env->slot.rear_left().y()) /
                                (park_env->slot.rear_right().x() - park_env->slot.rear_left().x())),
                      box_width, 2); // 底边中点为中心,box_width宽,0.8m长
    Vec2d t_left_point = park_env->slot.front_left() - slot_x_direction * FLAGS_apa_vert_rotate_down_target_x +
                         slot_y_direction * FLAGS_apa_vert_rotate_down_target_y;
    Vec2d t_right_point = park_env->slot.front_right() + slot_x_direction * FLAGS_apa_vert_rotate_down_target_x +
                          slot_y_direction * FLAGS_apa_vert_rotate_down_target_y;

    Vec2d roi_area_center = park_env->slot.front_mid();

    Box2d roi_area_uss_bottom = Box2d(roi_area_center, park_env->slot.slot_angle() + M_PI_2,
                                      2. * VehicleConfigHelper::Instance()->GetConfig().vehicle_param().overalllength(),
                                      2 * VehicleConfigHelper::Instance()->GetConfig().vehicle_param().overalllength());

    MLOG(PARKING_PLANNING, INFO) << "roi_area_uss_bottom.fl," << roi_area_uss_bottom.GetAllCorners()[0].x() << " "
                                 << roi_area_uss_bottom.GetAllCorners()[0].y();
    MLOG(PARKING_PLANNING, INFO) << "roi_area_uss_bottom.fr," << roi_area_uss_bottom.GetAllCorners()[1].x() << " "
                                 << roi_area_uss_bottom.GetAllCorners()[1].y();
    MLOG(PARKING_PLANNING, INFO) << "roi_area_uss_bottom.rr," << roi_area_uss_bottom.GetAllCorners()[2].x() << " "
                                 << roi_area_uss_bottom.GetAllCorners()[2].y();
    MLOG(PARKING_PLANNING, INFO) << "roi_area_uss_bottom.rl," << roi_area_uss_bottom.GetAllCorners()[3].x() << " "
                                 << roi_area_uss_bottom.GetAllCorners()[3].y();

    double jud_head = M_PI / 6; // 重规划roi区域航向

    std::unordered_map<ObstacleLineTypeEnum, int> obs_line_left_type_cnt;
    std::unordered_map<ObstacleLineTypeEnum, int> obs_line_right_type_cnt;
    std::unordered_map<ObstacleLineTypeEnum, int> obs_line_bottom_type_cnt;

    for (size_t i = 0; i < obstacle_line_ptr_vector_slot_frame_.size(); ++i) {
        for (size_t j = 0; j < obstacle_line_ptr_vector_slot_frame_[i]->points().size(); ++j) {
            const Vec2d &obstacle_point = obstacle_line_ptr_vector_slot_frame_[i]->points()[j];
            if (!has_overlap(obstacle_point, park_env->map_boundary)) {
                park_env->removed_obstacle_points.push_back(obstacle_point);
                continue;
            }
            if (has_overlap(obstacle_point, left_area)) {
                park_env->obstacle_info.left_side_free = false;
                obs_line_left_type_cnt[obstacle_line_ptr_vector_slot_frame_[i]->type()]++;
            }

            if (has_overlap(obstacle_point, right_area)) {

                park_env->obstacle_info.right_side_free = false;
                obs_line_right_type_cnt[obstacle_line_ptr_vector_slot_frame_[i]->type()]++;
            }
            if (has_overlap(obstacle_point, upper_area)) {
                park_env->obstacle_info.upper_free = false;
            }
            if (has_overlap(obstacle_point, bottom_area)) {
                obs_line_bottom_type_cnt[obstacle_line_ptr_vector_slot_frame_[i]->type()]++;
            }
            if (has_overlap(obstacle_point, remove_obstacle_line_box)) {
                // if (obstacle_point.y() > park_env->slot.front_left().y() - 1.2 * park_env->slot.length()) {
                //     park_env->obstacle_info.vision_obstacle_invade = true;
                //     if (obstacle_point.x() < park_env->slot.center().x()) {
                //         park_env->obstacle_info.left_side_free = false;
                //     } else {
                //         park_env->obstacle_info.right_side_free = false;
                //     }
                // }
                continue;
                // if(virtual_slot_box.IsPointIn(obstacle_point)){
                //     continue;
                // }
            }
            tmp_obstacle_points.push_back(obstacle_line_ptr_vector_slot_frame_[i]->points()[j]);
            tmp_obstacle_points_for_decider.push_back(obstacle_line_ptr_vector_slot_frame_[i]->points()[j]);
        }
    }

    // 左侧是什么类型的接地线
    std::vector<ObstacleLineTypeEnum> obs_line_left_type_cnt_vector;
    for (auto obs_line : obs_line_left_type_cnt) {
        obs_line_left_type_cnt_vector.emplace_back(obs_line.first);
    }
    if (!obs_line_left_type_cnt_vector.empty()) {
        sort(obs_line_left_type_cnt_vector.begin(), obs_line_left_type_cnt_vector.end(),
             [&](const ObstacleLineTypeEnum &a, const ObstacleLineTypeEnum &b) -> bool {
                 return obs_line_left_type_cnt[a] > obs_line_left_type_cnt[b];
             });
        park_env->obstacle_info.left_obstacle_line_type = get_obstacle_line_type(obs_line_left_type_cnt_vector[0]);
    }
    // 右侧是什么类型的接地线
    std::vector<ObstacleLineTypeEnum> obs_line_right_type_cnt_vector;
    for (auto obs_line : obs_line_right_type_cnt) {
        obs_line_right_type_cnt_vector.emplace_back(obs_line.first);
    }
    if (!obs_line_right_type_cnt_vector.empty()) {
        sort(obs_line_right_type_cnt_vector.begin(), obs_line_right_type_cnt_vector.end(),
             [&](const ObstacleLineTypeEnum &a, const ObstacleLineTypeEnum &b) -> bool {
                 return obs_line_right_type_cnt[a] > obs_line_right_type_cnt[b];
             });
        park_env->obstacle_info.right_obstacle_line_type = get_obstacle_line_type(obs_line_right_type_cnt_vector[0]);
    }

    // 底边是什么类型的接地线
    std::vector<ObstacleLineTypeEnum> obs_line_bottom_type_cnt_vector;
    for (auto obs_line : obs_line_bottom_type_cnt) {
        obs_line_bottom_type_cnt_vector.emplace_back(obs_line.first);
    }
    if (!obs_line_bottom_type_cnt_vector.empty()) {
        sort(obs_line_bottom_type_cnt_vector.begin(), obs_line_bottom_type_cnt_vector.end(),
             [&](const ObstacleLineTypeEnum &a, const ObstacleLineTypeEnum &b) -> bool {
                 return obs_line_bottom_type_cnt[a] > obs_line_bottom_type_cnt[b];
             });
        park_env->obstacle_info.bottom_obstacle_line_type = get_obstacle_line_type(obs_line_bottom_type_cnt_vector[0]);
    }

    for (size_t i = 0; i < uss_obstacle_ptr_vector_slot_frame_.size(); ++i) {
        for (size_t j = 0; j < uss_obstacle_ptr_vector_slot_frame_[i]->points().size(); ++j) {
            const Vec2d &obstacle_point = uss_obstacle_ptr_vector_slot_frame_[i]->points()[j];
            if (!has_overlap(obstacle_point, park_env->map_boundary)) {
                park_env->removed_obstacle_points.push_back(obstacle_point);
                continue;
            }
            if (has_overlap(obstacle_point, bottom_area) &&
                has_overlap(Vec2d(park_env->start_pose.x(), park_env->start_pose.y()), roi_area_uss_bottom) &&
                park_env->start_pose.theta() > park_env->slot.heading() - jud_head &&
                park_env->start_pose.theta() < park_env->slot.heading() + jud_head) {
                MLOG(PARKNING_PLANNING, INFO) << "进入roi_area_uss_bottom区域";
                this->y_line_bottom_uss_ =
                    (this->y_line_bottom_uss_ > obstacle_point.y()) ? this->y_line_bottom_uss_ : obstacle_point.y();
            }
            if (has_overlap(obstacle_point, slot_box)) {
                park_env->obstacle_info.uss_obstacle_invade = true;
                // if (obstacle_point.x() < park_env->slot.center().x()) {
                //     park_env->obstacle_info.left_side_free = false;
                // } else {
                //     park_env->obstacle_info.right_side_free = false;
                // }
            }
            if (has_overlap(obstacle_point, left_area)) {
                park_env->obstacle_info.left_side_free = false;
            }
            if (has_overlap(obstacle_point, right_area)) {
                park_env->obstacle_info.right_side_free = false;
            }
            if (has_overlap(obstacle_point, upper_area)) {
                park_env->obstacle_info.upper_free = false;
            }
            // 底边uss点在roi区域范围外时终态不考虑,在范围内时考虑

            if (!virtual_slot_box.IsPointIn(obstacle_point) ||
                current_car_polygon.DistanceTo(obstacle_point) < confidence_distance) {
                tmp_obstacle_points.push_back(obstacle_point);
            }
        }
    }
    for (size_t i = 0; i < static_obstacle_ptr_vector_slot_frame_.size(); ++i) {
        auto edges = static_obstacle_ptr_vector_slot_frame_[i]->edges();
        for (const auto &edge : edges) {
            if (!has_overlap(edge, park_env->map_boundary)) {
                continue;
            }
            if (has_overlap(edge, slot_box)) {
                // OD入侵车位
                park_env->obstacle_info.vision_obstacle_invade = true;
                if (edge.start().x() < park_env->slot.center().x()) {
                    park_env->obstacle_info.left_side_free = false;
                    // od 入侵
                    if (fabs(edge.start().x() - park_env->slot.center().x()) < park_env->slot.width() / 2 ||
                        fabs(edge.end().x() - park_env->slot.center().x()) < park_env->slot.width() / 2) {
                        park_env->obstacle_info.left_obstacle_line_type = "OD";
                    }
                } else {
                    park_env->obstacle_info.right_side_free = false;
                    if (fabs(edge.start().x() - park_env->slot.center().x()) < park_env->slot.width() / 2 ||
                        fabs(edge.end().x() - park_env->slot.center().x()) < park_env->slot.width() / 2) {
                        park_env->obstacle_info.left_obstacle_line_type = "OD";
                    }
                }
                // continue; //删除车位内的OD边
            }
            if (has_overlap(edge, left_area)) {
                park_env->obstacle_info.left_side_free = false;
            }
            if (has_overlap(edge, right_area)) {
                park_env->obstacle_info.right_side_free = false;
            }
            if (has_overlap(edge, upper_area)) {
                park_env->obstacle_info.upper_free = false;
            }
            tmp_obstacle_lines.push_back(edge);
        }
    }
    for (size_t i = 0; i < static_obstacle_ptr_vector_slot_frame_.size(); ++i) {
        auto edges = static_obstacle_ptr_vector_slot_frame_[i]->edges();
        for (const auto &edge : edges) {
            std::vector<Vec2d> edge_points{edge.start(), edge.end()};
            for (const auto &obstacle_point : edge_points) {
                if (!has_overlap(obstacle_point, park_env->map_boundary)) {
                    park_env->removed_obstacle_points.push_back(obstacle_point);
                    continue;
                }
                if (has_overlap(obstacle_point, slot_box)) {
                    if (obstacle_point.y() > park_env->slot.front_left().y() - 0.7 * park_env->slot.length()) {
                        park_env->obstacle_info.vision_obstacle_invade = true;
                        if (obstacle_point.x() < park_env->slot.center().x()) {
                            park_env->obstacle_info.left_side_free = false;
                        } else {
                            park_env->obstacle_info.right_side_free = false;
                        }
                    }
                    continue;
                }
                if (has_overlap(obstacle_point, left_area)) {
                    park_env->obstacle_info.left_side_free = false;
                }
                if (has_overlap(obstacle_point, right_area)) {
                    park_env->obstacle_info.right_side_free = false;
                }
                if (has_overlap(obstacle_point, upper_area)) {
                    park_env->obstacle_info.upper_free = false;
                }
                tmp_obstacle_edge_points.push_back(obstacle_point);
            }
        }
    }

    for (size_t i = 0; i < dynamic_obstacle_ptr_vector_slot_frame_.size(); ++i) {
        auto edges = dynamic_obstacle_ptr_vector_slot_frame_[i]->edges();
        for (const auto &edge : edges) {
            if (!has_overlap(edge, park_env->map_boundary)) {
                continue;
            }
            tmp_dynamic_obstacle_lines.push_back(edge);
        }
    }

    for (size_t i = 0; i < dynamic_obstacle_ptr_vector_slot_frame_.size(); ++i) {
        auto edges = dynamic_obstacle_ptr_vector_slot_frame_[i]->edges();
        for (const auto &edge : edges) {
            std::vector<Vec2d> edge_points{edge.start(), edge.end()};
            for (const auto &obstacle_point : edge_points) {
                if (!has_overlap(obstacle_point, park_env->map_boundary)) {
                    park_env->removed_obstacle_points.push_back(obstacle_point);
                    continue;
                }
                tmp_dynamic_obstacle_edge_points.push_back(obstacle_point);
            }
        }
    }

    //  通过障碍物信息计算T区
    Vec2d foot_point;
    double lane_width = 7.0;
    Vec2d lane_upper_point = park_env->slot.front_left() + 7.0 * slot_vertical_direction;
    auto left_area_corners = left_area.GetAllCorners(); //顺序是右下，右上，左上，左下
    auto left_area_right_line = LineSegment2d(left_area_corners[0], left_area_corners[1]);
    auto left_area_upper_line = LineSegment2d(left_area_corners[1], left_area_corners[2]);
    auto left_area_x_min = left_area_right_line.DistanceTo(t_left_point);
    auto left_area_y_min = left_area_upper_line.DistanceTo(t_left_point);
    auto right_area_corners = right_area.GetAllCorners();
    auto right_area_left_line = LineSegment2d(right_area_corners[2], right_area_corners[3]);
    auto right_area_upper_line = LineSegment2d(right_area_corners[1], right_area_corners[2]);
    auto right_area_x_min = right_area_left_line.DistanceTo(t_right_point);
    auto right_area_y_min = right_area_upper_line.DistanceTo(t_right_point);
    for (auto p : right_area_corners) {
        MLOG(PARKING_PLANNING, INFO) << "right_area_corners " << p.x() << " " << p.y();
    }
    for (const auto &point : tmp_obstacle_points) {
        if (lane_area.IsPointIn(point)) {
            double dis = park_env->slot.front_edge().GetPerpendicularFoot(point, &foot_point);
            if (dis < lane_width) {
                lane_upper_point = point;
                lane_width = dis;
            }
        }
        if (left_area.IsPointIn(point)) {
            auto x_min = left_area_right_line.DistanceTo(point);
            if (x_min < left_area_x_min) {
                t_left_point = t_left_point + slot_horizontal_direction * (left_area_x_min - x_min);
                left_area_x_min = x_min;
            }

            auto y_min = left_area_upper_line.DistanceTo(point);
            if (y_min < left_area_y_min) {
                t_left_point = t_left_point + slot_y_direction * (left_area_y_min - y_min);
                left_area_y_min = y_min;
            }
        }
        if (right_area.IsPointIn(point)) {
            auto x_min = right_area_left_line.DistanceTo(point);
            if (x_min < right_area_x_min) {
                t_right_point = t_right_point - slot_horizontal_direction * (right_area_x_min - x_min);
                right_area_x_min = x_min;
            }

            auto y_min = right_area_upper_line.DistanceTo(point);
            if (y_min < right_area_y_min) {
                t_right_point = t_right_point + slot_y_direction * (right_area_y_min - y_min);
                right_area_y_min = y_min;
            }
        }
    }
    for (const auto &point : tmp_obstacle_edge_points) {
        if (lane_area.IsPointIn(point)) {
            double dis = park_env->slot.front_edge().GetPerpendicularFoot(point, &foot_point);
            if (dis < lane_width) {
                lane_upper_point = point;
                lane_width = dis;
            }
        }
        if (left_area.IsPointIn(point)) {
            auto x_min = left_area_right_line.DistanceTo(point);
            if (x_min < left_area_x_min) {
                t_left_point = t_left_point + slot_horizontal_direction * (left_area_x_min - x_min);
                left_area_x_min = x_min;
            }

            auto y_min = left_area_upper_line.DistanceTo(point);
            if (y_min < left_area_y_min) {
                t_left_point = t_left_point + slot_y_direction * (left_area_y_min - y_min);
                left_area_y_min = y_min;
            }
        }
        if (right_area.IsPointIn(point)) {
            auto x_min = right_area_left_line.DistanceTo(point);
            if (x_min < right_area_x_min) {
                t_right_point = t_right_point - slot_horizontal_direction * (right_area_x_min - x_min);
                right_area_x_min = x_min;
            }

            auto y_min = right_area_upper_line.DistanceTo(point);
            if (y_min < right_area_y_min) {
                t_right_point = t_right_point + slot_y_direction * (right_area_y_min - y_min);
                right_area_y_min = y_min;
            }
        }
    }

    auto t_road_left_edge_left = t_left_point - 5.0 * slot_x_direction;
    park_env->t_shape.road_left_edge = LineSegment2d(t_road_left_edge_left, t_left_point);
    MLOG(PARKING_PLANNING, INFO) << "t_road_left_edge_left: " << t_road_left_edge_left.x() << " "
                                 << t_road_left_edge_left.y();
    MLOG(PARKING_PLANNING, INFO) << "t_left_point: " << t_left_point.x() << " " << t_left_point.y();
    MLOG(PARKING_PLANNING, INFO) << "t_right_point: " << t_right_point.x() << " " << t_right_point.y();
    park_env->slot.right_edge().GetPerpendicularFoot(t_right_point, &foot_point);
    auto t_road_right_edge_right = t_right_point + 9.0 * slot_x_direction;
    park_env->t_shape.road_right_edge = LineSegment2d(t_road_right_edge_right, t_right_point);

    auto t_slot_left_length = park_env->slot.rear_edge().GetPerpendicularFoot(t_left_point, &foot_point) /
                              std::sin(park_env->slot.slot_angle());

    auto t_slot_right_length = park_env->slot.rear_edge().GetPerpendicularFoot(t_right_point, &foot_point) /
                               std::sin(park_env->slot.slot_angle());

    park_env->t_shape.slot_left_edge =
        LineSegment2d(t_left_point, t_left_point - t_slot_left_length * slot_y_direction);
    park_env->t_shape.slot_right_edge =
        LineSegment2d(t_right_point, t_right_point - t_slot_right_length * slot_y_direction);
    double saft_y = park_env->t_shape.slot_left_edge.end().y() + 0.0;
    if (park_env->slot.has_limiter()) {
        //当没有阻车器的时候，T的底边是车位的底边，
        //当有阻车器的时候，T的底边是阻车器的位置减去一个固定数值，这样来预防rotate_down越过阻车器。
        saft_y = std::max(park_env->slot.limiter().max_y() - 0.3, saft_y);
    }
    double bottom_x_min = park_env->t_shape.slot_left_edge.end().x();
    double bottom_x_max = park_env->t_shape.slot_right_edge.end().x();
    park_env->t_shape.slot_bottom_edge = LineSegment2d(Vec2d(bottom_x_min, saft_y), Vec2d(bottom_x_max, saft_y));

    lane_upper_point = Vec2d(lane_upper_point.x(), lane_upper_point.y());
    auto t_upper_line_left = park_env->t_shape.road_left_edge.GetPerpendicularFoot(lane_upper_point, &foot_point);
    auto t_upper_line_right = park_env->t_shape.road_right_edge.GetPerpendicularFoot(lane_upper_point, &foot_point);

    park_env->t_shape.upper_line =
        LineSegment2d(t_road_left_edge_left + slot_vertical_direction * t_upper_line_left,
                      t_road_right_edge_right + slot_vertical_direction * t_upper_line_right);

    park_env->t_shape.c_point = t_right_point - slot_x_direction * 0.05 + slot_y_direction * 0.05;
    park_env->t_shape.road_width = FLAGS_apa_vert_default_road_width;

    if (FLAGS_apa_vert_park_env_use_t) {
        park_env->obstacle_points.push_back(park_env->t_shape.road_left_edge.end());
        park_env->obstacle_points.push_back(park_env->t_shape.road_right_edge.end());
        park_env->obstacle_points.push_back(park_env->t_shape.slot_bottom_edge.start());
        park_env->obstacle_points.push_back(park_env->t_shape.slot_bottom_edge.end());

        park_env->obstacle_edges.push_back(park_env->t_shape.road_left_edge);
        park_env->obstacle_edges.push_back(park_env->t_shape.slot_left_edge);

        park_env->obstacle_edges.push_back(park_env->t_shape.road_right_edge);
        park_env->obstacle_edges.push_back(park_env->t_shape.slot_right_edge);
        park_env->obstacle_edges.push_back(park_env->t_shape.upper_line);
        park_env->obstacle_edges.push_back(park_env->t_shape.slot_bottom_edge);

        std::vector<Vec2d> polygon_points{
            park_env->t_shape.upper_line.start(),      park_env->t_shape.upper_line.end(),
            park_env->t_shape.road_right_edge.start(), park_env->t_shape.road_right_edge.end(),
            park_env->t_shape.slot_right_edge.end(),   park_env->t_shape.slot_left_edge.end(),
            park_env->t_shape.slot_left_edge.start(),  park_env->t_shape.road_left_edge.start()};

        Polygon2d t_shape_polygon(polygon_points);

        for (const auto &point : tmp_obstacle_points) {
            if (t_shape_polygon.IsPointIn(point) || t_shape_polygon.IsPointOnBoundary(point)) {
                park_env->obstacle_points_for_decider.push_back(point);

                if (!virtual_slot_box.IsPointIn(point) || current_car_polygon.DistanceTo(point) < confidence_distance) {
                    park_env->obstacle_points.push_back(point);
                }
            }
        }

        for (const auto &point : tmp_obstacle_edge_points) {
            if (t_shape_polygon.IsPointIn(point) || t_shape_polygon.IsPointOnBoundary(point)) {
                park_env->obstacle_points.push_back(point);
            }
        }

        for (const auto &line : tmp_obstacle_lines) {
            if (t_shape_polygon.HasOverlap(line) && !virtual_slot_box.HasOverlap(line)) {
                park_env->obstacle_edges.push_back(line);
            }
        }

        // for(const auto& point: tmp_dynamic_obstacle_edge_points)
        // {
        //     if(t_shape_polygon.IsPointIn(point) ||
        //        t_shape_polygon.IsPointOnBoundary(point))
        //     {
        //         park_env->obstacle_points_for_decider.push_back(point);
        //     }
        // }

        for (const auto &line : tmp_dynamic_obstacle_lines) {
            if (t_shape_polygon.HasOverlap(line)) {
                park_env->obstacle_edges_for_decider.push_back(line);
            }
        }
    } else {
        park_env->obstacle_points_for_decider = tmp_obstacle_points_for_decider;
        park_env->obstacle_points = tmp_obstacle_points;
        park_env->obstacle_edges = tmp_obstacle_lines;
        park_env->obstacle_edges_for_decider = tmp_dynamic_obstacle_lines;
        for (const auto &edge_point : tmp_obstacle_edge_points) {
            park_env->obstacle_points.push_back(edge_point);
        }
        // for(const auto& edge_point: tmp_dynamic_obstacle_edge_points)
        // {
        //     park_env->obstacle_points_for_decider.push_back(edge_point);
        // }
    }

    MLOG(PARKING_PLANNING, INFO) << "left side status: " << park_env->obstacle_info.left_side_free;
    MLOG(PARKING_PLANNING, INFO) << "right side status: " << park_env->obstacle_info.right_side_free;
    MLOG(PARKING_PLANNING, INFO) << "upper side status: " << park_env->obstacle_info.upper_free;
    MLOG(PARKING_PLANNING, INFO) << "vision_obstacle_invade: " << park_env->obstacle_info.vision_obstacle_invade;
    MLOG(PARKING_PLANNING, INFO) << "uss_obstacle_invade: " << park_env->obstacle_info.uss_obstacle_invade;
}

void ObstacleManager::collect_data_and_construct_T_para(std::shared_ptr<ParkEnvironment> park_env,
                                                        const bool &is_construct_t_shape) {

    // double filter_obs_length = FLAGS_apa_para_bottom_obs_filter_box_length * park_env->slot.length();
    double filter_obs_length = 0.2;
    auto slot_x_angle = std::atan((park_env->slot.front_right().y() - park_env->slot.front_left().y()) /
                                  (park_env->slot.front_right().x() - park_env->slot.front_left().x()));
    Vec2d slot_x_direction = park_env->slot.front_right() - park_env->slot.front_left();
    slot_x_direction.Normalize();
    Vec2d slot_y_direction = park_env->slot.front_left() - park_env->slot.rear_left();
    slot_y_direction.Normalize();

    Vec2d filter_obs_line_start = park_env->slot.rear_left() + filter_obs_length * slot_y_direction;
    Vec2d filter_obs_line_end = park_env->slot.rear_right() + filter_obs_length * slot_y_direction;
    LineSegment2d filter_obs_line = LineSegment2d(filter_obs_line_start, filter_obs_line_end);

    double right_nearest_dist = park_env->slot.width();
    double left_nearest_dist = park_env->slot.width();

    double side_area_width = FLAGS_apa_para_update_tshape_consider_y + park_env->slot.length() - filter_obs_length;
    double left_side_area_length = left_nearest_dist + 0.5 * park_env->slot.width();
    double right_side_area_length = left_side_area_length;
    Vec2d left_side_center = park_env->slot.front_left() +
                             (0.5 * park_env->slot.width() - left_nearest_dist) * slot_x_direction / 2.0 +
                             (FLAGS_apa_para_update_tshape_consider_y - (park_env->slot.length() - filter_obs_length)) *
                                 slot_y_direction / 2.0;
    Box2d left_side_area(left_side_center, slot_x_angle, left_side_area_length, side_area_width);
    Vec2d right_side_center =
        park_env->slot.front_right() + ((right_nearest_dist - 0.5 * park_env->slot.width()) * slot_x_direction / 2.0) +
        ((FLAGS_apa_para_update_tshape_consider_y - (park_env->slot.length() - filter_obs_length)) * slot_y_direction /
         2.0);

    Box2d right_side_area(right_side_center, slot_x_angle, right_side_area_length, side_area_width);

    std::vector<Vec2d> tmp_obstacle_points;
    std::vector<Vec2d> tmp_obstacle_points_for_decider;
    std::vector<Vec2d> tmp_obstacle_edge_points;
    std::vector<Vec2d> tmp_dynamic_obstacle_edge_points;
    std::vector<LineSegment2d> tmp_obstacle_lines;
    std::vector<LineSegment2d> tmp_dynamic_obstacle_lines;

    bool is_in_slot = false;
    if (park_env->slot.box().IsPointIn(park_env->start_pose.translation()) ||
        park_env->slot.box().IsPointOnBoundary(park_env->start_pose.translation())) {
        is_in_slot = true;
    }
    Vec2d foot_point;
    double fusion_line_area_width = FLAGS_apa_para_update_tshape_consider_y + park_env->slot.length();
    Vec2d fusion_line_left_side_center =
        park_env->slot.front_left() - (0.5 * park_env->slot.width()) * slot_x_direction / 2.0 +
        (FLAGS_apa_para_update_tshape_consider_y - park_env->slot.length()) * slot_y_direction / 2.0;
    double fusion_line_left_side_area_length = 0.5 * park_env->slot.width();
    Box2d fusion_line_left_side_area =
        Box2d(fusion_line_left_side_center, slot_x_angle, fusion_line_left_side_area_length, fusion_line_area_width);

    Vec2d fusion_line_right_side_center =
        park_env->slot.front_right() + (0.5 * park_env->slot.width()) * slot_x_direction / 2.0 +
        (FLAGS_apa_para_update_tshape_consider_y - park_env->slot.length()) * slot_y_direction / 2.0;
    double fusion_line_right_side_area_length = 0.5 * park_env->slot.width();
    Box2d fusion_line_right_side_area =
        Box2d(fusion_line_right_side_center, slot_x_angle, fusion_line_right_side_area_length, fusion_line_area_width);
    for (size_t i = 0; i < obstacle_line_ptr_vector_slot_frame_.size(); ++i) {
        for (size_t j = 0; j < obstacle_line_ptr_vector_slot_frame_[i]->points().size(); ++j) {
            const Vec2d &obstacle_point = obstacle_line_ptr_vector_slot_frame_[i]->points()[j];
            if (!has_overlap(obstacle_point, park_env->map_boundary)) {
                park_env->removed_obstacle_points.push_back(obstacle_point);
                continue;
            }

            if (filter_obs_line.ProductOntoUnit(obstacle_point) < 0) {
                continue;
            }

            if (park_env->slot.box().IsPointIn(obstacle_point) ||
                park_env->slot.box().IsPointOnBoundary(obstacle_point)) {
                park_env->removed_obstacle_points.push_back(obstacle_point);
                continue;
            }

            if (fusion_line_right_side_area.IsPointIn(obstacle_point)) {
                double new_right_nearest_dist =
                    park_env->slot.right_edge().ProductOntoUnit(obstacle_point) < 0
                        ? park_env->slot.right_edge().GetPerpendicularFoot(obstacle_point, &foot_point)
                        : -park_env->slot.right_edge().GetPerpendicularFoot(obstacle_point, &foot_point);
                right_nearest_dist = std::min(new_right_nearest_dist, right_nearest_dist);
                MLOG(PARKING_PLANNING, DEBUG) << "right_nearest_dist: " << right_nearest_dist
                                              << " obs.x: " << obstacle_point.x() << " obs.y: " << obstacle_point.y();
            } else if (fusion_line_left_side_area.IsPointIn(obstacle_point)) {
                double new_left_nearest_dist =
                    park_env->slot.left_edge().ProductOntoUnit(obstacle_point) > 0
                        ? park_env->slot.left_edge().GetPerpendicularFoot(obstacle_point, &foot_point)
                        : -park_env->slot.left_edge().GetPerpendicularFoot(obstacle_point, &foot_point);
                left_nearest_dist = std::min(new_left_nearest_dist, left_nearest_dist);
            }
            tmp_obstacle_points.push_back(obstacle_line_ptr_vector_slot_frame_[i]->points()[j]);
            tmp_obstacle_points_for_decider.push_back(obstacle_line_ptr_vector_slot_frame_[i]->points()[j]);
        }
    }
    // TODO: ROI区域,车辆在ROI区域时,uss点不被清除,否则被清除,不再使用uss点识别tshape的左右两边
    Box2d roi_uss_area =
        Box2d(park_env->slot.center(), slot_x_angle, 2. * park_env->slot.width(), 1. * park_env->slot.length());
    double jud_head_uss = M_PI / 6;
    std::vector<Vec2d> uss_obstacle_points;
    for (size_t i = 0; i < uss_obstacle_ptr_vector_slot_frame_.size(); ++i) {
        for (size_t j = 0; j < uss_obstacle_ptr_vector_slot_frame_[i]->points().size(); ++j) {
            const Vec2d &obstacle_point = uss_obstacle_ptr_vector_slot_frame_[i]->points()[j];
            if (!has_overlap(obstacle_point, park_env->map_boundary)) {
                park_env->removed_obstacle_points.push_back(obstacle_point);
                continue;
            }

            uss_obstacle_points.emplace_back(obstacle_point);
            if (roi_uss_area.IsPointIn(Vec2d(park_env->start_pose.x(), park_env->start_pose.y())) &&
                park_env->start_pose.theta() > -jud_head_uss && park_env->start_pose.theta() < jud_head_uss) {

                left_side_center =
                    park_env->slot.front_left() +
                    (0.5 * park_env->slot.width() - left_nearest_dist) * slot_x_direction / 2.0 +
                    (FLAGS_apa_para_update_tshape_consider_y - (park_env->slot.length() - filter_obs_length)) *
                        slot_y_direction / 2.0;
                left_side_area_length = left_nearest_dist + 0.5 * park_env->slot.width();
                left_side_area = Box2d(left_side_center, slot_x_angle, left_side_area_length, side_area_width);

                right_side_center =
                    park_env->slot.front_right() +
                    (right_nearest_dist - 0.5 * park_env->slot.width()) * slot_x_direction / 2.0 +
                    (FLAGS_apa_para_update_tshape_consider_y - (park_env->slot.length() - filter_obs_length)) *
                        slot_y_direction / 2.0;
                right_side_area_length = right_nearest_dist + 0.5 * park_env->slot.width();
                right_side_area = Box2d(right_side_center, slot_x_angle, right_side_area_length, side_area_width);

                if (right_side_area.IsPointIn(obstacle_point)) {
                    right_nearest_dist =
                        park_env->slot.right_edge().ProductOntoUnit(obstacle_point) < 0
                            ? park_env->slot.right_edge().GetPerpendicularFoot(obstacle_point, &foot_point)
                            : -park_env->slot.right_edge().GetPerpendicularFoot(obstacle_point, &foot_point);
                } else if (left_side_area.IsPointIn(obstacle_point)) {
                    left_nearest_dist =
                        park_env->slot.left_edge().ProductOntoUnit(obstacle_point) > 0
                            ? park_env->slot.left_edge().GetPerpendicularFoot(obstacle_point, &foot_point)
                            : -park_env->slot.left_edge().GetPerpendicularFoot(obstacle_point, &foot_point);
                }
            }
            tmp_obstacle_points.push_back(uss_obstacle_ptr_vector_slot_frame_[i]->points()[j]);
        }
    }

    LineSegment2d mid_edge(park_env->slot.rear_mid(), park_env->slot.front_mid());
    double od_side_area_width = FLAGS_apa_para_update_tshape_consider_y + park_env->slot.length();
    for (size_t i = 0; i < static_obstacle_ptr_vector_slot_frame_.size(); ++i) {
        if (is_in_slot && (park_env->parking_task.park_mode() != ParkingTask::PARK_OUT)) {
            break;
        }
        auto edges = static_obstacle_ptr_vector_slot_frame_[i]->edges();
        for (const auto &edge : edges) {
            if (!has_overlap(edge, park_env->map_boundary)) {
                continue;
            }
            if (mid_edge.ProductOntoUnit(edge.start()) * mid_edge.ProductOntoUnit(edge.end()) < 0) {
                tmp_obstacle_lines.push_back(edge);
                continue;
            }
            if (park_env->slot.front_edge().ProductOntoUnit(edge.center()) > 0) {
                tmp_obstacle_lines.push_back(edge);
                continue;
            }
            Vec2d obstacle_point;
            // OD在右侧
            if (mid_edge.GetPerpendicularFoot(edge.start(), &foot_point) <
                mid_edge.GetPerpendicularFoot(edge.end(), &foot_point)) {
                obstacle_point = edge.start();
            } else {
                obstacle_point = edge.end();
            }

            left_side_center =
                park_env->slot.front_left() +
                (0.5 * park_env->slot.width() - left_nearest_dist) * slot_x_direction / 2.0 +
                (FLAGS_apa_para_update_tshape_consider_y - (park_env->slot.length())) * slot_y_direction / 2.0;
            left_side_area_length = left_nearest_dist + 0.5 * park_env->slot.width();
            left_side_area = Box2d(left_side_center, slot_x_angle, left_side_area_length, od_side_area_width);

            right_side_center =
                park_env->slot.front_right() +
                (right_nearest_dist - 0.5 * park_env->slot.width()) * slot_x_direction / 2.0 +
                (FLAGS_apa_para_update_tshape_consider_y - (park_env->slot.length())) * slot_y_direction / 2.0;
            right_side_area_length = right_nearest_dist + 0.5 * park_env->slot.width();
            right_side_area = Box2d(right_side_center, slot_x_angle, right_side_area_length, od_side_area_width);

            if (right_side_area.IsPointIn(obstacle_point)) {
                right_nearest_dist =
                    park_env->slot.right_edge().ProductOntoUnit(obstacle_point) < 0
                        ? park_env->slot.right_edge().GetPerpendicularFoot(obstacle_point, &foot_point)
                        : -park_env->slot.right_edge().GetPerpendicularFoot(obstacle_point, &foot_point);
            } else if (left_side_area.IsPointIn(obstacle_point)) {
                left_nearest_dist = park_env->slot.left_edge().ProductOntoUnit(obstacle_point) > 0
                                        ? park_env->slot.left_edge().GetPerpendicularFoot(obstacle_point, &foot_point)
                                        : -park_env->slot.left_edge().GetPerpendicularFoot(obstacle_point, &foot_point);
            }
            tmp_obstacle_lines.push_back(edge);
        }
    }

    for (size_t i = 0; i < static_obstacle_ptr_vector_slot_frame_.size(); ++i) {
        if (is_in_slot && (park_env->parking_task.park_mode() != ParkingTask::PARK_OUT)) {
            break;
        }
        auto edges = static_obstacle_ptr_vector_slot_frame_[i]->edges();
        for (const auto &edge : edges) {
            std::vector<Vec2d> edge_points{edge.start(), edge.end()};
            for (const auto &obstacle_point : edge_points) {
                if (!has_overlap(obstacle_point, park_env->map_boundary)) {
                    park_env->removed_obstacle_points.push_back(obstacle_point);
                    continue;
                }
                tmp_obstacle_edge_points.push_back(obstacle_point);
            }
        }
    }

    for (size_t i = 0; i < dynamic_obstacle_ptr_vector_slot_frame_.size(); ++i) {
        auto edges = dynamic_obstacle_ptr_vector_slot_frame_[i]->edges();
        for (const auto &edge : edges) {
            if (!has_overlap(edge, park_env->map_boundary)) {
                continue;
            }
            tmp_dynamic_obstacle_lines.push_back(edge);
        }
    }

    for (size_t i = 0; i < dynamic_obstacle_ptr_vector_slot_frame_.size(); ++i) {
        auto edges = dynamic_obstacle_ptr_vector_slot_frame_[i]->edges();
        for (const auto &edge : edges) {
            std::vector<Vec2d> edge_points{edge.start(), edge.end()};
            for (const auto &obstacle_point : edge_points) {
                if (!has_overlap(obstacle_point, park_env->map_boundary)) {
                    park_env->removed_obstacle_points.push_back(obstacle_point);
                    continue;
                }
                tmp_dynamic_obstacle_edge_points.push_back(obstacle_point);
            }
        }
    }

    double max_extend_length = (park_env->slot.width() >= FLAGS_apa_para_standard_slot_width)
                                   ? (FLAGS_apa_para_extend_slot_ratio * park_env->slot.width())
                                   : (1.2 * FLAGS_apa_para_extend_slot_ratio * park_env->slot.width());
    double left_expand_dist = left_nearest_dist;
    double right_expand_dist = right_nearest_dist;
    MLOG(PARKING_PLANNING, INFO) << "left_nearest_dist: " << left_nearest_dist;
    MLOG(PARKING_PLANNING, INFO) << "right_nearest_dist: " << right_nearest_dist;
    if (right_nearest_dist + left_nearest_dist <= max_extend_length) {
        // 限制后向车位的借入空间
        left_expand_dist = std::min(FLAGS_apa_para_max_backward_borrow_length, left_nearest_dist);
    } else if (left_nearest_dist <= 0.0) {
        // 优先扩展前向车位的空间
        right_expand_dist = max_extend_length + std::fabs(left_nearest_dist);
    } else {
        right_expand_dist = std::min(right_nearest_dist, max_extend_length);
        left_expand_dist = std::max((max_extend_length - right_nearest_dist), 0.0);
        // 限制后向车位的借入空间
        left_expand_dist = std::min(FLAGS_apa_para_max_backward_borrow_length, left_expand_dist);
    }
    MLOG(PARKING_PLANNING, INFO) << "left_expand_dist: " << left_expand_dist;
    MLOG(PARKING_PLANNING, INFO) << "right_expand_dist: " << right_expand_dist;

    if ((left_expand_dist + right_expand_dist) >= (0.5 * park_env->slot.width())) {
        if (left_expand_dist <= right_expand_dist) {
            park_env->obstacle_info.right_side_free = true;
            park_env->obstacle_info.left_side_free = false;
        } else {
            park_env->obstacle_info.left_side_free = true;
            park_env->obstacle_info.right_side_free = false;
        }
    } else {
        park_env->obstacle_info.left_side_free = false;
        park_env->obstacle_info.right_side_free = false;
    }

    //计算T区底边更新所需考虑的uss点区域
    double bottom_nearest_dist = 0.0;
    Box2d uss_bottom_shift_area = get_uss_bottom_shift_area(park_env, left_expand_dist, right_expand_dist);
    // 更新T区底边
    LineSegment2d t_shape_bottom_edge = park_env->slot.rear_edge();
    Vec2d uss_foot_point;
    for (auto obstacle_point : uss_obstacle_points) {
        t_shape_bottom_edge = LineSegment2d(park_env->slot.rear_left() + bottom_nearest_dist * slot_y_direction,
                                            park_env->slot.rear_right() + bottom_nearest_dist * slot_y_direction);
        if ((uss_bottom_shift_area.IsPointIn(obstacle_point) ||
             uss_bottom_shift_area.IsPointOnBoundary(obstacle_point)) &&
            t_shape_bottom_edge.ProductOntoUnit(obstacle_point) > 0) {
            bottom_nearest_dist = park_env->slot.rear_edge().GetPerpendicularFoot(obstacle_point, &uss_foot_point);
        }
    }
    double bottom_shift = bottom_nearest_dist;
    MLOG(PARKING_PLANNING, INFO) << "bottom_shift: " << bottom_shift;

    double parkC_corner_distance = 0.0;
    double parkD_corner_distance = 0.0;
    if (is_construct_t_shape) {
        parkC_corner_distance = FLAGS_apa_para_corner_distance;
        parkD_corner_distance = -park_env->slot.length();
        auto slot_right_edge_start = park_env->slot.front_right() + right_expand_dist * slot_x_direction;
        auto slot_left_edge_start = park_env->slot.front_left() - left_expand_dist * slot_x_direction;
        auto slot_left_edge_end =
            park_env->slot.rear_left() - left_expand_dist * slot_x_direction + bottom_shift * slot_y_direction;
        auto parkC_obstacle_points = tmp_obstacle_points;
        for (auto &edge : tmp_obstacle_lines) {
            auto length = edge.length();
            double delta_l = 0.1;
            double discrete_point_num = fabs(std ::ceil(length / delta_l)) + 1;
            for (size_t i = 0; i < discrete_point_num; ++i) {
                double s = (i * delta_l) > length ? length : (i * delta_l);
                auto point = edge.start() + s * edge.unit_direction();
                parkC_obstacle_points.emplace_back(point);
            }
        }
        for (auto obstacle_point : parkC_obstacle_points) {
            // 障碍物坐标和C角点的相对距离
            double delta_x = obstacle_point.x() - slot_right_edge_start.x();
            double delta_y = obstacle_point.y() - slot_right_edge_start.y();
            if (delta_x < FLAGS_apa_para_wall_case_consider_dis_top_x && delta_x >= 0 &&
                obstacle_point.y() > slot_left_edge_end.y() &&
                obstacle_point.y() < slot_right_edge_start.y() + FLAGS_apa_para_wall_case_consider_dis_top_y) {
                parkC_corner_distance = std::max(delta_y, parkC_corner_distance);
            }
            double delta_d_x = obstacle_point.x() - slot_left_edge_start.x();
            double delta_d_y = obstacle_point.y() - slot_left_edge_start.y();
            if (delta_d_x > -(0.5 * park_env->slot.width()) && delta_d_x <= 0 &&
                obstacle_point.y() > slot_left_edge_end.y() && obstacle_point.y() < slot_left_edge_start.y()) {
                parkD_corner_distance = std::max(delta_d_y, parkD_corner_distance);
            }
        }
    }
    double top_expand_dist = std::min(parkC_corner_distance, 0.0);
    double top_d_expand_dist = std::max(std::min(parkD_corner_distance, 0.0), -1.0);
    VehicleModel vehicle_model =
        VehicleModel(0.0, 0.0, FLAGS_apa_vehicle_model_front_x_cut, FLAGS_apa_vehicle_model_front_y_cut,
                     FLAGS_apa_vehicle_model_rear_x_cut, FLAGS_apa_vehicle_model_rear_y_cut);
    vehicle_model.update_pose(park_env->start_pose);
    auto virtual_park_d =
        park_env->slot.front_left() - left_expand_dist * slot_x_direction + top_d_expand_dist * slot_y_direction;
    if (vehicle_model.polygon().IsPointIn(virtual_park_d) ||
        vehicle_model.polygon().IsPointOnBoundary(virtual_park_d)) {
        top_d_expand_dist = std::min(parkD_corner_distance, 0.0);
    }
    MLOG(PARKING_PLANNING, INFO) << "top_d_expand_dist: " << top_d_expand_dist;
    double top_bound =
        park_env->start_pose.y() + 1.0 + (VehicleConfigHelper::GetConfig().vehicle_param().overallwidth() / 2.0);
    double road_width = std::max(FLAGS_apa_para_default_road_width, top_bound);

    // parall target decision
    // test

    this->cal_parallel_yline_bottom_uss(park_env);
    if (park_env->parking_task.park_mode() == ParkingTask::PARK_IN) {
        if (this->y_line_bottom_uss_ > -3 && FLAGS_apa_para_target_decider_model && this->y_line_bottom_uss_ < -0.3) {
            bottom_shift = -(park_env->slot.front_left().y() - this->y_line_bottom_uss_ -
                             (park_env->slot.front_left().y() - park_env->slot.rear_left().y()));
        }
    } else if (park_env->parking_task.park_mode() == ParkingTask::PARK_OUT) {
        bottom_shift = -FLAGS_apa_apra_parkout_bottom_shift_flat;
    }

    MLOG(PARKING_PLANNING, INFO) << "bottom_shift," << bottom_shift;

    park_env->t_shape.road_left_edge = LineSegment2d(
        park_env->slot.front_left() - 10 * slot_x_direction + top_d_expand_dist * slot_y_direction,
        park_env->slot.front_left() - left_expand_dist * slot_x_direction + top_d_expand_dist * slot_y_direction);

    park_env->t_shape.slot_left_edge = LineSegment2d(
        park_env->slot.front_left() - left_expand_dist * slot_x_direction + top_d_expand_dist * slot_y_direction,
        park_env->slot.rear_left() - left_expand_dist * slot_x_direction + bottom_shift * slot_y_direction);

    park_env->t_shape.road_right_edge = LineSegment2d(
        park_env->slot.front_right() + 10 * slot_x_direction + top_expand_dist * slot_y_direction,
        park_env->slot.front_right() + right_expand_dist * slot_x_direction + top_expand_dist * slot_y_direction);

    park_env->t_shape.slot_right_edge = LineSegment2d(
        park_env->slot.front_right() + right_expand_dist * slot_x_direction + top_expand_dist * slot_y_direction,
        park_env->slot.rear_right() + right_expand_dist * slot_x_direction + bottom_shift * slot_y_direction);

    park_env->t_shape.upper_line =
        LineSegment2d(park_env->slot.front_left() - 10 * slot_x_direction + road_width * slot_y_direction,
                      park_env->slot.front_right() + 10 * slot_x_direction + road_width * slot_y_direction);

    park_env->t_shape.slot_bottom_edge = LineSegment2d(
        park_env->slot.rear_left() - left_expand_dist * slot_x_direction + bottom_shift * slot_y_direction,
        park_env->slot.rear_right() + right_expand_dist * slot_x_direction + bottom_shift * slot_y_direction);

    park_env->t_shape.road_width = park_env->t_shape.upper_line.start().y();
    park_env->t_shape.c_point =
        Vec2d(park_env->t_shape.slot_right_edge.start().x(),
              park_env->slot.front_right().y() + parkC_corner_distance + FLAGS_apa_para_park_c_corner_buffer);

    if (FLAGS_apa_park_env_use_t && is_construct_t_shape) {
        park_env->obstacle_points.push_back(park_env->t_shape.road_left_edge.end());
        park_env->obstacle_points.push_back(park_env->t_shape.road_right_edge.end());
        park_env->obstacle_points.push_back(park_env->t_shape.slot_bottom_edge.start());
        park_env->obstacle_points.push_back(park_env->t_shape.slot_bottom_edge.end());

        park_env->obstacle_edges.push_back(park_env->t_shape.road_left_edge);
        park_env->obstacle_edges.push_back(park_env->t_shape.slot_left_edge);

        park_env->obstacle_edges.push_back(park_env->t_shape.road_right_edge);
        park_env->obstacle_edges.push_back(park_env->t_shape.slot_right_edge);
        park_env->obstacle_edges.push_back(park_env->t_shape.upper_line);
        // park_env->obstacle_edges.push_back(t_shape_area.slot_bottom_edge);

        std::vector<Vec2d> polygon_points{
            park_env->t_shape.upper_line.start(),      park_env->t_shape.upper_line.end(),
            park_env->t_shape.road_right_edge.start(), park_env->t_shape.road_right_edge.end(),
            park_env->t_shape.slot_right_edge.end(),   park_env->t_shape.slot_left_edge.end(),
            park_env->t_shape.slot_left_edge.start(),  park_env->t_shape.road_left_edge.start()};

        Polygon2d t_shape_polygon(polygon_points);

        for (const auto &point : tmp_obstacle_points) {
            if (t_shape_polygon.IsPointIn(point) || t_shape_polygon.IsPointOnBoundary(point)) {
                // park_env->obstacle_points_for_decider.push_back(point);
                park_env->obstacle_points.push_back(point);
            }
        }

        for (const auto &point : tmp_obstacle_points_for_decider) {
            if (t_shape_polygon.IsPointIn(point) || t_shape_polygon.IsPointOnBoundary(point)) {
                park_env->obstacle_points_for_decider.push_back(point);
                // park_env->obstacle_points.push_back(point);
            }
        }

        for (const auto &point : tmp_obstacle_edge_points) {
            if (t_shape_polygon.IsPointIn(point) || t_shape_polygon.IsPointOnBoundary(point)) {
                park_env->obstacle_points.push_back(point);
            }
        }

        for (const auto &line : tmp_obstacle_lines) {
            if (t_shape_polygon.HasOverlap(line)) {
                park_env->obstacle_edges.push_back(line);
            }
        }

        for (const auto &point : tmp_dynamic_obstacle_edge_points) {
            if (t_shape_polygon.IsPointIn(point) || t_shape_polygon.IsPointOnBoundary(point)) {
                park_env->obstacle_points_for_decider.push_back(point);
            }
        }

        for (const auto &line : tmp_dynamic_obstacle_lines) {
            if (t_shape_polygon.HasOverlap(line)) {
                park_env->obstacle_edges_for_decider.push_back(line);
            }
        }
    } else {
        park_env->obstacle_points_for_decider = tmp_obstacle_points_for_decider;
        park_env->obstacle_points = tmp_obstacle_points;
        park_env->obstacle_edges = tmp_obstacle_lines;
        park_env->obstacle_edges_for_decider = tmp_dynamic_obstacle_lines;
        for (const auto &edge_point : tmp_obstacle_edge_points) {
            park_env->obstacle_points.push_back(edge_point);
        }
        for (const auto &edge_point : tmp_dynamic_obstacle_edge_points) {
            park_env->obstacle_points_for_decider.push_back(edge_point);
        }
    }
}

void ObstacleManager::collect_obstacle_world_frame(const PerceptionView &perception_view,
                                                   const PerceptionView &fusion_uss,
                                                   const PerceptionView &fusion_object,
                                                   std::vector<Vec2d> &obstacle_points,
                                                   std::vector<Box2d> &obstacle_boxes) {
    obstacle_points.clear();
    obstacle_boxes.clear();
    reset();

    for (int i = 0; i < perception_view.parking_obstacle_lines().obstacle_lines_size(); ++i) {
        auto odl_world = ObstacleLine::make(perception_view.parking_obstacle_lines().obstacle_lines(i));
        odl_world->init();
        obstacle_line_ptr_vector_world_frame_.push_back(odl_world);
    }

    for (int i = 0; i < fusion_uss.parking_obstacle_lines().obstacle_lines_size(); ++i) {
        auto od_point_world = ObstacleLine::make(fusion_uss.parking_obstacle_lines().obstacle_lines(i));
        od_point_world->init();
        uss_obstacle_ptr_vector_world_frame_.push_back(od_point_world);
    }

    for (int i = 0; i < fusion_object.obstacles_size(); ++i) {
        auto obstacle_world = ObstacleBox::make(fusion_object.obstacles(i));
        // if (fusion_object.obstacles(i).movestatus() != perception::Obstacle::MOVING) {
        static_obstacle_ptr_vector_world_frame_.push_back(obstacle_world);
        // } else {
        // dynamic_obstacle_ptr_vector_world_frame_.push_back(obstacle_world);

        // double vx = 0.0, vy = 0.0;
        // if (fusion_object.obstacles(i).has_odom_obs_vel()) {
        //     vx = fusion_object.obstacles(i).odom_obs_vel().x();
        //     vy = fusion_object.obstacles(i).odom_obs_vel().y();
        // } else {
        //     double linear_v = 1.0;
        //     vx = linear_v * std::cos(fusion_object.obstacles(i).odom_obs_theta());
        //     vy = linear_v * std::sin(fusion_object.obstacles(i).odom_obs_theta());
        // }
        // double max_prediction_duration = 5.0;
        // double kTimePiece = 1.0;
        // for (double pred_t = kTimePiece; pred_t <= max_prediction_duration; pred_t+= kTimePiece) {
        //     perception::Obstacle pred_obstacle;
        //     pred_obstacle.CopyFrom(fusion_object.obstacles(i));
        //     pred_obstacle.mutable_odom_obs_pos()->set_x(fusion_object.obstacles(i).odom_obs_pos().x() + vx * pred_t);
        //     pred_obstacle.mutable_odom_obs_pos()->set_y(fusion_object.obstacles(i).odom_obs_pos().y() + vy * pred_t);
        //     auto pred_obstacle_world = ObstacleBox::make(pred_obstacle);
        //     dynamic_obstacle_ptr_vector_world_frame_.push_back(pred_obstacle_world);
        // }
        // }
    }

    for (size_t i = 0; i < obstacle_line_ptr_vector_world_frame_.size(); ++i) {
        for (size_t j = 0; j < obstacle_line_ptr_vector_world_frame_[i]->points().size(); ++j) {
            const Vec2d &obstacle_point = obstacle_line_ptr_vector_world_frame_[i]->points()[j];
            obstacle_points.push_back(obstacle_point);
        }
    }

    for (size_t i = 0; i < uss_obstacle_ptr_vector_world_frame_.size(); ++i) {
        for (size_t j = 0; j < uss_obstacle_ptr_vector_world_frame_[i]->points().size(); ++j) {
            const Vec2d &obstacle_point = uss_obstacle_ptr_vector_world_frame_[i]->points()[j];
            obstacle_points.push_back(obstacle_point);
        }
    }

    for (size_t i = 0; i < static_obstacle_ptr_vector_world_frame_.size(); ++i) {
        const Box2d &obstacle_box = static_obstacle_ptr_vector_world_frame_[i]->get_box();
        obstacle_boxes.push_back(obstacle_box);
    }

    for (size_t i = 0; i < dynamic_obstacle_ptr_vector_world_frame_.size(); ++i) {
        const Box2d &obstacle_box = dynamic_obstacle_ptr_vector_world_frame_[i]->get_box();
        obstacle_boxes.push_back(obstacle_box);
    }
}

bool ObstacleManager::has_overlap(const Vec2d &obstacle_point, const Box2d &boundary) {
    if (boundary.IsPointIn(obstacle_point) || boundary.IsPointOnBoundary(obstacle_point)) {
        return true;
    }
    return false;
}

bool ObstacleManager::has_overlap(const LineSegment2d &obstacle_line, const Box2d &boundary) {
    if (boundary.HasOverlap(obstacle_line)) {
        return true;
    }
    return false;
}

std::vector<ObstacleLine::Ptr>
ObstacleManager::expansion_obstacle_by_zone(const std::vector<ObstacleLine::Ptr> &obstacle_lines,
                                            const Pose2d &start_pose) {
    if (obstacle_lines.empty()) {
        return obstacle_lines;
    }
    auto vehicle_config = VehicleConfigHelper::GetConfig().vehicle_param();
    double rear_to_center = vehicle_config.overalllength() / 2.0 - vehicle_config.rear_edge_to_center();
    Pose2d center_pose = Pose2d(start_pose.x() + rear_to_center * std::cos(start_pose.theta()),
                                start_pose.y() + rear_to_center * std::sin(start_pose.theta()), start_pose.theta());
    std::vector<ObstacleLine::Ptr> expand_obstacle_lines;
    for (size_t i = 0; i < obstacle_lines.size(); ++i) {
        ObstacleLine::Ptr shift_obstacle_line = std::make_shared<ObstacleLine>();
        shift_obstacle_line->mutable_points().clear();
        shift_obstacle_line->set_type(obstacle_lines[i]->type());
        if ((obstacle_lines[i]->type() == ObstacleLineTypeEnum::WALL ||
             obstacle_lines[i]->type() == ObstacleLineTypeEnum::PILLAR) &&
            (obstacle_lines[i]->points()[0].y() > start_pose.y())) {
            std::vector<Vec2d> old_obstacle_points = obstacle_lines[i]->points();
            for (size_t j = 0; j < old_obstacle_points.size(); ++j) {
                Vec2d new_obstacle_point = translate_obstacle_point(old_obstacle_points[j], center_pose);
                shift_obstacle_line->mutable_points().emplace_back(new_obstacle_point);
            }
        } else {
            shift_obstacle_line->mutable_points() = obstacle_lines[i]->points();
        }
        expand_obstacle_lines.emplace_back(shift_obstacle_line);
    }
    return expand_obstacle_lines;
}

Vec2d ObstacleManager::translate_obstacle_point(const Vec2d &obstacle_point, const Pose2d &center_pose) {
    Vec2d shift_obstacle_point = obstacle_point;
    Vec2d uss_point;
    for (size_t i = 0; i < uss_obstacle_ptr_vector_slot_frame_.size(); ++i) {
        for (size_t j = 0; j < uss_obstacle_ptr_vector_slot_frame_[i]->points().size(); ++j) {
            uss_point = uss_obstacle_ptr_vector_slot_frame_[i]->points()[j];
            if (std::hypot(uss_point.y() - obstacle_point.y(), uss_point.x() - obstacle_point.x()) < 1.0) {
                return obstacle_point;
            }
        }
    }
    double obs_dis_to_center = std::hypot(center_pose.y() - obstacle_point.y(), center_pose.x() - obstacle_point.x());
    Vec2d direction = center_pose.translation() - obstacle_point;
    direction.Normalize();
    shift_obstacle_point = obstacle_point + obs_dis_to_center * 0.05 * direction;
    MLOG(PARKING_PLANNING, DEBUG) << "shift_obs.x: " << shift_obstacle_point.x()
                                  << " shift_obs.y: " << shift_obstacle_point.y();
    MLOG(PARKING_PLANNING, DEBUG) << "obs.x: " << obstacle_point.x() << " obs.y: " << obstacle_point.y();
    return shift_obstacle_point;
}

Box2d ObstacleManager::get_uss_bottom_shift_area(std::shared_ptr<ParkEnvironment> park_env,
                                                 const double &left_expand_dist, const double &right_expand_dist) {
    auto vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
    auto slot_x_angle = std::atan((park_env->slot.front_right().y() - park_env->slot.front_left().y()) /
                                  (park_env->slot.front_right().x() - park_env->slot.front_left().x()));
    Vec2d slot_x_direction = park_env->slot.front_right() - park_env->slot.front_left();
    slot_x_direction.Normalize();
    Vec2d slot_y_direction = park_env->slot.front_left() - park_env->slot.rear_left();
    slot_y_direction.Normalize();
    Vec2d uss_bottom_shift_center =
        park_env->slot.rear_mid() + (0.5 * park_env->slot.length() - 5.0) * slot_y_direction / 2.0;
    Box2d uss_bottom_shift_area(uss_bottom_shift_center, slot_x_angle,
                                park_env->slot.width() - 2 * FLAGS_apa_para_update_tshape_consider_x,
                                5.0 + 0.5 * park_env->slot.length());
    // if (!is_construct_t_shape) {
    //     return uss_bottom_shift_area;
    // }
    double rear_shift = vehicle_param.rear_edge_to_center() - vehicle_param.overalllength() / 2.0;
    Vec2d rear_center = park_env->slot.front_left() + (0.5 * park_env->slot.width() + rear_shift) * slot_x_direction -
                        (0.5 * park_env->slot.length()) * slot_y_direction;
    auto slot_angle = std::atan((park_env->slot.front_right().y() - park_env->slot.front_left().y()) /
                                (park_env->slot.front_right().x() - park_env->slot.front_left().x()));
    Pose2d raw_target_pose = Pose2d(rear_center.x(), rear_center.y(), slot_angle);
    VehicleModel uss_vehicle_model(0.0, 0.0);
    uss_vehicle_model.update_pose(raw_target_pose);
    double lateral_shift = 0.0;
    Vec2d uss_foot_point;
    LineSegment2d left_edge = LineSegment2d(park_env->slot.front_left() - left_expand_dist * slot_x_direction,
                                            park_env->slot.rear_left() - left_expand_dist * slot_x_direction);
    LineSegment2d right_edge = LineSegment2d(park_env->slot.front_right() + right_expand_dist * slot_x_direction,
                                             park_env->slot.rear_right() + right_expand_dist * slot_x_direction);
    double left_free_distance = left_edge.ProductOntoUnit(uss_vehicle_model.rear_left()) > 0
                                    ? left_edge.GetPerpendicularFoot(uss_vehicle_model.rear_left(), &uss_foot_point)
                                    : -left_edge.GetPerpendicularFoot(uss_vehicle_model.rear_left(), &uss_foot_point);
    double right_free_distance =
        right_edge.ProductOntoUnit(uss_vehicle_model.front_left()) < 0
            ? right_edge.GetPerpendicularFoot(uss_vehicle_model.front_left(), &uss_foot_point)
            : -right_edge.GetPerpendicularFoot(uss_vehicle_model.front_left(), &uss_foot_point);
    MLOG(PARKING_PLANNING, INFO) << "left_free_distance: " << left_free_distance;
    MLOG(PARKING_PLANNING, INFO) << "right_free_distance: " << right_free_distance;
    double safe_distance = FLAGS_apa_para_safe_buffer_target_pose_to_t_shape;
    if (left_free_distance < safe_distance) {
        if ((right_free_distance + left_free_distance) >= 2 * safe_distance) {
            lateral_shift = safe_distance - left_free_distance;
        } else {
            lateral_shift = (right_free_distance - left_free_distance) / 2;
        }
    }
    if (right_free_distance < safe_distance) {
        if ((right_free_distance + left_free_distance) >= 2 * safe_distance) {
            lateral_shift = right_free_distance - safe_distance;
        } else {
            lateral_shift = (right_free_distance - left_free_distance) / 2;
        }
    }
    Vec2d center_x_direction = rear_center + (lateral_shift - rear_shift) * slot_x_direction;
    Vec2d center_y_direction =
        park_env->slot.rear_mid() + (0.5 * park_env->slot.length() - 5.0) * slot_y_direction / 2.0;
    uss_bottom_shift_center = Vec2d(center_x_direction.x(), center_y_direction.y());
    MLOG(PARKING_PLANNING, INFO) << "uss_bottom_shift_center.x: " << uss_bottom_shift_center.x()
                                 << " y: " << uss_bottom_shift_center.y();
    uss_bottom_shift_area = Box2d(uss_bottom_shift_center, slot_x_angle, vehicle_param.overalllength() + 0.4,
                                  5.0 + 0.5 * park_env->slot.length());
    return uss_bottom_shift_area;
}

std::string ObstacleManager::get_obstacle_line_type(const ObstacleLineTypeEnum &obstacle_line_type) {
    std::string bottom_obstacle_line_type = "UNKNOWN_STATUS";
    switch (obstacle_line_type) {
    case ObstacleLineTypeEnum::UNKNOWN_STATUS:
        bottom_obstacle_line_type = "UNKNOWN_STATUS";
        break;
    case ObstacleLineTypeEnum::EDGE:
        bottom_obstacle_line_type = "EDGE";
        break;
    case ObstacleLineTypeEnum::PASS:
        bottom_obstacle_line_type = "PASS";
        break;
    case ObstacleLineTypeEnum::WALL:
        bottom_obstacle_line_type = "WALL";
        break;
    case ObstacleLineTypeEnum::PILLAR:
        bottom_obstacle_line_type = "PILLAR";
        break;
    case ObstacleLineTypeEnum::CURB:
        bottom_obstacle_line_type = "CURB";
        break;
    case ObstacleLineTypeEnum::CAR_EDGE:
        bottom_obstacle_line_type = "CAR_EDGE";
        break;
    case ObstacleLineTypeEnum::OTHER:
        bottom_obstacle_line_type = "OTHER";
        break;
    default:
        break;
    }
    return bottom_obstacle_line_type;
}

PLANNING_NAMESPACE_END
