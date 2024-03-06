#include "vehicle_model.h"

PLANNING_NAMESPACE_START

VehicleModel::VehicleModel(double flat) {
    double lat_flat = flat;
    double lon_front_flat = flat;
    double lon_rear_flat = flat;
    init(lat_flat, lon_front_flat, lon_rear_flat, 0.0, 0.0, 0.0, 0.0);
}

VehicleModel::VehicleModel(double lat_flat, double lon_flat, double front_x_cut, double front_y_cut, double rear_x_cut,
                           double rear_y_cut) {
    double lon_front_flat = lon_flat;
    double lon_rear_flat = lon_flat;
    init(lat_flat, lon_front_flat, lon_rear_flat, front_x_cut, front_y_cut, rear_x_cut, rear_y_cut);
}

VehicleModel::VehicleModel(double lat_flat, double lon_front_flat, double lon_rear_flat, double front_x_cut,
                           double front_y_cut, double rear_x_cut, double rear_y_cut) {
    init(lat_flat, lon_front_flat, lon_rear_flat, front_x_cut, front_y_cut, rear_x_cut, rear_y_cut);
}

void VehicleModel::init(double lat_flat, double lon_front_flat, double lon_rear_flat, double front_x_cut,
                        double front_y_cut, double rear_x_cut, double rear_y_cut) {
    double half_width = 0.5 * VehicleConfigHelper::Instance()->GetConfig().vehicle_param().overallwidth() + lat_flat;
    double front_to_rear =
        VehicleConfigHelper::Instance()->GetConfig().vehicle_param().front_edge_to_center() + lon_front_flat;
    double back_to_rear =
        VehicleConfigHelper::Instance()->GetConfig().vehicle_param().rear_edge_to_center() + lon_rear_flat;

    origin_corners_.clear();
    origin_cut_corners_.clear();

    origin_cut_corners_.emplace_back(Vec2d(-back_to_rear + rear_x_cut, half_width));
    origin_cut_corners_.emplace_back(Vec2d(-back_to_rear, half_width - rear_y_cut));
    origin_cut_corners_.emplace_back(Vec2d(-back_to_rear, -half_width + rear_y_cut));
    origin_cut_corners_.emplace_back(Vec2d(-back_to_rear + rear_x_cut, -half_width));
    origin_cut_corners_.emplace_back(Vec2d(front_to_rear - front_x_cut, -half_width));
    origin_cut_corners_.emplace_back(Vec2d(front_to_rear, -half_width + front_y_cut));
    origin_cut_corners_.emplace_back(Vec2d(front_to_rear, half_width - front_y_cut));
    origin_cut_corners_.emplace_back(Vec2d(front_to_rear - front_x_cut, half_width));

    origin_corners_.emplace_back(Vec2d(-back_to_rear, half_width));
    origin_corners_.emplace_back(Vec2d(-back_to_rear, -half_width));
    origin_corners_.emplace_back(Vec2d(front_to_rear, -half_width));
    origin_corners_.emplace_back(Vec2d(front_to_rear, half_width));

    corners_ = origin_corners_;
    cut_corners_ = origin_cut_corners_;
    construct_edges();
}

Vec2d VehicleModel::front_left() const { return corners_[3]; }

Vec2d VehicleModel::front_right() const { return corners_[2]; }

Vec2d VehicleModel::rear_right() const { return corners_[1]; }

Vec2d VehicleModel::rear_left() const { return corners_[0]; }

void VehicleModel::update_pose(const Pose2d &pose) {
    corners_.clear();
    corners_.resize(origin_corners_.size());
    for (size_t i = 0; i < origin_corners_.size(); ++i) {
        corners_[i] = pose * origin_corners_[i];
    }
    construct_edges();

    cut_corners_.clear();
    cut_corners_.resize(origin_cut_corners_.size());
    for (size_t i = 0; i < origin_cut_corners_.size(); ++i) {
        cut_corners_[i] = pose * origin_cut_corners_[i];
    }
}

const std::vector<Vec2d> &VehicleModel::corners() const { return corners_; }

const std::vector<LineSegment2d> &VehicleModel::edges() const { return edges_; }

const std::vector<Vec2d> &VehicleModel::origin_corners() const { return origin_corners_; }

const Polygon2d &VehicleModel::polygon() const { return polygon_; }

void VehicleModel::construct_edges() {
    edges_.clear();
    for (size_t i = 0; i < corners_.size(); ++i) {
        edges_.emplace_back(corners_.at(i), corners_.at((i + 1) % (corners_.size())));
    }
    polygon_ = Polygon2d(corners_);
}

const std::vector<Vec2d> &VehicleModel::cut_corners() const { return cut_corners_; }
PLANNING_NAMESPACE_END