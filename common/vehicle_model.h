#ifndef PLANNING_VEHICLE_MODEL_H
#define PLANNING_VEHICLE_MODEL_H

#include "pose2d.h"

PLANNING_NAMESPACE_START

class VehicleModel {
  public:
    explicit VehicleModel(double flat = 0.0);

    VehicleModel(double lat_flat, double lon_flat, double front_x_cut = 0.0, double front_y_cut = 0.0,
                 double rear_x_cut = 0.0, double rear_y_cut = 0.0);

    VehicleModel(double lat_flat, double lon_front_flat, double lon_rear_flat, double front_x_cut, double front_y_cut,
                 double rear_x_cut, double rear_y_cut);

    void update_pose(const Pose2d &pose);

    const std::vector<Vec2d> &corners() const;

    const std::vector<Vec2d> &cut_corners() const;

    const std::vector<LineSegment2d> &edges() const;

    const std::vector<Vec2d> &origin_corners() const;

    const Polygon2d &polygon() const;

    Vec2d front_left() const;

    Vec2d front_right() const;

    Vec2d rear_right() const;

    Vec2d rear_left() const;

  private:
    void init(double lat_flat, double lon_front_flat, double lon_rear_flat, double front_x_cut, double front_y_cut,
              double rear_x_cut, double rear_y_cut);

    void construct_edges();

    Polygon2d polygon_;
    std::vector<Vec2d> origin_corners_;
    std::vector<Vec2d> corners_;
    std::vector<LineSegment2d> edges_;
    std::vector<Vec2d> origin_cut_corners_;
    std::vector<Vec2d> cut_corners_;
};

PLANNING_NAMESPACE_END

#endif // PLANNING_VEHICLE_MODEL_H
