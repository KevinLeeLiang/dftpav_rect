//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : gird_map.h
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//

#ifndef GIRD_MAP_H
#define GIRD_MAP_H

#include "grid_collision_detection.h"
#include <eigen3/Eigen/Eigen>
#include <vector>

typedef Eigen::Vector2d POINT2D;

class Box2d {
private:
  POINT2D point_lf_;
  POINT2D point_rf_;
  POINT2D point_lr_;
  POINT2D point_rr_;
  double xmin_;
  double ymin_;
  double xmax_;
  double ymax_;
  double length_;
  double width_;
  POINT2D center_;
  void init(){
    point_lf_ = POINT2D(center_.x() - length_ / 2, center_.y() + width_ / 2);
    point_rf_ = POINT2D(center_.x() + length_ / 2, center_.y() + width_ / 2);
    point_lr_ = POINT2D(center_.x() - length_ / 2, center_.y() - width_ / 2);
    point_rr_ = POINT2D(center_.x() + length_ / 2, center_.y() - width_ / 2);
  }
public:
  Box2d() {
    xmin_ = 0;
    ymin_ = 0;
    xmax_ = 0;
    ymax_ = 0;
    center_ = POINT2D(0., 0.);
    length_ = 0.;
    width_ = 0.;
    init();
  }
  Box2d(POINT2D plf, POINT2D prf, POINT2D plr, POINT2D prr)
      : point_lf_(plf), point_lr_(plr), point_rf_(prf), point_rr_(prr) {
    length_ = prr.x() - prf.x();
  }
  Box2d(POINT2D c, double _length, double _width)
      : center_(c), length_(_length), width_(_width) {
    init();
    xmin_ = point_lr_.x();
    ymin_ = point_lr_.y();
    xmax_ = point_rf_.x();
    ymax_ = point_rf_.y();
  }
  Box2d(double _xmin, double _ymin, double _xmax, double _ymax)
      : xmin_(_xmin), ymin_(_ymin), xmax_(_xmax), ymax_(_ymax) {
    center_ = POINT2D(xmin_ + (xmax_ - xmin_)/2, ymin_ + (ymax_ - ymin_)/2);
    length_ = xmax_ - xmax_;
    width_ = ymax_ - ymin_;
    init();
  }

  POINT2D center() { return center_; }
  POINT2D point_left_front() { return point_lf_; }
  POINT2D point_left_rear() { return point_lr_; }
  POINT2D point_right_front() { return point_rf_; }
  POINT2D point_right_rear() { return point_rr_; }
  double x_min() { return xmin_; }
  double y_min() { return ymin_; }
  double x_max() { return xmax_; }
  double y_max() { return ymax_; }
  double width() { return width_; }
  double length() { return length_; }
};

class GridMap : public GridCollisionDetection {
public:
  GridMap();
  ~GridMap();
  void BuildGridMap(const std::vector<POINT2D> &obs, Box2d &map_bound);
  bool CheckIfCollisionUsingLine(const POINT2D p1, const POINT2D p2, const double resolution);

public:
  std::vector<std::vector<int>> grid_map;

  uint64_t map_length_;
  uint64_t map_width_;
};

#endif // GIRD_MAP_H
