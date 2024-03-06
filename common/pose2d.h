#pragma once

#include "planning_typedef.h"

PLANNING_NAMESPACE_START

class Rot2 {
  public:
    explicit Rot2(double theta = 0.0);

    Rot2(double sin_theta, double cos_theta);

    static double normalize_angle(const double angle);

    static Rot2 identity();

    static double extract_yaw_from_rotation_matrix(const Eigen::Matrix3d &rot);

    Rot2 inverse() const;

    Vec2d rotate(const Vec2d &vec) const;

    Vec2d unrotate(const Vec2d &vec) const;

    Eigen::Matrix2d matrix() const {
        Eigen::Matrix2d R;
        R << cos_theta_, -sin_theta_, sin_theta_, cos_theta_;
        return R;
    }

    double sin_theta() const { return sin_theta_; }

    double cos_theta() const { return cos_theta_; }

    double tan_theta() const { return sin_theta_ / cos_theta_; }

    double theta() const { return theta_; }

    double degree() const { return theta() / M_PI * 180.0; }

    Vec2d unit() const { return Vec2d(cos_theta_, sin_theta_); }

    Vec2d operator*(const Vec2d &vec) const { return rotate(vec); }

    // R1*R2
    Rot2 transform_from(const Rot2 &rot) const {
        return Rot2(sin_theta_ * rot.cos_theta_ + cos_theta_ * rot.sin_theta_,
                    cos_theta_ * rot.cos_theta_ - sin_theta_ * rot.sin_theta_);
    }

    // R1^t*R2
    Rot2 transform_to(const Rot2 &rot) const { return inverse() * rot; }

    Rot2 operator*(const Rot2 &rot) const { return transform_from(rot); }

    static double theta(double sin_theta, double cos_theta) { return ::atan2(sin_theta, cos_theta); }

    static double angle(const Rot2 &rot1, const Rot2 &rot2) { return (rot1.inverse() * rot2).theta(); }

    static double angle(const double &theta1, const double &theta2) {
        return (Rot2(theta1).inverse() * Rot2(theta2)).theta();
    }

  private:
    double theta_;
    double cos_theta_;
    double sin_theta_;
};

class Pose2d {
  public:
    explicit Pose2d(double x = 0.0, double y = 0.0, double theta = 0.0);

    Pose2d(const Rot2 &r, const Vec2d &t);

    Eigen::Isometry3d to_iso3d() const;

    static Pose2d from_iso3d(const Eigen::Isometry3d &pose3d);

    static Pose2d from_proto_pose(const haomo::hidelivery::localization::Pose &proto_pose);

    Pose2d mirror_injection() const;

    double x() const { return translation_.x(); }

    double y() const { return translation_.y(); }

    Rot2 rotation() const { return rot_; }

    Vec2d translation() const { return translation_; }

    double theta() const { return rot_.theta(); }

    double degree() const { return rot_.degree(); }

    static Pose2d identity();

    Pose2d inverse() const;

    // P^t * point
    LineSegment2d transform_to(const LineSegment2d &line) const;

    // ObstacleLine transform_to(const ObstacleLine& line) const;

    Vec2d transform_to(const Vec2d &vec) const;

    Box2d transform_to(const Box2d &box) const;

    // ObstacleBox transform_to(const ObstacleBox& box) const;

    // P1^t * P2
    Pose2d transform_to(const Pose2d &p1) const;

    // ObstacleLine transform_from(const ObstacleLine& line) const;

    LineSegment2d transform_from(const LineSegment2d &line) const;

    Vec2d transform_from(const Vec2d &vec) const;

    Box2d transform_from(const Box2d &box) const;

    // ObstacleBox transform_from(const ObstacleBox& box) const;

    // T*P1
    Pose2d transform_from(const Pose2d &p1) const;

    LineSegment2d operator*(const LineSegment2d &line) const { return transform_from(line); }

    Vec2d operator*(const Vec2d &vec) const { return transform_from(vec); }

    Box2d operator*(const Box2d &box) const { return transform_from(box); }

    Pose2d operator*(const Pose2d &p1) const { return transform_from(p1); }

    // ObstacleLine operator *(const ObstacleLine& line) const
    // {
    //     return transform_from(line);
    // }

    // ObstacleBox operator *(const ObstacleBox& box) const
    // {
    //     return transform_from(box);
    // }

    friend std::ostream &operator<<(std::ostream &out, const Pose2d &pose) {
        out << "x: " << pose.x() << ", y: " << pose.y() << ", theta: " << pose.theta() / M_PI * 180.0;
        return out;
    }

  private:
    Vec2d translation_;
    Rot2 rot_;
};

PLANNING_NAMESPACE_END
