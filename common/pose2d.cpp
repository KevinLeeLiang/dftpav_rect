#include "parking_planning/apa/common/pose2d.h"

PLANNING_NAMESPACE_START

Rot2::Rot2(double theta)
    : theta_(normalize_angle(theta))
    , cos_theta_(std::cos(theta_))
    , sin_theta_(std::sin(theta_)) {}

Rot2::Rot2(double sin_theta, double cos_theta)
    : theta_(theta(sin_theta, cos_theta))
    , cos_theta_(cos_theta)
    , sin_theta_(sin_theta) {}

double Rot2::normalize_angle(const double angle) {
    if (angle > M_PI || angle < -M_PI) {
        double a = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (a < 0.0) {
            a += (2.0 * M_PI);
        }
        return a - M_PI;
    }

    return angle;
}

Rot2 Rot2::identity() { return Rot2(0.0); }

double Rot2::extract_yaw_from_rotation_matrix(const Eigen::Matrix3d &rot) {
    Eigen::Vector3d euler{rot.eulerAngles(2, 1, 0)};
    double theta = euler(0);
    if ((std::abs(euler(1)) + std::abs(euler(2))) > (0.8 * M_PI)) {
        theta = Rot2::normalize_angle(euler(0) + M_PI);
    } else {
        theta = Rot2::normalize_angle(euler(0));
    }

    return theta;
}

Rot2 Rot2::inverse() const { return Rot2(-sin_theta_, cos_theta_); }

Vec2d Rot2::rotate(const Vec2d &p) const {
    return Vec2d(cos_theta_ * p.x() + -sin_theta_ * p.y(), sin_theta_ * p.x() + cos_theta_ * p.y());
}

Vec2d Rot2::unrotate(const Vec2d &p) const {
    return Vec2d(cos_theta_ * p.x() + sin_theta_ * p.y(), -sin_theta_ * p.x() + cos_theta_ * p.y());
}

Pose2d::Pose2d(double x, double y, double theta)
    : translation_(x, y)
    , rot_(theta) {}

Pose2d::Pose2d(const Rot2 &r, const Vec2d &t)
    : translation_(t)
    , rot_(r) {}

Eigen::Isometry3d Pose2d::to_iso3d() const {
    Eigen::Isometry3d pose3 = Eigen::Isometry3d::Identity();
    pose3.linear() = Eigen::AngleAxisd(theta(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose3.translation() = Eigen::Vector3d(x(), y(), 0.0);

    return pose3;
}

Pose2d Pose2d::from_iso3d(const Eigen::Isometry3d &pose3d) {
    Eigen::Vector3d euler{pose3d.linear().eulerAngles(2, 1, 0)};
    double theta = euler(0);
    if ((std::abs(euler(1)) + std::abs(euler(2))) > (0.8 * M_PI)) {
        theta = Rot2::normalize_angle(euler(0) + M_PI);
    } else {
        theta = Rot2::normalize_angle(euler(0));
    }

    return Pose2d(pose3d.translation().x(), pose3d.translation().y(), theta);
};

Pose2d Pose2d::from_proto_pose(const haomo::hidelivery::localization::Pose &proto_pose) {
    Eigen::Isometry3d pose3d = Eigen::Isometry3d::Identity();
    pose3d.linear() = Eigen::Quaterniond(proto_pose.odom_attitude().qw(), proto_pose.odom_attitude().qx(),
                                         proto_pose.odom_attitude().qy(), proto_pose.odom_attitude().qz())
                          .toRotationMatrix();

    pose3d.translation() =
        Eigen::Vector3d(proto_pose.odom_position().x(), proto_pose.odom_position().y(), proto_pose.odom_position().z());

    return from_iso3d(pose3d);
};

Pose2d Pose2d::mirror_injection() const { return Pose2d(-x(), y(), (M_PI - theta())); }

Pose2d Pose2d::identity() { return Pose2d(0.0, 0.0, 0.0); }

Pose2d Pose2d::inverse() const {
    return Pose2d(rot_.inverse(), rot_.unrotate(Vec2d(-translation_.x(), -translation_.y())));
}

// T^t * object
LineSegment2d Pose2d::transform_to(const LineSegment2d &line) const {
    return LineSegment2d(transform_to(line.start()), transform_from(line.end()));
}

// ObstacleLine Pose2d::transform_to(const ObstacleLine& line) const
// {
//     return ObstacleLine(transform_to(line.start()),
//                          transform_from(line.end()),
//                          line.type());
// }

Vec2d Pose2d::transform_to(const Vec2d &vec) const { return rot_.unrotate(vec - translation_); }

Box2d Pose2d::transform_to(const Box2d &box) const {
    return Box2d(transform_to(box.center()), (rot_.inverse() * Rot2(box.heading())).theta(), box.length(), box.width());
}

// ObstacleBox Pose2d::transform_to(const ObstacleBox& box) const
// {
//     return ObstacleBox(transform_to(box.center()),
//                       (rot_.inverse()* Rot2(box.heading())).theta(),
//                       box.length(), box.width(), box.type());
// }

// T^t*P1
Pose2d Pose2d::transform_to(const Pose2d &p1) const {
    return Pose2d(rot_.inverse() * p1.rot_, rot_.inverse() * (p1.translation_ - translation_));
}

// ObstacleLine Pose2d::transform_from(const ObstacleLine& line) const
// {
//     return ObstacleLine(transform_from(line.start()),
//                         transform_from(line.end()),
//                         line.type());
// }

LineSegment2d Pose2d::transform_from(const LineSegment2d &line) const {
    return LineSegment2d(transform_from(line.start()), transform_from(line.end()));
}

Vec2d Pose2d::transform_from(const Vec2d &vec) const { return rot_ * vec + translation_; }

Box2d Pose2d::transform_from(const Box2d &box) const {
    return Box2d(transform_from(box.center()), (rot_ * Rot2(box.heading())).theta(), box.length(), box.width());
}

// ObstacleBox Pose2d::transform_from(const ObstacleBox& box) const
// {
//     return ObstacleBox(transform_from(box.center()),
//                       (rot_* Rot2(box.heading())).theta(),
//                       box.length(), box.width(), box.type());
// }

// T*P1
Pose2d Pose2d::transform_from(const Pose2d &p1) const {
    Rot2 r = rot_ * p1.rot_;
    Vec2d t = rot_ * p1.translation_ + translation_;

    return Pose2d(r, t);
}

PLANNING_NAMESPACE_END