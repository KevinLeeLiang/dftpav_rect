#ifndef PLANNING_OBSTACLE_H
#define PLANNING_OBSTACLE_H

#include "pose2d.h"
#include "parking_planning/common/planning_gflags.h"
#include "parking_planning/apa/common/parking_slot.h"

PLANNING_NAMESPACE_START

using haomo::hidelivery::perception::ObstacleLine3D;
using haomo::hidelivery::perception::PerceptionView;

struct ObstacleInfo {
    bool left_side_free = true;
    bool right_side_free = true;
    bool upper_free = true;
    bool vision_obstacle_invade = false;
    bool uss_obstacle_invade = false;
    std::string bottom_obstacle_line_type = "UNKNOWN_STATUS";
    std::string left_obstacle_line_type = "UNKNOWN_STATUS";
    std::string right_obstacle_line_type = "UNKNOWN_STATUS";
};

class ObstacleBox {
  public:
    DEFINE_SHARDED_PTR(ObstacleBox)

    explicit ObstacleBox(const haomo::hidelivery::perception::Obstacle &obstacle, double flat_thresh = 0.0);

    static Ptr make(const haomo::hidelivery::perception::Obstacle &obstacle, double flat_thresh = 0.0);

    static Ptr make();

    void mirror_injection();

    ObstacleBox() = default;

    void init();

    const std::vector<Vec2d> &corner() const;

    const std::vector<LineSegment2d> &edges() const;

    Box2d get_box() const;

    Ptr transform(const Pose2d &pose) const;

    Ptr transform(const Eigen::Isometry3d &se3) const;

  private:
    double theta_;
    Eigen::Vector3d center_;
    Eigen::Vector3d size_;
    std::vector<Eigen::Vector3d> corners_;
    std::vector<Vec2d> corners_2d_;
    std::vector<LineSegment2d> edges_;
    Box2d box_;
};

enum ObstacleLineTypeEnum {
    UNKNOWN_STATUS = 0,
    EDGE = 1,     // 边缘
    PASS = 2,     // 可通行
    WALL = 3,     // 墙
    PILLAR = 4,   // 柱子
    CURB = 5,     // 路沿
    CAR_EDGE = 6, // 车辆边缘
    OTHER = 20,   // 未知
};
class ObstacleLine {
  public:
    DEFINE_SHARDED_PTR(ObstacleLine)

    explicit ObstacleLine(const ObstacleLine3D &odl_3d);

    static Ptr make(const ObstacleLine3D &odl_3d);

    static Ptr make();

    void mirror_injection();

    ObstacleLine() = default;

    void init(double interpolate_thresh = 0.3);

    const std::vector<Vec2d> &points() const;

    std::vector<Vec2d> &mutable_points() { return points_2d_interpolated_; };

    const std::vector<Eigen::Vector3d> &origin_points() const;

    const ObstacleLineTypeEnum type() const { return type_; };

    void set_type(ObstacleLineTypeEnum type) { type_ = type; };

    const haomo::hidelivery::perception::ObstacleLineFunction func() const { return func_; }

    void set_func(haomo::hidelivery::perception::ObstacleLineFunction func) { func_ = func; }

    bool inited() const;

    Ptr transform(const Pose2d &pose) const;

    Ptr transform(const Eigen::Isometry3d &se3) const;

    friend ObstacleLine operator*(const Pose2d &pose, const ObstacleLine &line_local) {
        Eigen::Isometry3d se3 = pose.to_iso3d();
        return se3 * line_local;
    }

    friend ObstacleLine operator*(const Eigen::Isometry3d &se3, const ObstacleLine &line_local) {
        ObstacleLine line_global;
        line_global.points_origin_.reserve(line_local.points_origin_.size());
        for (size_t i = 0; i < line_local.points_origin_.size(); ++i) {
            Eigen::Vector3d point = se3 * line_local.points_origin_[i];
            line_global.points_origin_.push_back(point);
        }

        return line_global;
    }

  private:
    bool is_init_ = false;
    std::vector<Eigen::Vector3d> points_origin_;
    std::vector<Eigen::Vector3d> points_interpolated_;
    std::vector<Vec2d> points_2d_interpolated_;
    ObstacleLineTypeEnum type_ = ObstacleLineTypeEnum::UNKNOWN_STATUS;
    haomo::hidelivery::perception::ObstacleLineFunction func_ = haomo::hidelivery::perception::NONE_FUNCTION;
};

struct TShapeArea;
struct ParkEnvironment;

class ObstacleManager {
  public:
    explicit ObstacleManager(double interpolate_thresh = 0.3);

    std::vector<ObstacleLine::Ptr> get_obstacle_lines(CoordinateType coord = CoordinateType::Slot) const;

    std::vector<ObstacleLine::Ptr> get_uss_obstacle(CoordinateType coord = CoordinateType::Slot) const;

    std::vector<ObstacleBox::Ptr> get_static_obstacle_box(CoordinateType coord = CoordinateType::Slot) const;

    std::vector<ObstacleBox::Ptr> get_dynamic_obstacle_box(CoordinateType coord = CoordinateType::Slot) const;

    void update(const PerceptionView &perception_view, const PerceptionView &fusion_uss,
                const PerceptionView &fusion_object, const Eigen::Isometry3d &T_wb,
                const Eigen::Isometry3d &T_body_slot, bool right_park);

    void reset();

    bool valid() const;

    static bool has_overlap(const Vec2d &obstacle_point, const Box2d &boundary);

    static bool has_overlap(const LineSegment2d &obstacle_line, const Box2d &boundary);

    void collect_data_and_construct_T(std::shared_ptr<ParkEnvironment> park_env, const bool &is_construct_t_shape);

    void collect_data_and_construct_T_vert(std::shared_ptr<ParkEnvironment> park_env);

    void collect_data_and_construct_T_para(std::shared_ptr<ParkEnvironment> park_env, const bool &is_construct_t_shape);

    void collect_obstacle_world_frame(const PerceptionView &perception_view, const PerceptionView &fusion_uss,
                                      const PerceptionView &fusion_object, std::vector<Vec2d> &obstacle_points,
                                      std::vector<Box2d> &obstacle_boxes);

    std::vector<ObstacleLine::Ptr> expansion_obstacle_by_zone(const std::vector<ObstacleLine::Ptr> &obstacle_lines,
                                                              const Pose2d &start_pose);

    Vec2d translate_obstacle_point(const Vec2d &obstacle_point, const Pose2d &center_pose);

    Box2d get_uss_bottom_shift_area(std::shared_ptr<ParkEnvironment> park_env, const double &left_expand_dist,
                                    const double &right_expand_dist);

    std::string get_obstacle_line_type(const ObstacleLineTypeEnum &obstacle_line_type);

    double get_parallel_yline_bottom_uss() const { return y_line_bottom_uss_; };

  private:
    void cal_parallel_yline_bottom_uss(std::shared_ptr<ParkEnvironment> park_env);

  private:
    double interpolate_thresh_ = 0.3;
    bool valid_ = false;
    std::vector<ObstacleLine::Ptr> obstacle_line_ptr_vector_world_frame_;
    std::vector<ObstacleLine::Ptr> obstacle_line_ptr_vector_body_frame_;
    std::vector<ObstacleLine::Ptr> obstacle_line_ptr_vector_slot_frame_;

    std::vector<ObstacleLine::Ptr> uss_obstacle_ptr_vector_world_frame_;
    std::vector<ObstacleLine::Ptr> uss_obstacle_ptr_vector_body_frame_;
    std::vector<ObstacleLine::Ptr> uss_obstacle_ptr_vector_slot_frame_;

    std::vector<ObstacleBox::Ptr> static_obstacle_ptr_vector_world_frame_;
    std::vector<ObstacleBox::Ptr> static_obstacle_ptr_vector_body_frame_;
    std::vector<ObstacleBox::Ptr> static_obstacle_ptr_vector_slot_frame_;

    std::vector<ObstacleBox::Ptr> dynamic_obstacle_ptr_vector_world_frame_;
    std::vector<ObstacleBox::Ptr> dynamic_obstacle_ptr_vector_body_frame_;
    std::vector<ObstacleBox::Ptr> dynamic_obstacle_ptr_vector_slot_frame_;
    double y_line_bottom_uss_ = -10.;
    std::string bottom_obstacle_line_type_ = "UNKNOWN_STATUS";
};

PLANNING_NAMESPACE_END

#endif // PLANNING_OBSTACLE_H
