//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : geometry.h
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//

#ifndef GEOMETRY_H
#define GEOMETRY_H
#include "parking_planning/apa/common/planning_typedef.h"
#include "parking_planning/apa/common/pose2d.h"

namespace HA {
enum class ObstacleType : int {
    VIRTUAL = 0,
    BOUNDARY = 1,  // map boundary
    ROADBOARD = 2, //
    PARKINGLINE = 3,
    WALL = 4,
    CAR = 5,
    HUMAN = 6,
    FREESPACE = 7,
    NONE = 8,
};

class ObstacleLineTmp : public LineSegment2d {
  public:
    ObstacleLineTmp(const Vec2d &start, const Vec2d &end, const ObstacleType obs_type = ObstacleType::NONE)
        : LineSegment2d(start, end)
        , type_(obs_type) {
        init_max_min();
    };

    ObstacleLineTmp(const LineSegment2d &line_obs, const ObstacleType obs_type = ObstacleType::NONE)
        : LineSegment2d(line_obs)
        , type_(obs_type) {
        init_max_min();
    };

    ObstacleLineTmp() = default;

    ~ObstacleLineTmp() = default;
    ;

    ObstacleType type() const { return type_; };

    bool is_physical() const {
        return type_ == ObstacleType::CAR || type_ == ObstacleType::HUMAN || type_ == ObstacleType::WALL ||
               type_ == ObstacleType::FREESPACE;
    }

    double min_x() const { return min_x_; }
    double max_x() const { return max_x_; }
    double min_y() const { return min_y_; }
    double max_y() const { return max_y_; }

  private:
    void init_max_min() {
        min_x_ = std::min(start().x(), end().x());
        max_x_ = std::max(start().x(), end().x());
        min_y_ = std::min(start().y(), end().y());
        max_y_ = std::max(start().y(), end().y());
    }

    ObstacleType type_;
    double min_x_;
    double max_x_;
    double min_y_;
    double max_y_;
};

class ObstacleBoxTmp : public Box2d {
  public:
    ObstacleBoxTmp(const Vec2d &center, const float heading, const float length, const float width,
                   const ObstacleType obs_type = ObstacleType::NONE)
        : Box2d(center, heading, length, width)
        , type_(obs_type){};

    ObstacleBoxTmp(const Box2d &box_obs, const ObstacleType obs_type = ObstacleType::NONE)
        : Box2d(box_obs)
        , type_(obs_type){};

    ~ObstacleBoxTmp(){};

    ObstacleType type() const { return type_; };

    std::vector<LineSegment2d> GetAllEdges() const {
        auto corners = GetAllCorners();
        std::vector<LineSegment2d> ret;
        for (int i = 0; i < (int)corners.size() - 1; ++i) {
            ret.emplace_back(corners[i], corners[i + 1]);
        }
        ret.emplace_back(corners.back(), corners.front());
        return ret;
    }

  private:
    ObstacleType type_;
};

inline bool operator==(const Pose2d &lhs, const Pose2d &rhs) {
    return (lhs.x() == rhs.x() && lhs.y() == rhs.y() && lhs.theta() == rhs.theta());
}

inline bool operator!=(const Pose2d &lhs, const Pose2d &rhs) { return !operator==(lhs, rhs); }
};

#endif // GEOMETRY_H
