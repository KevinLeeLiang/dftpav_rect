#ifndef PLANNING_PLANNING_TYPEDEF_H
#define PLANNING_PLANNING_TYPEDEF_H

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "math/vec2d.h"
#include "math/line_segment2d.h"
#include "math/box2d.h"
#include "planning_macro.h"
#include "data.h"
#include "parking_planning/common/vehicle_config/vehicle_config_helper.h"
#include "math/polygon2d.h"

PLANNING_NAMESPACE_START

using haomo::hidelivery::GearPosition;
using haomo::hidelivery::planning::PathPoint;
using haomo::hidelivery::planning::TrajectoryPoint;
using haomo::hios::GearBoxInfoPb;
using namespace haomo::hidelivery::math;

enum ParkingStatus {
    IDLE = 0,
    PARK_IN_OBSTACLE_BREAK = 1,
    PARK_OUT_OBSTACLE_BREAK = 2,
    PARKING_IN_FORWARD = 3,
    PARKING_IN_BACKWARD = 4,
    PARKING_IN_DONE = 5,
    PARKING_IN_ERROR_FOR_OBS = 6,
    PARKING_IN_ERROR_FOR_ROTATE_NUM = 7,
    PARKING_OUT = 8,
    PARKING_OUT_DONE = 9,
    PARKING_OUT_ERROR_FOR_OBS = 10,
    PARKING_OUT_ERROR_FOR_ROTATE_NUM = 11,
    PARKING_EMERGENCY_DONE = 12,
    PARKING_IN_LAST = 13,
    PARKING_OUT_LAST = 14,
};

enum ParkType {
    UNDEFINE = 0,
    VERTICAL = 1,
    PARALLEL = 2,
    OBLIQUE = 3,
    HEADING = 4,
};

enum class CoordinateType {
    World = 0,
    Body,
    Slot,
};

enum class SlotCoordinateType {
    Front_Left = 0,
    Front_Center = 1,
    Rear_Center = 2,
    Rear_Left = 3,
    Slot_Center = 4,
};

enum class ObsState {
    UNDEFINE = 0,
    HARD_BRAKE = 1,
    SOFT_BRAKE = 2,
    NONE_OBS = 3,
    SLOW_DOWN = 4,
};

enum ReplanCmd {
    First_Plan = 0,
    Obs_Stuck = 1,
    Pursuit_Error = 2,
    Slot_Update = 3,
    Static_Adjust = 4,
    Dynamic_Adjust = 5,
};

enum PathReqStatus {
    NO_NEED = 0,
    WAITING = 1,
    SUCCESS = 2,
    FAILURE = 3,
};

PLANNING_NAMESPACE_END

#endif // PLANNING_PLANNING_TYPEDEF_H
