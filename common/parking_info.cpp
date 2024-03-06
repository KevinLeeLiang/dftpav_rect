#include "parking_info.h"

PLANNING_NAMESPACE_START

void ParkingInfo::clear() {
    park_env_ = std::make_shared<ParkEnvironment>();
    park_env_backup_ = std::make_shared<ParkEnvironment>();
    obstacle_decider_result_ = std::make_shared<ObstacleDeciderResult>();
    replan_data_ = std::make_shared<ReplanData>();
    path_req_ = std::make_shared<PathReqData>();
    pipeline_data_ = std::make_shared<PipelineData>();
    park_output_ = std::make_shared<ParkOutput>();
}

void ParkingInfo::clear_all() {
    clear();
    global_path_info_ = std::make_shared<Path>();
    current_path_info_ = std::make_shared<Path>();

    global_path_.clear();
    published_path_.clear();
    published_trajcetory_.clear();
}

PLANNING_NAMESPACE_END