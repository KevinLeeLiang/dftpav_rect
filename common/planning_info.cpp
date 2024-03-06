#include "planning_info.h"

PLANNING_NAMESPACE_START

void PlanningInfo::clear() {
    plan_env_ = std::make_shared<PlanEnvironment>();
    path_plan_debug_ = std::make_shared<PathPlanDebug>();
    is_select_candidate_ = false;
}

void PlanningInfo::clear_all() {
    clear();
    global_path_info_ = std::make_shared<Path>();
}

PLANNING_NAMESPACE_END