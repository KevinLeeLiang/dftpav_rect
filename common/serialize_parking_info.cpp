#include "serialize_parking_info.h"
#include "math_helper.h"

PLANNING_NAMESPACE_START

void PlanningLogHelper::log_to_file(const std::string &data) {
    if (!init_) {
        return;
    }
    static int cnt = 0;
    if (cnt % N_ == 0) {
        std::ofstream fout(log_file_name_, std::ios::app);
        if (!fout.is_open()) {
            return;
        }
        fout << data << std::endl;
        fout.close();
        cnt = 0;
    }
    cnt++;
}

void PlanningLogHelper::init(const std::string &log_file_name) {
    init_ = false;
    log_file_name_ = log_file_name;
    std::ofstream fout(log_file_name_);
    if (fout.is_open()) {
        init_ = true;
    }
    fout.close();
}
void PlanningLogHelper::set_N(int N) { N_ = N; }
void PlanningLogHelper::reset() { init_ = false; }

SerializeParkingInfo::SerializeParkingInfo() {
    // 初始化点的信息和附加信息
    // std::map<std::string, std::vector<std::tuple<double, double, double>>>
    //     _points;
    // std::map<std::string, std::string> _extra_info;
}

void SerializeParkingInfo::add_points(std::string topic_name, const Vec2d point) {
    std::tuple<double, double, double> temp_point(point.x(), point.y(), 0);
    if (_points.find(topic_name) == _points.end()) {
        std::vector<std::tuple<double, double, double>> point_vector;
        _points[topic_name] = point_vector;
    }
    _points[topic_name].push_back(temp_point);
}

void SerializeParkingInfo::add_points(std::string topic_name, double x, double y, double v) {
    std::tuple<double, double, double> temp_point(x, y, v);
    if (_points.find(topic_name) == _points.end()) {
        std::vector<std::tuple<double, double, double>> point_vector;
        _points[topic_name] = point_vector;
    }
    _points[topic_name].push_back(temp_point);
}

void SerializeParkingInfo::add_extra_info(std::string key, std::string value) { _extra_info.push_back({key, value}); }

void SerializeParkingInfo::clear() {
    _points.clear();
    _extra_info.clear();
}

void SerializeParkingInfo::show_json_string(ParkingInfo::Ptr parking_info) {
    clear();

    auto current_loc = parking_info->park_env()->start_pose;
    add_points("park_ego_location", current_loc.x(), current_loc.y(), current_loc.theta());

    MLOG(PARKING_PLANNING, INFO) << "current_loc.x(): " << current_loc.x();
    MLOG(PARKING_PLANNING, INFO) << "current_loc.y(): " << current_loc.y();
    MLOG(PARKING_PLANNING, INFO) << "current_loc.theta(): " << current_loc.theta();

    auto target_loc = parking_info->park_env()->target_pose;
    add_points("target_pose", target_loc.x(), target_loc.y(), target_loc.theta());
    if (parking_info->obstacle_decider_result()->is_adjust_valid) {
        auto adjusted_pose = parking_info->obstacle_decider_result()->adjusted_pose;
        add_points("adjusted_pose", adjusted_pose.x(), adjusted_pose.y(), adjusted_pose.theta());
    }

    auto slot = parking_info->park_env()->slot;

    add_points("park_solt", slot.rear_left());
    add_points("park_solt", slot.rear_right());
    add_points("park_solt", slot.front_right());
    add_points("park_solt", slot.front_left());

    auto t_shape = parking_info->park_env()->t_shape;
    add_points("t_shape", t_shape.slot_left_edge.start());
    add_points("t_shape", t_shape.slot_left_edge.end());
    add_points("t_shape", t_shape.slot_right_edge.end());
    add_points("t_shape", t_shape.slot_right_edge.start());
    add_points("t_shape", t_shape.road_right_edge.start());
    add_points("t_shape", t_shape.upper_line.end());
    add_points("t_shape", t_shape.upper_line.start());
    add_points("t_shape", t_shape.road_left_edge.start());

    if (slot.has_limiter()) {
        add_points("park_slot_limiter", slot.limiter().left());
        add_points("park_slot_limiter", slot.limiter().right());
    }

    auto c_point = parking_info->park_env()->t_shape.c_point;
    add_points("rotate_down_C_point", c_point.x(), c_point.y(), 0.0);

    double rear_edge_to_center = VehicleConfigHelper::GetConfig().vehicle_param().rear_edge_to_center();
    double overalllength = VehicleConfigHelper::GetConfig().vehicle_param().overalllength();
    double overallwidth = VehicleConfigHelper::GetConfig().vehicle_param().overallwidth();
    double lat_inflation = (parking_info->park_env()->slot.park_type() == ParkType::VERTICAL ||
                            parking_info->park_env()->slot.park_type() == ParkType::OBLIQUE)
                               ? FLAGS_apa_vert_decider_lat_inflation
                               : FLAGS_apa_para_decider_lat_inflation;

    add_points("rear_edge_to_center", rear_edge_to_center, 0, 0);
    add_points("overalllength", overalllength, 0, 0);
    add_points("overallwidth", overallwidth, 0, 0);
    add_points("front_x_cut", FLAGS_apa_vehicle_model_front_x_cut, 0, 0);
    add_points("front_y_cut", FLAGS_apa_vehicle_model_front_y_cut, 0, 0);
    add_points("rear_x_cut", FLAGS_apa_vehicle_model_rear_x_cut, 0, 0);
    add_points("rear_y_cut", FLAGS_apa_vehicle_model_rear_y_cut, 0, 0);
    add_points("lat_inflation", lat_inflation, 0, 0);

    auto &debug_info = parking_info->mutable_park_output()->debug_info;

    if (debug_info.obstacle_points_size() > 0) {
        auto T_slot_world = (parking_info->park_env()->T_world_body * parking_info->park_env()->T_body_slot).inverse();
        for (int i = 0; i < debug_info.obstacle_points_size(); i++) {
            auto p_raw = debug_info.obstacle_points()[i];
            Vec2d p = Vec2d(p_raw.x(), p_raw.y());
            auto p_slot = MathHelper::transform_vec2d(p, T_slot_world, !parking_info->park_env()->right_park);
            add_points("path_collision_point", p_slot.x(), p_slot.y(), 0.0);
        }
    }

    auto obs_res = parking_info->mutable_obstacle_decider_result();
    if (debug_info.blocked_points_size() > 0) {
        add_points("blocked_point", obs_res->obs_point);
        // for (int i = 0; i < debug_info.blocked_points_size(); i++) {
        //     auto p_raw = debug_info.blocked_points()[i];
        //     Vec2d p = Vec2d(p_raw.x(), p_raw.y());
        //     auto p_slot = MathHelper::transform_vec2d(p, parking_info->park_env()->T_world_slot.inverse(), false);
        //     if (!parking_info->park_env()->right_park) {
        //         p_slot = Vec2d(-p_slot.x(), p_slot.y());
        //     }
        //     add_points("blocked_point", p_slot.x(), p_slot.y(), 0.0);
        // }
    }

    if (obs_res->obs_state == ObsState::HARD_BRAKE || obs_res->obs_state == ObsState::SOFT_BRAKE ||
        obs_res->obs_state == ObsState::SLOW_DOWN) {
        add_points("obs_point", obs_res->obs_point);
    }

    if (obs_res->relevant_obs.size() == 4) {
        for (auto p : obs_res->relevant_obs[0]) {
            add_points("obs_point_left", p);
        }

        for (auto p : obs_res->relevant_obs[1]) {
            add_points("obs_point_right", p);
        }

        for (auto p : obs_res->relevant_obs[2]) {
            add_points("obs_point_front", p);
        }

        for (auto p : obs_res->relevant_obs[3]) {
            add_points("obs_point_rear", p);
        }
        // add_extra_info("obs_point_left size", std::to_string(obs_res->relevant_obs[0].size()));
        // add_extra_info("obs_point_right size", std::to_string(obs_res->relevant_obs[1].size()));
        // add_extra_info("obs_point_front size", std::to_string(obs_res->relevant_obs[2].size()));
        // add_extra_info("obs_point_rear size", std::to_string(obs_res->relevant_obs[3].size()));
    }

    for (auto &p : parking_info->park_output()->vertical_planner_search_points) {
        add_points("search_points", p);
    }

    // add str info
    std::string planing_res = parking_info->global_path_info()->success ? "成功" : "失败";
    double lane_y = std::max(0.0, t_shape.slot_right_edge.start().y());
    double lane_width = std::abs(t_shape.upper_line.end().y() - lane_y);
    double slot_space = std::abs(t_shape.slot_left_edge.start().x() - t_shape.slot_right_edge.start().x());
    add_extra_info("t_shape_lane_width", std::to_string(lane_width));
    add_extra_info("t_shape_slot_space", std::to_string(slot_space));

    Pose2d start_to_target = parking_info->park_env()->target_pose.inverse() * parking_info->park_env()->start_pose;
    double error_x = fabs(start_to_target.y());
    double error_heading = fabs(start_to_target.theta());
    add_extra_info("error_x", std::to_string(error_x));
    add_extra_info("error_heading", std::to_string(error_heading));

    add_extra_info("failure_reson", debug_info.failure_reason());
    add_extra_info("is_active", std::to_string(parking_info->is_active()));
    add_extra_info("perception_stamp", std::to_string(parking_info->get_perception_stamp()));
    add_extra_info("path planning result", planing_res);
    add_extra_info("plan_method", parking_info->park_output()->debug_info.planner_method());
    add_extra_info("gear_switch_count", std::to_string(parking_info->global_path_info()->gear_switch_count));
    add_extra_info("total_length", std::to_string(parking_info->global_path_info()->total_length));

    add_extra_info("max_free_length", std::to_string(obs_res->max_free_length));

    auto park_output = parking_info->park_output();
    add_extra_info("obs_state", park_output->obs_state);

    add_extra_info("replan_trigger", std::to_string(parking_info->replan_data()->replan_trigger));
    add_extra_info("replan_reason", park_output->replan_reason);

    add_extra_info("planned_gear", std::to_string(parking_info->planned_gear()));
    add_extra_info("chassis_gear", std::to_string(parking_info->chassis_gear()));

    add_extra_info("parking_state", std::to_string(park_output->parking_state));

    add_extra_info("speed", std::to_string(parking_info->vehicle_speed()));

    add_extra_info("is_adjust_valid", std::to_string(parking_info->obstacle_decider_result()->is_adjust_valid));

    for (auto &path_point_vec : parking_info->global_path()) {
        for (const auto &p : path_point_vec) {
            add_points("global_path", p.x(), p.y(), p.theta());
        }
    }

    for (auto &raw_path_point_vec : parking_info->raw_global_path()) {
        for (const auto &p : raw_path_point_vec) {
            add_points("raw_global_path", p.x(), p.y(), p.theta());
        }
    }

    auto published_path = parking_info->published_path();
    for (auto &p : published_path) {
        add_points("published_path", p.x(), p.y(), p.theta());
    }

    for (auto &obs : parking_info->park_env()->obstacle_points) {
        add_points("park_park_obs", obs.x(), obs.y(), 0);
    }

    for (auto &line : parking_info->park_env()->obstacle_edges) {
        add_points("obstacle_edges", line.start());
        add_points("obstacle_edges", line.end());
    }
    for (auto &raw_obs_line : parking_info->park_env()->raw_obstacle_line_slot_vector) {
        for (auto &raw_obs : raw_obs_line->points()) {
            add_points("raw_obs", raw_obs.x(), raw_obs.y(), 0);
        }
    }

    std::string json_string = "json_start{";

    if (!_points.empty()) {
        std::map<std::string, std::vector<std::tuple<double, double, double>>>::iterator iter = _points.begin();
        while (iter != _points.end()) {
            json_string += ("\"" + iter->first + "\": [");
            int num_point = 0;
            for (uint i = 0; i < iter->second.size(); i++) {
                std::tuple<double, double, double> point = iter->second[i];
                std::string point_str = "";
                point_str = "[" + std::to_string(std::get<0>(point)) + "," + std::to_string(std::get<1>(point)) + "," +
                            std::to_string(std::get<2>(point)) + "]";
                json_string += point_str;
                if (i != iter->second.size() - 1) {
                    json_string += ",";
                }
                num_point++;
            }
            json_string += "]";
            ++iter;
            if (iter != _points.end()) {
                json_string += ",";
            }
        }
    }
    if (!_extra_info.empty()) {
        json_string += ",";
        auto iter = _extra_info.begin();
        uint i = 0;
        while (iter != _extra_info.end()) {
            json_string += ("\"" + iter->first + "\": \"" + iter->second + "\"");
            if (i != _extra_info.size() - 1) {
                json_string += ",";
            }
            ++i;
            ++iter;
        }
    }

    json_string += "}json_end";
    MLOG(PARKING_PLANNING, INFO) << json_string;
#ifdef BUILD_WITH_PARK_VISUALIZATION
    PlanningLogHelper::instance()->log_to_file(json_string);
#endif
}

void SerializeParkingInfo::show_json_string(PlanningInfo::Ptr planning_info) {
    auto current_loc = planning_info->plan_env()->start_pose;
    add_points("park_ego_location", current_loc.x(), current_loc.y(), current_loc.theta());
    auto target_loc = planning_info->plan_env()->target_pose;
    add_points("target_pose", target_loc.x(), target_loc.y(), target_loc.theta());

    auto slot = planning_info->plan_env()->slot;

    add_points("park_solt", slot.rear_left());
    add_points("park_solt", slot.rear_right());
    add_points("park_solt", slot.front_right());
    add_points("park_solt", slot.front_left());

    auto t_shape = planning_info->plan_env()->t_shape;
    add_points("t_shape", t_shape.slot_left_edge.start());
    add_points("t_shape", t_shape.slot_left_edge.end());
    add_points("t_shape", t_shape.slot_right_edge.end());
    add_points("t_shape", t_shape.slot_right_edge.start());
    add_points("t_shape", t_shape.road_right_edge.start());
    add_points("t_shape", t_shape.upper_line.end());
    add_points("t_shape", t_shape.upper_line.start());
    add_points("t_shape", t_shape.road_left_edge.start());

    if (slot.has_limiter()) {
        add_points("park_slot_limiter", slot.limiter().left());
        add_points("park_slot_limiter", slot.limiter().right());
    }

    auto c_point = planning_info->plan_env()->t_shape.c_point;
    add_points("rotate_down_C_point", c_point.x(), c_point.y(), 0.0);

    double rear_edge_to_center = VehicleConfigHelper::GetConfig().vehicle_param().rear_edge_to_center();
    double overalllength = VehicleConfigHelper::GetConfig().vehicle_param().overalllength();
    double overallwidth = VehicleConfigHelper::GetConfig().vehicle_param().overallwidth();
    double lat_inflation = (planning_info->plan_env()->slot.park_type() == ParkType::VERTICAL ||
                            planning_info->plan_env()->slot.park_type() == ParkType::OBLIQUE)
                               ? FLAGS_apa_vert_vehicle_lat_inflation
                               : FLAGS_apa_para_decider_lat_inflation;

    add_points("rear_edge_to_center", rear_edge_to_center, 0, 0);
    add_points("overalllength", overalllength, 0, 0);
    add_points("overallwidth", overallwidth, 0, 0);
    add_points("front_x_cut", FLAGS_apa_vehicle_model_front_x_cut, 0, 0);
    add_points("front_y_cut", FLAGS_apa_vehicle_model_front_y_cut, 0, 0);
    add_points("rear_x_cut", FLAGS_apa_vehicle_model_rear_x_cut, 0, 0);
    add_points("rear_y_cut", FLAGS_apa_vehicle_model_rear_y_cut, 0, 0);
    add_points("lat_inflation", lat_inflation, 0, 0);

    auto plan_debug = planning_info->path_plan_debug();
    for (auto &p : plan_debug->obstacle_points) {
        add_points("path_collision_point", p);
    }
    for (auto &p : plan_debug->vertical_planner_search_points) {
        add_points("search_points", p);
    }

    for (auto &path_point_vec : planning_info->global_path_info()->real_path) {
        for (const auto &p : path_point_vec) {
            add_points("global_path", p.x(), p.y(), p.theta());
        }
    }

    for (auto &obs : planning_info->plan_env()->obstacle_points) {
        add_points("park_park_obs", obs.x(), obs.y(), 0);
    }

    for (auto &line : planning_info->plan_env()->obstacle_edges) {
        add_points("obstacle_edges", line.start());
        add_points("obstacle_edges", line.end());
    }

    if (planning_info->plan_env()->is_candidate_valid && planning_info->plan_env()->candidate_pose.size() > 0) {
        auto adjusted_pose = planning_info->plan_env()->candidate_pose[0];
        add_points("adjusted_pose", adjusted_pose.x(), adjusted_pose.y(), adjusted_pose.theta());
    }

    // add str info
    std::string path_plan_type_str;
    switch (planning_info->path_plan_type()) {
    case PathPlanType::FIRST_PATH_PLAN:
        path_plan_type_str = "FIRST_PATH_PLAN";
        break;
    case PathPlanType::NORMAL_REPLAN:
        path_plan_type_str = "NORMAL_REPLAN";
        break;
    case PathPlanType::ROI_STATIC_REPLAN:
        path_plan_type_str = "ROI_STATIC_REPLAN";
        break;
    case PathPlanType::ROI_DYNAMIC_REPLAN:
        path_plan_type_str = "ROI_DYNAMIC_REPLAN";
        break;
    default:
        path_plan_type_str = "UNKNOWN_TYPE";
    }

    std::string slot_type = "UNKNOWN_TYPE";
    switch (planning_info->plan_env()->slot.park_type()) {
    case ParkType::VERTICAL:
        slot_type = "VERTICAL";
        break;
    case ParkType::PARALLEL:
        slot_type = "PARALLEL";
        break;
    case ParkType::OBLIQUE:
        slot_type = "OBLIQUE";
        break;
    case ParkType::HEADING:
        slot_type = "HEADING";
        break;
    default:
        slot_type = "UNKNOWN_TYPE";
    }

    std::string planing_res = planning_info->global_path_info()->success ? "成功" : "失败";
    double lane_y = std::max(0.0, t_shape.slot_right_edge.start().y());
    double lane_width = std::abs(t_shape.upper_line.end().y() - lane_y);
    double slot_space = std::abs(t_shape.slot_left_edge.start().x() - t_shape.slot_right_edge.start().x());

    add_extra_info("perception_stamp", std::to_string(planning_info->get_perception_stamp()));
    add_extra_info("path planning result", planing_res);
    add_extra_info("slot_type", slot_type);
    add_extra_info("path_plan_type", path_plan_type_str);
    add_extra_info("plan_method", plan_debug->planner_method);
    add_extra_info("failure_reson", plan_debug->failure_reason);
    add_extra_info("gear_switch_count", std::to_string(planning_info->global_path_info()->gear_switch_count));
    add_extra_info("total_length", std::to_string(planning_info->global_path_info()->total_length));
    add_extra_info("chassis_gear", std::to_string(planning_info->chassis_gear()));
    add_extra_info("t_shape_lane_width", std::to_string(lane_width));
    add_extra_info("t_shape_slot_space", std::to_string(slot_space));
    add_extra_info("is_adjust_valid", std::to_string(planning_info->plan_env()->is_candidate_valid));

    std::string json_string = "json_start{";

    if (!_points.empty()) {
        std::map<std::string, std::vector<std::tuple<double, double, double>>>::iterator iter = _points.begin();
        while (iter != _points.end()) {
            json_string += ("\"" + iter->first + "\": [");
            int num_point = 0;
            for (uint i = 0; i < iter->second.size(); i++) {
                std::tuple<double, double, double> point = iter->second[i];
                std::string point_str = "";
                point_str = "[" + std::to_string(std::get<0>(point)) + "," + std::to_string(std::get<1>(point)) + "," +
                            std::to_string(std::get<2>(point)) + "]";
                json_string += point_str;
                if (i != iter->second.size() - 1) {
                    json_string += ",";
                }
                num_point++;
            }
            json_string += "]";
            ++iter;
            if (iter != _points.end()) {
                json_string += ",";
            }
        }
    }
    if (!_extra_info.empty()) {
        json_string += ",";
        auto iter = _extra_info.begin();
        uint i = 0;
        while (iter != _extra_info.end()) {
            json_string += ("\"" + iter->first + "\": \"" + iter->second + "\"");
            if (i != _extra_info.size() - 1) {
                json_string += ",";
            }
            ++i;
            ++iter;
        }
    }

    json_string += "}json_end";
    MLOG(PARKING_PLANNING, INFO) << json_string;
#ifdef BUILD_WITH_PARK_VISUALIZATION
    PlanningLogHelper::instance()->log_to_file(json_string);
#endif
}

PLANNING_NAMESPACE_END
