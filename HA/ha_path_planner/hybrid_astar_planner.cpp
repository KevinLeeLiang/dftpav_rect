//
// Created by garen_lee on 2024/1/26.
/**
 ******************************************************************************
 * @file           : hybrid_astar_planner.cpp.cc
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/26
 ******************************************************************************
 */
//

#include "hybrid_astar_planner.h"
#include "../ha_config/ha_planning_config.h"
#include "utils.h"

PLANNING_NAMESPACE_START

static const double deg2rad = M_PI / 180.0;
static const double bc2ab = 1.8;
static const double hasearch_flat = 2;

HybridAStarPlanner::HybridAStarPlanner(const size_t max_node_num) {
    for (size_t i = 0; i < max_node_num; ++i) {
        nodes_vec_.emplace_back(std::make_shared<HybridAStarNode>());
    }
    nodes_size_ = 0;
    this->start_time_ = 0;
    reeds_shepp_path_ = std::make_shared<ReedSheppPath>();
    result_.clear();
    //  vehicle_model_ = VehicleModel(
    //      FLAGS_apa_vert_vehicle_lat_inflation,
    //      FLAGS_apa_vert_vehicle_lon_inflation, FLAGS_apa_vehicle_model_front_x_cut,
    //      FLAGS_apa_vehicle_model_front_y_cut, FLAGS_apa_vehicle_model_rear_x_cut,
    //      FLAGS_apa_vehicle_model_rear_y_cut);
    vehicle_model_ = VehicleModel(FLAGS_apa_vert_vehicle_lat_inflation, FLAGS_apa_vert_vehicle_lon_inflation);
    park_type_ = ParkType::VERTICAL;
    grid_map_ptr_ = std::make_shared<GridMap>();
    this->x_left_boun_ = -8;
    this->x_right_boun_ = 10;
    this->y_bot_boun_ = -5;

    delta_t_ = HAPlanningConfig::instance()->hybrid_astar_param().delta_t;
    max_delta_angle_ = HAPlanningConfig::instance()->max_delta_angle();
    wheel_base_ = VehicleConfigHelper::GetConfig().vehicle_param().wheelbase();
    front_edge_to_rear_ = VehicleConfigHelper::GetConfig().vehicle_param().front_edge_to_center();
    right_edge_to_center_ = VehicleConfigHelper::GetConfig().vehicle_param().overallwidth() / 2;
    back_edge_to_rear_ = VehicleConfigHelper::GetConfig().vehicle_param().rear_edge_to_center();
    left_edge_to_center_ = VehicleConfigHelper::GetConfig().vehicle_param().overallwidth() / 2;
    next_node_num_ = HAPlanningConfig::instance()->hybrid_astar_param().next_node_num; // default: 13
    max_zigzag_allowd_ = HAPlanningConfig::instance()->hybrid_astar_param().max_zigzag_allowd;
    // visualize
    verbose = HAPlanningConfig::instance()->hybrid_astar_param().verbose;
    display_points = HAPlanningConfig::instance()->hybrid_astar_param().display_points;
    reed_shepp_generator_ = std::make_unique<ReedShepp>();
}

HybridAStarPlanner::HybridAStarPlanner(const Pose2d &goal_pose, double goal_v, const Box2d &map_boundary,
                                       const std::vector<LineSegment2d> &map) {
    xy_grid_resolution_ = HAPlanningConfig::instance()->hybrid_astar_param().xy_grid_resolution;
    phi_grid_resolution_ = HAPlanningConfig::instance()->hybrid_astar_param().phi_grid_resolution;
    step_size_ = HAPlanningConfig::instance()->hybrid_astar_param().step_size;
    delta_t_ = HAPlanningConfig::instance()->hybrid_astar_param().delta_t;
    max_delta_angle_ = HAPlanningConfig::instance()->max_delta_angle();
    wheel_base_ = VehicleConfigHelper::GetConfig().vehicle_param().wheelbase();
    front_edge_to_rear_ = VehicleConfigHelper::GetConfig().vehicle_param().front_edge_to_center();
    right_edge_to_center_ = VehicleConfigHelper::GetConfig().vehicle_param().overallwidth() / 2;
    back_edge_to_rear_ = VehicleConfigHelper::GetConfig().vehicle_param().rear_edge_to_center();
    left_edge_to_center_ = VehicleConfigHelper::GetConfig().vehicle_param().overallwidth() / 2;
    next_node_num_ = HAPlanningConfig::instance()->hybrid_astar_param().next_node_num;
    // visualize
    verbose = HAPlanningConfig::instance()->hybrid_astar_param().verbose;
    display_points = HAPlanningConfig::instance()->hybrid_astar_param().display_points;

    park_type_ = ParkType::VERTICAL;

    // init local coordinate system
    Vec2d tmp_origin_corner = map_boundary.GetAllCorners().at(3);
    local_frame_pose_ = Pose2d(tmp_origin_corner.x(), tmp_origin_corner.y(), map_boundary.heading());

    x_bound_ = map_boundary.length();
    y_bound_ = map_boundary.width();

    nodes_size_ = 0;
    reeds_shepp_path_ = std::make_shared<ReedSheppPath>();
    reed_shepp_generator_ = std::make_unique<ReedShepp>();

    end_node_ = std::make_shared<HybridAStarNode>(goal_pose.x(), goal_pose.y(), goal_pose.theta(), goal_v, 0);

    const double &max_delta_angle_rear = HAPlanningConfig::instance()->max_delta_angle();
    if (HAPlanningConfig::instance()->car_param().enable_multiple_steer_modes) {
        double min_wheel_base_offset =
            wheel_base_ * std::tan(-max_delta_angle_rear * deg2rad) /
            (std::tan(max_delta_angle_ * deg2rad) - std::tan(-max_delta_angle_rear * deg2rad));
        wheel_base_offset_options_.push_back(min_wheel_base_offset);
        wheel_base_offset_options_.push_back(0);
        wheel_base_offset_options_.push_back(100.0 * wheel_base_);
    } else {
        wheel_base_offset_options_.push_back(0);
    }
    result_.clear();
    result_.status = HA::SBPStatus::EXCEPTION;
    max_zigzag_allowd_ = HAPlanningConfig::instance()->hybrid_astar_param().max_zigzag_allowd;
}

void HybridAStarPlanner::buildGridMap(const std::vector<LineSegment2d> &obs_lines, const std::vector<Vec2d> &obs_pts,
                                      const Box2d &map_boundary) {
    std::vector<Vec2d> obs_s;
    MLOG(PARKING_PLANNING, INFO) << "map_bound min_x," << map_boundary.min_x() << ",min_y," << map_boundary.min_y()
                                 << ",max_x," << map_boundary.max_x() << ",max_y," << map_boundary.max_y();
    MLOG(PARKING_PLANNING, INFO) << "center x," << map_boundary.center_x() << ",y," << map_boundary.center_y();
    auto isInMap = [](const Vec2d &p, const Box2d &map_bound) -> bool {
        return (p.x() > map_bound.min_x() && p.x() < map_bound.max_x()) &&
               (p.y() > map_bound.min_y() && p.y() < map_bound.max_y());
    };
    auto getPoint = [](double step, const LineSegment2d &line) -> Vec2d {
        return Vec2d(line.start().x() + step * line.cos_heading(), line.start().y() + step * line.sin_heading());
    };
    for (auto &line : obs_lines) {
        double step = 0.0;
        while (step < line.length()) {
            auto obs_pt = getPoint(step, line);
            if (isInMap(obs_pt, map_boundary)) {
                obs_s.push_back(obs_pt);
            }
            step = step + FLAGS_apa_ha_obstacle_grid_resolution;
        }
    }
    obs_s.insert(obs_s.end(), obs_pts.begin(), obs_pts.end());
    this->grid_map_ptr_->BuildGridMap(obs_s, map_boundary);
}

/**
 * @brief               - 更新HAPlanner的参数,如地图,search步长,网格分辨率等
 * @param goal_pose     - {Pose2d}终点位姿
 * @param goal_v        - {double}终点速度
 * @param map_boundary  - {const Box2d}地图的边界
 * @param map           - {const std::vector<LineSegment2d> &}线型地图(暂时不用)
 */
void HybridAStarPlanner::update(const Pose2d &goal_pose, double goal_v, const Box2d &map_boundary,
                                const std::vector<LineSegment2d> &map) {

    xy_grid_resolution_ = FLAGS_apa_ha_xy_grid_resolution;
    phi_grid_resolution_ = FLAGS_apa_ha_phi_grid_resolution;
    step_size_ = FLAGS_apa_ha_step_size;

    // init local coordinate system
    Vec2d tmp_origin_corner = map_boundary.GetAllCorners().at(3);
    local_frame_pose_ = Pose2d(tmp_origin_corner.x(), tmp_origin_corner.y(), map_boundary.heading());
    x_bound_ = map_boundary.length();
    y_bound_ = map_boundary.width();

    nodes_size_ = 0;

    const double &max_delta_angle_rear = HAPlanningConfig::instance()->max_delta_angle(); // 最大后轮转角
    wheel_base_offset_options_.clear();
    // 多轴转向
    if (HAPlanningConfig::instance()->car_param().enable_multiple_steer_modes) {
        double min_wheel_base_offset =
            wheel_base_ * std::tan(-max_delta_angle_rear * deg2rad) /
            (std::tan(max_delta_angle_ * deg2rad) - std::tan(-max_delta_angle_rear * deg2rad));
        wheel_base_offset_options_.push_back(min_wheel_base_offset);
        wheel_base_offset_options_.push_back(0);
        wheel_base_offset_options_.push_back(100.0 * wheel_base_);
    } else { // 前轮转向
        wheel_base_offset_options_.push_back(0);
    }
    // result_.clear();
    result_.status = SBPStatus::EXCEPTION;
    reeds_shepp_path_ = std::make_shared<ReedSheppPath>();
    reed_shepp_generator_.reset(new ReedShepp);
    this->y_up_boun_ = 7;
    this->y_bot_boun_ = -5;
}

/**
 * @brief               - 为规划器内部变量起点位姿状态赋值
 * @param start_pose    - {Pose2d start_pose} 赋值的起点位姿状态
 * @param start_vel     - {double}起点速度
 */
void HybridAStarPlanner::set_start_pose(const Pose2d &start_pose, double start_vel) {
    start_node_ = std::make_shared<HybridAStarNode>(start_pose.x(), start_pose.y(), start_pose.theta(), start_vel, 0);
    start_node_->zigzags = 0;
}

/**
 * @brief               - 为规划器内部变量终点位姿状态赋值
 * @param target_pose   - {Pose2d} 赋值的终点位姿状态
 */
void HybridAStarPlanner::set_target_pose(const Pose2d &target_pose) {
    end_node_ = std::make_shared<HybridAStarNode>(target_pose.x(), target_pose.y(), target_pose.theta(), 0, 0);
}

/**
 * @brief           - HA规划算法接口
 * @param obs_ptrs  - {const std::vector<SBPObstacleInterface::Ptr>
 * &}规划环境障碍物
 * @return          - {bool} 规划成功返回true否则返回false
 */
bool HybridAStarPlanner::plan(const std::vector<SBPObstacleInterface::Ptr> &obs_ptrs) {
    // std::cout << "HybridAstar::Plan is called" << std::endl;
    // int size_tmp = 0;
    MLOG(PARKING_PLANNING, INFO) << "HybridAStarPlanner Plan Start, input obs size: " << obs_ptrs.size();
    auto s_time = ::os::Time::now().to_usec();

    bool is_finial_plan = true;
    MLOG(PARKING_PLANNING, INFO) << "!!!!!!!!is finial plan:" << is_finial_plan;
    MLOG(PARKING_PLANNING, INFO) << "!!!!!!!! ";
    obs_ptrs_ = obs_ptrs;

    if (start_node_ == nullptr || end_node_ == nullptr) {
        MLOG(PARKING_PLANNING, INFO) << "HybridAStarPlanner::Start node or End "
                                        "nocheck_collision_realde not set ";
        result_.status = HA::SBPStatus::INFEASIBLE;
        return false;
    }

    if (check_collision(start_node_, false)) {
        MLOG(PARKING_PLANNING, INFO) << "HybridAStarPlanner::Plan failed for collision at start node";
        result_.status = HA::SBPStatus::INFEASIBLE;
        return false;
    }
    if (is_finial_plan && check_collision(end_node_, false)) {
        MLOG(PARKING_PLANNING, INFO) << "HybridAStarPlanner::Plan failed for collision at end node";
        result_.status = HA::SBPStatus::INFEASIBLE;
        return false;
    }
    MLOG(PARKING_PLANNING, INFO) << "this->y_up_boun_," << this->y_up_boun_ << ",this->y_bot_boun_,"
                                 << this->y_bot_boun_ << ",this->x_left_boun_," << this->x_left_boun_
                                 << ",this->x_right_boun_," << this->x_right_boun_;
    //    searchPoints_ = std::vector<Pose2d>();

    Compare::target = *end_node_;
    Compare::cmp_frame_pose_ = local_frame_pose_;
    Compare cmp(xy_grid_resolution_, HAPlanningConfig::instance()->hybrid_astar_param().grid_dijkstra_xy_resolution);

    ////////
    // 初始化的内部参数,最优队列采用的是无障碍物的启发和有障碍物启发的混合,加权合
    cmp.loadFrom(obs_ptrs_, x_bound_, y_bound_);
    cmp.run_dijkstra(x_bound_, y_bound_);
    ///

    std::priority_queue<SearchNode::Ptr, std::vector<SearchNode::Ptr>, Compare> pq; // priority queue
    pq.push(start_node_);


    std::unordered_map<size_t, SearchNode::Ptr> close_set_;
    std::unordered_map<size_t, SearchNode::Ptr> open_set_;
    open_set_.insert(std::make_pair(start_node_->get_index(), start_node_));

    uint32_t search_iter = 0;
    uint32_t search_iter2 = 0;
    int iter = 0;
    while (!pq.empty()) {
        uint64_t check_time = ::os::Time::now().to_usec();
        uint64_t plan_time = (check_time - start_time_);

        // if (iter > PlanningConfig::instance()->hybrid_astar_param().max_iter_max
        // || plan_time > FLAGS_apa_ha_search_time_max) {
        if (plan_time > FLAGS_apa_ha_search_time_max) {
            MLOG(PARKING_PLANNING, ERROR)
                << "hybrid astar exceeds over time vs max time:" << plan_time << " : " << FLAGS_apa_ha_search_time_max;
            result_.status = HA::SBPStatus::TIMEOUT;
            break;
        }
        SearchNode::Ptr current = pq.top();
        pq.pop();
        iter++;

        // for visualization
        if (verbose == VERBOSE_DEBUG) {
            if (display_points == BI_PRINT) {
                searchPoints_.emplace_back(current->x, current->y, current->theta);
            } else if (display_points == FORWARD_PRINT && current->vel > 0) {
                searchPoints_.emplace_back(current->x, current->y, current->theta);
            } else if (display_points == BACKWARD_PRINT && current->vel < 0) {
                searchPoints_.emplace_back(current->x, current->y, current->theta);
            }
        }
        if (current->previous && analytic_expansion(current, obs_ptrs)) {
            SearchNode::Ptr sbp_nodes_back = sbp_rspath_.get_nodes().back();
            combine_trajectory(&result_, sbp_nodes_back);
            MLOG(PARKING_PLANNING, INFO) << "Hybridastar reached with cost = " << current->trajcost
                                         << ", heuristic cost = " << current->heuristic_cost_;
            result_.status = HA::SBPStatus::SUCCESS;
            break;
        }
        search_iter2++;

        if (current->previous && current == this->end_node_ && !is_finial_plan) {
            MLOG(PARKING_PLANNING, INFO) << "已到达中继点,更新HA规划器终点";
            //            SearchNode::Ptr sbp_nodes_back = sbp_rspath_.get_nodes().back();
            combine_trajectory(&result_, current);
            MLOG(PARKING_PLANNING, INFO) << "Hybridastar reached with cost = " << current->trajcost
                                         << ", heuristic cost = " << current->heuristic_cost_;
            result_.status = HA::SBPStatus::SUCCESS;
            std::vector<Pose2d> tmp;
            for (SearchNode::Ptr Dummy = current; Dummy; Dummy = Dummy->previous) {
                double x_ref = Dummy->x + Dummy->wheel_base_offset_ * std::cos(Dummy->theta);
                double y_ref = Dummy->y + Dummy->wheel_base_offset_ * std::sin(Dummy->theta);
                tmp.push_back(Pose2d(x_ref, y_ref, Utils::normalize_angle(Dummy->theta)));
            }
            std::reverse(tmp.begin(), tmp.end());
            this->searchPath_.insert(searchPath_.end(), tmp.begin(), tmp.end());
            break;
        }

        if (close_set_.find(current->get_index()) != close_set_.end()) {
            MLOG(PARKING_PLANNING, INFO) << "????";
            continue;
        }

        close_set_.insert(std::make_pair(current->get_index(), current));

        std::vector<SearchNode::Ptr> nexts = get_next_states(current);
        for (const auto &next_node : nexts) {
            if (next_node->x > this->x_right_boun_ || next_node->x < this->x_left_boun_ ||
                next_node->y < this->y_bot_boun_ || next_node->y > this->y_up_boun_) {
                continue;
            }
            if (!check_collision(next_node)) {
                search_iter++;
                if (close_set_.find(next_node->get_index()) != close_set_.end()) {
                    continue;
                }
                if (open_set_.find(next_node->get_index()) == open_set_.end()) {
                    // next_node->side_diff = CalcSideDiff(next_node);
                    next_node->set_traj_cost();

                    next_node->obs_cost = current->obs_cost;
                    if (!reed_shepp_generator_->ShortestRSP(next_node, end_node_, reeds_shepp_path_)) {
                        continue;
                    } else {
                        sbp_rspath_.update(next_node, reeds_shepp_path_);
                        next_node->heuristic_cost_ = sbp_rspath_.get_cost(obs_ptrs_, vehicle_model_);
                    }
                    pq.push(next_node);
                    open_set_.insert(std::make_pair(next_node->get_index(), next_node));
                }
            } else {
            }
        }
    }

    if (pq.empty()) {
        MLOG(PARKING_PLANNING, INFO) << "pq.empty, iter, " << iter;

        result_.status = SBPStatus::INFEASIBLE;
    }
    MLOG(PARKING_PLANNING, INFO) << "HybridAStarPlanner check_collision num = " << search_iter
                                 << ", analytic_expansion num = " << search_iter2;
    result_.iteration_times = iter;
    result_.debug_string = "iteration_times = " + std::to_string(iter);
    // double chrono_ms = 0;
    // result_.debug_string += ", time_per_iter = " +
    //                         std::to_string(chrono_ms /
    //                         result_.iteration_times);
    MLOG(PARKING_PLANNING, INFO) << "HybridAStarPlanner Plan run time: "
                                 << (::os::Time::now().to_usec() - s_time) * 1e-3 << "ms";

    return result_.status == HA::SBPStatus::SUCCESS;
}

/**
 * @brei            - 将search结果和当前node进行结合拼接,得到最终结果到result
 * @param result    - {SBPResult *}HA算法的最终结果
 * @param current   - {const SearchNode::Ptr}当前的搜索节点
 */
void HybridAStarPlanner::combine_trajectory(SBPResult *result, const SearchNode::Ptr &current) {
    //    result->clear();
    SBPResult tmp;
    double x_ref = 0.0;
    double y_ref = 0.0;
    for (SearchNode::Ptr Dummy = current; Dummy; Dummy = Dummy->previous) {
        x_ref = Dummy->x + Dummy->wheel_base_offset_ * std::cos(Dummy->theta);
        y_ref = Dummy->y + Dummy->wheel_base_offset_ * std::sin(Dummy->theta);
        tmp.x.push_back(x_ref);
        tmp.y.push_back(y_ref);
        tmp.phi.push_back(Utils::normalize_angle(Dummy->theta));
        tmp.wheel_base_offset.push_back(Dummy->wheel_base_offset_);
        tmp.v.push_back(Dummy->vel);
        tmp.steer.push_back(Dummy->kappa);
    }
    std::reverse(tmp.x.begin(), tmp.x.end());
    std::reverse(tmp.y.begin(), tmp.y.end());
    std::reverse(tmp.phi.begin(), tmp.phi.end());
    std::reverse(tmp.wheel_base_offset.begin(), tmp.wheel_base_offset.end());
    std::reverse(tmp.v.begin(), tmp.v.end());
    std::reverse(tmp.steer.begin(), tmp.steer.end());
    result->x.insert(result->x.end(), tmp.x.begin(), tmp.x.end());
    result->y.insert(result->y.end(), tmp.y.begin(), tmp.y.end());
    result->phi.insert(result->phi.end(), tmp.phi.begin(), tmp.phi.end());
    result->wheel_base_offset.insert(result->wheel_base_offset.end(), tmp.wheel_base_offset.begin(),
                                     tmp.wheel_base_offset.end());
    result->v.insert(result->v.end(), tmp.v.begin(), tmp.v.end());
    result->steer.insert(result->steer.end(), tmp.steer.begin(), tmp.steer.end());
}

/**
 * @brief               - 检测当前node是否发生碰撞
 * @param current_state - {const SearchNode::Ptr &}当前node
 * @return              - {bool}如果发生碰撞返回true,否则返回false
 */
bool HybridAStarPlanner::check_collision_real(const SearchNode::Ptr &current_state) {
    // 这里是把当前searchnode的车辆角点 转换到map_boundary，
    // 如果超出范围就判定是collision

    if (this->CheckInBound(current_state))
        return true;

    for (const auto &obs_ptr : obs_ptrs_) {
        if (obs_ptr->check_collision(current_state, vehicle_model_)) {
            return true;
        }
    }
    return false;
}

/**
 * @brief                   - 检查节点是否在地图内
 * @param current_state     - {const SearchNode::Ptr &}被检查的节点
 * @return                  - {bool} true在地图内,false不在地图内
 */
bool HybridAStarPlanner::CheckInBound(const SearchNode::Ptr &current_state) {
    VehicleModel tmp = this->vehicle_model_;
    tmp.update_pose(Pose2d(current_state->x, current_state->y, current_state->theta));
    for (size_t i = 0; i < tmp.corners().size(); i++) {
        Vec2d cor_2d = tmp.corners().at(i);
        Pose2d cur_2D = Utils::tf2d(local_frame_pose_, Pose2d(cor_2d.x(), cor_2d.y(), current_state->theta));
        if (cur_2D.x() < 0 || cur_2D.x() >= x_bound_ || cur_2D.y() < 0 || cur_2D.y() >= y_bound_) {
            return true;
        }
    }
    return false;
}

/**
 * @brief               - 检测当前node是否发生碰撞
 * @param current_state - {const SearchNode::Ptr &}当前node
 * @return              - {bool}如果发生碰撞返回true,否则返回false
 */
bool HybridAStarPlanner::check_collision(const SearchNode::Ptr &current_state) {
    static SearchNode::Ptr tmp_check_node = std::make_shared<HybridAStarNode>();
    int gear = (current_state->vel < 0.0) ? -1 : 1;
    const SearchNode &pos = *current_state;

    if (pos.previous != nullptr) {
        const SearchNode &previous_pos = *pos.previous;
        int previous_gear = (previous_pos.vel < 0.0) ? -1 : 1;
        if (previous_gear != gear && previous_pos.previous) {
            double x_previous_safe = previous_pos.x + previous_gear *
                                                          HAPlanningConfig::instance()->car_param().lon_inflation_max *
                                                          cos(previous_pos.theta);
            double y_previous_safe = previous_pos.y + previous_gear *
                                                          HAPlanningConfig::instance()->car_param().lon_inflation_max *
                                                          sin(previous_pos.theta);
            tmp_check_node->set_node(x_previous_safe, y_previous_safe, previous_pos.theta);
            if (FLAGS_apa_ha_check_collision_method == 0) {
                for (const auto &obs_ptr : obs_ptrs_) {
                    if (obs_ptr->check_collision(tmp_check_node, vehicle_model_)) {
                        return true;
                    }
                }
            } else if (FLAGS_apa_ha_check_collision_method == 1) {
                if (this->grid_map_ptr_->configurationTest(tmp_check_node->x, tmp_check_node->y,
                                                           tmp_check_node->theta)) {
                    return true;
                }
            }
        }
    }
    if (FLAGS_apa_ha_check_collision_method == 0) {
        for (const auto &obs_ptr : obs_ptrs_) {
            if (obs_ptr->check_collision(current_state, vehicle_model_)) {
                return true;
            }
        }
    } else if (FLAGS_apa_ha_check_collision_method == 1) {
        if (this->grid_map_ptr_->configurationTest(current_state->x, current_state->y, current_state->theta)) {
            return true;
        }
    }
    return false;
}

/**
 * @brief                   - 搜索节点的碰撞检测
 * @param current_state     - {const SearchNode::Ptr &}被检测的搜索节点
 * @param test              - {bool} 是否进行可视化log打印
 * @return                  - {bool} true发生碰撞,false不发生碰撞
 */
bool HybridAStarPlanner::check_collision(const SearchNode::Ptr &current_state, const bool test) {
    static SearchNode::Ptr tmp_check_node = std::make_shared<HybridAStarNode>();
    int gear = (current_state->vel < 0.0) ? -1 : 1;
    const SearchNode &pos = *current_state;

    if (pos.previous != nullptr) {
        const SearchNode &previous_pos = *pos.previous;
        int previous_gear = (previous_pos.vel < 0.0) ? -1 : 1;
        if (previous_gear != gear && previous_pos.previous) {
            double x_previous_safe = previous_pos.x + previous_gear *
                                                          HAPlanningConfig::instance()->car_param().lon_inflation_max *
                                                          cos(previous_pos.theta);
            double y_previous_safe = previous_pos.y + previous_gear *
                                                          HAPlanningConfig::instance()->car_param().lon_inflation_max *
                                                          sin(previous_pos.theta);
            tmp_check_node->set_node(x_previous_safe, y_previous_safe, previous_pos.theta);
            if (FLAGS_apa_ha_check_collision_method == 0) {
                for (const auto &obs_ptr : obs_ptrs_) {
                    if (obs_ptr->check_collision(tmp_check_node, vehicle_model_)) {

                        return true;
                    }
                }
            } else if (FLAGS_apa_ha_check_collision_method == 1) {
                if (this->grid_map_ptr_->configurationTest(tmp_check_node->x, tmp_check_node->y, tmp_check_node->theta),
                    test) {
                    return true;
                }
            }
        }
    }
    if (FLAGS_apa_ha_check_collision_method == 0) {
        for (const auto &obs_ptr : obs_ptrs_) {
            if (obs_ptr->check_collision(current_state, vehicle_model_)) {
                return true;
            }
        }
    } else if (FLAGS_apa_ha_check_collision_method == 1) {
        if (this->grid_map_ptr_->configurationTest(current_state->x, current_state->y, current_state->theta, test)) {
            return true;
        }
    }
    return false;
}

/**
 * @breif                   - 使用rs曲线链接当前的节点current_node和end_node,连上后碰撞检测通过返回true
 * @param current_node      - {const SearchNode::Ptr &}搜索的当前节点
 * @param end_node          - {const SearchNode::Ptr &}本次HA规划的终点节点
 * @param obs_ptrs          - {const std::vector<SBPObstacleInterface::Ptr> &} 障碍物数据
 * @return                  - {bool} 用rs曲线链接当前搜索节点current_node到本次HA规划的终点节点end_node且不碰装,返回true
 */
bool HybridAStarPlanner::analytic_expansion(const SearchNode::Ptr &current_node, const SearchNode::Ptr &end_node,
                                            const std::vector<SBPObstacleInterface::Ptr> &obs_ptrs) {
    //使用RS曲线连接current_node和end_node

    bool flag = !reed_shepp_generator_->ShortestRSP(current_node, end_node, reeds_shepp_path_,
                                                    ReedShepp::Param(true, true, true, false, false, false));
    if (flag) {

        // std::cout << "AnalyticExpansion failed" << std::endl;
        return false;
    }

    // prefer small curvature
    //    double radius = calc_circular_radius(current_node, end_node);
    //    radius -= 0.1; // 转弯半径范围[min_r+0.1, min_r *2]
    //    bool small_kappa_rs_succeed = false;
    //    if (radius > VehicleConfigHelper::Instance()->GetConfig().vehicle_param().minturningradius() &&
    //        radius < VehicleConfigHelper::Instance()->GetConfig().vehicle_param().minturningradius() * 2) {
    //        //    ReedShepp small_kappa_rs_generator(1.0 / radius, step_size_);
    //        ReedShepp small_kappa_rs_generator;
    //        small_kappa_rs_succeed = small_kappa_rs_generator.ShortestRSP(
    //            current_node, end_node, reeds_shepp_path_, ReedShepp::Param(true, true, true, true, true, true));
    //    }
    //    // 当前设置 max_analytic_expansion_length =-1, 不考虑连接RS的路径长度
    //    if (PlanningConfig::instance()->hybrid_astar_param().max_analytic_expansion_length > 0 &&
    //        reeds_shepp_path_->total_length >
    //        PlanningConfig::instance()->hybrid_astar_param().max_analytic_expansion_length) {
    //        return false;
    //    }

    int zigzag_rs = 0;
    if ((current_node->vel > 0) != reeds_shepp_path_->gear[0]) {
        zigzag_rs += 1;
    }
    for (size_t i = 1; i < reeds_shepp_path_->gear.size(); ++i) {
        if (reeds_shepp_path_->gear[i] != reeds_shepp_path_->gear[i - 1]) {
            zigzag_rs += 1;
        }
    }
    // if(end_node_->vel * (reeds_shepp_path_->gear.back()? 1:-1) < 0) {
    //   zigzag_rs += 1;
    // }
    bool is_analytic_expantion_zigzag =
        !HAPlanningConfig::instance()->hybrid_astar_param().enable_analytic_expansion && zigzag_rs > 0;
    bool is_zigzag_too_much =
        is_analytic_expantion_zigzag ||
        zigzag_rs + current_node->zigzags > HAPlanningConfig::instance()->hybrid_astar_param().max_zigzag_allowd;

    sbp_rspath_.update(current_node, reeds_shepp_path_);
    auto nodes = sbp_rspath_.get_nodes();
    for (const auto &node : nodes) {
        if (is_node_out_of_range(node)) {
            return false;
        }
    }
    if (FLAGS_apa_ha_check_collision_method == 0) {
        flag = !sbp_rspath_.check_collision(step_size_, obs_ptrs, vehicle_model_);
    } else if (FLAGS_apa_ha_check_collision_method == 1) {
        flag = !sbp_rspath_.check_collision(step_size_, this->grid_map_ptr_);
    }

    if (!is_zigzag_too_much && flag) {
        reeds_shepp_to_end_ = *reeds_shepp_path_;
        std::vector<Pose2d> tmp;
        for (SearchNode::Ptr Dummy = current_node; Dummy; Dummy = Dummy->previous) {
            double x_ref = Dummy->x + Dummy->wheel_base_offset_ * std::cos(Dummy->theta);
            double y_ref = Dummy->y + Dummy->wheel_base_offset_ * std::sin(Dummy->theta);
            tmp.push_back(Pose2d(x_ref, y_ref, Utils::normalize_angle(Dummy->theta)));
        }
        std::reverse(tmp.begin(), tmp.end());
        this->searchPath_.insert(searchPath_.end(), tmp.begin(), tmp.end());
        return true;
    }
    return false;
}

/**
 * @brief               - 当前节点是否可以通过rs曲线直接到达终点且无碰撞
 * @param current_node  - {const SearchNode::Ptr&}当前node
 * @param obs_ptrs      - {const std::vector<SBPObstacleInterface::Ptr>&}障碍物队列
 * @return              - {bool} 如果能直接安全到达终点返回true,否则返回false
 */
bool HybridAStarPlanner::analytic_expansion(const SearchNode::Ptr &current_node,
                                            const std::vector<SBPObstacleInterface::Ptr> &obs_ptrs) {
    //使用RS曲线连接current_node和end_node

    bool flag = !reed_shepp_generator_->ShortestRSP(current_node, end_node_, reeds_shepp_path_,
                                                    ReedShepp::Param(true, true, true, true, true, true));
    if (flag) {

        // std::cout << "AnalyticExpansion failed" << std::endl;
        return false;
    }

    // prefer small curvature
    double radius = calc_circular_radius(current_node, end_node_);
    radius -= 0.1; // 转弯半径范围[min_r+0.1, min_r *2]
    bool small_kappa_rs_succeed = false;
    if (radius > VehicleConfigHelper::Instance()->GetConfig().vehicle_param().minturningradius() &&
        radius < VehicleConfigHelper::Instance()->GetConfig().vehicle_param().minturningradius() * 2) {
        //    ReedShepp small_kappa_rs_generator(1.0 / radius, step_size_);
        ReedShepp small_kappa_rs_generator;
        small_kappa_rs_succeed = small_kappa_rs_generator.ShortestRSP(
            current_node, end_node_, reeds_shepp_path_, ReedShepp::Param(true, true, true, true, true, true));
    };

    // 当前设置 max_analytic_expansion_length =-1, 不考虑连接RS的路径长度
    if (HAPlanningConfig::instance()->hybrid_astar_param().max_analytic_expansion_length > 0 &&
        reeds_shepp_path_->total_length >
            HAPlanningConfig::instance()->hybrid_astar_param().max_analytic_expansion_length) {
        return false;
    }

    int zigzag_rs = 0;
    if ((current_node->vel > 0) != reeds_shepp_path_->gear[0]) {
        zigzag_rs += 1;
    }
    for (size_t i = 1; i < reeds_shepp_path_->gear.size(); ++i) {
        if (reeds_shepp_path_->gear[i] != reeds_shepp_path_->gear[i - 1]) {
            zigzag_rs += 1;
        }
    }

    bool is_analytic_expantion_zigzag =
        !HAPlanningConfig::instance()->hybrid_astar_param().enable_analytic_expansion && zigzag_rs > 0;
    bool is_zigzag_too_much = is_analytic_expantion_zigzag || zigzag_rs + current_node->zigzags > max_zigzag_allowd_;
    sbp_rspath_.update(current_node, reeds_shepp_path_);
    auto nodes = sbp_rspath_.get_nodes();
    for (const auto &node : nodes) {
        if (is_node_out_of_range(node)) {
            MLOG(PARKING_PLANNING, INFO) << "is node out of range";
            return false;
        }
    }
    if (FLAGS_apa_ha_check_collision_method == 0) {
        flag = !sbp_rspath_.check_collision(step_size_, obs_ptrs, vehicle_model_);
    } else if (FLAGS_apa_ha_check_collision_method == 1) {
        flag = !sbp_rspath_.check_collision(step_size_, this->grid_map_ptr_);
    }

    if (!is_zigzag_too_much && flag) {
        reeds_shepp_to_end_ = *reeds_shepp_path_;
        if (small_kappa_rs_succeed) {
            // std::cout << "small_kappa_rs_succeed ";
        }
        return true;
    }
    return false;
}

/**
 * @brief           - 从当前node计算下一个状态空间节点队列
 * @param current   - {const SearchNode::Ptr &}当前搜索节点
 * @return          - {std::vector<SearchNode::Ptr>}当前节点可达的下一状态空间节点队列
 */

std::vector<SearchNode::Ptr> HybridAStarPlanner::get_next_states(const SearchNode::Ptr &current) {
    std::vector<SearchNode::Ptr> next;
    double next_x = 0.0;
    double next_y = 0.0;
    double next_theta = 0.0;
    double alpha = 0.0;
    double beta = 0.0;
    double R = 0.0;

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double delta_step_size = step_size_ * 2.0;

    const double &max_delta_angle_rear = HAPlanningConfig::instance()->max_delta_angle();

    double max_wheel_base_offset = wheel_base_ * std::tan(max_delta_angle_rear * deg2rad) /
                                   (std::tan(max_delta_angle_ * deg2rad) - std::tan(max_delta_angle_rear * deg2rad));

    for (size_t i = 0; i < wheel_base_offset_options_.size(); ++i) {
        double wheel_base_offset = wheel_base_offset_options_[i];
        if (current->vel != 0 && current->wheel_base_offset_ <= 0 && wheel_base_offset > 0) {
            continue;
        }
        double max_delta_angle_apply = max_delta_angle_;
        if (wheel_base_offset > max_wheel_base_offset) {
            max_delta_angle_apply = max_delta_angle_rear;
        }

        if (next_node_num_ % 2 == 0 || next_node_num_ < 2) {
            // throw std::invalid_argument("invalid next_node_num!");
        }
        double initial_travel_step = -step_size_;
        double terminal_travel_step = step_size_;
        if (HAPlanningConfig::instance()->hybrid_astar_param().step_direction == -1) {
            terminal_travel_step = 0;
        } else if (HAPlanningConfig::instance()->hybrid_astar_param().step_direction == 1) {
            initial_travel_step = step_size_;
        }
        for (double traveled_distance = initial_travel_step; traveled_distance < terminal_travel_step + 0.0001;
             traveled_distance += delta_step_size) {
            if (traveled_distance * current->vel > 0) {
                if (wheel_base_offset != current->wheel_base_offset_) {
                    continue;
                }
            }

            double max_delta_rate_equivalent = HAPlanningConfig::instance()->max_delta_angle() *
                                               (wheel_base_ + wheel_base_offset) * std::cos(current->delta * deg2rad) *
                                               std::cos(current->delta * deg2rad) / wheel_base_;

            double max_delta_rate_apply =
                std::min(HAPlanningConfig::instance()->max_delta_angle(), max_delta_rate_equivalent);
            double steer_lower = std::max(current->delta - max_delta_rate_apply, -max_delta_angle_apply);
            double steer_upper = std::min(current->delta + max_delta_rate_apply, max_delta_angle_apply);

            if (traveled_distance * current->vel < 0) {
                steer_lower = -max_delta_angle_apply;
                steer_upper = max_delta_angle_apply;
            }
            // next_node_num_ 默认值是13
            double delta_step = (steer_upper - steer_lower) / (double)(next_node_num_ - 1);
            if (delta_step <= 1e-6) {
                // throw std::invalid_argument("invalid steer limits!");
            }

            // transform the state w.r.t virtual wheel base
            x = current->x - wheel_base_offset * std::cos(current->theta);
            y = current->y - wheel_base_offset * std::sin(current->theta);
            theta = current->theta;

            for (alpha = steer_lower; alpha < steer_upper + 0.001; alpha += delta_step) {
                // MLOG(PARKING_PLANNING, INFO)<< "alpha " << alpha;
                if (std::abs(alpha) < 1e-6) {
                    alpha = alpha > 0 ? 1e-6 : -1e-6;
                }
                R = (wheel_base_ + wheel_base_offset) / std::tan(alpha * deg2rad);
                if (R == 0) {
                    MLOG(PARKING_PLANNING, ERROR) << "[HyAstar calc error] R == 0";
                    continue;
                }

                // MLOG(PARKING_PLANNING, INFO)<< "R " << R;
                beta = traveled_distance / R;
                next_theta = Utils::normalize_angle(theta + beta);
                next_x = x + R * (std::cos(theta) * std::sin(beta) - std::sin(theta) * (1 - std::cos(beta)));
                next_y = y + R * (std::sin(theta) * std::sin(beta) + std::cos(theta) * (1 - std::cos(beta)));

                // transform the state w.r.t virtual wheel base back
                next_x += wheel_base_offset * std::cos(next_theta);
                next_y += wheel_base_offset * std::sin(next_theta);

                if (nodes_size_ < nodes_vec_.size()) {
                    nodes_vec_[nodes_size_]->set_node(next_x, next_y, next_theta, traveled_distance, alpha, 1 / R,
                                                      wheel_base_offset);
                } else {
                    nodes_vec_.emplace_back(std::shared_ptr<HybridAStarNode>(new HybridAStarNode(
                        next_x, next_y, next_theta, traveled_distance, alpha, 1 / R, wheel_base_offset)));
                }
                nodes_vec_[nodes_size_]->previous = current;
                nodes_vec_[nodes_size_]->zigzags = current->zigzags;
                if (traveled_distance * current->vel < 0) {
                    nodes_vec_[nodes_size_]->zigzags = current->zigzags + 1;
                }

                if (nodes_vec_[nodes_size_]->zigzags <= max_zigzag_allowd_) {
                    next.emplace_back(nodes_vec_[nodes_size_]);
                    nodes_size_++;
                }
            }
        }
    }
    return next;
}

/**
 * @brief   - 获取规划器内部变量HA search过的点,用于可视化debug
 * @return  - {std::vector<Pose2d>}规划器内部变量HA search过的点
 */
std::vector<Pose2d> HybridAStarPlanner::get_search_points() { return searchPoints_; }

/**
 * @brief   - 获取规划器内部变量规划结果变量
 * @return  - {SBPResult} 规划器内部变量规划结果变量
 */
SBPResult HybridAStarPlanner::get_result() { return result_; }

/**
 * @brief               - 节点是否超出了地图边界,这关系到obs的启发式搜索是否越界
 * @param current_node  - {const SearchNode::Ptr&}当前node
 * @return              - {bool}如果超出来地图边界返回true,否则返回false
 */
bool HybridAStarPlanner::is_node_out_of_range(const SearchNode::Ptr &current_node) {
    Pose2d cur_2D = Utils::tf2d(local_frame_pose_, Pose2d(current_node->x, current_node->y, current_node->theta));
    if (cur_2D.x() < 0 || cur_2D.x() >= x_bound_ || cur_2D.y() < 0 || cur_2D.y() >= y_bound_) {
        return true;
    }
    return false;
}

double HybridAStarPlanner::calc_circular_radius(const SearchNode::Ptr &node_a, const SearchNode::Ptr &node_b) {
    double turn_radius = 1e6;
    double diff_theta = Utils::normalize_angle(node_b->theta - node_a->theta);
    // double average_theta = std::atan2(std::sin(node_b->theta) +
    // std::sin(node_a->theta), std::cos(node_b->theta) +
    // std::cos(node_a->theta)); Vec2d diff_vec(node_b->x - node_a->x, node_b->y -
    // node_a->y);
    double denominator =
        std::cos(node_a->theta) * std::sin(node_b->theta) - std::sin(node_a->theta) * std::cos(node_b->theta);
    // if(std::abs(std::cos(diff_vec.Angle() - average_theta)) < std::cos(1e-2))
    // {
    //   return false;
    // }

    if (std::abs(denominator) < 1e-6) {
        return turn_radius;
    }
    double length_a =
        ((node_b->x - node_a->x) * std::sin(node_b->theta) - (node_b->y - node_a->y) * std::cos(node_b->theta)) /
        denominator;
    double length_b =
        ((node_b->x - node_a->x) * std::sin(node_a->theta) - (node_b->y - node_a->y) * std::cos(node_a->theta)) /
        denominator;

    // if(std::abs(diff_theta) > M_PI_2)
    // {
    //   return false;
    // }

    if (length_a * length_b > 0) {
        return turn_radius;
    }
    // double sin_diff_2 = std::sin(diff_theta / 2);
    // double turn_radius = std::abs(sin_diff_2) < 1e-6 ? 1e6 :
    // std::hypot(node_b->x - node_a->x, node_b->y - node_a->y) / 2 / sin_diff_2;
    turn_radius = std::min(std::abs(length_a), std::abs(length_b)) * std::abs(std::tan(diff_theta / 2));
    return std::abs(turn_radius);
}

bool HybridAStarPlanner::Plan(const ParkEnvironment::ConstPtr &park_env_ptr, Path::Ptr &path) {
    this->set_start_pose(park_env_ptr->start_pose);
    this->set_target_pose(park_env_ptr->target_pose);
    return true;
}

/**
 * @brief           - 将SBPResult的结果封装到Path::Ptr中
 * @param ha_res    - {SBPResult *}ha search的结果
 * @param path      - {Path::Ptr &}HA Planner输出结果
 */
void HybridAStarPlanner::SetResult(SBPResult *ha_res, Path::Ptr &path) {
    MLOG(PARKING_PLANNING, INFO) << "HybridAStarPlanner::SetResult";
    int v_direct = 1;
    Pose2d p_last;
    if (ha_res->x.size() <= 1) {
        path->success = false;
        path->total_length = 0;
        path->gear_switch_count = 0;
        p_last = Pose2d(0, 0, 0);
        return;
    } else {
        path->success = true;
        p_last = Pose2d(ha_res->x.at(0), ha_res->y.at(0), ha_res->phi.at(0));

        if (ha_res->v.at(0) == 0)
            v_direct = ha_res->v.at(1) / fabs(ha_res->v.at(1));
        else {
            v_direct = ha_res->v.at(0) / fabs(ha_res->v.at(0));
        }
    }
    PathSegment s;
    s.start_pose = p_last;
    double seg_length = 0;
    path->gear_switch_count = path->path_segments.size();
    for (size_t i = 0; i < ha_res->x.size(); i++) {
        double v = ha_res->v.at(i);
        MLOG(PARKING_PLANNING, INFO) << "v," << v << ",x," << ha_res->x.at(i) << ",y," << ha_res->y.at(i);
        PathPoint tmp;
        if (i != 0 &&
            (((v * v_direct) < 0 && i != ha_res->x.size() - 1) || i == ha_res->x.size() - 1)) { // 要么换档,要么到最后
            s.path_type = PathType::SMOOTH;
            if (v_direct >= 0 && i != ha_res->x.size() - 1) {
                s.gear = GearBoxInfoPb::GEAR_DRIVE;
            } else {
                if (i == ha_res->x.size() - 1 ) {
                    if (i >= 1){
                        if (ha_res->v.at(i - 1) < 0) {
                            s.gear = GearBoxInfoPb::GEAR_REVERSE;
                        } else {
                            s.gear = GearBoxInfoPb::GEAR_DRIVE;
                        }
                    }else {
                        s.gear = GearBoxInfoPb::GEAR_DRIVE;
                    }

                }else {
                    if (v_direct >= 0){
                        s.gear = GearBoxInfoPb::GEAR_DRIVE;
                    } else {
                        s.gear = GearBoxInfoPb::GEAR_REVERSE;
                    }
                }
            }
            if ((v * v_direct) < 0 && i != ha_res->x.size() - 1) { // 换档
                // 现将线段加入到segments再将前一个点加入到这一段
                s.length = seg_length;
                s.target_pose =  Pose2d(ha_res->x.at(i - 1), ha_res->y.at(i - 1), ha_res->phi.at(i - 1));
                path->gear_switch_count++;
                path->path_segments.push_back(s);
                //将前一个点和这个点都加进来
                s.path_points.clear();
                seg_length = 0;
                s.start_pose = Pose2d(ha_res->x.at(i - 1), ha_res->y.at(i - 1), ha_res->phi.at(i - 1));
                tmp.set_s(0);
                v_direct = int(v / fabs(v));
                tmp.set_x(ha_res->x.at(i - 1));
                tmp.set_y(ha_res->y.at(i - 1));
                tmp.set_theta(ha_res->phi.at(i - 1));
                tmp.set_s(seg_length);
                tmp.set_kappa(ha_res->steer.at(i - 1));
                s.path_points.push_back(tmp);
                v_direct = int(v / fabs(v));

                tmp.set_x(ha_res->x.at(i));
                tmp.set_y(ha_res->y.at(i));
                tmp.set_theta(ha_res->phi.at(i));
                seg_length = seg_length + hypot(p_last.x() - ha_res->x.at(i),
                                                p_last.y() - ha_res->y.at(i)); // 计算上一个点到当前点距离
                tmp.set_s(seg_length);
                tmp.set_kappa(ha_res->steer.at(i));
                s.path_points.push_back(tmp);
                p_last = Pose2d(ha_res->x.at(i), ha_res->y.at(i), ha_res->phi.at(i));

            } else if (i == ha_res->x.size() - 1) { // 到最后
                seg_length = seg_length + hypot(p_last.x() - ha_res->x.at(i),
                                                p_last.y() - ha_res->y.at(i)); // 计算上一个点到当前点距离
                s.target_pose = Pose2d(ha_res->x.at(i), ha_res->y.at(i), ha_res->phi.at(i));
                s.length = seg_length;
                tmp.set_x(ha_res->x.at(i));
                tmp.set_y(ha_res->y.at(i));
                tmp.set_theta(ha_res->phi.at(i));

                seg_length = seg_length + hypot(p_last.x() - ha_res->x.at(i),
                                                p_last.y() - ha_res->y.at(i)); // 计算上一个点到当前点距离
                tmp.set_s(seg_length);
                tmp.set_kappa(ha_res->steer.at(i));
                s.path_points.push_back(tmp);
                p_last = Pose2d(ha_res->x.at(i), ha_res->y.at(i), ha_res->phi.at(i));
                path->path_segments.push_back(s);
                path->target_pose = s.target_pose;
            }
            path->total_length = path->total_length + s.length;
        } else { // 中间点
            tmp.set_x(ha_res->x.at(i));
            tmp.set_y(ha_res->y.at(i));
            tmp.set_theta(ha_res->phi.at(i));

            seg_length = seg_length + hypot(p_last.x() - ha_res->x.at(i),
                                            p_last.y() - ha_res->y.at(i)); // 计算上一个点到当前点距离
            tmp.set_s(seg_length);
            tmp.set_kappa(ha_res->steer.at(i));
            s.path_points.push_back(tmp);
            p_last = Pose2d(ha_res->x.at(i), ha_res->y.at(i), ha_res->phi.at(i));
        }
    }
    //  for (size_t i = 0; i < path->path_segments.size(); i ++){
    //    double s = 0;
    //    for(size_t j = 0; j < path->path_segments.at(i).path_points.size();
    //    j++){
    //      s = s+j*step_size_;
    //      path->path_segments.at(i).path_points.at(j).set_s(s);
    //    }
    //  }

    MLOG(PARKING_PLANNING, INFO) << "finish HybridAStarPlanner::SetResult";
}



PLANNING_NAMESPACE_END