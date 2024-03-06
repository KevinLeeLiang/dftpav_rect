//
// Created by garen_lee on 2024/1/23.
/**
 ******************************************************************************
 * @file           : ha_path_plan.cpp.cc
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/23
 ******************************************************************************
 */
//

#include "ha_path_plan.h"
#include "ha_path_planner/utils.h"
PLANNING_NAMESPACE_START

HAPathPlan::HAPathPlan(const size_t max_node_num) {
    this->ha_planner_ = std::make_shared<HybridAStarPlanner>(max_node_num);
    this->is_rev_ = true;
}

void HAPathPlan::Plan(const PlanningInfo::Ptr &planning_info, Path::Ptr &path) {
    MLOG(PARKING_PLANNING, INFO) << "HAPathPlan::Plan Plan Start";
    obs_ptrs_.clear();
    uint64_t start_time = ::os::Time::now().to_usec();
    this->ha_planner_->setPlanStartTime(start_time);
    this->ha_planner_->clear_result();

    auto park_env_ptr = planning_info->plan_env();

    MLOG(PARKING_PLANNING, INFO) << "start_pose x: " << park_env_ptr->start_pose.x();
    MLOG(PARKING_PLANNING, INFO) << "start_pose y: " << park_env_ptr->start_pose.y();
    MLOG(PARKING_PLANNING, INFO) << "start_pose theta: " << park_env_ptr->start_pose.theta();

    MLOG(PARKING_PLANNING, INFO) << "target_pose x: " << park_env_ptr->target_pose.x();
    MLOG(PARKING_PLANNING, INFO) << "target_pose y: " << park_env_ptr->target_pose.y();
    MLOG(PARKING_PLANNING, INFO) << "target_pose theta: " << park_env_ptr->target_pose.theta();
    if (fabs(park_env_ptr->start_pose.x() - park_env_ptr->target_pose.x()) < 1e-5 &&
        fabs(park_env_ptr->start_pose.y() - park_env_ptr->target_pose.y()) < 1e-5 &&
        fabs(park_env_ptr->start_pose.theta() - park_env_ptr->target_pose.theta()) < 1e-5) {
        path->success = false;
        MLOG(PARKING_PLANNING, ERROR) << "start_pose and target_pose are the same location, no need to plan";
        return;
    }
    // 构建障碍物模型
    // update(park_env_ptr->target_pose, goal_v, map_boundary, map);

    MLOG(PARKING_PLANNING, INFO) << "obstacle_edges size: " << park_env_ptr->obstacle_edges.size();
    MLOG(PARKING_PLANNING, INFO) << "obstacle_points size: " << park_env_ptr->obstacle_points.size();

    std::vector<ObstacleLineTmp> lines;
    lines.reserve(park_env_ptr->obstacle_edges.size());
    for (auto &edge : park_env_ptr->obstacle_edges) {
        lines.push_back(ObstacleLineTmp(edge.start(), edge.end()));
    }
    // 车位底边加入障碍物,将一定范围外的车位顶边也加入障碍物,来确保车辆超出一定x范围的移动空间在通道内
    LineSegment2d bottum_line(Vec2d(park_env_ptr->slot.rear_left().x() - 1, park_env_ptr->slot.rear_left().y() + 0.5),
                              Vec2d(park_env_ptr->slot.rear_left().x() - 10, park_env_ptr->slot.rear_left().y() + 0.5));
    ObstacleLineTmp bottum_obsline(bottum_line, ObstacleType::WALL);
    lines.emplace_back(bottum_obsline);
    LineSegment2d top_line(Vec2d(park_env_ptr->slot.front_left().x() - 7, park_env_ptr->slot.front_left().y()),
                           Vec2d(park_env_ptr->slot.front_left().x() - 15, park_env_ptr->slot.front_left().y()));
    ObstacleLineTmp top_obsline(top_line, ObstacleType::WALL);
    lines.emplace_back(top_obsline);
    LineSegment2d left_line(Vec2d(park_env_ptr->slot.front_left().x() - 7, park_env_ptr->slot.front_left().y()),
                            Vec2d(park_env_ptr->slot.front_left().x() - 7, park_env_ptr->slot.rear_left().y()));
    ObstacleLineTmp left_obsline(left_line, ObstacleType::WALL);
    lines.emplace_back(left_obsline);
    obs_ptrs_.emplace_back(std::make_shared<SBPObstacleLine>(lines));
    if (park_env_ptr->obstacle_points.size() > 0) {
        auto sbp_obs_points = std::make_shared<SBPObstaclePoint>(park_env_ptr->obstacle_points);
        obs_ptrs_.push_back(sbp_obs_points);
    }

    // 构建 grid map
    map_boundary_ = Box2d(park_env_ptr->slot.front_right(), park_env_ptr->slot.slot_angle(), 80, 80);

    std::vector<LineSegment2d> edge = park_env_ptr->obstacle_edges;
    double slot_mid_lx, slot_mid_ly, slot_mid_rx, slot_mid_ry;
    slot_mid_lx = park_env_ptr->slot.rear_left().x();
    slot_mid_ly = park_env_ptr->slot.rear_left().y() +
                  (park_env_ptr->slot.front_left().y() - park_env_ptr->slot.rear_left().y()) / 2;
    slot_mid_rx = park_env_ptr->slot.rear_right().x();
    slot_mid_ry = park_env_ptr->slot.rear_right().y() +
                  (park_env_ptr->slot.front_right().y() - park_env_ptr->slot.rear_right().y()) / 2;

    edge.push_back(LineSegment2d(Vec2d(park_env_ptr->slot.rear_left().x() - 0.2, park_env_ptr->slot.rear_left().y()),
                                 Vec2d(slot_mid_lx - 0.2, slot_mid_ly)));
    edge.push_back(LineSegment2d(Vec2d(park_env_ptr->slot.rear_right().x() + 0.2, park_env_ptr->slot.rear_right().y()),
                                 Vec2d(slot_mid_rx + 0.2, slot_mid_ry)));
    edge.emplace_back(bottum_line);
    edge.emplace_back(top_line);
    edge.emplace_back(left_line);
    this->buildGridMap(edge, park_env_ptr->obstacle_points, map_boundary_);
    // TODO: 建图完成后进行规划
    MLOG(PARKING_PLANNING, INFO) << "ha path plan start pose x," << park_env_ptr->start_pose.x() << ",y,"
                                 << park_env_ptr->start_pose.y() << ",theta," << park_env_ptr->start_pose.theta();
    MLOG(PARKING_PLANNING, INFO) << "ha path plan target pose x," << park_env_ptr->target_pose.x() << ",y,"
                                 << park_env_ptr->target_pose.y() << ",theta," << park_env_ptr->target_pose.theta();
    if (is_rev_) {
        this->ha_planner_->set_target_pose(
            Pose2d(park_env_ptr->start_pose.x(), park_env_ptr->start_pose.y(), park_env_ptr->start_pose.theta()));
        this->ha_planner_->set_start_pose(park_env_ptr->target_pose);
    } else {

        this->ha_planner_->set_start_pose(
            Pose2d(park_env_ptr->start_pose.x(), park_env_ptr->start_pose.y(), park_env_ptr->start_pose.theta()));
        this->ha_planner_->set_target_pose(park_env_ptr->target_pose);
    }
    this->ha_planner_->update(park_env_ptr->target_pose, 0, map_boundary_);
    this->ha_planner_->plan(obs_ptrs_);

    auto res = this->ha_planner_->get_result();
    if (res.status == HA::SBPStatus::SUCCESS) {
        path->success = true;

        path->start_pose = park_env_ptr->start_pose;
        path->target_pose = park_env_ptr->target_pose;
        path->total_length = path->gear_switch_count = 1;
        std::vector<PathPoint> path_points;

        MLOG(PARKING_PLANNING, INFO) << "res.x " << res.x.size();
        MLOG(PARKING_PLANNING, INFO) << "res.y " << res.y.size();
        MLOG(PARKING_PLANNING, INFO) << "res.phi " << res.phi.size();
        MLOG(PARKING_PLANNING, INFO) << "res.steer " << res.steer.size();
        MLOG(PARKING_PLANNING, INFO) << "res.accumulated_s " << res.accumulated_s.size();

        size_t size = res.x.size();
        this->SetResult(&res, path);
        path->start_pose = park_env_ptr->start_pose;
        path->target_pose = park_env_ptr->target_pose;
        samplePath(planning_info, path);
        // this->hybrid_astar_planner_->SetResult(&res, path);
        for (size_t i = 0; i < size; i++) {
            PathPoint path_point;
            path_point.set_x(res.x[i]);
            path_point.set_y(res.y[i]);
            path_point.set_theta(res.phi[i]);
            path_point.set_kappa(res.steer[i]);

            MLOG(PARKING_PLANNING, INFO) << res.steer[i] << " HA_PATH p," << res.x[i] << "," << res.y[i] << ","
                                         << res.phi[i];
            path_points.push_back(std::move(path_point));
        }
        path->merge(path_points);
    } else {
        path->success = false;
    }
    uint64_t end_time = ::os::Time::now().to_usec() - start_time;
    for (size_t i = 0; i < this->ha_planner_->get_search_points().size(); i++) {
        MLOG(PARKING_PLANNING, INFO) << "SearchPoints p," << this->ha_planner_->get_search_points()[i].x() << ","
                                     << this->ha_planner_->get_search_points()[i].y() << ","
                                     << this->ha_planner_->get_search_points()[i].theta();
    }
    MLOG(PARKING_PLANNING, INFO) << "TShape upline," << park_env_ptr->t_shape.upper_line.start().x() << ","
                                 << park_env_ptr->t_shape.upper_line.start().y() << ","
                                 << park_env_ptr->t_shape.upper_line.end().x() << ","
                                 << park_env_ptr->t_shape.upper_line.end().y();
    MLOG(PARKING_PLANNING, INFO) << "TShape roadleft," << park_env_ptr->t_shape.road_left_edge.start().x() << ","
                                 << park_env_ptr->t_shape.road_left_edge.start().y() << ","
                                 << park_env_ptr->t_shape.road_left_edge.end().x() << ","
                                 << park_env_ptr->t_shape.road_left_edge.end().y();
    MLOG(PARKING_PLANNING, INFO) << "TShape roadright," << park_env_ptr->t_shape.road_right_edge.start().x() << ","
                                 << park_env_ptr->t_shape.road_right_edge.start().y() << ","
                                 << park_env_ptr->t_shape.road_right_edge.end().x() << ","
                                 << park_env_ptr->t_shape.road_right_edge.end().y();
    MLOG(PARKING_PLANNING, INFO) << "TShape slotleft," << park_env_ptr->t_shape.slot_left_edge.start().x() << ","
                                 << park_env_ptr->t_shape.slot_left_edge.start().y() << ","
                                 << park_env_ptr->t_shape.slot_left_edge.end().x() << ","
                                 << park_env_ptr->t_shape.slot_left_edge.end().y();
    MLOG(PARKING_PLANNING, INFO) << "TShape slotbottom," << park_env_ptr->t_shape.slot_bottom_edge.start().x() << ","
                                 << park_env_ptr->t_shape.slot_bottom_edge.start().y() << ","
                                 << park_env_ptr->t_shape.slot_bottom_edge.end().x() << ","
                                 << park_env_ptr->t_shape.slot_bottom_edge.end().y();
    MLOG(PARKING_PLANNING, INFO) << "TShape slotright," << park_env_ptr->t_shape.slot_right_edge.start().x() << ","
                                 << park_env_ptr->t_shape.slot_right_edge.start().y() << ","
                                 << park_env_ptr->t_shape.slot_right_edge.end().x() << ","
                                 << park_env_ptr->t_shape.slot_right_edge.end().y();
    for (size_t i = 0; i < lines.size(); i++) {
        MLOG(PARKING_PLANNING, INFO) << "obsline l," << lines.at(i).start().x() << "," << lines.at(i).start().y() << ","
                                     << lines.at(i).end().x() << "," << lines.at(i).end().y();
    }
    for (size_t i = 0; i < park_env_ptr->obstacle_points.size(); i++) {
        MLOG(PARKING_PLANNING, INFO) << "obspoint p," << park_env_ptr->obstacle_points.at(i).x() << ","
                                     << park_env_ptr->obstacle_points.at(i).y();
    }
    if (path->success == true) {
        auto search_path = this->ha_planner_->get_search_path();
        for (size_t i = 0; i < search_path.size(); i++) {
            auto tmp = search_path.at(i);
            MLOG(PARKING_PLANNING, INFO) << "searchPath p," << tmp.x() << "," << tmp.y() << "," << tmp.theta();
        }
        MLOG(PARKING_PLANNING, INFO) << "HA plan Time:" << end_time << " HA Plan Success";
    } else {
        MLOG(PARKING_PLANNING, INFO) << "HA plan Time:" << end_time << " HA Plan Failed";
    }
    MLOG(PARKING_PLANNING, INFO) << "HA Plan Finish";

    path->print();
}

void HAPathPlan::buildGridMap(const std::vector<LineSegment2d> &obs_lines, const std::vector<Vec2d> &obs_pts,
                              const Box2d &map_boundary) {
    auto start_time = ::os::Time::now().to_usec();
    this->ha_planner_->buildGridMap(obs_lines, obs_pts, map_boundary);
    MLOG(PARKING_PLANNING, INFO) << "HyAstar BuildGridMap run Time:"
                                 << (::os::Time::now().to_usec() - start_time) * 1e-3 << "ms";
}

void HAPathPlan::SetResult(SBPResult *ha_res, Path::Ptr &path) {
    // 将ha结果加入到path中
    this->ha_planner_->SetResult(ha_res, path);
}

void HAPathPlan::samplePath(const PlanningInfo::Ptr &planning_info, Path::Ptr &path) {
    size_t path_size = path->path_segments.size();
    MLOG(PARKING_PLANNING, INFO) << "ORIGIN DATA";
    auto segs = path->path_segments;
    for (size_t i = 0; i < segs.size(); i++) {
        MLOG(PARKING_PLANNING, INFO) << "SEG i," << i;
        auto seg = segs[i];
        for (size_t j = 0; j < seg.path_points.size(); ++j) {
            auto p = seg.path_points[j];
            MLOG(PARKING_PLANNING, INFO) << "path p x," << p.x() << ",y," << p.y() << ",t," << p.theta() << ",s,"
                                         << p.s();
        }
    }
    MLOG(PARKING_PLANNING, INFO) << "Interpolation";
    for (size_t i = 0; i < path_size; ++i) {
        auto &path_segment = path->path_segments[i];
        std::vector<PathPoint> path_tmp = samplePath(planning_info, path_segment);
        if (this->is_rev_) {
            segs[path_size - i - 1].path_points = path_tmp;
            auto e = path_segment.start_pose;
            auto s = path_segment.target_pose;
            segs[path_size - i - 1].start_pose = s;
            segs[path_size - i - 1].target_pose = e;
            if (path_segment.gear == GearBoxInfoPb::GEAR_DRIVE)
                segs[path_size - i - 1].gear = GearBoxInfoPb::GEAR_REVERSE;
            else
                segs[path_size - i - 1].gear = GearBoxInfoPb::GEAR_DRIVE;
        } else {
            segs[i].path_points = path_tmp;
        }
    }
    path->path_segments = segs;
    MLOG(PARKING_PLANNING, INFO) << "ha path point test!";
    for (size_t i = 0; i < segs.size(); i++) {
        auto seg = segs[i];
        MLOG(PARKING_PLANNING, INFO) << "SEG i," << i << ",gear," << seg.gear;
        for (size_t j = 0; j < seg.path_points.size(); ++j) {
            auto p = seg.path_points[j];
            MLOG(PARKING_PLANNING, INFO) << "path p x," << p.x() << ",y," << p.y() << ",t," << p.theta() << ",s,"
                                         << p.s();
        }
    }
}

std::vector<PathPoint> HAPathPlan::samplePath(PlanningInfo::Ptr planning_info, const PathSegment &path_segment) {
    std::vector<PathPoint> path_result;
    double delta_s = FLAGS_apa_output_trajectory_length_resolution;
    double total_length = path_segment.length + delta_s * 1.0e-6;
    int t = int(total_length / delta_s);
    double delt = total_length / t;
    for (double s = 0; s <= total_length + 1.0e-6; s += delt) {
        auto pt = PathPlanUtils::EvaluateByS(path_segment.path_points, s);
        if (this->is_rev_) {
            pt.set_s(total_length - s);
            path_result.insert(path_result.begin(), pt);
        } else
            path_result.push_back(pt);
    }
    // path_result = path_segment.path_points;
    if (!planning_info->plan_env()->right_park) {
        for (auto &p : path_result) {
            p.set_kappa(p.kappa() * -1.0);
        }
    }
    return path_result;
}

PLANNING_NAMESPACE_END