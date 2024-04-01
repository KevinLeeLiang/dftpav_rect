#include "HA/grid_map/grid_map.h"
#include "common.h"
#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <memory>
POINT3D getPoint3dFromStringLine(std::string line) {
  std::vector<std::string> tokens;
  std::istringstream iss(line);
  std::string token;

  while (std::getline(iss, token, ',')) {
    tokens.push_back(token);
  }
  return POINT3D(std::stod(tokens[0]), std::stod(tokens[1]), std::stod(tokens[2]));
}

POINT2D getPoint2dFromStringLine(std::string line) {
  std::vector<std::string> tokens;
  std::istringstream iss(line);
  std::string token;

  while (std::getline(iss, token, ',')) {
    tokens.push_back(token);
  }
  return POINT2D(std::stod(tokens[0]), std::stod(tokens[1]));
}

std::vector<POINT2D> getObstacleEnvTest() {
  std::vector<POINT2D> obs;
  std::ifstream file;// 打开名为example.txt的文件
  file.open("../Sim/example.txt");
  if (file.is_open()) {// 检查文件是否成功打开
    std::string line;
    while (std::getline(file, line)) {// 逐行读取文件内容
      obs.push_back(getPoint2dFromStringLine(line));
    }

    file.close();// 关闭文件
  } else {
    std::cerr << "无法打开文件" << std::endl;// 打开文件失败时输出错误信息
  }

  return obs;
}

std::vector<POINT3D> getStatesTest() {
  std::vector<Eigen::Vector3d> states;
  std::ifstream file;// 打开名为example.txt的文件
  file.open("../Sim/path.txt");
  if (file.is_open()) {// 检查文件是否成功打开
    std::string line;
    while (std::getline(file, line)) {// 逐行读取文件内容
      states.push_back(getPoint3dFromStringLine(line));
    }

    file.close();// 关闭文件
  } else {
    std::cerr << "无法打开文件" << std::endl;// 打开文件失败时输出错误信息
  }

  return states;
}

std::vector<Eigen::MatrixXd> getRectangleConst(Box2d map_bound, std::vector<POINT2D> obs, std::vector<POINT3D> statelist) {
  std::vector<Eigen::MatrixXd> hPolys_;
  std::shared_ptr<GridMap> grid_map = std::make_shared<GridMap>();
  grid_map->BuildGridMap(obs, map_bound);

  double resolution = 0.2;
  double step = resolution * 1.0;
  double limitBound = 10.0;
  // generate a rectangle for this state px py yaw
  for (const auto state : statelist) {
    // generate a hPoly
    Eigen::MatrixXd hPoly;
    hPoly.resize(4, 4);
    Eigen::Matrix<int, 4, 1> NotFinishTable =
        Eigen::Matrix<int, 4, 1>(1, 1, 1, 1);
    Eigen::Vector2d sourcePt = state.head(2);
    Eigen::Vector2d rawPt = sourcePt;
    double yaw = state[2];
    bool test = false;
    common::VehicleParam sourceVp, rawVp;
    Eigen::Matrix2d egoR;
    egoR << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
    common::VehicleParam vptest;
    grid_map->configurationTest(state.x(), state.y(), yaw);

    Eigen::Vector4d expandLength;
    expandLength << 0.0, 0.0, 0.0, 0.0;
    // dcr width length
    while (NotFinishTable.norm() > 0) {
      //+dy  +dx -dy -dx
      for (int i = 0; i < 4; i++) {
        if (!NotFinishTable[i])
          continue;
        // get the new source and vp
        Eigen::Vector2d NewsourcePt = sourcePt;
        common::VehicleParam NewsourceVp = sourceVp;
        Eigen::Vector2d point1, point2, newpoint1, newpoint2;

        bool isocc = false;
        switch (i) {
          //+dy
          case 0:
            point1 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length() / 2.0 +
                                                       sourceVp.d_cr(),
                                                       sourceVp.width() / 2.0);
            point2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length() / 2.0 +
                                                       sourceVp.d_cr(),
                                                       sourceVp.width() / 2.0);
            newpoint1 =
                sourcePt +
                egoR * Eigen::Vector2d(sourceVp.length() / 2.0 + sourceVp.d_cr(),
                                       sourceVp.width() / 2.0 + step);
            newpoint2 =
                sourcePt +
                egoR * Eigen::Vector2d(-sourceVp.length() / 2.0 + sourceVp.d_cr(),
                                       sourceVp.width() / 2.0 + step);
            // 1 new1 new1 new2 new2 2
            isocc = grid_map->CheckIfCollisionUsingLine(point1, newpoint1,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            isocc = grid_map->CheckIfCollisionUsingLine(newpoint1, newpoint2,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            isocc = grid_map->CheckIfCollisionUsingLine(newpoint2, point2,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            expandLength[i] += step;
            if (expandLength[i] >= limitBound) {
              NotFinishTable[i] = 0.0;
              break;
            }
            NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(0, step / 2.0);
            NewsourceVp.set_width(NewsourceVp.width() + step);
            sourcePt = NewsourcePt;
            sourceVp = NewsourceVp;
            break;
            //+dx
          case 1:
            point1 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length() / 2.0 +
                                                       sourceVp.d_cr(),
                                                       -sourceVp.width() / 2.0);
            point2 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length() / 2.0 +
                                                       sourceVp.d_cr(),
                                                       sourceVp.width() / 2.0);
            newpoint1 =
                sourcePt + egoR * Eigen::Vector2d(step + sourceVp.length() / 2.0 +
                                                  sourceVp.d_cr(),
                                                  -sourceVp.width() / 2.0);
            newpoint2 =
                sourcePt + egoR * Eigen::Vector2d(step + sourceVp.length() / 2.0 +
                                                  sourceVp.d_cr(),
                                                  sourceVp.width() / 2.0);
            // 1 new1 new1 new2 new2 2
            isocc = grid_map->CheckIfCollisionUsingLine(point1, newpoint1,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            isocc = grid_map->CheckIfCollisionUsingLine(newpoint1, newpoint2,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            isocc = grid_map->CheckIfCollisionUsingLine(newpoint2, point2,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            expandLength[i] += step;
            if (expandLength[i] >= limitBound) {
              NotFinishTable[i] = 0.0;
              break;
            }
            NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(step / 2.0, 0.0);
            NewsourceVp.set_length(NewsourceVp.length() + step);
            sourcePt = NewsourcePt;
            sourceVp = NewsourceVp;
            break;
            //-dy
          case 2:
            point1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length() / 2.0 +
                                                       sourceVp.d_cr(),
                                                       -sourceVp.width() / 2.0);
            point2 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length() / 2.0 +
                                                       sourceVp.d_cr(),
                                                       -sourceVp.width() / 2.0);
            newpoint1 =
                sourcePt +
                egoR * Eigen::Vector2d(-sourceVp.length() / 2.0 + sourceVp.d_cr(),
                                       -sourceVp.width() / 2.0 - step);
            newpoint2 =
                sourcePt +
                egoR * Eigen::Vector2d(sourceVp.length() / 2.0 + sourceVp.d_cr(),
                                       -sourceVp.width() / 2.0 - step);
            // 1 new1 new1 new2 new2 2
            isocc = grid_map->CheckIfCollisionUsingLine(point1, newpoint1,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            isocc = grid_map->CheckIfCollisionUsingLine(newpoint1, newpoint2,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            isocc = grid_map->CheckIfCollisionUsingLine(newpoint2, point2,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            expandLength[i] += step;
            if (expandLength[i] >= limitBound) {
              NotFinishTable[i] = 0.0;
              break;
            }
            NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(0, -step / 2.0);
            NewsourceVp.set_width(NewsourceVp.width() + step);
            sourcePt = NewsourcePt;
            sourceVp = NewsourceVp;
            break;
            //-dx
          case 3:
            point1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length() / 2.0 +
                                                       sourceVp.d_cr(),
                                                       sourceVp.width() / 2.0);
            point2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length() / 2.0 +
                                                       sourceVp.d_cr(),
                                                       -sourceVp.width() / 2.0);
            newpoint1 =
                sourcePt + egoR * Eigen::Vector2d(-sourceVp.length() / 2.0 +
                                                  sourceVp.d_cr() - step,
                                                  sourceVp.width() / 2.0);
            newpoint2 =
                sourcePt + egoR * Eigen::Vector2d(-sourceVp.length() / 2.0 +
                                                  sourceVp.d_cr() - step,
                                                  -sourceVp.width() / 2.0);
            // 1 new1 new1 new2 new2 2
            isocc = grid_map->CheckIfCollisionUsingLine(point1, newpoint1,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            isocc = grid_map->CheckIfCollisionUsingLine(newpoint1, newpoint2,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            isocc = grid_map->CheckIfCollisionUsingLine(newpoint2, point2,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            expandLength[i] += step;
            if (expandLength[i] >= limitBound) {
              NotFinishTable[i] = 0.0;
              break;
            }
            NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(-step / 2.0, 0.0);
            NewsourceVp.set_length(NewsourceVp.length() + step);
            sourcePt = NewsourcePt;
            sourceVp = NewsourceVp;
            break;
        }
      }
    }
    Eigen::Vector2d point1, norm1;
    point1 =
        rawPt + egoR * Eigen::Vector2d(rawVp.length() / 2.0 + rawVp.d_cr() +
                                       expandLength[1],
                                       rawVp.width() / 2.0 + expandLength[0]);
    norm1 << -sin(yaw), cos(yaw);
    hPoly.col(0).head<2>() = norm1;
    hPoly.col(0).tail<2>() = point1;
    Eigen::Vector2d point2, norm2;
    // point2 =
    // sourcePt+egoR*Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
    point2 =
        rawPt + egoR * Eigen::Vector2d(rawVp.length() / 2.0 + rawVp.d_cr() +
                                       expandLength[1],
                                       -rawVp.width() / 2.0 - expandLength[2]);
    norm2 << cos(yaw), sin(yaw);
    hPoly.col(1).head<2>() = norm2;
    hPoly.col(1).tail<2>() = point2;
    Eigen::Vector2d point3, norm3;
    // point3 =
    // sourcePt+egoR*Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
    point3 =
        rawPt + egoR * Eigen::Vector2d(-rawVp.length() / 2.0 + rawVp.d_cr() -
                                       expandLength[3],
                                       -rawVp.width() / 2.0 - expandLength[2]);
    norm3 << sin(yaw), -cos(yaw);
    hPoly.col(2).head<2>() = norm3;
    hPoly.col(2).tail<2>() = point3;
    Eigen::Vector2d point4, norm4;
    // point4 =
    // sourcePt+egoR*Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
    point4 =
        rawPt + egoR * Eigen::Vector2d(-rawVp.length() / 2.0 + rawVp.d_cr() -
                                       expandLength[3],
                                       rawVp.width() / 2.0 + expandLength[0]);
    norm4 << -cos(yaw), -sin(yaw);
    hPoly.col(3).head<2>() = norm4;
    hPoly.col(3).tail<2>() = point4;
    hPolys_.push_back(hPoly);
  }
  return hPolys_;
}

void OutputFile(const std::vector<Eigen::MatrixXd>hPolys) {
  std::ofstream outFile("../Sim/output.txt"); // 打开输出文件流

  if (outFile.is_open()) { // 检查文件是否成功打开
    for (const auto& hpoly : hPolys) {
      outFile << hpoly(2, 0) << "," << hpoly(3, 0) << "," << hpoly(2, 1) << "," << hpoly(3, 1) << ","
              << hpoly(2, 2) << "," << hpoly(3, 2) << "," << hpoly(2, 3) << "," << hpoly(3, 3) << "\n"; // 将每行数据写入文件，并在末尾添加换行符
    }

    outFile.close(); // 关闭文件流
    std::cout << "Data has been written to output.txt" << std::endl;
  } else {
    std::cerr << "Error opening the file." << std::endl;
  }
}


ErrorType RunMINCOParking(){
  //TO DO
  traj_container_.clearSingul();
  Eigen::MatrixXd flat_finalState(2, 3),  flat_headState(2,3);
  Eigen::VectorXd ego_piece_dur_vec;
  Eigen::MatrixXd ego_innerPs;
  ROS_WARN("begin to run minco");
  nav_msgs::Path debug_msg0,debug_msg1;
  display_hPolys_.clear();
  double worldtime =  head_state_.time_stamp;
  double basetime = 0.0;

  /*try to merge optimization process*/
  std::vector<std::vector<Eigen::MatrixXd>> sfc_container;
  std::vector<int> singul_container;
  Eigen::VectorXd duration_container;
  std::vector<Eigen::MatrixXd> waypoints_container;
  std::vector<Eigen::MatrixXd> iniState_container,finState_container;
  duration_container.resize(kino_trajs_.size());

  for(unsigned int i = 0; i < kino_trajs_.size(); i++){
    double timePerPiece = traj_piece_duration_;
    plan_utils::FlatTrajData kino_traj = kino_trajs_.at(i);
    singul_container.push_back(kino_traj.singul);
    std::vector<Eigen::Vector3d> pts = kino_traj.traj_pts;
    plan_utils::MinJerkOpt initMJO;
    plan_utils::Trajectory initTraj;
    int piece_nums;
    double initTotalduration = 0.0;
    for(const auto pt : pts){
      initTotalduration += pt[2];
    }
    piece_nums = std::max(int(initTotalduration / timePerPiece + 0.5),2);
    timePerPiece = initTotalduration / piece_nums;
    ego_piece_dur_vec.resize(piece_nums);
    ego_piece_dur_vec.setConstant(timePerPiece);
    duration_container[i] = timePerPiece * piece_nums;
    ego_innerPs.resize(2, piece_nums-1);
    std::vector<Eigen::Vector3d> statelist;
    double res_time = 0;
    for(int i = 0; i < piece_nums; i++ ){
      int resolution;
      if(i==0||i==piece_nums-1){
        resolution = dense_traj_res;
      }
      else{
        resolution = traj_res;
      }
      for(int k = 0; k <= resolution; k++){
        double t = basetime+res_time + 1.0*k/resolution*ego_piece_dur_vec[i];
        Eigen::Vector3d pos = kino_path_finder_->evaluatePos(t);
        statelist.push_back(pos);
        if(k==resolution && i!=piece_nums-1){
          ego_innerPs.col(i) = pos.head(2);
        }
      }
      res_time += ego_piece_dur_vec[i];
    }
    std::cout<<"s: "<<kino_traj.singul<<"\n";
    double tm1 = ros::Time::now().toSec();
    getRectangleConst(statelist);
    sfc_container.push_back(hPolys_);
    display_hPolys_.insert(display_hPolys_.end(),hPolys_.begin(),hPolys_.end());
    waypoints_container.push_back(ego_innerPs);
    iniState_container.push_back(kino_traj.start_state);
    finState_container.push_back(kino_traj.final_state);
    basetime += initTotalduration;
    //visualization
    initMJO.reset(ego_piece_dur_vec.size());
    initMJO.generate(ego_innerPs, timePerPiece,kino_traj.start_state, kino_traj.final_state);
    initTraj = initMJO.getTraj(kino_traj.singul);
    {
      for(double t  = 0.0; t <= initTraj.getTotalDuration(); t+=0.01){
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = initTraj.getPos(t)[0];//x
        pose.pose.position.y = initTraj.getPos(t)[1];//y
        pose.pose.position.z = 0.2;
        debug_msg1.poses.push_back(pose);
      }
      debug_msg1.header.frame_id = "map";
    }
    Debugtraj1Pub.publish(debug_msg1);
  }
  Debugtraj1Pub.publish(debug_msg1);

  double t1= ros::Time::now().toSec();
  if(map_itf_->GetMovingObsTraj(&sur_discretePoints)!=kSuccess){
    return kWrongStatus;
  }
  ConverSurroundTrajFromPoints(sur_discretePoints,&surround_trajs);
  double t2 = ros::Time::now().toSec();
  std::cout<<"convert time: "<<(t2-t1)<<std::endl;
  ploy_traj_opt_->setSurroundTrajs(&surround_trajs);
  // ploy_traj_opt_->setSurroundTrajs(NULL);
  std::cout<<"try to optimize!\n";

  int flag_success = ploy_traj_opt_->OptimizeTrajectory(iniState_container, finState_container,
                                                        waypoints_container,duration_container,
                                                        sfc_container,  singul_container,worldtime,0.0);
  std::cout<<"optimize ended!\n";



  if (flag_success)
  {
    std::cout << "[PolyTrajManager] Planning success ! " << std::endl;
    for(unsigned int i = 0; i < kino_trajs_.size(); i++){
      traj_container_.addSingulTraj( (*ploy_traj_opt_->getMinJerkOptPtr())[i].getTraj(singul_container[i]), worldtime, ego_id_); // todo time
      std::cout<<"init duration: "<<duration_container[i]<<std::endl;
      std::cout<<"pieceNum: " << waypoints_container[i].cols() + 1 <<std::endl;
      std::cout<<"optimized total duration: "<<(*ploy_traj_opt_->getMinJerkOptPtr())[i].getTraj(1).getTotalDuration()<<std::endl;
      std::cout<<"optimized jerk cost: "<<(*ploy_traj_opt_->getMinJerkOptPtr())[i].getTrajJerkCost()<<std::endl;
      worldtime = traj_container_.singul_traj.back().end_time;
    }
  }
  else{
    ROS_ERROR("[PolyTrajManager] Planning fails! ");
    return kWrongStatus;
  }
  if(is_init){
    //reset the timeStamp
    for(auto & it:traj_container_.singul_traj ){
      it.start_time = ros::Time::now().toSec()-head_state_.time_stamp+it.start_time;
      it.end_time = ros::Time::now().toSec()-head_state_.time_stamp+it.end_time;
    }
  }

  return kSuccess;

}


int main() {
  std::cout << "Hello, World!" << std::endl;
  std::vector<Eigen::MatrixXd> hPolys;
  Box2d map_bound = Box2d(POINT2D(25, 25), 100, 100);
  std::vector<POINT2D> obs = getObstacleEnvTest();
  std::vector<POINT3D> statelist = getStatesTest();
  hPolys = getRectangleConst(map_bound, obs, statelist);
//  hPolys = tttt(map_bound, obs, statelist);
  OutputFile(hPolys);

  for (int i = 0; i < hPolys.size(); i++) {
    std::cout << hPolys[i] << std::endl;
    std::cout << std::endl;
  }
  return 0;
}
