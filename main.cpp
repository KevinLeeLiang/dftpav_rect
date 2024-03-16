#include "HA/grid_map/grid_map.h"
#include "common.h"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <fstream>

void getRectangleConst(Box2d map_bound, std::vector<POINT2D> &obs,
                       std::vector<Eigen::Vector3d> statelist,
                       std::vector<Eigen::MatrixXd> hPolys_) {
  hPolys_.clear();
  GridMap grid_map;
  grid_map.BuildGridMap(obs, map_bound);

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
    grid_map.configurationTest(state.x(), state.y(), yaw);

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
            isocc = grid_map.CheckIfCollisionUsingLine(point1, newpoint1,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            isocc = grid_map.CheckIfCollisionUsingLine(newpoint1, newpoint2,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            isocc = grid_map.CheckIfCollisionUsingLine(newpoint2, point2,
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
            isocc = grid_map.CheckIfCollisionUsingLine(point1, newpoint1,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            isocc = grid_map.CheckIfCollisionUsingLine(newpoint1, newpoint2,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            isocc = grid_map.CheckIfCollisionUsingLine(newpoint2, point2,
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
            isocc = grid_map.CheckIfCollisionUsingLine(point1, newpoint1,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            isocc = grid_map.CheckIfCollisionUsingLine(newpoint1, newpoint2,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            isocc = grid_map.CheckIfCollisionUsingLine(newpoint2, point2,
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
            isocc = grid_map.CheckIfCollisionUsingLine(point1, newpoint1,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            isocc = grid_map.CheckIfCollisionUsingLine(newpoint1, newpoint2,
                                                       resolution / 2.0);
            if (isocc) {
              NotFinishTable[i] = 0.0;
              break;
            }
            isocc = grid_map.CheckIfCollisionUsingLine(newpoint2, point2,
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
  };
}



POINT2D getPoint2dFromStringLine(std::string line) {
  std::vector<std::string>tokens;
  std::istringstream iss(line);
  std::string token;

  while (std::getline(iss, token, ',')) {
    tokens.push_back(token);
  }
  return POINT2D(std::stod(tokens[0]), std::stod(tokens[1]));
}

std::vector<POINT2D> getObstacleEnvTest(Box2d & map_bound) {
  map_bound = Box2d(POINT2D(0, 0), 50, 50);
  std::vector<POINT2D> obs;
  std::ifstream file; // 打开名为example.txt的文件
  file.open("../Sim/example.txt");
  if (file.is_open()) { // 检查文件是否成功打开
    std::string line;
    while (std::getline(file, line)) { // 逐行读取文件内容
      obs.push_back(getPoint2dFromStringLine(line));
    }

    file.close(); // 关闭文件
  } else {
    std::cerr << "无法打开文件" << std::endl; // 打开文件失败时输出错误信息
  }

  return obs;
}



int main() {
  std::cout << "Hello, World!" << std::endl;
  std::vector<Eigen::Vector3d> statelist;
  std::vector<Eigen::MatrixXd> hPolys;

  Box2d map_bound = Box2d(POINT2D(0, 0), 50, 50);
  std::vector<POINT2D> obs = getObstacleEnvTest(map_bound);

  getRectangleConst(map_bound, obs, statelist, hPolys);
  for (int i = 0; i < hPolys.size(); i++) {
    std::cout << "i," << hPolys[i](0, 0) << "," << hPolys[i](0, 1)
              << "," << hPolys[i](1, 0) << "," << hPolys[i](1, 1)
              << "," << hPolys[i](2, 0) << "," << hPolys[i](2, 1)
              << "," << hPolys[i](3, 0) << "," << hPolys[i](3, 1) << std::endl;
  }
  return 0;
}
