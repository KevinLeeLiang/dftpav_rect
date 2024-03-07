#include <iostream>
#include <eigen3/Eigen/Eigen>
#include "common.h"
#include "HA/grid_map/grid_map.h"

void getRectangleConst(std::vector<Eigen::Vector3d> statelist, std::vector<Eigen::MatrixXd> hPolys_) {
  hPolys_.clear();
  GridMap grid_map;
  grid_map.BuildGridMap();
  GridMap2D grid_map;
  map_itf_->GetObstacleMap(&grid_map);
  double resolution = grid_map.dims_resolution(0);
  double step = resolution * 1.0;
  double limitBound = 10.0;
  //generate a rectangle for this state px py yaw
  for(const auto state : statelist){
    //generate a hPoly
    Eigen::MatrixXd hPoly;
    hPoly.resize(4, 4);
    Eigen::Matrix<int,4,1> NotFinishTable = Eigen::Matrix<int,4,1>(1,1,1,1);
    Eigen::Vector2d sourcePt = state.head(2);
    Eigen::Vector2d rawPt = sourcePt;
    double yaw = state[2];
    bool test = false;
    common::VehicleParam sourceVp,rawVp;
    Eigen::Matrix2d egoR;
    egoR << cos(yaw), -sin(yaw),
        sin(yaw), cos(yaw);
    common::VehicleParam vptest;
    map_itf_->CheckIfCollisionUsingPosAndYaw(vptest,state,&test);

    Eigen::Vector4d expandLength;
    expandLength << 0.0, 0.0, 0.0, 0.0;
    //dcr width length
    while(NotFinishTable.norm()>0){
      //+dy  +dx -dy -dx
      for(int i = 0; i<4; i++){
        if(!NotFinishTable[i]) continue;
        //get the new source and vp
        Eigen::Vector2d NewsourcePt = sourcePt;
        common::VehicleParam NewsourceVp = sourceVp;
        Eigen::Vector2d point1,point2,newpoint1,newpoint2;

        bool isocc = false;
        switch (i)
        {
          //+dy
          case 0:
            point1 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
            point2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
            newpoint1 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0+step);
            newpoint2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0+step);
            //1 new1 new1 new2 new2 2
            map_itf_->CheckIfCollisionUsingLine(point1,newpoint1,&isocc,resolution/2.0);
            if(isocc){
              NotFinishTable[i] = 0.0;
              break;
            }
            map_itf_->CheckIfCollisionUsingLine(newpoint1,newpoint2,&isocc,resolution/2.0);
            if(isocc){
              NotFinishTable[i] = 0.0;
              break;
            }
            map_itf_->CheckIfCollisionUsingLine(newpoint2,point2,&isocc,resolution/2.0);
            if(isocc){
              NotFinishTable[i] = 0.0;
              break;
            }
            expandLength[i] += step;
            if(expandLength[i] >= limitBound){
              NotFinishTable[i] = 0.0;
              break;
            }
            NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(0,step/2.0);
            NewsourceVp.set_width(NewsourceVp.width() + step);
            sourcePt = NewsourcePt;
            sourceVp = NewsourceVp;
            break;
            //+dx
          case 1:
            point1 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
            point2 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
            newpoint1 = sourcePt + egoR * Eigen::Vector2d(step+sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
            newpoint2 = sourcePt + egoR * Eigen::Vector2d(step+sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
            //1 new1 new1 new2 new2 2
            map_itf_->CheckIfCollisionUsingLine(point1,newpoint1,&isocc,resolution/2.0);
            if(isocc){
              NotFinishTable[i] = 0.0;
              break;
            }
            map_itf_->CheckIfCollisionUsingLine(newpoint1,newpoint2,&isocc,resolution/2.0);
            if(isocc){
              NotFinishTable[i] = 0.0;
              break;
            }
            map_itf_->CheckIfCollisionUsingLine(newpoint2,point2,&isocc,resolution/2.0);
            if(isocc){
              NotFinishTable[i] = 0.0;
              break;
            }
            expandLength[i] += step;
            if(expandLength[i] >= limitBound){
              NotFinishTable[i] = 0.0;
              break;
            }
            NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(step/2.0,0.0);
            NewsourceVp.set_length(NewsourceVp.length() + step);
            sourcePt = NewsourcePt;
            sourceVp = NewsourceVp;
            break;
            //-dy
          case 2:
            point1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
            point2 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
            newpoint1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0-step);
            newpoint2 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0-step);
            //1 new1 new1 new2 new2 2
            map_itf_->CheckIfCollisionUsingLine(point1,newpoint1,&isocc,resolution/2.0);
            if(isocc){
              NotFinishTable[i] = 0.0;
              break;
            }
            map_itf_->CheckIfCollisionUsingLine(newpoint1,newpoint2,&isocc,resolution/2.0);
            if(isocc){
              NotFinishTable[i] = 0.0;
              break;
            }
            map_itf_->CheckIfCollisionUsingLine(newpoint2,point2,&isocc,resolution/2.0);
            if(isocc){
              NotFinishTable[i] = 0.0;
              break;
            }
            expandLength[i] += step;
            if(expandLength[i] >= limitBound){
              NotFinishTable[i] = 0.0;
              break;
            }
            NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(0,-step/2.0);
            NewsourceVp.set_width(NewsourceVp.width() + step);
            sourcePt = NewsourcePt;
            sourceVp = NewsourceVp;
            break;
            //-dx
          case 3:
            point1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
            point2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
            newpoint1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr()-step,sourceVp.width()/2.0);
            newpoint2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr()-step,-sourceVp.width()/2.0);
            //1 new1 new1 new2 new2 2
            map_itf_->CheckIfCollisionUsingLine(point1,newpoint1,&isocc,resolution/2.0);
            if(isocc){
              NotFinishTable[i] = 0.0;
              break;
            }
            map_itf_->CheckIfCollisionUsingLine(newpoint1,newpoint2,&isocc,resolution/2.0);
            if(isocc){
              NotFinishTable[i] = 0.0;
              break;
            }
            map_itf_->CheckIfCollisionUsingLine(newpoint2,point2,&isocc,resolution/2.0);
            if(isocc){
              NotFinishTable[i] = 0.0;
              break;
            }
            expandLength[i] += step;
            if(expandLength[i] >= limitBound){
              NotFinishTable[i] = 0.0;
              break;
            }
            NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(-step/2.0,0.0);
            NewsourceVp.set_length(NewsourceVp.length() + step);
            sourcePt = NewsourcePt;
            sourceVp = NewsourceVp;
            break;
        }
      }
    }
    Eigen::Vector2d point1,norm1;
    point1 = rawPt+egoR*Eigen::Vector2d(rawVp.length()/2.0+rawVp.d_cr()+expandLength[1],rawVp.width()/2.0+expandLength[0]);
    norm1 << -sin(yaw), cos(yaw);
    hPoly.col(0).head<2>() = norm1;
    hPoly.col(0).tail<2>() = point1;
    Eigen::Vector2d point2,norm2;
    // point2 = sourcePt+egoR*Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
    point2 = rawPt+egoR*Eigen::Vector2d(rawVp.length()/2.0+rawVp.d_cr()+expandLength[1],-rawVp.width()/2.0-expandLength[2]);
    norm2 << cos(yaw), sin(yaw);
    hPoly.col(1).head<2>() = norm2;
    hPoly.col(1).tail<2>() = point2;
    Eigen::Vector2d point3,norm3;
    // point3 = sourcePt+egoR*Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
    point3 = rawPt+egoR*Eigen::Vector2d(-rawVp.length()/2.0+rawVp.d_cr()-expandLength[3],-rawVp.width()/2.0-expandLength[2]);
    norm3 << sin(yaw), -cos(yaw);
    hPoly.col(2).head<2>() = norm3;
    hPoly.col(2).tail<2>() = point3;
    Eigen::Vector2d point4,norm4;
    // point4 = sourcePt+egoR*Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
    point4 = rawPt+egoR*Eigen::Vector2d(-rawVp.length()/2.0+rawVp.d_cr()-expandLength[3],rawVp.width()/2.0+expandLength[0]);
    norm4 << -cos(yaw), -sin(yaw);
    hPoly.col(3).head<2>() = norm4;
    hPoly.col(3).tail<2>() = point4;
    hPolys_.push_back(hPoly);
  };
}
int main() {
  std::cout << "Hello, World!" << std::endl;
  std::vector<Eigen::Vector3d> statelist;
  std::vector<Eigen::MatrixXd> hPolys;
  bool is_ok = getRectangleConst(statelist, hPolys);
  if (is_ok) {
    std::cout << "have safe h polys" << std::endl;
  } else {
    std::cout << "Error! no safe h polys" << std::endl;
  }
  return 0;
}
