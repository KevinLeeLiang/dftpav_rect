//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : gird_map.cpp.cc
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//

#include "grid_map.h"
#include <limits>

GridMap::GridMap() {
  this->map_length_ = 0;
  this->map_width_ = 0;
  this->updateLookupTable();
}

GridMap::~GridMap() {
  //        for (size_t i = 0; i < this->map_length_; i++) {
  //          delete bin_map[i];
  //        }
  //        delete[]bin_map;
}

void GridMap::BuildGridMap(const std::vector<POINT2D> &obs,
                           const Box2d &map_bound) {
  float obstacle_grid_resolution = Constants::cellSize;
  this->map_width_ = uint64_t(map_bound.width() / obstacle_grid_resolution);
  this->map_length_ = uint64_t(map_bound.length() / obstacle_grid_resolution);
  this->grid_map.clear();
  this->grid_map.resize(this->map_length_);

  for (size_t i = 0; i < this->grid_map.size(); i++) {
    this->grid_map.at(i).resize(this->map_width_, 0);
  }
  for (const auto &p : obs) {
    uint64_t index_x =
        uint64_t((p.x() - map_bound.x_min()) / obstacle_grid_resolution);
    if (index_x >= this->map_length_)
      continue;
    uint64_t index_y =
        uint64_t((p.y() - map_bound.y_min()) / obstacle_grid_resolution);
    if (index_y >= this->map_width_)
      continue;
    grid_map[index_x][index_y] = 1;
  }

  std::cout << "map_bound" << std::endl;
  std::cout << map_bound.x_min() << "," << map_bound.y_min() << ","
            << map_bound.x_max() << "," << map_bound.y_max() << ","
            << map_bound.Width() << "," << map_bound.Length() << std::endl;
  std::cout << "grid_map start" << std::endl;
  //    std::string s;
  //    for (size_t i = 0; i < grid_map.size(); i++) {
  //        for (size_t j = 0; j < grid_map.at(i).size(); j++) {
  //            s = s + ",";
  //            s = s + std::to_string(grid_map[i][j]);
  //        };
  //        MLOG(PARKING_PLANNING, INFO) << s;
  //        s = "";
  //    }
  this->setGridOri(map_bound.x_min(), map_bound.y_min());
  this->setGridSize(this->map_width_, this->map_length_);
  this->updateGrid(this->grid_map);
  this->updateLookupTable();
}
