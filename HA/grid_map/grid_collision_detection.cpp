//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : grid_collision_detection.cpp.cc
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//

#include "grid_collision_detection.h"

PLANNING_NAMESPACE_START

void GridCollisionDetection::setGridSize(int width, int length){
    this->width_ = width;
    this->length_ = length;
}

void GridCollisionDetection::clear() {
    //    for (int i = 0; i < width_; ++i)
    //    {
    //        delete [] grid[i];
    //    }
    //    delete [] grid;
}

void GridCollisionDetection::updateGrid(const std::vector<std::vector<int>> &map) {
    this->grid.clear();
    for (size_t i = 0; i < map.size(); ++i) {
        this->grid.push_back(map[i]);
    }
    // grid.swap(map);
}

bool GridCollisionDetection::configurationTest(double x, double y, double t, bool test) {
    // MLOG(PARKING_PLANNING, INFO) << "GridCollisionDetection::configurationTest grid.size():" << this->grid.size();
    int X = (int)((x - this->x_min_) / Constants::cellSize);
    int Y = (int)((y - this->y_min_) / Constants::cellSize);
    int iX = (int)((x / Constants::cellSize - (long)(x / Constants::cellSize)) * Constants::positionResolution);
    iX = iX > 0 ? iX : 0;
    int iY = (int)((y / Constants::cellSize - (long)(y / Constants::cellSize)) * Constants::positionResolution);
    iY = iY > 0 ? iY : 0;

    if (t < 0.0)
        t += 2 * M_PI;

    if (t > 2 * M_PI)
        t -= 2 * M_PI;

    int iT = (int)(t / Constants::deltaHeadingRad);
    int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
    int cX;
    int cY;
    if (this->grid.size() == 0) {
        return true;
    }
    if (test) {
        MLOG(PARKING_PLANNING, INFO) << "Vehicle_Grid_Model_Start";
        std::string s = "vehicle ";
        for (int i = 0; i < collisionLookuptable[idx].length; ++i) {

            cX = (X + collisionLookuptable[idx].pos[i].x);
            cY = (Y + collisionLookuptable[idx].pos[i].y); //对应完整的格子
            s = s + std::to_string(cX) + "," + std::to_string(cY) + ";";
            if (cX >= 0 && (unsigned int)cX < length_ / Constants::cellSize && cY >= 0 &&
                (unsigned int)cY < width_ / Constants::cellSize) {
                continue;
            }
        }
        MLOG(PARKING_PLANNING, INFO) << s;
        MLOG(PARKING_PLANNING, INFO) << "Vehicle_Grid_Model_End";
    }
    for (int i = 0; i < collisionLookuptable[idx].length; ++i) {

        cX = (X + collisionLookuptable[idx].pos[i].x);
        cY = (Y + collisionLookuptable[idx].pos[i].y); //对应完整的格子

        if (cX >= 0 && (unsigned int)cX <= length_ / Constants::cellSize && cY >= 0 &&
            (unsigned int)cY <= width_ / Constants::cellSize) {
            // std::cout << "cx," <<cX << ",cy," << cY <<std::endl;
            if ((unsigned int)cX >= grid.size())
                continue;
            if ((unsigned int)cY >= grid.at(cX).size())
                continue;
            if (grid[cX][cY]) {
                return true;
            }
        }
    }
    return false;
}

void GridCollisionDetection::setGridOri(const double x_min, const double y_min) {
    this->x_min_ = x_min;
    this->y_min_ = y_min;
}

void GridCollisionDetection::updateLookupTable() {
    memcpy(collisionLookuptable, SingletonLookup::GetInstance()->collisionLookuptable, sizeof(collisionLookuptable));
}

PLANNING_NAMESPACE_END