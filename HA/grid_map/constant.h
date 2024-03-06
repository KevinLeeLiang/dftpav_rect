//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : constant.h
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//

#ifndef CONSTANT_H
#define CONSTANT_H
#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#else
#endif

namespace Constants {

static const float cellSize = 0.1;
static const float cSize = 1.0;

// vehicle parameter

static const float width = 8.0;   // lookup未使用
static const float length = 12.0; // lookup未使用

static const int headings = 360; // 360 / 72 = 5 rad
static const float deltaHeadingRad = M_PI * 2.0 / headings;
static const int maxIterations = 10000;

static const float wSmoothness = 1.0;
static const float wObstacle = 1.0;

static const float alpha = 0.5;

static const float totalWeight = wObstacle + wSmoothness;

// bounding box size length/width
static const int positionResolution = 3;
static const int positions = positionResolution * positionResolution;
static const float node_radius = 0.25;

struct relPos {
    /// the x position relative to the center
    int x;
    /// the y position relative to the center
    int y;
    relPos():x(0),y(0){}
    relPos(int _x, int _y):x(_x),y(_y){}
};

struct config {
    /// the number of cells occupied by this configuration of the vehicle
    int length;
    //  wq 修改数组大小为256，对应车辆膨胀后的面积对应的网格数量
    // relPos pos[64];
    relPos pos[7200];
    config():length(1){}
    config(int _length):length(_length){}
};

} // namespace Constants

#endif // NODE_ERROR_CONSTANT_H
