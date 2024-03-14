//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : singleton_lookup.cpp.cc
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//

#include "singleton_lookup.h"
#include <math.h>
#include <vector>

SingletonLookup *SingletonLookup::m_plan_config = nullptr;
common::VehicleParam *SingletonLookup::ptr_veh_mode_ = nullptr;

struct point {
    double x;
    double y;
};

static void collisionLookup(Constants::config *lookup, common::VehicleParam* ptr_veh_mode) {
    // float length = vehicle_param.length + vehicle_param.length_expand;
    float cellsize = Constants::cellSize;
    float width = ptr_veh_mode->width() / cellsize;

    float width_half = (width / 2); //膨胀后的半车宽

    float length_rear = (ptr_veh_mode->length()/2) /
                        cellsize; //后轴到后端边缘长度

    float length_front = (ptr_veh_mode->length()/2) /
                         cellsize;

    // int bbSize = (int)((sqrt(width_half * width_half + length_front*
    // length_front) + 2) /
    //                                                    Constants::cellSize);
    //                                                    //后轴中点到左(右)前角长度
    int bbSize = (int)((sqrt(width_half * width_half + length_front * length_front) + 2) / Constants::cSize);
    // cell size
    const int size = bbSize * 2;
    //  const int cSize = Constants::cSize;

    // ______________________
    // VARIABLES FOR ROTATION
    // center of the rectangle
    point c;
    point temp;
    // points of the rectangle
    point p[4];
    point nP[4];

    // turning angle
    double theta;

    // ____________________________
    // VARIABLES FOR GRID TRAVERSAL
    // vector for grid traversal
    point t;
    point start;
    point end;
    // cell index
    int X;
    int Y;
    // t value for crossing vertical and horizontal boundary
    double tMaxX;
    double tMaxY;
    // t value for width/heigth of cell
    double tDeltaX;
    double tDeltaY;
    // positive or negative step direction
    int stepX;
    int stepY;
    // grid
    std::vector<char> cSpace;
    cSpace.resize(size * size);
    // bool cSpace[size * size];// vehicle occupied grid area
    bool inside = false;
    int hcross1 = 0;
    int hcross2 = 0;

    // _____________________________
    // VARIABLES FOR LOOKUP CREATION
    int count = 0;
    const int positionResolution = Constants::positionResolution;
    const int positions = positionResolution * positionResolution; // 100

    std::vector<point> points(positions);

    // generate all discrete positions within one cell
    for (int i = 0; i < positionResolution; ++i) {
        for (int j = 0; j < positionResolution; ++j) {
            points[positionResolution * i + j].x =
                static_cast<float>(j) / positionResolution; // 1.f / positionResolution * j;
            points[positionResolution * i + j].y =
                static_cast<float>(i) / positionResolution; // 1.f / positionResolution * i;
        }
    }

    auto w_sz = width_half / Constants::cSize;
    auto lr_sz = length_rear / Constants::cSize;
    auto lf_sz = length_front / Constants::cSize;
    for (int q = 0; q < positions; ++q) {
        // set the starting angle to zero;
        theta = 0;

        // set points of rectangle (vehicle center) transform the center of vehicle
        c.x = (double)size / 2 + points[q].x;
        c.y = (double)size / 2 + points[q].y;

        // wq 修改车辆轮廓计算参数与分离轴计算参数一致

        p[0].x = int(c.x - lr_sz);
        p[0].y = int(c.y - w_sz);

        p[1].x = int(c.x - lr_sz);
        p[1].y = int(c.y + w_sz);

        p[2].x = int(c.x + lf_sz);
        p[2].y = int(c.y + w_sz);

        p[3].x = int(c.x + lf_sz);
        p[3].y = int(c.y - w_sz);

        for (int o = 0; o < Constants::headings; ++o) {
            // initialize cSpace
            // for (int i = 0; i < size; ++i) {
            //     for (int j = 0; j < size; ++j) {
            //         cSpace[i * size + j] = false;
            //     }
            // }
            std::fill(cSpace.begin(), cSpace.end(), false);

            // shape rotation
            for (int j = 0; j < 4; ++j) {
                // translate point to origin
                temp.x = p[j].x - c.x;
                temp.y = p[j].y - c.y;

                // rotate and shift back
                nP[j].x = temp.x * cos(theta) - temp.y * sin(theta) + c.x;
                nP[j].y = temp.x * sin(theta) + temp.y * cos(theta) + c.y;
            } // rotate the vehicle

            // create the next angle
            // theta -= Constants::deltaHeadingRad;
            theta += Constants::deltaHeadingRad;

            // cell traversal clockwise
            for (int k = 0; k < 4; ++k) {
                // create the vectors clockwise
                if (k < 3) {
                    start = nP[k];
                    end = nP[k + 1];
                } else {
                    start = nP[k];
                    end = nP[0];
                }

                // set indexes
                X = (int)start.x;
                Y = (int)start.y;

                cSpace[Y * size + X] = true;
                t.x = end.x - start.x;
                t.y = end.y - start.y;

                stepX = t.x > 0; // represent the increase direction
                stepY = t.y > 0;

                if (stepX == 0)
                    stepX = -1;

                if (stepY == 0)
                    stepY = -1;

                // width and height normalized by t
                if (t.x != 0) {
                    tDeltaX = 1.f / fabs(t.x);
                } else {
                    tDeltaX = 1000;
                }

                if (t.y != 0) {
                    tDeltaY = 1.f / fabs(t.y);
                } else {
                    tDeltaY = 1000;
                }

                // set maximum traversal values
                if (stepX > 0) {
                    tMaxX = tDeltaX * (1 - (start.x - (long)start.x));
                } else {

                    if ((start.x - (long)start.x) == 0) {
                        tMaxX = tDeltaX * 0.1;
                    } else {
                        tMaxX = tDeltaX * (start.x - (long)start.x);
                    }
                    //          tMaxX = tDeltaX * (start.x - (long)start.x);
                }

                if (stepY > 0) {
                    tMaxY = tDeltaY * (1 - (start.y - (long)start.y));
                } else {
                    if ((start.y - (long)start.y) == 0) {
                        tMaxY = tDeltaY * 0.1;
                    } else {
                        tMaxY = tDeltaY * (start.y - (long)start.y);
                    }
                    //          tMaxY = tDeltaY * (start.y - (long)start.y);
                }

                while ((int)end.x != X || (int)end.y != Y) {
                    // only increment x if the t length is smaller and the result will be
                    // closer to the goal
                    if (tMaxX < tMaxY && std::abs(X + stepX - (int)end.x) < std::abs(X - (int)end.x)) {
                        tMaxX = tMaxX + tDeltaX;
                        X = X + stepX;
                        cSpace[Y * size + X] = true;
                        // only increment y if the t length is smaller and the result will
                        // be closer to the goal
                    } else if (tMaxY < tMaxX && std::abs(Y + stepY - (int)end.y) < std::abs(Y - (int)end.y)) {
                        tMaxY = tMaxY + tDeltaY;
                        Y = Y + stepY;
                        cSpace[Y * size + X] = true;
                    } else if (2 >= std::abs(X - (int)end.x) + std::abs(Y - (int)end.y)) {
                        if (std::abs(X - (int)end.x) > std::abs(Y - (int)end.y)) {
                            X = X + stepX;
                            cSpace[Y * size + X] = true;
                        } else {
                            Y = Y + stepY;
                            cSpace[Y * size + X] = true;
                        }
                    } else {
                        // this SHOULD NOT happen
                        // std::cout << "\n--->tie occured, please check for error in
                        // script\n"; // need to modify
                        break;
                    }
                }
            }

            // FILL THE SHAPE
            for (int i = 0; i < size; ++i) {
                // set inside to false
                inside = false;

                for (int j = 0; j < size; ++j) {

                    // determine horizontal crossings
                    for (int k = 0; k < size; ++k) {
                        if (cSpace[i * size + k] && !inside) {
                            hcross1 = k;
                            inside = true;
                        }

                        if (cSpace[i * size + k] && inside) {
                            hcross2 = k;
                        }
                    }

                    // if inside fill
                    if (j > hcross1 && j < hcross2 && inside) {
                        cSpace[i * size + j] = true;
                    }
                }
            }

            // GENERATE THE ACTUAL LOOKUP
            count = 0;

            for (int i = 0; i < size; ++i) {
                for (int j = 0; j < size; ++j) {
                    if (cSpace[i * size + j]) {
                        lookup[q * Constants::headings + o].pos[count].x = j - (int)c.x;
                        lookup[q * Constants::headings + o].pos[count].y = i - (int)c.y;
                        count++;
                    }
                }
            }

            lookup[q * Constants::headings + o].length = count;
        }
    }

}

SingletonLookup *SingletonLookup::GetInstance() {
    if (nullptr == m_plan_config) {
        m_plan_config = new SingletonLookup();
        ptr_veh_mode_ = new common::VehicleParam();
        if (!m_plan_config->getInitStatus())
            m_plan_config->init();
    }

    return m_plan_config;
}

void SingletonLookup::init() {
    if (nullptr == m_plan_config)
        return;

    is_init = true;
    // 初始化lookup
    collisionLookup(collisionLookuptable, ptr_veh_mode_);
}

