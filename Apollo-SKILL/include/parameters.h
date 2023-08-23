#ifndef PARAMETERS_H_
#define PARAMETERS_H_
#include "calc.h"
#include "vex.h"
#include <math.h>
#include <fstream>

const double PI = M_PI;
const float WheelRadius = 3.492;                // 主动轮半径 cm
const float TrackingWheelRadius = 3.492;        // 定位轮半径 cm

const double LEncoderAngle = acos(157.4157 / 220);//44.31°
const double REncoderAngle = acos(158.4213/ 220);//43.9375°

const double targetPosX = 15;
const double targetPosY = 297;
const double diff=10;
const double ofset=1;
// const double targetPosX = -26.95;
// const double targetPosY = 293.977;

extern bool CollectFlag;
extern bool isL;

const float RefreshTime = 10;                   // 刷新时间 ms
const float positionRefreshTime = 10;           // 定位刷新时间 ms

extern mutex m;

const int CENTER_FOV = 150;
// centre at 158
// extern bool fourBarFlag;

// #define VisionSensor__SIG_TAR VisionSensor__SIG_RED
// #define VisionSensor__SIG_TAR VisionSensor__SIG_BLUE

#define COMPETITION_LEFT

#ifdef RED_ALLIANCE
const float color_range[2] = {340, 50};
#define VisionSensor__SIG_TAR VisionSensor__SIG_RED
#endif

#ifdef BLUE_ALLIANCE
const float color_range[2] = {190, 280};
#define VisionSensor__SIG_TAR VisionSensor__SIG_BLUE
#endif

#endif