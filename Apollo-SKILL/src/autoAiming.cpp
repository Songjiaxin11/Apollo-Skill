#include "basic-functions.h"
#include "PID.h"
#include "parameters.h"
#include "chassis.h"
#include "position.h"
#include "auto-functions.h"
#include "autoAiming.h"
#include <iostream>
using namespace std;

bool isAiming = false;
int offset = 0;
bool isAiming_vision = false;
// int autonAiming()
// {
//     float output = 0;
//     auto pid = PID();
//     pid.setTarget(CENTER_FOV + offset);
//     pid.setCoefficient(0.35, 0, 0.85);
    
//     // pid.setCoefficient(2, 0, 0);
//     pid.setIMax(25);
//     pid.setIRange(10);
//     pid.setErrorTolerance(5);
//     pid.setDTolerance(5);
//     pid.setJumpTime(100);
//     while (true)
//     {
//         pid.setTarget(CENTER_FOV + offset);
//         VisionSensor.takeSnapshot(VisionSensor__SIG_TAR);
//         pid.update(VisionSensor.largestObject.centerX);
//         output = pid.getOutput();
//         if (isAiming)
//         {
//             // if ((Point(300.777, 25.0472) - Position::getInstance()->getPos()).mod() <= 180)
//             Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), -output);

//             // else
//                 // aimAt(300.777, 25.0472);
//             // cout << "isAiming" << endl;
//             cout << "pid: " << output << endl;
//         }
//         this_thread::sleep_for(20);
//     }
//     return 1;
// }

int autonAiming()
{
    Point _tarPos = Point(targetPosX, targetPosY);
    Point pos = Position::getInstance()->getPos();
    double _tarAng = 90 - (_tarPos - pos).dir();
    _tarAng += 180;
    while (_tarAng >= 360)
        _tarAng -= 360;
    while (_tarAng < 0)
        _tarAng += 360;

    auto pid = PID();
    pid.setTarget(_tarAng);
    pid.setCoefficient(2, 0.2, 8);
    // pid.setCoefficient(2, 0, 0);
    pid.setIMax(40);
    pid.setIRange(10);
    pid.setErrorTolerance(2);
    pid.setDTolerance(5);
    pid.setJumpTime(100);
    MyTimer mytimer;

    while (true)
    {
        double speedX = Position::getInstance()->getXSpeed();
        double speedY = Position::getInstance()->getYSpeed();
        double dist = (Position::getInstance()->getPos() - Point(targetPosX, targetPosY)).mod();
        double discSpeed = 420; // cm * s ^ -1
        // 出射速度 460 ~ 480 cm/s
        double t = dist / discSpeed; /*????*/

        _tarPos = Point(targetPosX - speedX * t, targetPosY - speedY * t);
        pos = Position::getInstance()->getPos();
        _tarAng = 90 - (_tarPos - pos).dir();
        _tarAng += 180;
        double d = _tarAng - IMUHeading();
        while (d < -180){
            d += 360;
            _tarAng += 360;
        }
        while (d > 180){
            d -= 360;
            _tarAng -= 360;
        }
        pid.setTarget(_tarAng);
        pid.update(IMUHeading());
        double output = pid.getOutput(); 
        if(abs(output) >= 60) output = 60 * sign(output);     

        if (isAiming)
        {
            aimPreciselyAt(targetPosX - speedX * t, targetPosY - speedY * t);
            // aimPreciselyAt(targetPosX, targetPosY);
            Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), output);
        }
        this_thread::sleep_for(20);
    }
    return 1;
}

void setAimingStatus(bool _input)
{
    isAiming = _input;
    offset = 0;
}

void setAimingStatus(bool _input, int _offset)
{
    isAiming = _input;
    offset = _offset;
}



void setVisionAimingStatus(bool _input)
{
    isAiming_vision = _input;
    offset = 5;
}

int autonAiming_vision()
{
    float output = 0;
    auto pid = PID();
    pid.setTarget(CENTER_FOV + offset);
    pid.setCoefficient(0.35, 0, 0.85);
    
    // pid.setCoefficient(2, 0, 0);
    pid.setIMax(25);
    pid.setIRange(10);
    pid.setErrorTolerance(5);
    pid.setDTolerance(5);
    pid.setJumpTime(100);
    while (true)
    {
        pid.setTarget(CENTER_FOV + offset);
        VisionSensor.takeSnapshot(VisionSensor__SIG_TAR);
        pid.update(VisionSensor.largestObject.centerX);
        output = pid.getOutput();
        if (isAiming_vision)
        {
            // if ((Point(300.777, 25.0472) - Position::getInstance()->getPos()).mod() <= 180)
            Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), -output);

            // else
                // aimAt(300.777, 25.0472);
            // cout << "isAiming" << endl;
            // cout << "pid: " << output << endl;
        }
        this_thread::sleep_for(20);
    }
    return 1;
}
