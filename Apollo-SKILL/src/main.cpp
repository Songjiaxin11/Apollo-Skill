/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       SJTU VEX                                                  */
/*    Created:      Wed Dec 21 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "basic-functions.h"
#include "auto-functions.h"
#include "usercontrol.h"
#include "position.h"
#include "autonomous.h"
#include "flyWheel.h"
#include "chassis.h"
#include "debugger.h"
#include "autoAiming.h"
#include "controller.h"
#include "my-timer.h"
#include <stdlib.h>
#include <iostream>
#include "adjusment.h"
using namespace std;
using namespace vex;
// #define COMPETITION
// MyTimer auto_time;
#ifdef COMPETITION
competition Competition;
#endif
// 定义Competiton会奇妙进入手动程序

int main()
{
    // vexcodeInit();
    cout << "start" << endl;
    Inertial.calibrate();
    waitUntil(!Inertial.isCalibrating());
    this_thread::sleep_for(200);
    cout << "calibrated!" << endl;
    Controller.Screen.setCursor(5, 1);
    Controller.Screen.print("         calibrated!");
    Position *p = Position::getInstance();
    setAimingStatus(false);

    setFlyWheelSpeed(0);
    thread FlyWheelControl(updateFlyWheel);
    thread UpdatePos(updatePosition);
    thread UpdateChassis(updateChassis);
    thread AutonAiming(autonAiming);
    // thread AutonAimingVision(autonAiming_vision);
    thread TController(defineController);
    thread TestDistance(updateDistance);
    // thread ShootInAuto(updateDistanceInAuto);
    // Position::getInstane()->setGlobalPosition(45, 35, 45);
    Position::getInstance()->setGlobalPosition(0, 0);

#ifdef COMPETITION
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
#endif

#ifdef SKILL
    Position::getInstance()->setGlobalPosition(104.894, 19.0832);
#endif
    // usercontrol();
    // moveToWithHeading(0, 100, 0, 100);

#ifdef debug
    debugControl();
#endif

    while (true)
    {
        Point pos = Position::getInstance()->getPos();
#ifdef TEST
        if (press_B)
        {
            autonomous();
            press_B = false;
        }
#endif
        // Brain.Screen.setCursor(18, 2);
        // Brain.Screen.print("%.3lf  s", auto_time.getTimeDouble() / 1000);
        // cout << auto_time.getTimeDouble() << " s" << endl;
        Brain.Screen.setCursor(2, 2);
        Brain.Screen.print("%.3lf %.3lf %.3lf", pos._x, pos._y, IMUHeading());
        // cout << p->getPos()._x << " " << p->getPos()._y << " " << IMUHeading() << endl;
         
        this_thread::sleep_for(10);
    }
}
