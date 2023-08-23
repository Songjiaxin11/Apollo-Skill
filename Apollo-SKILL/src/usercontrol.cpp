#include "vex.h"
#include "basic-functions.h"
#include "controller.h"
#include "adjusment.h"
#include "parameters.h"
#include "flyWheel.h"
#include "position.h"
#include "chassis.h"
#include "usercontrol.h"
#include "autoAiming.h"
#include <iostream>
#include "autonomous.h"
using namespace std;
using namespace vex;

bool isUsrCtl = true, last_isUsrCtl = true;
/**
 * @brief 底盘车头控制
 *
 */
void baseControl()
{
    double a1, a3, a4;
    a1 = (abs(A1) < 10) ? 0 : A1;
    a3 = (abs(A3) < 10) ? 0 : A3;
    a4 = (abs(A4) < 10) ? 0 : A4;
    Vector v = Vector(a4, a3);
    if (v.mod() > 100)
        v = v / v.mod() * 100;
    Chassis::getInstance()->manualSetRobotVel(v, a1);
}

/**
 * @brief 底盘人头控制
 *
 */
void baseControlbyHeading()
{
    double a1, a3, a4;
    int AA = abs(A1);
    // a1 = (abs(A1) < 10) ? 0 : A1;
    // GYW感觉舒服的旋转,曲线上升
    if (abs(A1) < 2)
        a1 = 0;
    if (abs(A1) >= 2)
    {
        a1 = (AA * AA / 123 - 4 * AA / 123 + 4 / 123) * A1 / abs(A1);
    }
    a3 = (abs(A3) < 20) ? 0 : A3;
    a4 = (abs(A4) < 20) ? 0 : A4;
    Vector v = Vector(a4, a3);
    if (v.mod() > 100)
        v = v / v.mod() * 100;
    Chassis::getInstance()->manualSetWorldVel(v, a1);
}

/**
 * @brief 撞墙重置坐标
 * 按键:X
 */
void initializeVel()
{
    Point pos;
    pos = Position::getInstance()->getPos();
    if (press_X)
    {
        press_X = false;
        Position::getInstance()->setGlobalPosition(0, 0);
        Inertial.setHeading(0, rotationUnits::deg);
        Inertial.setRotation(0, rotationUnits::deg);
        clearBrainScr();
        Brain.Screen.setCursor(2, 2);
        Brain.Screen.print("%.3lf %.3lf %.3lf", pos._x, pos._y, IMUHeading());
    }
}

void intakerControl()
{
    if (R1 && !RIGHT&&!L1)
        moveIntaker(90);
    else if ((RIGHT && !R1)||((L1 && !R1)))
        moveIntaker(-100);
    else
        moveIntaker(0);
}

void intakerLift()
{
    if (Y)
    {
        Piston_IntakerLifter.set(true);
    }
    else
    {
        Piston_IntakerLifter.set(false);
    }
}

void anglerControl()
{

    if (R2)
    {
        Piston_Angler.set(true);
    }
    else
    {
        Piston_Angler.set(false);
    }
}

void triggerControl()
{
    if (L2)
    {
        Piston_Trigger.set(true);
        delay(50);
        Piston_Trigger.set(false);
        delay(119);
    }
}

void flyWheelControl()
{

    if (R2 && !B)
    {
        setFlyWheelSpeed(324-15);
    }
    if (B && !R2)
    {
        setFlyWheelSpeed(0);
    }
    if (!B && !R2)
    {
        setFlyWheelSpeed(329-18);
    }
}

void deployerControl()
{
    if (UP && LEFT)
    {
        Piston_Deployer.set(true);
        setFlyWheelSpeed(0);
    }
    else
    {
        Piston_Deployer.set(false);
    }
}

/**
 * @brief 全场定位自瞄
 * 按键: A2下拉至底端
 */
void positionAiming()
{
    static MyTimer timerTrigger;
    static bool flag = false, lastFlag = false;
    if (A2 > -80)
    {
        timerTrigger.reset();
        flag = false;
    }
    else
    {
        if (!flag && timerTrigger.getTime() > 200)
        {
            flag = true;
        }
    }

    if (flag)
    {
        setAimingStatus(true);
    }
    if (lastFlag && !flag)
    {
        setAimingStatus(false);
        Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);
    }

    lastFlag = flag;
    // if (A)
    // {
    //     setAimingStatus(true);
    // }
    // else
    // {
    //     setAimingStatus(false);
    //     Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);
    // }
}

void head45Degree()
{
    static MyTimer timerTrigger;
    static bool flag = false, lastFlag = false;
    if (A2 > -80)
    {
        timerTrigger.reset();
        flag = false;
    }
    else
    {
        if (!flag && timerTrigger.getTime() > 500)
        {
            flag = true;
        }
    }

    if (flag)
    {
        // turnToLock(135);
        turnToLock(45);
    }
    if (lastFlag && !flag)
    {
        Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);
    }

    lastFlag = flag;
}

void WallShoot()
{

    static MyTimer timerL, timerR;
    static bool flagL = false, lastFlagL = false;
    static bool flagR = false, lastFlagR = false;
    // static float LAngel = 180 + 15.72 - 1.22, RAngel = 256 - 180+10;
    float LAngel, RAngel;
#ifdef SKILL
    LAngel = 15.72 - 1.22, RAngel = 256 + 10;
#endif
#ifdef COMPETITION_LEFT_RED
    LAngel = 180 + 15.72 - 1.22, RAngel = 256 - 180 + 10;
#endif
#ifdef COMPETITION_LEFT_BLUE
    LAngel = 180 + 15.72 - 1.22, RAngel = 256 - 180 + 10;
#endif
    if (A2 < 100)
    {
        timerR.reset();
        flagR = false;
    }
    else
    {
        if (!flagR && timerR.getTime() > 50)
        {
            flagR = true;
        }
    }

    if (A2 > -100)
    {
        timerL.reset();
        flagL = false;
    }
    else
    {
        if (!flagL && timerL.getTime() > 50)
        {
            flagL = true;
        }
    }

    if (flagL)
    {
        turnToLock(LAngel);
    }
    if (lastFlagL && !flagL)
    {
        Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);
    }

    if (flagR)
    {
        turnToLock(RAngel);
    }
    if (lastFlagR && !flagR)
    {
        Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);
    }

    lastFlagL = flagL;
    lastFlagR = flagR;
}

void usrCtlThread(void *childThread)
{
    vex::thread *Thread = (vex::thread *)childThread;
    // Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);//original
    vex::thread *T = NULL;

    while (true)
    {

        if (press_A) // 按一下A 开自动, 再按一下关自动
        {
            isUsrCtl = !isUsrCtl;
            press_A = false;
            cout << "A Pressed" << endl;
            if (T == NULL)
            {
                T = new thread(autonomous);
                cout << "111" << endl;
            }
            else
            {
                T->interrupt();
                delete T;
                T = NULL;
                Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);
            }
        }

        if (isUsrCtl)
        {
            static bool brakeType = false;
            // 底盘控制
            // baseControl();
            baseControlbyHeading();

            // 其余组件控制
            intakerControl();
            anglerControl();
            triggerControl();
            intakerLift();
            flyWheelControl();
            deployerControl();
            WallShoot();
            // head45Degree(); // lock heading 也是A2下拉到底 和全场定位自瞄只能同时存在一个
            initializeVel();
            // positionAiming();

            // 视觉自瞄
          


        } //!!!!!!!!!!!!!!!!!!!!!!
        this_thread::sleep_for(RefreshTime);
    }
}

void autoCtlThread()
{
    // 单状态机实现，状态变量为全局变量，每一个状态完成一个非阻塞自动函数
    while (true)
    {
        this_thread::sleep_for(10);
    }
    return;
}

void usercontrol()
{

    Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);
    Chassis::getInstance()->chassisBrake(vex::brakeType::hold);
    Chassis::getInstance()->setStopBrakeType(brakeType::hold);
    // Position::getInstance()->setGlobalPosition(45, 35);
    setFlyWheelSpeed(0);
    setAimingStatus(false);
    // this_thread::sleep_for(1000);
    cout << "enter user control" << endl;

    thread AutoCtl(autoCtlThread);
    usrCtlThread(&AutoCtl);

    while (true)
    {

        this_thread::sleep_for(10);
        //     // Controller.Screen.setCursor(5, 1);
        //     // Controller.Screen.print(isUsrCtl ? "USR" : "AUTO");
        //     // Point pos = Position::getInstance()->getPos();
        //     // Brain.Screen.clearScreen();
        //     // Brain.Screen.setCursor(2, 2);
        //     // Brain.Screen.print("%.3lf %.3lf %.3lf", pos._x, pos._y, IMUHeading());
        //     // cout << p->getPos()._x << " " << p->getPos()._y << " " << IMUHeading() << endl;00
        //     // std::cout << pos._x << " " << pos._y << " " << IMUHeading() << endl;

        //     //     // 左右定位轮旋转角度
        //     //     // Position *Mileage = Position::getInstance();
        //     //     // Brain.Screen.setCursor(2, 2);
        //     //     // Brain.Screen.print("%.2f %.2f", Mileage->getLMileage(), Mileage->getRMileage());
    }
}
// 111