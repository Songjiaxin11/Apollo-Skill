#include "debugger.h"
#include "basic-functions.h"
#include "robot-config.h"
#include "geometry.h"
#include "usercontrol.h"
#include "position.h"
#include "adjusment.h"
#include "parameters.h"
#include "controller.h"
#include "auto-functions.h"
#include <iostream>
#include "autonomous.h"
#include "chassis.h"
using namespace std;

void debugControl()
{
    enum STATE
    {
        USR = 1,
        GYRO,
        DEG,
        ENC,
        CalRidus,
        FLYW,
        VelData,
        TESTPID
    } state;

    // debug需要的局部变量
    float Lsum, Rsum;
    state = USR;
    int testFlyVel = 0;
    int sum = 1;
    Point pos;
    Position *p = Position::getInstance();

    while (true)
    {
        switch (state)
        {
        // 手动控制【1】
        case USR:
            // baseControl();
            baseControlbyHeading();
            pos = Position::getInstance()->getPos();
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("%lf %lf %lf", pos._x, pos._y, IMUHeading());
            Brain.Screen.setCursor(2, 1);
            Brain.Screen.print("%lf %lf\n", -EncoderL.rotation(degrees), -EncoderR.rotation(degrees));
            std::cout << pos._x << " " << pos._y << " " << IMUHeading() << endl;

            if (press_B)
            {
                press_B = false;
                Position::getInstance()->setGlobalPosition(0, 0);
                Inertial.setHeading(0, rotationUnits::deg);
                Inertial.setRotation(0, rotationUnits::deg);
                clearBrainScr();
                clearControllerScr();
                std::cout << pos._x << " " << pos._y << " " << IMUHeading() << endl;
            }

            if (press_DOWN)
            {
                press_DOWN = false;
                state = GYRO;
                clearBrainScr();
                clearControllerScr();
            }
            if (press_UP)
            {
                press_UP = false;
                state = TESTPID;
                clearBrainScr();
                clearControllerScr();
                Controller.Screen.setCursor(5, 1);
                Controller.Screen.print(CollectFlag ? "        YES" : "        NO ");
                Lsum = Rsum = 0;
            }
            break;
        // 手动测试陀螺仪精度【2】
        case GYRO:
            baseControl();
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("%f", Inertial.rotation());
            Brain.Screen.setCursor(2, 1);
            Brain.Screen.print("%f", IMUHeading());
            cout << Inertial.rotation() << " " << IMUHeading() << endl;
            if (press_B)
            {
                press_B = false;
                Inertial.setRotation(0, vex::rotationUnits::deg);
            }

            if (press_DOWN)
            {
                press_DOWN = false;
                state = DEG;
                clearBrainScr();
                clearControllerScr();
            }
            if (press_UP)
            {
                press_UP = false;
                state = USR;
                clearBrainScr();
                clearControllerScr();
            }
            break;
        // 手动计算定位轮安装角度【3】
        case DEG:
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("Left: %.2f", p->getLMileage());
            Brain.Screen.setCursor(2, 1);
            Brain.Screen.print("Right: %.2f", p->getRMileage());
            cout << "Left: " << p->getLMileage() << " "
                 << "Right: " << p->getRMileage() << endl;
            if (press_A)
            {
                press_A = false;
                Controller.Screen.setCursor(5, 3);
                Controller.Screen.print("%.2f", acos(abs(p->getLMileage() / 220)) * 180 / M_PI);
                Controller.Screen.setCursor(5, 12);
                Controller.Screen.print("%.2f", acos(abs(p->getRMileage() / 220)) * 180 / M_PI);
                Controller.Screen.setCursor(5, 18);
                Controller.Screen.print("%d", sum);
            }

            if (press_B)
            {
                press_B = false;
                EncoderL.setPosition(0, vex::rotationUnits::deg);
                EncoderR.setPosition(0, vex::rotationUnits::deg);
                clearBrainScr();
            }

            if (press_DOWN)
            {
                press_DOWN = false;
                state = ENC;
                clearBrainScr();
                clearControllerScr();
            }
            if (press_UP)
            {
                press_UP = false;
                state = GYRO;
                clearBrainScr();
                clearControllerScr();
            }
            break;
        // 测试定位轮编码器旋转一圈的精度【4】
        case ENC:
            Controller.Screen.setCursor(5, 3);
            Controller.Screen.print("L: %.1f", EncoderL.rotation(degrees));
            Controller.Screen.setCursor(5, 14);
            Controller.Screen.print("R: %.1f", EncoderR.rotation(degrees));
            std::cout << "L: " << EncoderL.rotation(degrees) << "R: " << EncoderR.rotation(degrees) << endl;

            if (press_B)
            {
                press_B = false;
                EncoderL.setPosition(0, vex::rotationUnits::deg);
                EncoderR.setPosition(0, vex::rotationUnits::deg);
                EncoderL.setRotation(0, vex::rotationUnits::deg);
                EncoderR.setRotation(0, vex::rotationUnits::deg);
                clearControllerScr();
            }

            if (press_DOWN)
            {
                press_DOWN = false;
                state = CalRidus;
                clearBrainScr();
                clearControllerScr();
                Lsum = Rsum = 0;
            }
            if (press_UP)
            {
                press_UP = false;
                state = DEG;
                clearBrainScr();
                clearControllerScr();
            }
            break;
        // 手动旋转计算定位轮子旋转中心【5】
        case CalRidus:
            baseControl();
            if (press_B)
            {
                press_B = false;
                Lsum = Rsum = 0;
                EncoderL.setPosition(0, vex::rotationUnits::deg);
                EncoderR.setPosition(0, vex::rotationUnits::deg);
            }

            Lsum = p->getLMileage();
            Rsum = p->getRMileage();
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("Lsum: %.2f", Lsum);
            Brain.Screen.setCursor(2, 1);
            Brain.Screen.print("Rsum: %.2f", Rsum);

            if (press_DOWN)
            {
                press_DOWN = false;
                state = FLYW;
                clearBrainScr();
                clearControllerScr();
            }
            if (press_UP)
            {
                press_UP = false;
                state = ENC;
                clearBrainScr();
                clearControllerScr();
            }
            break;
        // 飞轮曲线拟合（曲线结果输出在控制台哦）【6】
        case FLYW:
            if (press_A)
            {
                press_A = false;
                testFlyWheel();
            }

            if (press_DOWN)
            {
                press_DOWN = false;
                state = VelData;
                Controller.Screen.setCursor(5, 1);
                Controller.Screen.print(CollectFlag ? "        YES" : "        NO ");

                // Brain.Screen.clearScreen();
                clearBrainScr();
                clearControllerScr();
            }
            if (press_UP)
            {
                press_UP = false;
                state = CalRidus;
                clearBrainScr();
                clearControllerScr();
            }
            break;
        // 获取速度波形（曲线结果输出在txt文件）【7】
        case VelData:
            baseControlbyHeading();

            // stop collecting data
            if (press_A)
            {
                CollectFlag = false;
                if (isL)
                    cout << "----------Above is L----------" << endl;
                else
                    cout << "----------Above is R----------" << endl;
                Controller.Screen.setCursor(5, 1);
                Controller.Screen.print(CollectFlag ? "        YES" : "        NO ");
                press_A = false;
            }

            if (press_X)
            {
                press_X = false;
                CollectFlag = false;
                if (isL)
                    cout << "----------Above is L----------" << endl;
                else
                    cout << "----------Above is R----------" << endl;
                isL = !isL;
                Controller.Screen.setCursor(5, 1);
                Controller.Screen.print(isL ? "                      L" : "                      R");
            }

            // start collecting data
            if (press_B)
            {
                CollectFlag = true;
                Controller.Screen.setCursor(5, 1);
                Controller.Screen.print(CollectFlag ? "        YES" : "        NO ");
                press_B = false;
            }

            if (press_DOWN)
            {
                CollectFlag = false;
                press_DOWN = false;
                state = TESTPID;
                // Brain.Screen.clearScreen();
                clearBrainScr();
                clearControllerScr();
            }
            if (press_UP)
            {
                CollectFlag = false;
                press_UP = false;
                state = FLYW;
                clearBrainScr();
                clearControllerScr();
            }
            break;
        // 调试PID【8】
        case TESTPID:
            // baseControlbyHeading();
            // Chassis::getInstance()->chassisBrake(vex::brakeType::hold);
            // Chassis::getInstance()->setStopBrakeType(brakeType::hold);
            pos = Position::getInstance()->getPos();
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("%lf %lf %lf", pos._x, pos._y, IMUHeading());
            cout << pos._x << " " << pos._y << " " << IMUHeading() << endl;
            if (press_A)
            {
                press_A = false;
                // turnTo(180, 80);
                // moveToWithHeading(0, 0, 15, 50);
                // timerForwardWithHeading(120, 150, 0);
                // quickMoveToWithHeading(0, 0, 90, 90);
                turnTo(90,100);

                // Inertial.setRotation(0, vex::rotationUnits::deg);
            }

            if (press_Y)
            {
                press_Y = false;
                //(0, 80);
                // moveToWithHeading(0.291,11.83,310.59, 100);
                // timerForwardWithHeading(-40,300,0);
                // intake3DiscsWithPiston(6.5,-33.369,165.405);
                quickMoveToWithHeading(0, 50, 0, 90);
                // autonomous();

                // timerForwardWithHeading(200, 200, 0);
                // Piston_IntakerLifter.set(true);
                // moveToWithHeading(33.795,33.867,87.22, 100);
            }

            if (press_B)
            {
                Position::getInstance()->setGlobalPosition(0, 0);
                Inertial.setHeading(0, rotationUnits::deg);
                Inertial.setRotation(0, rotationUnits::deg);
                press_B = false;
            }

            if (press_DOWN)
            {
                CollectFlag = false;
                press_DOWN = false;
                state = USR;
                // Brain.Screen.clearScreen();
                clearBrainScr();
                clearControllerScr();
            }
            if (press_UP)
            {
                CollectFlag = false;
                press_UP = false;
                state = VelData;
                clearBrainScr();
                clearControllerScr();
            }
            break;
        }

        Controller.Screen.setCursor(5, 1);
        Controller.Screen.print("%d", state);
        this_thread::sleep_for(10);
    }
}

// #ifdef debug
// // cout<< Position::getInstance()->TrackingWheelLSpeed() << endl;
// std::cout << pos._x << " " << pos._y << " " << IMUHeading() << endl;
// if (UP && !last_UP){
//     Position *p = Position::getInstance();
//     // cout << p->getPos()._x << ", " << p->getPos()._y << ", " << IMUHeading() << endl;
// }
// if (DOWN && !last_DOWN){
//         PID adjPID;
//         adjPID.setCoefficient(0.6, 0, 0);
//         adjPID.setDTolerance(5);
//         adjPID.setErrorTolerance(2);
//         adjPID.setIMax(30);
//         adjPID.setIRange(5);
//         adjPID.setJumpTime(300);

//         // Inertial.setHeading(180, rotationUnits::deg);
//         // AutoPIDAdjustment(adjPID, 20, adjustGetPos, adjustForward, RefreshTime);
//         AutoPIDAdjustment(adjPID, 20, inertialHeading, moveClockwise, RefreshTime);
// }
// #endif