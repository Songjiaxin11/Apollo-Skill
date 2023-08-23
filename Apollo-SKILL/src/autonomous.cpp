#include "autonomous.h"
#include "auto-functions.h"
#include "chassis.h"
#include "position.h"
#include "basic-functions.h"
#include "flyWheel.h"
#include "vex.h"
#include <iostream>
#include "my-timer.h"
#include "adjusment.h"
using namespace std;
using namespace vex;
// 技能赛一定要打开deploy!!!
//  #define deploy

// 第一个三盘防挡
#define blocking

void testRoller()
{
    turnRoller();
    cout << "ending autonomous" << endl;
    // ChassisControl.interrupt();
    while (true)
    {
        Position *p = Position::getInstance();
        cout << p->getPos()._x << " " << p->getPos()._y << " " << IMUHeading() << endl;
        this_thread::sleep_for(100);
    }
}

/**
 * @brief 技能赛自动
 *
 */

#ifdef SKILL
void autonomous()
{
    thread ShootInAuto(updateDistanceInAuto);
    cout << "enter auto functions" << endl;
    Inertial.setHeading(0, rotationUnits::deg);
    Position *p = Position::getInstance();
    Chassis *c = Chassis::getInstance();
    // Chassis::getInstance()->chassisBrake(vex::brakeType::hold);
    // Chassis::getInstance()->setStopBrakeType(brakeType::hold);
    // 前进到导入台，清空
    /*------------------------------------------------------------------------------------------*/
    MyTimer timer;
    timer.reset();
    setFlyWheelSpeed(335 - diff);
    this_thread::sleep_for(150);
    moveIntaker(90 - 2 - 5);
    aimPreciselyAt(299.739, 21.2025, 0 + ofset);
    this_thread::sleep_for(50);
    shoot(3, 200);
    this_thread::sleep_for(100);
    shoot(3, 400); // original
    setFlyWheelSpeed(335 - 0.5 - diff);
    aimPreciselyAt(299.739, 21.2025, 3.8 + ofset);
    this_thread::sleep_for(50);
    shoot(4, 350);
    /*------------------------------------------------------------------------------------------*/
    // 三连
    quickMoveToWithHeading(204.73 - 5, 31.885, 45, 100); // first
    this_thread::sleep_for(50);
    quickMoveToWithHeading(204.73 - 5, 95.5197, 45, 60);
    setFlyWheelSpeed(325 - 6 - 2 - diff);
    aimPreciselyAt(299.739, 21.2025, 0.3 + ofset);
    moveIntaker(-100);
    shoot(1);
    setFlyWheelSpeed(342 - 2 - 2 - diff);
    this_thread::sleep_for(50);
    shoot(1);
    setFlyWheelSpeed(342 - 5 - 2 - diff);
    this_thread::sleep_for(50);
    shoot(1);

    /*------------------------------------------------------------------------------------------*/
    // 三斜
    moveIntaker(70);
    turnTo(45);
    // this_thread::sleep_for(100);
    quickMoveToWithHeading(172.269, 103.18 - 3 - 3, 45, 60);                // 第一个
    quickMoveToWithHeading(249.11 + 8 + 10 + 5, 185.445 - 16 + 15, 45, 60); // 第三个过冲
    timerForwardWithHeading(200, 300, 0);
    setFlyWheelSpeed(331 - 3 - diff);

    // 三竖
    Chassis::getInstance()->chassisBrake(vex::brakeType::hold);
    Chassis::getInstance()->setStopBrakeType(brakeType::hold);
    moveIntaker(-100);
    quickMoveToWithHeading(213.864, 119.094 - 3 - 3, 135, 100); //
    aimPreciselyAt(299.739, 21.2025, 0.7 + ofset);
    shoot(1);
    setFlyWheelSpeed(333 + 2 - diff);
    shoot(1);
    moveIntaker(70);
    setFlyWheelSpeed(333 + 3 + 3 - diff);
    shoot(1);

    this_thread::sleep_for(100);
    turnTo(135);
    this_thread::sleep_for(50);
    quickMoveToWithHeading(233.283, 117.792 - 1.5 - 3, 135, 70); // 第一个盘
    quickMoveToWithHeading(315.169 + 10 - 4, 117.233, 135, 70);
    quickMoveToWithHeading(315.169 + 10 - 4, 117.233 - 30 - 5, 135, 70);
    setFlyWheelSpeed(350 - 3 - 3 - diff);
    Piston_Angler.set(true);
    aimPreciselyAt(299.739, 21.2025, 3.8 + ofset);
    moveIntaker(-100);
    shoot(1);
    setFlyWheelSpeed(347 + 18 - diff);
    shoot(1);
    setFlyWheelSpeed(347 + 3 + 5 - diff);
    shoot(1);
    timerForwardWithHeading(200, 200, 0); // 倒退防撞
    Piston_Angler.set(false);
    Chassis::getInstance()->chassisBrake(vex::brakeType::coast);
    Chassis::getInstance()->setStopBrakeType(brakeType::coast);
    // 第一个三盘
    setFlyWheelSpeed(330.5 - diff);
    moveIntaker(85);
    quickMoveToWithHeading(289.68 - 3, 169.37 - 4 + 3, 236, 100); // 先停一下
    Piston_IntakerLifter.set(true);
    // quickMoveToWithHeading(291, 171, 225, 100);
    quickMoveToWithHeading(156.437 - 30 - 10, 81.734 - 30 - 9 - 5 - 4, 236, 60); // 第一个三盘
    this_thread::sleep_for(100);
    timerForwardWithHeading(100, 200, 0);
    this_thread::sleep_for(20);
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(175);
    Piston_IntakerLifter.set(true);
    this_thread::sleep_for(400); // 第一个
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(175);
    Piston_IntakerLifter.set(true);
    this_thread::sleep_for(400); // 第二个
    Piston_IntakerLifter.set(false);
    // this_thread::sleep_for(1000);
    // quickMoveToWithHeading(180, 0, 282.1, 100); // barrier外一点
    quickMoveToWithHeading(160, -15, 282.1, 100);
    moveIntaker(-100);
    // quickMoveToWithHeading(200, 0, 282.1, 100);
    quickMoveToWithHeading(214.645, -15, 255.96 + 3, 100);
    Piston_Angler.set(true);
    aimPreciselyAt(299.739, 21.2025, 3.5 + 3 + 8 + ofset);
    shoot(3);
    Piston_Angler.set(false);
    // 吃中线两盘
    moveIntaker(85);
    setFlyWheelSpeed(325 - 6 - 2 - diff);
    timerForwardWithHeading(100, 200, 0);
    quickMoveToWithHeading(113-30, 85-30, 315, 100);
    this_thread::sleep_for(50);
    timerForwardWithHeading(400, 100, 0);
    turnTo(0);
    this_thread::sleep_for(50);
    quickMoveToWithHeading(129, 110, 0, 100);
    timerForwardWithHeading(200, 100, 0);
    this_thread::sleep_for(50);
    moveIntaker(-100);
    quickMoveToWithHeading(177, 126, 310, 100);
    aimPreciselyAt(299.739, 21.2025, 3.5 + 3 + 8 + ofset);
    shoot(3);
    moveIntaker(85);
    /*
        // 中线三叠2
        setFlyWheelSpeed(333 - 2 - diff);
        moveIntaker(85);
        Piston_IntakerLifter.set(true);
        // quickMoveToWithHeading(84.148 - 10 - 6 - 2, 55.916 - 13 - 3, 300, 100);
        quickMoveToWithHeading(96.4867 - 20 - 8, 48.0103 - 20 + 8, 298.231, 100);
        timerForwardWithHeading(120, 200, 0);
        Piston_IntakerLifter.set(false);
        this_thread::sleep_for(175);
        Piston_IntakerLifter.set(true);
        this_thread::sleep_for(400); // 第一个
        Piston_IntakerLifter.set(false);
        this_thread::sleep_for(175);
        Piston_IntakerLifter.set(true);
        this_thread::sleep_for(400); // 第二个
        Piston_IntakerLifter.set(false);
        quickMoveToWithHeading(180 + 26 + 3, 1 - 5, 282.1, 100);
        moveIntaker(-100);
        // quickMoveToWithHeading(200,-5,282.1,100);
        // Piston_IntakerLifter.set(false);
        Piston_Angler.set(true);
        aimPreciselyAt(299.739, 21.2025, 1.5 + 2 + 2 + 4 + 1 + ofset);
        shoot(1);
        setFlyWheelSpeed(347 + 15 + 10 + 4 - diff);
        shoot(1);
        setFlyWheelSpeed(347 + 3 + 5 + 5 - diff);
        shoot(1);
        Piston_Angler.set(false);
        moveIntaker(85);
        setFlyWheelSpeed(0); //
        */

    // roller1
    Chassis::getInstance()->chassisBrake(vex::brakeType::hold);
    Chassis::getInstance()->setStopBrakeType(brakeType::hold);
    quickMoveToWithHeading(79.0851 - 35 + 5, 10 + 10, 190, 100);
    // quickMoveToWithHeading(79.0851 - 35 + 5 - 5, -3.52489 - 1, 190, 100); // 靠近roller
    quickMoveToWithHeading(79.0851 - 35, -3.52489 - 1 - 6, 190, 100);
    moveIntaker(-1);
    // timerForwardWithHeading(60, 300, 0); // 向前冲!
    this_thread::sleep_for(100);
    timerForwardWithHeading(60, 300, 0);
    Chassis::getInstance()->autoSetRobotVel(Vector(0, 20), 25);
    Motor_Intaker1.resetPosition();
    moveIntaker(-100);
    MyTimer timer2;
    timer2.reset();

    waitUntil(Motor_Intaker1.position(rotationUnits::deg) <= -220 && timer2.getTimeDouble() <= 800); // 转动Roller
    moveIntaker(1);
    this_thread::sleep_for(100); // 防止有的roller太松带着惯性滚
    timerForwardWithHeading(-200, 100, 0);
    this_thread::sleep_for(50);
    moveIntaker(100);

    // roller2
    quickMoveToWithHeading(1, 33.660, -100, 100);
    this_thread::sleep_for(300);
    moveIntaker(0);
    timerForwardWithHeading(40, 200, 0); // 向前冲!
    Chassis::getInstance()->autoSetRobotVel(Vector(0, 20), -25);
    Motor_Intaker1.resetPosition();
    moveIntaker(-100);
    MyTimer timer3;
    timer3.reset();

    waitUntil(Motor_Intaker1.position(rotationUnits::deg) <= -200 && timer3.getTimeDouble() <= 1000); // 转动Roller
    moveIntaker(0);
    timerForwardWithHeading(-100, 150, 0);
    this_thread::sleep_for(100);
    this_thread::sleep_for(200);

    // deploy
    quickMoveToWithHeading(34 - 10, 20 - 10, 225, 100);
    Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);
    ShootInAuto.interrupt();
#ifdef deploy
    Piston_Deployer.set(true);
    this_thread::sleep_for(1000);
    Piston_Deployer.set(false);
    this_thread::sleep_for(100);
    Piston_Deployer.set(true);
    this_thread::sleep_for(1000);
    Piston_Deployer.set(false);
    this_thread::sleep_for(100);
    Piston_Deployer.set(true);
    this_thread::sleep_for(200);
    Piston_Deployer.set(false);
#endif
    Controller.Screen.setCursor(5, 1);
    Controller.Screen.print(timer.getTimeDouble());
    cout << timer.getTimeDouble() << endl;
    this_thread::sleep_for(2000);
}
#endif

#ifdef COMPETITION_LEFT_RED // 三块
void autonomous()
{
    // 防四盘检测线程
    thread ShootInAuto(updateDistanceInAuto);
    MyTimer auto_time;
    cout << "enter auto functions" << endl;
    Inertial.setHeading(0, rotationUnits::deg);
    Position *p = Position::getInstance();
    Chassis *c = Chassis::getInstance();
    Chassis::getInstance()->chassisBrake(vex::brakeType::hold);
    Chassis::getInstance()->setStopBrakeType(brakeType::hold);
    setFlyWheelSpeed(396 - 16.999 + 3); // 开飞轮

    // 中线三盘
    moveIntaker(80);
    Piston_IntakerLifter.set(true);
    quickMoveToWithHeading(-5, 55, 0, 100);
    this_thread::sleep_for(100);
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(200);
    Piston_IntakerLifter.set(true);
    this_thread::sleep_for(300); // 第一个
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(200);
    Piston_IntakerLifter.set(true);
    this_thread::sleep_for(300); // 第二个
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(300);
    moveIntaker(80); // 重开intaker

#ifdef blocking
    timerForwardWithHeading(-100, 200, 0);
    this_thread::sleep_for(200);
    aimPreciselyAt(133.038, 262.32, 5);
#endif

    moveIntaker(-100); // 吐盘防卡
    this_thread::sleep_for(100);
    shoot(3);
    Chassis::getInstance()->chassisBrake(vex::brakeType::coast);
    Chassis::getInstance()->setStopBrakeType(brakeType::coast);
    this_thread::sleep_for(100);

    // 第二个三盘
    moveIntaker(80); // 重开intaker
    setFlyWheelSpeed(398 - 16.999 + 3 - 0.5);
    Piston_IntakerLifter.set(true);
    quickMoveToWithHeading(19.124, 32.16 - 4, 95.894, 100);
    this_thread::sleep_for(50);
    timerForwardWithHeading(90, 100 + 90, 0);
    this_thread::sleep_for(100);
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(175 + 25);
    Piston_IntakerLifter.set(true);
    this_thread::sleep_for(400); // 第一个
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(175 + 25);
    Piston_IntakerLifter.set(true);
    this_thread::sleep_for(400); // 第二个
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(300);
    aimPreciselyAt(133.038, 262.32, 1 + 1.5 + 5);
    moveIntaker(-100); // 吐盘防卡
    this_thread::sleep_for(100);
    shoot(3);

    // 中线一组盘
    moveIntaker(80); // 重开intaker
    setFlyWheelSpeed(383 - 16.999 + 5);
    this_thread::sleep_for(100);
    quickMoveToWithHeading(36.4156 - 2 - 1, 45 - 4 + 2.5 - 1, 15.113, 100); // 第一个
    this_thread::sleep_for(50);
    timerForwardWithHeading(120, 150 + 30, 0);
    this_thread::sleep_for(200);
    quickMoveToWithHeading(77.2274 + 8 + 2 - 2, 53.6367 + 2 + 2.5 - 3, 36.3957, 100); // 第二个
    this_thread::sleep_for(200);
    quickMoveToWithHeading(112.923 - 6 + 20, 30.4119 + 6, 107.682, 100); // 第三个不到
    this_thread::sleep_for(50);
    timerForwardWithHeading(120, 200, 0); // 向前吃
    this_thread::sleep_for(100);
    aimPreciselyAt(133.038, 262.32, 5.3);
    this_thread::sleep_for(100);
    moveIntaker(-100); // 吐盘防卡
    shoot(3);
    moveIntaker(80); // 重开intaker

    // barrier * 3
    setFlyWheelSpeed(395 - 16.999 + 10);
    quickMoveToWithHeading(118.158 - 2, -3.6123 + 2, 180, 100); // 第一个盘
    this_thread::sleep_for(50);
    quickMoveToWithHeading(118.158 - 31, -3.6123 - 34, 180, 100); // 第二个盘
    this_thread::sleep_for(100);
    quickMoveToWithHeading(55.1819 - 5, 19.255 - 5 - 5, 180, 100);
    aimPreciselyAt(133.038, 262.32, 4.8);
    this_thread::sleep_for(100);
    moveIntaker(-100);
    moveIntaker(80);
    shoot(3);
    this_thread::sleep_for(50);

    // 预装
    setFlyWheelSpeed(392 - 16.999 + 2);
    quickMoveToWithHeading(36.6641 - 3, -4.07135 - 3, 194.468, 100); // 预装1
    this_thread::sleep_for(50);
    quickMoveToWithHeading(25.2717 - 3 - 8, -15.5842 - 3 - 8, 191.813, 60); // 预装2
    this_thread::sleep_for(50);
    quickMoveToWithHeading(4.9605, 39.723, 211.093, 100);
    this_thread::sleep_for(50);
    aimPreciselyAt(133.038, 262.32, 3 - 1 + 4);
    this_thread::sleep_for(100);
    shoot(3);

    // roller
    Chassis::getInstance()->chassisBrake(vex::brakeType::hold);
    Chassis::getInstance()->setStopBrakeType(brakeType::hold);
    moveIntaker(0);                                             // intaker 停转 防止冲过去时正转roller
    quickMoveToWithHeading(-48.618, 12.1037 - 3, 223.746, 100); // 离roller还有一定距离
    timerForwardWithHeading(40, 300, 0);                        // 直接撞上roller
    this_thread::sleep_for(100);

    Motor_Intaker1.resetPosition();
    moveIntaker(-100);
    waitUntil(Motor_Intaker1.position(rotationUnits::deg) <= -50 + 20); // 转动Roller

    moveIntaker(0); // intaker停转 防止停留时继续反转

    // 低分区
    timerForwardWithHeading(-120, 200, 0); // 后退防撞roller
    moveIntaker(-100);                     // 开始吐盘推进低分区
    quickMoveToWithHeading(-40.35, 20, 135, 100);
    quickMoveToWithHeading(66.53, -85.72, 135, 100);

    // 初始化坐标
    moveIntaker(0);
    setFlyWheelSpeed(0); //
    this_thread::sleep_for(100);
    quickMoveToWithHeading(66.53 + 35, -85.72 - 5, 45, 100);
    this_thread::sleep_for(200);
    Point pos;
    pos = Position::getInstance()->getPos();
    Position::getInstance()->setGlobalPosition(0, 0);
    Inertial.setHeading(0, rotationUnits::deg);
    Inertial.setRotation(0, rotationUnits::deg);
    clearBrainScr();
    clearControllerScr();
    Brain.Screen.setCursor(2, 2);
    Brain.Screen.print("%.3lf %.3lf %.3lf", pos._x, pos._y, IMUHeading());
}
#endif

#ifdef COMPETITION_LEFT_BLUE // the Best 45s after Dallas at home
void autonomous()
{
    // 防四盘检测线程
    thread ShootInAuto(updateDistanceInAuto);
    MyTimer auto_time;
    cout << "enter auto functions" << endl;
    Inertial.setHeading(0, rotationUnits::deg);
    Position *p = Position::getInstance();
    Chassis *c = Chassis::getInstance();
    Chassis::getInstance()->chassisBrake(vex::brakeType::hold);
    Chassis::getInstance()->setStopBrakeType(brakeType::hold);
    setFlyWheelSpeed(396 - 16.999 + 3); // 开飞轮

    // 中线三盘
    moveIntaker(80);
    Piston_IntakerLifter.set(true);
    quickMoveToWithHeading(-5, 55, 0, 100);
    this_thread::sleep_for(100);
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(200);
    Piston_IntakerLifter.set(true);
    this_thread::sleep_for(300); // 第一个
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(200);
    Piston_IntakerLifter.set(true);
    this_thread::sleep_for(300); // 第二个
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(300);
    moveIntaker(80); // 重开intaker

#ifdef blocking
    timerForwardWithHeading(-100, 200, 0);
    this_thread::sleep_for(200);
    aimPreciselyAt(133.038, 262.32, 5);
#endif

    moveIntaker(-100); // 吐盘防卡
    this_thread::sleep_for(100);
    shoot(3);
    Chassis::getInstance()->chassisBrake(vex::brakeType::coast);
    Chassis::getInstance()->setStopBrakeType(brakeType::coast);
    this_thread::sleep_for(100);

    // 第二个三盘
    moveIntaker(80); // 重开intaker
    setFlyWheelSpeed(398 - 16.999 + 3 - 0.5);
    Piston_IntakerLifter.set(true);
    quickMoveToWithHeading(19.124, 32.16 - 4, 95.894, 100);
    this_thread::sleep_for(50);
    timerForwardWithHeading(90, 100 + 90, 0);
    this_thread::sleep_for(100);
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(175 + 25);
    Piston_IntakerLifter.set(true);
    this_thread::sleep_for(400); // 第一个
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(175 + 25);
    Piston_IntakerLifter.set(true);
    this_thread::sleep_for(400); // 第二个
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(300);
    aimPreciselyAt(133.038, 262.32, 1 + 1.5 + 5);
    moveIntaker(-100); // 吐盘防卡
    this_thread::sleep_for(100);
    shoot(3);

    // 中线一组盘
    moveIntaker(80); // 重开intaker
    setFlyWheelSpeed(383 - 16.999 + 5);
    this_thread::sleep_for(100);
    quickMoveToWithHeading(36.4156 - 2 - 1, 45 - 4 + 2.5 - 1, 15.113, 100); // 第一个
    this_thread::sleep_for(50);
    timerForwardWithHeading(120, 150 + 30, 0);
    this_thread::sleep_for(200);
    quickMoveToWithHeading(77.2274 + 8 + 2 - 2, 53.6367 + 2 + 2.5 - 3, 36.3957, 100); // 第二个
    this_thread::sleep_for(200);
    quickMoveToWithHeading(112.923 - 6 + 20, 30.4119 + 6, 107.682, 100); // 第三个不到
    this_thread::sleep_for(50);
    timerForwardWithHeading(120, 200, 0); // 向前吃
    this_thread::sleep_for(100);
    aimPreciselyAt(133.038, 262.32, 5.3);
    this_thread::sleep_for(100);
    moveIntaker(-100); // 吐盘防卡
    shoot(3);
    moveIntaker(80); // 重开intaker

    // barrier * 3
    setFlyWheelSpeed(395 - 16.999 + 10);
    quickMoveToWithHeading(118.158 - 2, -3.6123 + 2, 180, 100); // 第一个盘
    this_thread::sleep_for(50);
    quickMoveToWithHeading(118.158 - 31, -3.6123 - 34, 180, 100); // 第二个盘
    this_thread::sleep_for(100);
    quickMoveToWithHeading(55.1819 - 5, 19.255 - 5 - 5, 180, 100);
    aimPreciselyAt(133.038, 262.32, 4.8);
    this_thread::sleep_for(100);
    moveIntaker(-100);
    moveIntaker(80);
    shoot(3);
    this_thread::sleep_for(50);

    // 预装
    setFlyWheelSpeed(392 - 16.999 + 2);
    quickMoveToWithHeading(36.6641 - 3, -4.07135 - 3, 194.468, 100); // 预装1
    this_thread::sleep_for(50);
    quickMoveToWithHeading(25.2717 - 3 - 8, -15.5842 - 3 - 8, 191.813, 60); // 预装2
    this_thread::sleep_for(50);
    quickMoveToWithHeading(4.9605, 39.723, 211.093, 100);
    this_thread::sleep_for(50);
    aimPreciselyAt(133.038, 262.32, 3 - 1 + 4);
    this_thread::sleep_for(100);
    shoot(3);

    // roller
    Chassis::getInstance()->chassisBrake(vex::brakeType::hold);
    Chassis::getInstance()->setStopBrakeType(brakeType::hold);
    moveIntaker(0);                                             // intaker 停转 防止冲过去时正转roller
    quickMoveToWithHeading(-48.618, 12.1037 - 3, 223.746, 100); // 离roller还有一定距离
    timerForwardWithHeading(40, 300, 0);                        // 直接撞上roller
    this_thread::sleep_for(100);

    Motor_Intaker1.resetPosition();
    moveIntaker(-100);
    waitUntil(Motor_Intaker1.position(rotationUnits::deg) <= -50 + 20); // 转动Roller

    moveIntaker(0); // intaker停转 防止停留时继续反转

    // 低分区
    timerForwardWithHeading(-120, 200, 0); // 后退防撞roller
    moveIntaker(-100);                     // 开始吐盘推进低分区
    quickMoveToWithHeading(-40.35, 20, 135, 100);
    quickMoveToWithHeading(66.53, -85.72, 135, 100);

    // 初始化坐标
    moveIntaker(0);
    setFlyWheelSpeed(0); //
    this_thread::sleep_for(100);
    quickMoveToWithHeading(66.53 + 35, -85.72 - 5, 45, 100);
    this_thread::sleep_for(200);
    Point pos;
    pos = Position::getInstance()->getPos();
    Position::getInstance()->setGlobalPosition(0, 0);
    Inertial.setHeading(0, rotationUnits::deg);
    Inertial.setRotation(0, rotationUnits::deg);
    clearBrainScr();
    clearControllerScr();
    Brain.Screen.setCursor(2, 2);
    Brain.Screen.print("%.3lf %.3lf %.3lf", pos._x, pos._y, IMUHeading());
}
#endif
