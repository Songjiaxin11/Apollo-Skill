/**
 * @file trajectory.cpp
 * @author wcr
 * @brief 路径规划类
 * @version 0.1
 * @date 2023-02-14
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "trajectory.h"
#include "basic-functions.h"
#include "position.h"
#include <iostream>

using namespace std;

Trajectory *TrajectoryFactory::getPerciseTraj(
    Point _tarPos, double _tarAng,
    double _maxSpeed)
{ // 走到某个点,以某个角度,最大速度限制
  Trajectory *traj;
  Point pos = Position::getInstance()->getPos();
  double dis = (_tarPos - pos).mod();
  if (dis <= 20)
  {
    traj = new PIDTrajectory(_tarPos, _tarAng, _maxSpeed);
    traj->setPIDR(0.85, 0, 0.07, 7, 10);
    // traj->setPIDR(1, 0.8, 0, 7, 10);
    // traj->setPIDV(2.2, 1, 2.5, 20, 10);
    // traj->setPIDV(1.8, 0, 4.0, 20, 10);
    // traj->setPIDV(2.7, 0, 0, 20, 10);
    // traj->setPIDV(2.7, 0, 1.0, 20, 10);
    traj->setPIDV(50, 1, 100, 20, 10);
    PosPID dd;
    Point pos = Position::getInstance()->getPos();
    dd.update(pos);
    traj->setJumpTime(200);
    traj->setAngTol(1);
    traj->setTranTol(3);
  }
  else
  {
    traj = new NormalTrajectory(_tarPos, _tarAng, _maxSpeed);
    traj->setPIDR(0.85, 0, 0.07, 7, 10);
    // traj->setPIDV(2, 3.5, 2.5, 3, 50, 60, 10, 200);
    traj->setPIDV(2.8, 1, 2.5, 20, 10);
    traj->setJumpTime(200);
    traj->setAngTol(1);  // 3
    traj->setTranTol(3); // 8
  }
  return traj;
}

Trajectory *TrajectoryFactory::getPerciseTurnTraj(
    double _tarAng, double _maxSpeed)
{ // 准确转到某个角度,以某个最大转速限制
  Trajectory *traj;
  double ang = IMUHeading();
  double dang = _tarAng - ang;
  while (dang > 180)
  {
    _tarAng -= 360;
    dang -= 360;
  }
  while (dang < -180)
  {
    _tarAng += 360;
    dang += 360;
  }
  // 0.88 3.22344 0.16016
  traj = new PIDTrajectory(_tarAng, _maxSpeed);
  traj->setPIDR(0.85, 0, 0.07, 7, 10);//original
  // traj->setPIDR(1.5, 0, 1, 7, 10); //
  traj->setJumpTime(200);
  traj->setAngTol(1);
  return traj;
}

Trajectory *TrajectoryFactory::getQuickTraj(
    Point _tarPos, double _tarAng,
    double _maxSpeed)
{ // 非常快地走到某个点,以某个目标角度,最大转速限制
  Trajectory *traj;
  Point pos = Position::getInstance()->getPos();
  double dis = (_tarPos - pos).mod();
  if (dis <= 20)
  {
    traj = new PIDTrajectory(_tarPos, _tarAng, _maxSpeed);
    traj->setPIDR(0.85, 0, 0.07, 7, 10);
    traj->setPIDV(2.2, 1, 10 - 0.2, 20, 10);
    traj->setJumpTime(0);
    traj->setAngTol(3);
    traj->setTranTol(5);
  }
  else
  {
    traj = new NormalTrajectory(_tarPos, _tarAng, _maxSpeed);
    traj->setPIDR(0.85, 0, 0.07, 7, 10);
    // traj->setPIDV(4.4, 1, 14, 20, 10);//-0.2 100.15 虹口赛
    // traj->setPIDV(7.6, 1, 28, 20, 10); // 0.899 101.28
    traj->setPIDV(2.5, 0, 12, 20, 10); // original
    // traj->setPIDV(2.5-0.4, 0, 10, 20, 10); //
    // traj->setPIDV(6, 0, 0, 20, 10);
    // traj->setPIDV(3.86, 1, 15.8, 20, 10); // y+5
    // traj->setPIDV(4.4, 0.9, 14.4, 20, 10); // ok
    // traj->setPIDV(2.5, 1, 15.8, 20, 10); // y+5
    // traj->setPIDV(4, 0,20, 20, 10);
    traj->setJumpTime(0);
    traj->setAngTol(3);
    traj->setTranTol(3);
  }
  return traj;
}

Trajectory *TrajectoryFactory::getQuickTrajWithoutStop(Point _tarPos,
                                                       double _tarAng,
                                                       double _maxSpeed)
{
  Trajectory *traj;
  Point pos = Position::getInstance()->getPos();
  double dis = (_tarPos - pos).mod();
  if (dis <= 20)
  {
    traj = new PIDTrajectory(_tarPos, _tarAng, _maxSpeed, 0, 0);
    traj->setPIDR(0.85, 0, 0.07, 7, 10);
    traj->setPIDV(2.2, 1, 2.5, 20, 10);
    traj->setJumpTime(0);
    traj->setAngTol(1);
    traj->setTranTol(5);
  }
  else
  {
    traj = new NormalTrajectory(_tarPos, _tarAng, _maxSpeed, 0, 0);
    traj->setPIDR(0.85, 0, 0.07, 7, 10);
    traj->setPIDV(2.2, 1, 2.5, 20, 10);
    traj->setJumpTime(0);
    traj->setAngTol(3);
    traj->setTranTol(5);
  }
  return traj;
}

/**
 * @brief 初始化pid参数
 *
 */
void Trajectory::initPID()
{
  // pidv.setErrorTolerance(8);
  // pidv.setCoefficient(2, 1, 0);
  // pidv.setDTolerance(5);
  // pidv.setIMax(30);
  // pidv.setIRange(15);
  // pidv.setJumpTime(50);

  pidr.setErrorTolerance(3);
  // pidr.setCoefficient(1.275, 0, 2.45);
  pidr.setCoefficient(0.88, 3.22344, 0.16016);
  // 1.5 2.7
  pidr.setDTolerance(50);
  pidr.setIMax(60);
  pidr.setIRange(5);
  pidr.setJumpTime(200);

  // pidvs.setErrorTolerance(8);
  // pidvs.setCoefficient(3.3, 5, 0.2);
  // pidvs.setDTolerance(5);
  // pidvs.setIMax(30);
  // pidvs.setIRange(15);
  // pidvs.setJumpTime(50);

  pidrs.setErrorTolerance(3);
  // pidrs.setCoefficient(1.1, 4.45344, 0.181133);
  // pidrs.setCoefficient(1.5, 3.14286, 0.15);
  pidrs.setCoefficient(0.86, 4.20538, 0.117247);
  pidrs.setDTolerance(50);
  pidrs.setIMax(60);
  pidrs.setIRange(5);
  pidrs.setJumpTime(200);

  // // B楼
  pidv.setErrorTolerance(3);
  pidv.setCoefficient(2, 3.5, 2.5);
  // 4 2 6
  pidv.setDTolerance(50);
  pidv.setIMax(60);
  pidv.setIRange(10);
  pidv.setJumpTime(200);

  pidvs.setErrorTolerance(3);
  pidvs.setCoefficient(2.5, 3.5, 1.3);
  pidvs.setDTolerance(50);
  pidvs.setIMax(60);
  pidvs.setIRange(15);
  pidvs.setJumpTime(200);

  // pidr.setErrorTolerance(2);
  // pidr.setCoefficient(1.45, 2, 1.8);
  // //1.5 2.7
  // pidr.setDTolerance(3);
  // pidr.setIMax(30);
  // pidr.setIRange(5);
  // pidr.setJumpTime(300);
}

void Trajectory::setJumpTime(double t) { jumpTime = t; }

void Trajectory::setTranTol(double tol) { tranTol = tol; }

void Trajectory::setAngTol(double tol) { angTol = tol; }

void Trajectory::setPIDV(float _p, float _i, float _d, float _IMax,
                         float _IRange)
{
  pidv.setCoefficient(_p, _i, _d);
  pidv.setIMax(_IMax);
  pidv.setIRange(_IRange);

  // cout<<_p<<" "<<_i<<" "<<_d<<endl;
}

void Trajectory::setPIDR(float _p, float _i, float _d, float _IMax,
                         float _IRange)
{
  pidr.setCoefficient(_p, _i, _d);
  pidr.setIMax(_IMax);
  pidr.setIRange(_IRange);
}

/**
 * @brief 软启动
 *
 */
void Trajectory::softStartV()
{
  Point pos = Position::getInstance()->getPos();
  double accLength = (targetPos - initPos).mod() * accPctv;
  double passedLength = (pos - initPos).mod();
  double startSpeed;
  Vector d = (targetPos - pos);
  if (d.mod() == 0)
    startSpeed = 0;
  else
  {
    startSpeed = 35;
    d = d / d.mod();
  }
  double speed =
      startSpeed + (maxSpeed - startSpeed) * (passedLength / accLength);
  velocity = d * speed;
}

/**
 * @brief 角度软启动
 *
 */
void Trajectory::softStartR()
{
  double da = (targetAng - initAng);
  if (da > 180)
    da -= 360;
  if (da < -180)
    da += 360;
  double passedAng = (IMUHeading() - initAng);
  if (passedAng > 180)
    passedAng -= 360;
  if (passedAng < -180)
    passedAng += 360;
  double leftAng = (targetAng - IMUHeading());
  if (leftAng > 180)
    leftAng -= 360;
  if (leftAng < -180)
    leftAng += 360;
  double accAng = da * accPctr;
  double startSpeedR;
  if (da == 0)
    startSpeedR = 0;
  else
    startSpeedR = 35;
  double speedR =
      (startSpeedR + (maxSpeed - startSpeedR) * abs(passedAng / accAng)) *
      sign(leftAng);
  velocityR = speedR;
}

/**
 * @brief 匀速
 *
 */
void Trajectory::constSpeedV()
{
  Point pos = Position::getInstance()->getPos();
  Vector d = targetPos - pos;
  if (d.mod() != 0)
    d = d / d.mod();
  velocity = d * maxSpeed;
}

/**
 * @brief 角度匀速
 *
 */
void Trajectory::constSpeedR()
{
  double da = (targetAng - IMUHeading());
  if (da > 180)
    da -= 360;
  if (da < -180)
    da += 360;
  velocityR = sign(da) * maxSpeed;
}

/**
 * @brief 位移pid
 *
 */
void Trajectory::pidControlV(PosPID &pid)
{
  Point pos = Position::getInstance()->getPos();
  pid.update(pos);
  double speed = pid.getOutput();
  // if (pid.targetArrived())
  //     speed = 0;
  if (speed > 100)
    speed = 100;
  Vector d = (targetPos - pos);
  if (d.mod() != 0)
    d = d / d.mod();
  velocity = speed * d;
  // cout << "speed: " << speed << endl;
  // cout << "d: " << d._x << " " << d._y << endl;
  // cout << "v: " << velocity._x << " " << velocity._y << endl;
  // cout << "vmod: " << velocity.mod() << endl;
  if (abs(velocity.mod()) <= 0.0005)
    return;
  // cout << "1111111111111" << endl;
  if (velocity.mod() > maxSpeed)
    velocity = velocity / velocity.mod() * maxSpeed;
}

/**
 * @brief 角度pid
 *
 */
void Trajectory::pidControlR(PID &pid)
{
  double ang = IMUHeading();
  double da = (targetAng - ang);
  while (da > 180)
  {
    da -= 360;
    ang += 360;
    pid.setTarget(targetAng);
  }
  while (da < -180)
  {
    da += 360;
    ang -= 360;
    pid.setTarget(targetAng);
  }
  pid.update(ang);
  velocityR = pid.getOutput();
  if (abs(velocityR) > maxSpeed)
    velocityR = sign(velocityR) * maxSpeed;
  if (abs(velocityR) < 1)
    velocityR = 0;
}

bool Trajectory::targetArrived() { return varrived && rarrived; }

Vector Trajectory::getVelocity() { return velocity; }

double Trajectory::getVelocityR() { return velocityR; }

void Trajectory::update()
{
  updateStage();
  updateVelocity();
}

void Trajectory::updateArriveState()
{
  if (stageV != STOP && stageV != DONOTHING)
  {
    Point pos = Position::getInstance()->getPos();
    double leftLength = (targetPos - pos).mod();
    if (abs(leftLength) <=
        tranTol)
    { // Exit when staying in tolerated region and maintaining a
      // low enough speed for enough time
      if (timer1.getTime() >= jumpTime)
        varrived = true;
    }
    else
    {
      timer1.reset();
      varrived = false;
    }
  }

  if (stageR != STOP && stageR != DONOTHING)
  {
    double leftAng = (targetAng - IMUHeading());
    while (leftAng > 180)
      leftAng -= 360;
    while (leftAng < -180)
      leftAng += 360;
    if (abs(leftAng) <=
        angTol)
    { // Exit when staying in tolerated region and maintaining a low
      // enough speed for enough time
      if (timer2.getTime() >= jumpTime)
        rarrived = true;
    }
    else
    {
      timer2.reset();
      rarrived = false;
    }
  }
}

/**
 * @brief Construct a new NormalTrajectory:: NormalTrajectory object
 *
 * @param _tarPos 目标位置
 * @param _tarAng 目标角度
 * @param _maxSpeed 最大速度
 */
NormalTrajectory::NormalTrajectory(Point _tarPos, double _tarAng,
                                   double _maxSpeed)
{
  initPID();
  targetPos = _tarPos;
  pidv.setTarget(_tarPos);
  pidvs.setTarget(_tarPos);
  targetAng = _tarAng;
  pidr.setTarget(_tarAng);
  pidrs.setTarget(_tarAng);
  maxSpeed = _maxSpeed;
  if (maxSpeed < 35)
    maxSpeed = 35;
  accPctv = 0.2;
  decPctv = 0.5;

  initPos = Position::getInstance()->getPos();
  initAng = IMUHeading();
  stageV = stageR = BEGIN;
  varrived = rarrived = false;
}

/**
 * @brief Construct a new NormalTrajectory:: NormalTrajectory object
 *
 * @param _tarAng 目标角度
 * @param _maxSpeed 最大速度
 */
NormalTrajectory::NormalTrajectory(double _tarAng, double _maxSpeed)
{
  initPID();
  double d = _tarAng - IMUHeading();
  while (d > 180)
  {
    d -= 360;
    _tarAng -= 360;
  }
  while (d < -180)
  {
    d += 360;
    _tarAng += 360;
  }
  targetAng = _tarAng;
  pidr.setTarget(_tarAng);
  pidrs.setTarget(_tarAng);
  maxSpeed = _maxSpeed;
  if (maxSpeed < 35)
    maxSpeed = 35;
  accPctv = 0.05;
  decPctv = 0.35;
  initPos = Position::getInstance()->getPos();
  initAng = IMUHeading();
  stageR = BEGIN;
  stageV = DONOTHING;
  varrived = rarrived = false;
}

/**
 * @brief Construct a new NormalTrajectory:: NormalTrajectory object
 *
 * @param _tarPos 目标位置
 * @param _tarAng 目标角度
 * @param _maxSpeed 最大速度
 * @param _accPctv 加速段占比
 * @param _decPctv 减速段占比
 */
NormalTrajectory::NormalTrajectory(Point _tarPos, double _tarAng,
                                   double _maxSpeed, double _accPctv,
                                   double _decPctv)
{
  initPID();
  targetPos = _tarPos;
  pidv.setTarget(_tarPos);
  pidvs.setTarget(_tarPos);
  targetAng = _tarAng;
  pidr.setTarget(_tarAng);
  pidrs.setTarget(_tarAng);
  maxSpeed = _maxSpeed;
  if (maxSpeed < 35)
    maxSpeed = 35;
  accPctv = _accPctv;
  decPctv = _decPctv;
  initPos = Position::getInstance()->getPos();
  initAng = IMUHeading();
  stageV = stageR = BEGIN;
  varrived = rarrived = false;
}

/**
 * @brief 状态转移函数的速度部分，更新速度值
 *
 * @details 自动机设定了整个运动过程分为 开始 - 加速 - 匀速 - 减速 - 结束 阶段
 *
 *
 */
void NormalTrajectory::updateVelocity()
{
  // cout << "N updateV" << endl;
  switch (stageV)
  {
  case BEGIN: // calculate durations and path lengths
    break;
  case ACCEL:
    softStartV();
    break;
  case CONSTSPEED:
    constSpeedV();
    break;
  case DECEL: // m_pid.setTarget(totalPath);
    pidControlV(pidv);
    break;
  case STOP:
    velocity = Vector(0, 0);
    break;
  default:
    break;
  }
  switch (stageR)
  {
  case BEGIN: // calculate durations and path lengths
    break;
  case DECEL: // m_pid.setTarget(totalPath);
    pidControlR(pidr);
    break;
  case STOP:
    velocityR = 0;
    break;
  default:
    break;
  }
  if (abs(velocityR) >= 60)
    velocityR = sign(velocityR) * 60;
  if (targetArrived())
  {
    velocity = Vector(0, 0);
    velocityR = 0;
  }
  updateArriveState();
}

/**
 * @brief 更新状态
 *
 */
void NormalTrajectory::updateStage()
{
  // cout << "N updateStage" << endl;
  Point pos = Position::getInstance()->getPos();
  double leftLength = (targetPos - pos).mod();
  double accLength = (targetPos - initPos).mod() * accPctv;
  double passedLength = (pos - initPos).mod();
  double cstLength = (targetPos - initPos).mod() * decPctv;
  switch (stageV)
  {
  case BEGIN:
    stageV = ACCEL;
    cout << "stageV ACCEL" << endl;
    break;
  case ACCEL:
  {
    if (passedLength >= accLength || velocity.mod() >= maxSpeed)
    {
      stageV = CONSTSPEED;
      cout << "stageV CONSTSPEED" << endl;
    }
    break;
  }
  case CONSTSPEED:
  {
    if (leftLength <= cstLength)
    {
      stageV = DECEL;
      cout << "stageV DECEL" << endl;
    }
    break;
  }
  case DECEL:
    if (pidv.targetArrived())
    {
      // stageV = STOP;
      // cout << "stageV STOP" << endl;
    }
    break;
  case STOP:
    break;
  default:
    break;
  }

  double leftAng = (targetAng - IMUHeading());
  if (leftAng > 180)
    leftAng -= 360;
  if (leftAng < -180)
    leftAng += 360;
  double da = (targetAng - initAng);
  if (da > 180)
    da -= 360;
  if (da < -180)
    da += 360;
  double passedAng = (IMUHeading() - initAng);
  if (passedAng > 180)
    passedAng -= 360;
  if (passedAng < -180)
    passedAng += 360;
  double accAng = da * accPctr;
  double cstAng = da * decPctr;

  switch (stageR)
  {
  case BEGIN:
    stageR = DECEL;
    break;
  case DECEL:
    break;
  case STOP:
    break;
  default:
    break;
  }
}

/**
 * @brief Construct a new PIDTrajectory:: PIDTrajectory object
 *
 * @param _tarPos 目标位置
 * @param _tarAng 目标角度
 * @param _maxSpeed 最大速度
 */
PIDTrajectory::PIDTrajectory(Point _tarPos, double _tarAng, double _maxSpeed)
{
  initPID();
  targetPos = _tarPos;
  pidv.setTarget(_tarPos);
  pidvs.setTarget(_tarPos);
  targetAng = _tarAng;
  pidr.setTarget(_tarAng);
  pidrs.setTarget(_tarAng);
  maxSpeed = _maxSpeed;
  if (maxSpeed < 35)
    maxSpeed = 35;
  accPctv = 0.05;
  decPctv = 0.35;
  initPos = Position::getInstance()->getPos();
  initAng = IMUHeading();
  stageV = stageR = BEGIN;
  varrived = rarrived = false;
}

/**
 * @brief Construct a new PIDTrajectory:: PIDTrajectory object
 *
 * @param _tarAng 目标角度
 * @param _maxSpeed 最大速度
 */
PIDTrajectory::PIDTrajectory(double _tarAng, double _maxSpeed)
{
  double d = _tarAng - IMUHeading();
  while (d > 180)
  {
    d -= 360;
    _tarAng -= 360;
  }
  while (d < -180)
  {
    d += 360;
    _tarAng += 360;
  }
  targetAng = _tarAng;
  pidr.setTarget(_tarAng);
  // pidrs.setTarget(_tarAng);
  maxSpeed = _maxSpeed;
  if (maxSpeed < 35)
    maxSpeed = 35;
  accPctv = 0.05;
  decPctv = 0.35;
  initPos = Position::getInstance()->getPos();
  initAng = IMUHeading();
  stageR = BEGIN;
  stageV = DONOTHING;
  rarrived = false;
  varrived = true;
}

/**
 * @brief Construct a new PIDTrajectory:: PIDTrajectory object
 *
 * @param _tarPos 目标位置
 * @param _tarAng 目标角度
 * @param _maxSpeed 最大速度
 * @param _accPctv 加速段占比
 * @param _decPctv 减速段占比
 */
PIDTrajectory::PIDTrajectory(Point _tarPos, double _tarAng, double _maxSpeed,
                             double _accPctv, double _decPctv)
{
  initPID();
  targetPos = _tarPos;
  pidv.setTarget(_tarPos);
  pidvs.setTarget(_tarPos);
  targetAng = _tarAng;
  pidr.setTarget(_tarAng);
  pidrs.setTarget(_tarAng);
  maxSpeed = _maxSpeed;
  if (maxSpeed < 35)
    maxSpeed = 35;
  accPctv = _accPctv;
  decPctv = _decPctv;
  initPos = Position::getInstance()->getPos();
  initAng = IMUHeading();
  stageV = stageR = BEGIN;
  varrived = rarrived = false;
}

/**
 * @brief 状态转移函数的速度部分，更新速度值
 *
 * @details 自动机设定了整个运动过程分为 开始 - 加速 - 匀速 - 减速 - 结束 阶段
 *
 *
 */
void PIDTrajectory::updateVelocity()
{
  switch (stageV)
  {
  case BEGIN: // calculate durations and path lengths
    break;
  case DECEL: // m_pid.setTarget(totalPath);
    pidControlV(pidv);
    break;
  case STOP:
    velocity = Vector(0, 0);
    break;
  default:
    break;
  }
  switch (stageR)
  {
  case BEGIN: // calculate durations and path lengths
    break;
  case DECEL: // m_pid.setTarget(totalPath);
    pidControlR(pidr);
    break;
  case STOP:
    velocityR = 0;
    break;
  default:
    break;
  }
  if (abs(velocityR) >= 60)
    velocityR = sign(velocityR) * 60;
  if (targetArrived())
  {
    velocity = Vector(0, 0);
    velocityR = 0;
  }
  updateArriveState();
}

/**
 * @brief 更新状态
 *
 */
void PIDTrajectory::updateStage()
{
  Point pos = Position::getInstance()->getPos();
  double leftLength = (targetPos - pos).mod();
  double accLength = (targetPos - initPos).mod() * accPctv;
  double passedLength = (pos - initPos).mod();
  double cstLength = (targetPos - initPos).mod() * decPctv;
  switch (stageV)
  {
  case BEGIN:
    stageV = DECEL;
    break;
  case DECEL:
    break;
  case STOP:
    break;
  default:
    break;
  }

  double leftAng = (targetAng - IMUHeading());
  if (leftAng > 180)
    leftAng -= 360;
  if (leftAng < -180)
    leftAng += 360;
  double da = (targetAng - initAng);
  if (da > 180)
    da -= 360;
  if (da < -180)
    da += 360;
  double passedAng = (IMUHeading() - initAng);
  if (passedAng > 180)
    passedAng -= 360;
  if (passedAng < -180)
    passedAng += 360;
  double accAng = da * accPctr;
  double cstAng = da * decPctr;

  switch (stageR)
  {
  case BEGIN:
    stageR = DECEL;
    cout << "stageR DECEL" << endl;
    break;
  case DECEL:
  {
    break;
  }

  case STOP:
    break;
  default:
    break;
  }
}

// /**
//  * @brief 更新状态
//  *
//  */
// void PerciseTrajectory::updateStage()
// {
//     Point pos = Position::getInstance()->getPos();
//     double leftLength = (targetPos - pos).mod();
//     double accLength = (targetPos - initPos).mod() * accPctv;
//     double passedLength = (pos - initPos).mod();
//     double cstLength = (targetPos - initPos).mod() * decPctv;
//     switch (stageV){
//         case BEGIN:
//             if (leftLength <= 20){
//                 stageV = SHORTMOVE;
//                 cout << "stageV SHORTMOVE" << endl;
//             }
//             else{
//                 stageV = ACCEL;
//                 cout << "stageV ACCEL" << endl;
//             }
//             break;
//         case SHORTMOVE:
//             break;
//         case ACCEL:{
//             if (passedLength >= accLength || velocity.mod() >= maxSpeed){
//                 stageV = CONSTSPEED;
//                 cout << "stageV CONSTSPEED" << endl;
//             }
//             break;
//         }
//         case CONSTSPEED:{
//             if (leftLength <= cstLength){
//                 stageV = DECEL;
//                 cout << "stageV DECEL" << endl;
//             }
//             break;
//         }
//         case DECEL:
//             if (pidv.targetArrived()){
//                 // stageV = STOP;
//                 // cout << "stageV STOP" << endl;
//             }
//             break;
//         case STOP:
//             break;
//     }

//     double leftAng = (targetAng - IMUHeading());
//     if (leftAng > 180) leftAng -= 360;
//     if (leftAng < -180) leftAng += 360;
//     double da = (targetAng - initAng);
//     if (da > 180) da -= 360;
//     if (da < -180) da += 360;
//     double passedAng = (IMUHeading() - initAng);
//     if (passedAng > 180) passedAng -= 360;
//     if (passedAng < -180) passedAng += 360;
//     double accAng = da * accPctr;
//     double cstAng = da * decPctr;

//     switch (stageR){
//         case BEGIN:
//             if (abs(leftAng) <= 20){
//                 stageR = SHORTMOVE;
//                 cout << "stageR SHORTMOVE" << endl;
//             }
//             else{
//                 stageR = ACCEL;
//                 cout << "stageR ACCEL" << endl;
//             }
//             break;
//         case SHORTMOVE:
//             break;
//         case ACCEL:{
//             if (abs(passedAng) >= abs(accAng) || velocityR >= maxSpeed){
//                 stageR = CONSTSPEED;
//                 cout << "stageR CONSTSPEED" << endl;
//             }
//             break;
//         }
//         case CONSTSPEED:{
//             if (abs(leftAng) <= abs(cstAng)){
//                 stageR = DECEL;
//                 cout << "stageR DECEL" << endl;
//             }
//             break;
//         }
//         case DECEL:{
//             if (pidr.targetArrived()){
//                 // stageR = STOP;
//                 // cout << "stageR STOP" << endl;
//             }
//             break;
//         }

//         case STOP:
//             break;
//     }
// }
