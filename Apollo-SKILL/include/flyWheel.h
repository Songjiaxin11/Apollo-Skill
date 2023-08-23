// #ifndef FLYWHEEL_H_
// #define FLYWHEEL_H_
// extern double FlyWheel_Error;
// extern float FLYWHEEL_SPEED;
// float const Flywheel_P_Control_Domain = 10;
// float const kp = 80 / 10; // 1
// void setFlyWheelSpeed(double Target_Speed);
// void updateFlyWheelSpeed(double Target_Speed);
// double FlywheelVoltagetoVelocity(double percent);
// double FlywheelVelocitytoVoltage(double speed);
// void turnFlyWheelVoltage(float percent);
// double FlyWheelSpeed();
// void updateFlyWheel();
// #endif
#ifndef FLYWHEEL_H_
#define FLYWHEEL_H_
extern double FlyWheel_Error;
extern float FLYWHEEL_SPEED;
float const Flywheel_P_Control_Domain = 25;
// float const kp = 80 / 10;
float const kp = 0.2;
void setFlyWheelSpeed(double Target_Speed);
void changeFlyWheelSpeed(double Target_Speed);
void updateFlyWheelSpeed(double Target_Speed);
double FlywheelVoltagetoVelocity(double percent);
double FlywheelVelocitytoVoltage(double speed);
void turnFlyWheelVoltage(float percent);
double FlyWheelSpeed();
void updateFlyWheel();
#endif