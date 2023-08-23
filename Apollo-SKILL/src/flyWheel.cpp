#include "vex.h"
#include "robot-config.h"
#include "flyWheel.h"
#include "calc.h"
#include <iostream>
#include "position.h"
using namespace std;
double FlyWheel_Error;
float FLYWHEEL_SPEED = 0;
bool flywheel_speed_ok = false;
double FlywheelVoltagetoVelocity(double percent)
{
    return 7.6714 * percent - 18.733;
}

double FlywheelVelocitytoVoltage(double speed)
{
    return (speed + 18.733) / 7.6714;
}

void turnFlyWheelVoltage(float percent)
{
    Motor_FlyWheel2.spin(directionType::fwd, 127 * percent, voltageUnits::mV);
    Motor_FlyWheel1.spin(directionType::fwd, 127 * percent, voltageUnits::mV);
    return;
}

double FlyWheelSpeed()
{
    return (Motor_FlyWheel1.velocity(rpm) + Motor_FlyWheel2.velocity(rpm)) / 2;
}

void setFlyWheelSpeed(double Target_Speed){
    FLYWHEEL_SPEED = Target_Speed;
}

void changeFlyWheelSpeed(double Target_Speed){
    FLYWHEEL_SPEED = FLYWHEEL_SPEED + Target_Speed ;
}

void updateFlyWheelSpeed(double Target_Speed)
{
    // cout<<Target_Speed<<endl;
    FlyWheel_Error = Target_Speed - FlyWheelSpeed();
    if (Target_Speed == 0)
    {
        turnFlyWheelVoltage(0);
        return;
    }
    else
    {
        if (FlyWheel_Error >= Flywheel_P_Control_Domain && flywheel_speed_ok)
        {
            flywheel_speed_ok = false;
            // cout<<"1"<<endl;
        }
        if (FlyWheel_Error <= 0 && !flywheel_speed_ok)
        {
            flywheel_speed_ok = true;
            // cout<<"2"<<endl;
        }
        if (!flywheel_speed_ok)
        {
            turnFlyWheelVoltage(100);
            // cout<<"3"<<endl;
        }
        else
        {   
            if (FlyWheel_Error < -Flywheel_P_Control_Domain) turnFlyWheelVoltage(0);
            else
            {
                float P = 0.2 * FlyWheel_Error;
                if (fabs(P) > 5) P = sign(P) * 5;
                turnFlyWheelVoltage(FlywheelVelocitytoVoltage(Target_Speed) + P);
            }
        }

        // cout<<FlyWheel_Error<<endl;
        return;
    }
}

void updateFlyWheel(){
    while(true){
        updateFlyWheelSpeed(FLYWHEEL_SPEED);
    }
}