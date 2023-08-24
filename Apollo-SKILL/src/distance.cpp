#include "distance.h"
#include "vex.h"
#include "robot-config.h"
#include "my-timer.h"
#include <iostream>
#include "basic-functions.h"
#include "parameters.h"
#include "auto-functions.h"
using namespace vex;

void updateDistance()
{

    while (true)
    {
        if (Distance.isObjectDetected() && Distance.objectDistance(distanceUnits::cm) <= 8)
        {
            Controller.rumble("-");
        }
        if (!Distance.isObjectDetected() || Distance.objectDistance(distanceUnits::cm) >= 15)
        {
            Controller.rumble(" ");
        }
        this_thread::sleep_for(20);
    }
}

void updateDistanceInAuto()
{
    MyTimer mytimer;
    while (true)
    {
        if (Distance.isObjectDetected() && Distance.objectDistance(distanceUnits::cm) <= 8 && mytimer.getTime() >= 900)
        {
            moveIntaker(-100);
            turnTo(45);
            shoot(1);
            Controller.rumble("-");
        }

        if (!Distance.isObjectDetected() || Distance.objectDistance(distanceUnits::cm) >= 15)
        {
            mytimer.reset();
            double sz = 14;
            double v = sz / mytimer.getTime() * 1000;
            Controller.rumble(" ");
            Brain.Screen.setCursor(5, 5);
            Brain.Screen.print("%.3lf", Distance.objectDistance(distanceUnits::cm));
            // std::cout << Distance.objectDistance(distanceUnits::cm) << std::endl;
        }
        this_thread::sleep_for(20);
    }
}