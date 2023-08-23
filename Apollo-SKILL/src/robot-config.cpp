#include "vex.h"
#include <iostream>
#include "parameters.h"
using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
inertial Inertial = inertial(PORT3);

motor Motor_BaseLFU = motor(PORT7, ratio18_1, true);
motor Motor_BaseLFD = motor(PORT8, ratio18_1, false);
motor Motor_BaseLBU = motor(PORT10, ratio18_1, true);
motor Motor_BaseLBD = motor(PORT9, ratio18_1, false);

motor Motor_BaseRBU = motor(PORT11, ratio18_1, true);
motor Motor_BaseRBD = motor(PORT12, ratio18_1, false);
motor Motor_BaseRFU = motor(PORT16, ratio18_1, true);
motor Motor_BaseRFD = motor(PORT15, ratio18_1, false);

controller Controller = controller(primary);

// triport Expander = triport(PORT8);

encoder EncoderL = encoder(Brain.ThreeWirePort.C);
encoder EncoderR = encoder(Brain.ThreeWirePort.A);

motor Motor_Intaker1 = motor(PORT21, ratio18_1, true);
motor Motor_Intaker2 = motor(PORT4, ratio18_1, false);
motor Motor_FlyWheel1 = motor(PORT14, ratio6_1, true);
motor Motor_FlyWheel2 = motor(PORT1, ratio6_1, false);

digital_out Piston_Trigger = digital_out(Brain.ThreeWirePort.H);
digital_out Piston_Angler = digital_out(Brain.ThreeWirePort.F);
digital_out Piston_Deployer = digital_out(Brain.ThreeWirePort.G);
digital_out Piston_IntakerLifter = digital_out(Brain.ThreeWirePort.E);

vex::distance Distance = distance(PORT20);

#ifdef RED_ALLIANCE
vision VisionSensor = vision (PORT13, 115, VisionSensor__SIG_RED);
signature VisionSensor__SIG_RED = signature(1, 7059, 8121, 7590, -411, 143, -134, 7.8, 0); // VEX场馆
#endif

#ifdef BLUE_ALLIANCE
vision VisionSensor = vision (PORT13, 84, VisionSensor__SIG_BLUE);
// //red - 61 blue - 105
signature VisionSensor__SIG_BLUE = signature(2, -2397, -1723, -2060, 7159, 9039, 8098, 2.5, 0);// VEX场馆
// signature VisionSensor__SIG_BLUE = signature(1, -3131, -2443, -2786, 5605, 6909, 6258, 6.7, 0);
#endif

// vision VisionSensor = vision(PORT13, 105, VisionSensor__SIG_RED, VisionSensor__SIG_BLUE);//blue 84 red 115
// signature VisionSensor__SIG_RED = signature (1, 8101, 8529, 8316, -525, -87, -306, 4.6, 0);
// signature VisionSensor__SIG_RED = signature(1, 7059, 8121, 7590, -411, 143, -134, 7.8, 0); // VEX场馆
// signature VisionSensor__SIG_RED = signature (1, 6923, 8639, 7780, -1467, -929, -1198, 5, 0);
// signature VisionSensor__SIG_BLUE = signature (2, -1927, -1355, -1640, 5691, 6841, 6266, 2.1, 0);//手测,晃
// signature VisionSensor__SIG_BLUE = signature(1, -3131, -2443, -2786, 5605, 6909, 6258, 6.7, 0);
// signature VisionSensor__SIG_BLUE = signature(2, -2397, -1723, -2060, 7159, 9039, 8098, 2.5, 0);// VEX场馆
// VEXcode generated functions

// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void)
{
}
