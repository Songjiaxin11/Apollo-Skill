/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"

#define waitUntil(condition) \
  do                         \
  {                          \
    wait(5, msec);           \
  } while (!(condition))

#define repeat(iterations) \
  for (int iterator = 0; iterator < iterations; iterator++)

#define COMPETITION
//=======Please Choose Your ALLIANCE=======//
// #define RED_ALLIANCE
#define BLUE_ALLIANCE

//======Please Choose Your COMPETITION TYPE=======//
// #define COMPETITION_LEFT_RED
// #define COMPETITION_LEFT_BLUE
#define SKILL


//======Don't Touch it======//
// #define debug
// #define FIFTEEN
// #define COMPETITION_RIGHT
// #define TEST
