/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\lukec                                            */
/*    Created:      Sun Oct 24 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "Button.h"

using namespace vex;

int autonType = 0;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  int screenWidth = 480;
  int screenHeight = 272;

  /*
    Screen layout:

    REDLEFT     REDRIGHT

    BLUELEFT    BLUERIGHT
  */


  Button redRightButton(screenWidth / 2, 0, screenWidth / 2, screenHeight / 2, "ree", red);
  Button blueRightButton(screenWidth / 2, screenHeight / 2, screenWidth / 2, screenHeight / 2, "", blue);
  Button redLeftButton(0, 0, screenWidth / 2, screenHeight / 2, "", red);
  Button blueLeftButton(0, screenHeight / 2, screenWidth / 2, screenHeight / 2, "", blue);

  bool selected = false;
  bool rightSide = false;
  bool leftSide = false;

  bool drawn = false;

  redRightButton.draw();
  //blueRightButton.draw();
 // redLeftButton.draw();
 // blueLeftButton.draw();
  
}
