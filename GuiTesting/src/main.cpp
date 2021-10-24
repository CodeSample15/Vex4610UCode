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

  Button test(100, 100, 100, 100, "quack", purple);

  bool currentState = true;

  while(true) {
    if(test.isPressed() != currentState) { 
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1,1);
      Brain.Screen.print((test.isPressed() ? "Pressed" : "Not pressed"));
      test.draw();
      currentState = test.isPressed();
    }
  }
}
