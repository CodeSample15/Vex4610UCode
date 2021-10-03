/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\lukec                                            */
/*    Created:      Sun Oct 03 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// TestMotor            motor         1               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "PID.h"

using namespace vex;

void display()
{
  while(true) {
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Motor position %f", TestMotor.position(degrees));

    wait(15, msec);
    Brain.Screen.clearScreen();
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  thread t(display);
  //445
  TestMotor.setVelocity(100, percent);
  TestMotor.spinToPosition(100, degrees);
}
