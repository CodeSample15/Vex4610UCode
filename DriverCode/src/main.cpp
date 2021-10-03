/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\lukec                                            */
/*    Created:      Sun Sep 19 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RightFront           motor         10              
// RightBack            motor         20              
// LeftFront            motor         16              
// LeftBack             motor         12              
// Controller1          controller                    
// Lifter               motor         1               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

void ButtonCommands()
{
  while(true) {
    if(Controller1.ButtonA.pressing())
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  thread t(ButtonCommands);

  while(true) {
    double rightAmount = Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent);
    double leftAmount = Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent);
    //double rightAmount = Controller1.Axis2.position(percent);
    //double leftAmount = Controller1.Axis3.position(percent);

    //putting the output on a curve
    float rightMultiplier = 1;
    float leftMultiplier = 1;

    if(rightAmount < 0)
      rightMultiplier = -1;
    if(leftAmount < 0)
      leftMultiplier = -1;

    rightAmount = (rightAmount * rightAmount) / 100;
    leftAmount = (leftAmount * leftAmount) / 100;
    rightAmount *= rightMultiplier;
    leftAmount *= leftMultiplier;

    RightFront.setVelocity(rightAmount, vex::velocityUnits::pct);
    RightBack.setVelocity(rightAmount, vex::velocityUnits::pct);
    LeftFront.setVelocity(leftAmount, vex::velocityUnits::pct);
    LeftBack.setVelocity(leftAmount, vex::velocityUnits::pct);
    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);
  }

}
