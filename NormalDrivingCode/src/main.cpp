/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\lukec                                            */
/*    Created:      Sun Oct 24 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftBack             motor         20              
// LeftFront            motor         18              
// Conveyor             motor         17              
// RightBack            motor         11              
// RightFront           motor         15              
// Controller1          controller                    
// Lift                 motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  bool xPressing = false;
  bool goForward = true;

  Lift.setStopping(hold);
  Lift.setVelocity(70, percent);

  while(true) {
    double rightAmount;
    double leftAmount;

    if(goForward) {
      rightAmount = Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent);
      leftAmount = Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent);
    }
    else{
      rightAmount = -Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent);
      leftAmount = -Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent);
    }
    //double rightAmount = Controller1.Axis2.position(percent);
    //double leftAmount = Controller1.Axis3.position(percent);

    RightFront.setVelocity(rightAmount, vex::velocityUnits::pct);
    RightBack.setVelocity(rightAmount, vex::velocityUnits::pct);
    LeftFront.setVelocity(leftAmount, vex::velocityUnits::pct);
    LeftBack.setVelocity(leftAmount, vex::velocityUnits::pct);
    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);

    //controlling the lift
    if(Controller1.ButtonR1.pressing()) {
      Lift.spin(forward);
    }
    else if(Controller1.ButtonR2.pressing()) {
      Lift.spin(reverse);
    }
    else {
      Lift.stop();
    }

    //controlling the intake conveyor
    if(Controller1.ButtonL1.pressing()) { 
      Conveyor.setVelocity(90, percent);
      Conveyor.spin(forward);
    }
    else if(Controller1.ButtonL2.pressing()) {
      Conveyor.setVelocity(70, percent);
      Conveyor.spin(reverse);
    }
    else {
      Conveyor.stop();
    }

    //changing between forward and reverse directions
    if(!xPressing && Controller1.ButtonX.pressing()) { 
      xPressing = true;
      goForward = !goForward;
    }
    else if(!Controller1.ButtonX.pressing()) {
      xPressing = false;
    }
  }
}
