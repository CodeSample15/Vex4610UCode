/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Luke Crimi                                                */
/*    Created:      Tue Jan 25 2022                                           */
/*    Description:  Generic drive program that can be used for testing.       */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// RightFront           motor         3               
// LeftFront            motor         2               
// RightBack            motor         4               
// LeftBack             motor         5               
// FrontLift            motor         6               
// BackLift             motor         7               
// Tilter               motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

void printTemps() {
  //seperate thread for debugging
  while(true) {
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Front Right: %f", RightFront.temperature(percent));

    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Front Left: %f", LeftFront.temperature(percent));

    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Back Right: %f", RightBack.temperature(percent));

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Back Left: %f", LeftBack.temperature(percent));

    wait(15, msec);

    Brain.Screen.clearScreen();
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  thread t(printTemps);
  bool curveTurning = true;
  
  while(true) 
  {
    double rightAmount = 0;
    double leftAmount = 0;

    double turnAmount = Controller1.Axis1.position(percent);
    bool negative = turnAmount < 0;
    if(curveTurning) {
      turnAmount *= turnAmount;
      turnAmount /= 100;

      if(negative)
        turnAmount *= -1;
    }

    rightAmount = Controller1.Axis3.position(percent) - turnAmount;
    leftAmount = Controller1.Axis3.position(percent) + turnAmount;

    RightFront.setVelocity(rightAmount, percent);
    RightBack.setVelocity(rightAmount, percent);
    LeftFront.setVelocity(leftAmount, percent);
    LeftBack.setVelocity(leftAmount, percent);

    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);


    //if the robot has a lift
    FrontLift.setVelocity(100, percent);
    BackLift.setVelocity(100, percent);

    if(Controller1.ButtonR1.pressing()) {
      FrontLift.spin(forward); //raise the lift up
    }
    else if(Controller1.ButtonR2.pressing()) {
      FrontLift.spin(reverse); //lower the lift down
    }
    else {
      FrontLift.stop();
    }

    if(Controller1.ButtonL1.pressing()) {
      BackLift.spin(forward); //raise the lift up
    }
    else if(Controller1.ButtonL2.pressing()) {
      BackLift.spin(reverse); //lower the lift down
    }
    else {
      BackLift.stop();
    }


    //If the robot has a tilter
    Tilter.setVelocity(100, percent);

    if(Controller1.ButtonB.pressing()) {
      //move the tilter up
      Tilter.spin(forward);
    }
    else if(Controller1.ButtonDown.pressing()) {
      //move the tilter down
      Tilter.spin(reverse);
    }
    else {
      Tilter.stop();
    }

    wait(15, msec);
  }
}
