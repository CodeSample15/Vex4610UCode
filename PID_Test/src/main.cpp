/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\lukec                                            */
/*    Created:      Tue Sep 14 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftFront            motor         16              
// LeftBack             motor         12              
// RightFront           motor         10              
// RightBack            motor         20              
// Sensor               encoder       A, B            
// Controller1          controller                    
// Rotation             inertial      18              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "PID.h"

using namespace vex;

void display()
{
  while(true)
  {
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Sensor value: %f", RightFront.position(degrees));

    Brain.Screen.setCursor(1, 2);
    Brain.Screen.print("Rotation: %f", Rotation.orientation(roll, degrees));

    wait(15, msec);
    Brain.Screen.clearScreen();
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  thread t(display);

  Rotation.calibrate();
  wait(0.3, seconds);
  Rotation.setRotation(0, degrees);

  //test variables to just move forward
  int endLocation = 0;
  PID right(0.5, 0.1, 0.2, 20, 1, 100);
  PID left(0.5, 0.1, 0.2, 20, 1, 100);

  do {
    double RightOutput = right.calculate(Rotation.orientation(roll, degrees), endLocation);
    double LeftOutput = left.calculate(Rotation.orientation(roll, degrees), endLocation);

    if(abs((int)right.error) < 10) {
      RightOutput = 0;
      LeftOutput = 0;
    }

    RightFront.setVelocity(RightOutput * 10, vex::velocityUnits::pct);
    RightBack.setVelocity(RightOutput * 10, vex::velocityUnits::pct);
    LeftFront.setVelocity(LeftOutput * 10, vex::velocityUnits::pct);
    LeftBack.setVelocity(LeftOutput * 10, vex::velocityUnits::pct);
    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);

    this_thread::sleep_for(20); //dt
  } while(true);
  
  RightFront.stop();
  RightBack.stop();
  LeftFront.stop();
  LeftBack.stop();

}