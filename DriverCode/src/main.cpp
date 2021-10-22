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
// Rot                  inertial      18              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "PID.h"

using namespace vex;

double botRotation = 0;
double turnAmount;

void ButtonCommands()
{
  while(true) {
    if(Controller1.ButtonA.pressing()){

    }
  }
}

void display()
{
  while(true) {
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Rotation: %f", botRotation);

    Brain.Screen.setCursor(2,1);
    Brain.Screen.print("Bot Rotation Sensor: %f", Rot.rotation(degrees));

    Brain.Screen.setCursor(3,1);
    Brain.Screen.print("Rotation amount: %f", turnAmount);

    wait(15, msec);
    Brain.Screen.clearScreen();
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  //thread t(ButtonCommands);
  thread t(display);

  double Kp = 0.2;
  double Ki = 0.0001;
  double Kd = 0.80;
  double dt = 20;
  PID turnPid(Kp, Ki, Kd, dt, 40, 10);

  Rot.calibrate();
  wait(.5, seconds);

  while(true) {
    double rightAmount = Controller1.Axis3.position(percent);
    double leftAmount = Controller1.Axis3.position(percent);
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

    //feeding in rotation
    turnAmount = -Controller1.Axis1.position(percent);

    if(turnAmount != 0 || true) {
      botRotation -= turnAmount/15;
    }
    double output = turnPid.calculate(Rot.rotation(degrees), botRotation) * 4;

    RightFront.setVelocity(rightAmount - output, vex::velocityUnits::pct);
    RightBack.setVelocity(rightAmount - output, vex::velocityUnits::pct);
    LeftFront.setVelocity(leftAmount + output, vex::velocityUnits::pct);
    LeftBack.setVelocity(leftAmount + output, vex::velocityUnits::pct);
    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);

    this_thread::sleep_for(dt);
  }

}
