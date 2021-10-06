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
// LeftFront            motor         16              
// LeftBack             motor         12              
// RightFront           motor         10              
// RightBack            motor         20              
// Rotation             inertial      18              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "PID.h"

using namespace vex;

//helper function to move to a certain position using a right and left pid
void moveTo(PID leftPID, PID rightPID, int location)
{
  //reset rotation
  Rotation.setRotation(0, degrees);

  do {
    double rightSpeed = rightPID.calculate(RightFront.position(degrees) - Rotation.rotation(degrees), location);
    double leftSpeed = leftPID.calculate(LeftFront.position(degrees) + Rotation.rotation(degrees), location);

    RightFront.setVelocity(rightSpeed, percent);
    RightBack.setVelocity(rightSpeed, percent);
    LeftFront.setVelocity(leftSpeed, percent);
    LeftBack.setVelocity(leftSpeed, percent);
    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);

    this_thread::sleep_for(20); //dt
  } while(abs((int)rightPID.error) > 1 && abs((int)leftPID.error) > 1);

  RightFront.stop();
  RightBack.stop();
  LeftFront.stop();
  LeftBack.stop();
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  //calibrating inertial
  Rotation.calibrate();
  wait(0.3, seconds);

  //make PID objects
  double Kp = 0.45;
  double Ki = 0.01;
  double Kd = 0;
  double dt = 20;

  PID rightPid(Kp, Ki, Kd, dt);
  PID leftPid(Kp, Ki, Kd, dt);

  //Pick up first goal
  //TODO: code this part

  //move to other side of field and push neutral goal
  moveTo(leftPid, rightPid, 2000);

  //put down first goal

  //pick up second goal

  //move to other side of field and push neutral goal

  //put down second goal

  //pick up third goal

  //move to other side of field and push neutral goal

  //put down third goal

  //pick up fourth goal

  //move to other side of field

  //put down goal
}
