/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// RightFront           motor         9               
// RightBack            motor         21              
// LeftFront            motor         15              
// LeftBack             motor         11              
// Lift                 motor         13              
// Conveyor             motor         12              
// IndicatorLight       led           H               
// RotationSensor       rotation      14              
// InertialSensor       inertial      10              
// Tilter               motor         17              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  bool xPressing = false;
  bool goForward = true;
  double rightAmount;
  double leftAmount;

  Lift.setStopping(hold);
  Lift.setVelocity(70, percent);

  Tilter.setStopping(hold);

  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    /*
      *arrow up / down  = Tilter up and down
      *arcade drive
      *r1 + r2 = up down four bar
      *l1 + l2 = intake (in / out)
      *A = switch directions + light
    */

    //arcade drive
    if(goForward) {
      rightAmount = Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent);
      leftAmount = Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent);
      IndicatorLight.off();
    }
    else{
      rightAmount = -Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent);
      leftAmount = -Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent);
      IndicatorLight.on();
    }

    //tank drive (not used but here just in case)
    //double rightAmount = Controller1.Axis2.position(percent);
    //double leftAmount = Controller1.Axis3.position(percent);

    //moving the bot
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
    if(!xPressing && Controller1.ButtonA.pressing()) { 
      xPressing = true;
      goForward = !goForward;
    }
    else if(!Controller1.ButtonA.pressing()) {
      xPressing = false;
    } 

    //tilter
    if(Controller1.ButtonUp.pressing()) {
      Tilter.setVelocity(90, percent);
      Tilter.spin(reverse);
    }
    else if(Controller1.ButtonDown.pressing()) {
      Tilter.setVelocity(60, percent);
      Tilter.spin(forward);
    }
    else {
      Tilter.stop();
    }

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
