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
// ClamperL             digital_out   G               
// ClamperR             digital_out   F               
// TilterButton         bumper        E               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "PID.h"

using namespace vex;

int perfectTilterPosition = -110;

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

void drawStuff() {
  while (true) {
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Rotation: %f", InertialSensor.rotation(degrees));

    wait(15, msec);
    Brain.Screen.clearScreen();
  }
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  thread t(drawStuff);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  InertialSensor.calibrate();
  wait(4, seconds);
  InertialSensor.setRotation(0, degrees);
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

void turnWithPID(PID& turnPid, int amount, double speedModifier) 
{
  InertialSensor.setRotation(0, degrees);

  bool turnRight = false;

  if(amount > 0)
    turnRight = true;

  do {
    double speed = turnPid.calculate(InertialSensor.rotation(degrees), amount) * speedModifier;
    speed *= -1;


    if(turnRight) {
      RightBack.setVelocity(-speed, percent);
      RightFront.setVelocity(-speed, percent);
      LeftBack.setVelocity(speed, percent);
      LeftFront.setVelocity(speed, percent);
    }
    else {
      RightBack.setVelocity(speed, percent);
      RightFront.setVelocity(speed, percent);
      LeftBack.setVelocity(-speed, percent);
      LeftFront.setVelocity(-speed, percent);
    }

    RightBack.spin(forward);
    RightFront.spin(forward);
    LeftBack.spin(forward);
    LeftFront.spin(forward);
  } while(abs((int)turnPid.error) > 10);

  RightBack.stop();
  RightFront.stop();
  LeftBack.stop();
  LeftFront.stop();
}

void Clamp(int val) {
  ClamperL.set(val);
  ClamperR.set(val);
}

void Move(PID& pid, int amount, double speed) {
  RightFront.setPosition(0, degrees);

  do {
    double pidSpeed = pid.calculate(RightFront.position(degrees), amount) * speed;

    RightFront.setVelocity(pidSpeed, percent);
    RightBack.setVelocity(pidSpeed, percent);
    LeftBack.setVelocity(pidSpeed, percent);
    LeftFront.setVelocity(pidSpeed, percent);

    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);
  } while(pid.error > 10);

  RightFront.stop();
  LeftFront.stop();
  RightBack.stop();
  LeftBack.stop();
}

void Move(int amount, int speed) {
  RightFront.setPosition(0, degrees);

  while(abs((int)RightFront.position(degrees)) < amount) {
    RightFront.setVelocity(speed, percent);
    RightBack.setVelocity(speed, percent);
    LeftBack.setVelocity(speed, percent);
    LeftFront.setVelocity(speed, percent);

    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);
  }

  RightFront.stop();
  LeftFront.stop();
  RightBack.stop();
  LeftBack.stop();
}

void alignTilter(bool up) 
{
  if(up) {
    Tilter.setVelocity(90, percent);
    Tilter.spinToPosition(perfectTilterPosition, degrees);
  }
  else {
    Tilter.setVelocity(60, percent);
    Tilter.spinToPosition(0, degrees);
  }
}

void autonomous(void) {
  double Kp = 0.25;
  double Ki = 0.01;
  double Kd = 0.40;

  double turnKp = 0.25;
  double turnKi = 0;
  double turnKd = 0.40;

  PID driveTrainPID(Kp, Ki, Kd);
  PID turnPID(turnKp, turnKi, turnKd);

  Clamp(1);

  //resetting tilter position for consitancy when scoring rings into it
  while(!TilterButton.pressing()) {
    Tilter.setVelocity(30, percent);
    Tilter.spin(forward);
  }
  Tilter.setPosition(0, degrees);

  Tilter.stop();

  //TODO: code this part

  //pick up first goal
  Move(100, 30);
  Clamp(0);
  alignTilter(true);
  Move(100, -30);
  turnWithPID(turnPID, -90, 1);

  //move to other side of field and push neutral goal
  

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
  bool aPressing = false;
  bool xPressing = false;

  bool goForward = true;
  double rightAmount;
  double leftAmount;
  int clamp = 0;

  bool modifier = false;

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
      Conveyor.setVelocity(70, percent);
      Conveyor.spin(forward);
    }
    else if(Controller1.ButtonL2.pressing()) {
      Conveyor.setVelocity(60, percent);
      Conveyor.spin(reverse);
    }
    else {
      Conveyor.stop();
    }

    //changing between forward and reverse directions
    if(!aPressing && Controller1.ButtonA.pressing()) { 
      aPressing = true;
      goForward = !goForward;
    }
    else if(!Controller1.ButtonA.pressing()) {
      aPressing = false;
    }

    //manual button modifier boolean
    modifier = Controller1.ButtonY.pressing();

    //tilter
    if(Controller1.ButtonUp.pressing()) {
      Tilter.setVelocity(90, percent);
      Tilter.spin(reverse);
    }
    else if(Controller1.ButtonDown.pressing()) {
      Tilter.setVelocity(60, percent);
      Tilter.spin(forward);
    }
    else if(Controller1.ButtonRight.pressing()) {
      Tilter.setVelocity(90, percent);
      Tilter.spinToPosition(perfectTilterPosition, degrees);
    }
    else if(Controller1.ButtonDown.pressing()) {
      Tilter.setVelocity(60, percent);
      Tilter.spinToPosition(0, degrees);
    }
    else {
      Tilter.stop();
    }

    //clamper
    if(Controller1.ButtonX.pressing() && !xPressing) {
      xPressing = true;
    
      clamp = (clamp == 0 ? 1 : 0); //set the clamp value to 1 if it's equal to zero, and 0 if it's equal to 1 (the '?' is basically just an if else)
    }
    else if(!Controller1.ButtonX.pressing()) {
      xPressing = false;
    }
    ClamperL.set(clamp);
    ClamperR.set(clamp);

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
