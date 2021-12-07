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
// RotationSensor       rotation      14              
// InertialSensor       inertial      10              
// Tilter               motor         17              
// ClamperL             digital_out   G               
// ClamperR             digital_out   F               
// TilterButton         bumper        E               
// ArmBumper            bumper        H               
// Lift2                motor         18              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "PID.h"

using namespace vex;

int currentAutonSelection = 1; //auton selection before matches
const int perfectTilterPosition = -230;

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

    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Tilter angle: %f", Tilter.position(degrees));

    wait(15, msec);
    Brain.Screen.clearScreen();
  }
}

//calbacks for changing the selected auton
void upAuton() {
  currentAutonSelection++;
}

void downAuton() {
  currentAutonSelection--;
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

  //auton selection
  while(true) {
    //input for switching auton selection
    Controller1.ButtonRight.pressed(upAuton);
    Controller1.ButtonLeft.pressed(downAuton);

    //keeping the auton selection in range (3 different autons: 0, 1, and 2)
    if(currentAutonSelection > 2)
      currentAutonSelection=0;
    else if(currentAutonSelection < 0)
      currentAutonSelection = 2;

    //updating the controller screen
    if(currentAutonSelection == 0) {
      Controller1.Screen.print("Skills auton");
    }
    else if(currentAutonSelection == 1) {
      Controller1.Screen.print("Right side auton 1");
    }
    else if(currentAutonSelection == 2) {
      Controller1.Screen.print("Left side auton 1");
    }

    wait(200, msec);
    Controller1.Screen.clearScreen();
  }
}

void resetAll() 
{
  bool armSet = false;

  while(!ArmBumper.pressing() || !TilterButton.pressing()) {
    Lift.setVelocity(40, percent);
    Lift2.setVelocity(40, percent);

    Lift.spin(reverse);
    Lift2.spin(reverse);

    if(ArmBumper.pressing() && !armSet) {
      armSet = true;
      Lift.setPosition(0, degrees);
      Lift2.setPosition(0, degrees);
    }

    if(!TilterButton.pressing()) {
      Tilter.setVelocity(40, percent);
      Tilter.spin(forward);
    }

    if(ArmBumper.pressing()) {
      Lift.setStopping(hold);
      Lift2.setStopping(hold);
      Lift.stop();
      Lift2.stop();
    }

    wait(15, msec);
  }


  if(!armSet) {
    Lift.setPosition(0, degrees);
    Lift2.setPosition(0, degrees);
  }
  Lift.stop();
  Lift2.stop();

  Lift.setStopping(hold);
  Lift.setVelocity(100, percent);
  Lift2.setStopping(hold);
  Lift2.setVelocity(100, percent);

  Tilter.stop();
  wait(1, seconds);
  Tilter.setPosition(0, degrees);
  Tilter.setStopping(hold);

  ClamperL.set(1);
  ClamperR.set(1);
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

//Variables stored in heap
double Kp = 0.30;
double Ki = 0.00;
double Kd = 0.40;

double turnKp = 0.6;
double turnKi = 0;
double turnKd = 0.30;

PID driveTrainPID(Kp, Ki, Kd);
PID turnPID(turnKp, turnKi, turnKd);
PID tilterPID(0.5, 0, 0.4);

void turnWithPID(PID& turnPid, int amount, double speedModifier) 
{
  InertialSensor.setRotation(0, degrees);

  do {
    double speed = turnPid.calculate(InertialSensor.rotation(degrees), amount) * speedModifier;

    RightBack.setVelocity(-speed, percent);
    RightFront.setVelocity(-speed, percent);
    LeftBack.setVelocity(speed, percent);
    LeftFront.setVelocity(speed, percent);

    RightBack.spin(forward);
    RightFront.spin(forward);
    LeftBack.spin(forward);
    LeftFront.spin(forward);
  } while(abs((int)turnPid.error) > 4);

  RightBack.stop();
  RightFront.stop();
  LeftBack.stop();
  LeftFront.stop();
}

void Clamp(int val) {
  ClamperL.set(val);
  ClamperR.set(val);
}

//MOVE METHODS------------------------------------------------------------------------------------------------------------------------------
void Move(PID& pid, PID& turnPID, int amount, double speed) {
  RightFront.setPosition(0, degrees);
  RotationSensor.setPosition(0, degrees);

  do {
    double pidSpeed = pid.calculate(RightFront.position(degrees), amount) * speed;
    double turnSpeed = turnPID.calculate(RotationSensor.position(degrees), 0);

    RightFront.setVelocity(pidSpeed + turnSpeed, percent);
    RightBack.setVelocity(pidSpeed + turnSpeed, percent);
    LeftBack.setVelocity(pidSpeed - turnSpeed, percent);
    LeftFront.setVelocity(pidSpeed - turnSpeed, percent);

    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);
  } while(abs((int)pid.error) > 3);

  RightFront.stop();
  LeftFront.stop();
  RightBack.stop();
  LeftBack.stop();
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
  } while(abs((int)pid.error) > 3);

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
//END OF MOVE METHODS --------------------------------------------------------------------------------------------------------------


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

void outClamp() {
  Clamp(1);
}

void inClamp() {
  Clamp(0);
}

void setIntake(bool active) {
  if(active) {
    Conveyor.setVelocity(50, percent);
    Conveyor.spin(forward);
  }
  else {
    Conveyor.stop();
  }
}

//auton functions-----------------------------------------------------------------------------------------------------------------------
void SkillsAuton() {
  //pick up first goal
  Move(driveTrainPID, 100, 30);
  wait(0.3, seconds);
  inClamp();
  wait(0.3, seconds);
  alignTilter(true);
  Move(100, -30);
  turnWithPID(turnPID, -90, 1);
  Move(driveTrainPID, 1000, 40);

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

void RightSideOne() {
  //Right side auton
  Move(driveTrainPID, 500, 0.6);

  wait(0.3, seconds);
  inClamp();
  wait(0.3, seconds);
  Move(driveTrainPID, -240, 0.6);

  alignTilter(true);
  turnWithPID(turnPID, 140, 1);
  setIntake(true);

  Move(900, -30);
  wait(1, seconds);
  setIntake(false);
}

void LeftSideOne() {
  //nothing here yet for this auton
}

void autonomous(void) {
  //resetting tilter position for consitancy when scoring rings into it
  resetAll();

  //extend the clamp
  outClamp();

  //currentAutonSelection assigned in the pre-auton method
  if(currentAutonSelection == 0)
    SkillsAuton();
  else if(currentAutonSelection == 1)
    RightSideOne();
  else if(currentAutonSelection == 2)
    LeftSideOne();
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
  bool yPressing = false;

  bool goForward = true;
  double rightAmount;
  double leftAmount;
  int clamp = 0;

  bool modifier = false;
  bool up = false;
  bool down = false;
  double speed = 0;
  PID tilterPID(0.5, 0, 0.4);

  resetAll();

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
      //IndicatorLight.off();
    }
    else{
      rightAmount = -Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent);
      leftAmount = -Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent);
      //IndicatorLight.on();
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
      Lift2.spin(forward);
    }
    else if(Controller1.ButtonR2.pressing() && Lift.position(degrees) > 0) {
      Lift.spin(reverse);
      Lift2.spin(reverse);
    }
    else {
      Lift.stop();
      Lift2.stop();
    }

    //controlling the intake conveyor
    if(Controller1.ButtonL1.pressing()) { 
      Conveyor.setVelocity(65, percent);
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

    //tilter------------------------------------------------------------------------------------------
    //manual button modifier boolean
    modifier = Controller1.ButtonB.pressing();

    if(Controller1.ButtonRight.pressing() && modifier) {
      Tilter.setVelocity(100, percent);
      Tilter.spin(reverse);
      up = false;
      down = false;
    }
    else if(Controller1.ButtonDown.pressing() && modifier) {
      Tilter.setVelocity(60, percent);
      Tilter.spin(forward);
      up = false;
      down = false;
    }
    else if(Controller1.ButtonRight.pressing() && !modifier) {
      up = true;
      down = false;
    }
    else if(Controller1.ButtonDown.pressing() && !modifier) {
      up = false;
      down = true;
    }
    else {
      Tilter.stop();
    }

    if(up) {
      if(Tilter.position(degrees) > perfectTilterPosition) {
        speed = tilterPID.calculate(Tilter.position(degrees), perfectTilterPosition);
        Tilter.setVelocity(-speed, percent);
        Tilter.spin(reverse);
      }
      else {
        speed = tilterPID.calculate(Tilter.position(degrees), perfectTilterPosition);
        Tilter.setVelocity(-speed, percent);
        Tilter.spin(forward);
      }
    }

    if(down) {
      if(Tilter.position(degrees) > -0) {
        speed = tilterPID.calculate(Tilter.position(degrees), -0);
        Tilter.setVelocity(speed * 0.5, percent);
        Tilter.spin(reverse);
      }
      else {
        speed = tilterPID.calculate(Tilter.position(degrees), -0);
        Tilter.setVelocity(speed * 0.5, percent);
        Tilter.spin(forward);
      }
    }
    //end of tilter code ---------------------------------------------------------------------


    //clamper
    if(Controller1.ButtonY.pressing() && !yPressing) {
      yPressing = true;
    
      clamp = (clamp == 0 ? 1 : 0); //set the clamp value to 1 if it's equal to zero, and 0 if it's equal to 1 (the '?' is basically just an if else)
    }
    else if(!Controller1.ButtonY.pressing()) {
      yPressing = false;
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
