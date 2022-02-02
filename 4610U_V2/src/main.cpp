/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Luke Crimi                                                */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  4610U code for new bot                                    */
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
// BackClamp            digital_out   E               
// BackArmLimitSwitch   limit         F               
// FrontClamp           digital_out   G               
// Inertial             inertial      9               
// DistanceSensor       distance      10              
// Conveyor             motor         18              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "PID.h"
#include "AutonSelector.h"

using namespace vex;

// A global instance of competition
competition Competition;

void debugging() {
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



//PUT ALL METHODS AND INSTANCE VARIABLES HERE FOR CONTROLLING THE BOT IN BOTH AUTON AND DRIVER
//BELOW THIS LINE

AutonSelector selector; //for auton selection

int currentRotation = 0;

//clamps:
void openBackClamp() {
  BackClamp.set(0);
}
void closeBackClamp() {
  BackClamp.set(1);
}

void openFrontClamp() {
  FrontClamp.set(0);
}
void closeFrontClamp() {
  FrontClamp.set(1);
}

//for toggling the front and back clamper
bool backClamped = false;
bool frontClamped = false;

//callback functions for the proper buttons being pressed
void toggleFrontClamper() {
  if(frontClamped)
    openFrontClamp();
  else
    closeFrontClamp();

  frontClamped = !frontClamped;
}

void toggleBackClamper() {
  if(backClamped)
    openBackClamp();
  else
    closeBackClamp();

  backClamped = !backClamped;
}



//PIDs
PID drivePID(0.08, 0.0, 0.05, 15);
PID turnPID(0.55, 0.0, 0.15, 15);



//PUT ALL METHODS AND INSTANCE VARIABLES HERE FOR CONTROLLING THE BOT IN BOTH AUTON AND DRIVER
//ABOVE THIS LINE



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
  thread t(debugging); //start the debugging thread to view motor temps, positions, etc

  //calibrate inertial sensor
  Inertial.calibrate();
  wait(4.5, seconds);
  Inertial.setRotation(0, degrees);
  currentRotation = 0;

  BackLift.setStopping(hold);
  FrontLift.setStopping(hold);
  Tilter.setStopping(hold);

  //adding autons to the selector
  selector.add("Match Left", "(two center", "goals)");
  selector.add("", "", "");

  //auton selection using the left button
  bool pressing = false;
  while(true) {
    selector.display_autons();
    
    if(Controller1.ButtonLeft.pressing() && !pressing) {
      pressing = true;
      selector.iterate();
    }
    else {
      pressing = false;
    }

    wait(15, msec);
  }
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
  Inertial.setRotation(0, degrees);

  do {
    double speed = turnPid.calculate(Inertial.rotation(degrees), amount) * speedModifier;

    RightBack.setVelocity(-speed, percent);
    RightFront.setVelocity(-speed, percent);
    LeftBack.setVelocity(speed, percent);
    LeftFront.setVelocity(speed, percent);

    RightBack.spin(forward);
    RightFront.spin(forward);
    LeftBack.spin(forward);
    LeftFront.spin(forward);
  } while(abs((int)turnPid.error) > 2);

  RightBack.stop();
  RightFront.stop();
  LeftBack.stop();
  LeftFront.stop();

  //update the current rotation variable
  currentRotation += amount;

  if(currentRotation > 360)
    currentRotation -= 360;

  if(currentRotation < -360)
    currentRotation += 360;
}

void turnToRotation(PID& turnPID, int location, double speedModifier) 
{
  turnWithPID(turnPID, location - currentRotation, speedModifier);
  currentRotation = location;
}

//MOVE METHODS BELOW ======================================================================================================================================

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
  } while(abs((int)pid.error) > 2);

  RightFront.stop();
  LeftFront.stop();
  RightBack.stop();
  LeftBack.stop();
}

void Move(PID& pid, PID& turnPID, int amount, double speed) {
  RightFront.setPosition(0, degrees);
  Inertial.setRotation(0, degrees);

  do {
    double pidSpeed = pid.calculate(RightFront.position(degrees), amount) * speed;
    double turnSpeed = turnPID.calculate(Inertial.rotation(degrees), 0);

    RightFront.setVelocity(pidSpeed - turnSpeed, percent);
    RightBack.setVelocity(pidSpeed - turnSpeed, percent);
    LeftBack.setVelocity(pidSpeed + turnSpeed, percent);
    LeftFront.setVelocity(pidSpeed + turnSpeed, percent);

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

//MOVE METHODS ABOVE ======================================================================================================================================

//autons:

//SKILLS AUTONS:

//MATCH AUTONS:
void LeftTwoCenterGoals() 
{

}

void autonomous(void) {
  if(selector.getSelected() == 0) {
    LeftTwoCenterGoals();
  }
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

void usercontrol(void) 
{
  /*
    *BackLift = 6 bar

    Controls:
      *Analog sticks: movement of the bot (arcade mode)
      *ButtonR1 + ButtonR2: raising and lowering the front lift
      *ButtonL1 + ButtonL2: raising and lowering the back lift
      *ButtonB + ButtonDown: raising and lowering the tilter
      *ButtonY: toggle front clamp
      *ButtonRight: togggle back clamp
  */

  bool curveTurning = true; //should always be set to true except for special situations where you want less controllable turning
  
  while(true) 
  {
    //driving code + curve
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


    //LIFT CODE
    FrontLift.setVelocity(100, percent);
    BackLift.setVelocity(100, percent);

    //front
    if(Controller1.ButtonR1.pressing()) {
      FrontLift.spin(forward); //raise the lift up
    }
    else if(Controller1.ButtonR2.pressing()) {
      FrontLift.spin(reverse); //lower the lift down
    }
    else {
      FrontLift.stop();
    }

    //back
    if(Controller1.ButtonL1.pressing()) {
      BackLift.spin(forward); //raise the lift up
    }
    else if(Controller1.ButtonL2.pressing()) {
      BackLift.spin(reverse); //lower the lift down
    }
    else {
      BackLift.stop();
    }


    //TILTER CODE
    Tilter.setVelocity(50, percent);

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


    //CLAMPER CODE
    Controller1.ButtonY.pressed(toggleFrontClamper);
    Controller1.ButtonRight.pressed(toggleBackClamper);

    //CONVEYOR CODE
    Conveyor.setVelocity(100, percent);
    
    if(Controller1.ButtonA.pressing()) {
      Conveyor.spin(forward);
    }
    else {
      Conveyor.stop();
    }

    wait(20, msec);
  }
}





//main function below this




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
