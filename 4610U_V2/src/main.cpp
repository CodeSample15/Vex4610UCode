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
// FrontDistanceSensor  distance      17              
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

    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Tilter: %f", Tilter.temperature(percent));

    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("6 Bar Arm: %f", BackLift.temperature(percent));

    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("4 Bar Arm: %f", FrontLift.temperature(percent));

    wait(15, msec);

    Brain.Screen.clearScreen();
  }
}



//PUT ALL METHODS AND INSTANCE VARIABLES HERE FOR CONTROLLING THE BOT IN BOTH AUTON AND DRIVER
//BELOW THIS LINE

AutonSelector selector; //for auton selection

int currentRotation = 0;

//variables to keep track of certain speeds and whatnot
int conveyorSpeed = 70;
int tilterSpeed = 50;
int liftSpeed = 100;
int maxTurningSpeed = 50;

int tiltAmount = -30;

int lidarDistance = 16; //how far a mogoal has to be from the distance sensor before it clamps //used to be 17
int frontLidarDistance = 16;

bool stopThreads = false; //stop threads at the end of autons so that they don't carry over to the teleop period

bool autonRunning = false;

//clamps:
void openBackClamp() {
  BackClamp.set(0);
}
void closeBackClamp() {
  BackClamp.set(1);
}

void openFrontClamp() {
  FrontClamp.set(1);
}
void closeFrontClamp() {
  FrontClamp.set(0);
}

//tilters
void setFrontTilter(bool tilt) {
  Tilter.setVelocity(tilterSpeed, percent);

  if(tilt) {
    while(!BackArmLimitSwitch.pressing()) {
      BackLift.spin(reverse);
      wait(15, msec);
    }
    BackLift.stop();

    Tilter.spinToPosition(tiltAmount, degrees);
  }
  else {
    Tilter.spinToPosition(0, degrees);
  }
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

//for turning the intake on and off (autons)
void setIntake(bool intake) {
  if(intake) {
    Conveyor.setVelocity(conveyorSpeed, percent);
    Conveyor.spin(forward);
  }
  else {
    Conveyor.stop();
  }
}

//PIDs
PID drivePID(0.09, 0.00, 0.05, 15); //used to be 0.08 for p
PID turnPID(0.65, 0.00, 0.15, 15);


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

  //set the stopping modes of the lifts and tilter
  BackLift.setStopping(hold);
  FrontLift.setStopping(hold);
  Tilter.setStopping(hold);

  //reset positions
  BackLift.setPosition(0, degrees);
  FrontLift.setPosition(0, degrees);
  Tilter.setPosition(0, degrees);

  //adding autons to the selector
  selector.add("");

  //auton selection menu
  bool pressing = false;
  selector.display_autons();

  while(true) {
    selector.display_autons(); //update screen

    if(Controller1.ButtonUp.pressing() && !pressing) {
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

//reset for the tilter
void resetTilter() 
{
  //lift arm up
  BackLift.spinToPosition(20, degrees);

  //lower arm down
  while(!BackArmLimitSwitch.pressing() && !stopThreads) {
    BackLift.spin(reverse);
    Tilter.spin(forward);
    wait(15, msec);
  }
  Tilter.stop();
  BackLift.stop();

  //reset motor encoder on tilter
  Tilter.setPosition(0, degrees);
}

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

int MoveUntilClamp(int speed) {
  int startPosition = RightFront.position(degrees);

  while(DistanceSensor.objectDistance(mm) > lidarDistance && DistanceSensor.objectDistance(mm) != 0) {
    RightFront.setVelocity(speed, percent);
    RightBack.setVelocity(speed, percent);
    LeftBack.setVelocity(speed, percent);
    LeftFront.setVelocity(speed, percent);

    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);

    wait(15, msec);
  }

  int endPosition = RightFront.position(degrees);

  RightFront.stop();
  LeftFront.stop();
  RightBack.stop();
  LeftBack.stop();

  return abs(endPosition - startPosition);
}

//returns the total distance traveled by the bot to help with positioning
int MoveUntilClamp(int speed, int maxMove) {
  RightFront.setPosition(0, degrees);

  while(DistanceSensor.objectDistance(mm) > lidarDistance && DistanceSensor.objectDistance(mm) != 0) {
    RightFront.setVelocity(speed, percent);
    RightBack.setVelocity(speed, percent);
    LeftBack.setVelocity(speed, percent);
    LeftFront.setVelocity(speed, percent);

    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);

    if(abs((int)RightFront.position(degrees)) > maxMove) {
      break;
    }

    wait(15, msec);
  }

  RightFront.stop();
  LeftFront.stop();
  RightBack.stop();
  LeftBack.stop();

  return abs((int)RightFront.position(degrees));
}

//MOVE METHODS ABOVE ======================================================================================================================================

//SKILLS AUTONS:

/*
  ACTUAL AUTON METHOD CALLS BELOW

  *Uses the auton selector class to determine which program to run
*/
void autonomous(void) {
  autonRunning = true;

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

void hapticFeedBack() {
  bool detectingBack = false;
  bool detectingFront = false;

  while(!autonRunning) {
    //LIDAR DETECTION + HAPTIC FEEDBACK

    //back sensor
    if(DistanceSensor.objectDistance(mm) < lidarDistance && DistanceSensor.objectDistance(mm) != 0 && !detectingBack) {
      Controller1.rumble(".");
      detectingBack = true;
    }
    else if(DistanceSensor.objectDistance(mm) > lidarDistance || DistanceSensor.objectDistance(mm) == 0) {
      detectingBack = false;
    }

    //front sensor
    if(FrontDistanceSensor.objectDistance(mm) < frontLidarDistance && FrontDistanceSensor.objectDistance(mm) != 0 && !detectingFront) {
      Controller1.rumble("-");
      detectingFront = true;
    }
    else if(FrontDistanceSensor.objectDistance(mm) > frontLidarDistance || FrontDistanceSensor.objectDistance(mm) == 0) {
      detectingFront = false;
    }

    wait(5, msec);
  }
}

void usercontrol(void) 
{
  /*
    *BackLift = 6 bar

    Controls:
      *Analog sticks: movement of the bot (arcade mode)
      *ButtonR1 + ButtonR2: raising and lowering the front lift
      *ButtonL1 + ButtonL2: raising and lowering the back lift
      *ButtonDown + ButtonLeft: raising and lowering the tilter
      *ButtonB: intake
      *ButtonA: outtake
      *ButtonY: toggle front clamp
      *ButtonX: Modifier for slowing down the drive
      *ButtonRight: togggle back clamp
  */

  autonRunning = false;
  thread d(hapticFeedBack);

  bool curveTurning = true; //should always be set to true except for special situations where you want less controllable turning
  stopThreads = true; //stop any existing threads still running from auton
  
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

    if(turnAmount > maxTurningSpeed) {
      turnAmount = maxTurningSpeed;
    }

    //apply modifier for slow driving (X button)
    rightAmount = (Controller1.Axis3.position(percent) * (Controller1.ButtonX.pressing() ? .4 : 1) ) - turnAmount;
    leftAmount = (Controller1.Axis3.position(percent) * (Controller1.ButtonX.pressing() ? .4 : 1) ) + turnAmount;

    RightFront.setVelocity(rightAmount, percent);
    RightBack.setVelocity(rightAmount, percent);
    LeftFront.setVelocity(leftAmount, percent);
    LeftBack.setVelocity(leftAmount, percent);

    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);


    //LIFT CODE
    FrontLift.setVelocity(liftSpeed, percent);
    BackLift.setVelocity(liftSpeed, percent);

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
    else if(Controller1.ButtonL2.pressing() && !BackArmLimitSwitch.pressing()) {
      BackLift.spin(reverse); //lower the lift down
    }
    else {
      BackLift.stop();
    }


    //TILTER CODE
    Tilter.setVelocity(tilterSpeed, percent);

    if(Controller1.ButtonDown.pressing()) {
      //move the tilter up
      Tilter.spin(reverse);
    }
    else if(Controller1.ButtonLeft.pressing() && !BackArmLimitSwitch.pressing()) {
      //move the tilter down
      Tilter.spin(forward);
    }
    else {
      Tilter.stop();
    }


    //CLAMPER CODE
    Controller1.ButtonY.pressed(toggleFrontClamper);
    Controller1.ButtonRight.pressed(toggleBackClamper);

    //CONVEYOR CODE
    Conveyor.setVelocity(conveyorSpeed, percent);
    
    if(Controller1.ButtonB.pressing()) {
      Conveyor.spin(forward);
    }
    else if(Controller1.ButtonA.pressing()) {
      Conveyor.spin(reverse);
    }
    else {
      Conveyor.stop();
    }

    wait(15, msec);
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
