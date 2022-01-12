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
// RightFront           motor         8               
// RightBack            motor         21              
// LeftFront            motor         15              
// LeftBack             motor         11              
// Lift                 motor         13              
// Conveyor             motor         12              
// InertialSensor       inertial      10              
// Tilter               motor         17              
// ClamperL             digital_out   G               
// ClamperR             digital_out   F               
// TilterButton         bumper        E               
// ArmBumper            bumper        H               
// Lift2                motor         18              
// ClamperDistance      distance      19              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "PID.h"

using namespace vex;

int currentAutonSelection = 6; //auton selection before matches
const int perfectTilterPosition = -180;

int currentRotation = 0;

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

//for use in the preauton code to select an auton
bool rightPressing = false;
bool leftPressing = false;

void drawStuff() {
  while (true) {
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Motor temps:");

    Brain.Screen.setCursor(2,1);
    Brain.Screen.print("Front right: %f", RightFront.temperature());

    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Front left: %f", LeftFront.temperature());

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Back right: %f", RightBack.temperature());

    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Back left: %f", LeftBack.temperature());

    wait(15, msec);
    Brain.Screen.clearScreen();
  }
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  thread t(drawStuff); //start the drawStuff thread

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  InertialSensor.calibrate();
  wait(4, seconds);
  InertialSensor.setRotation(0, degrees);

  int autonCount = 6;

  //auton selection
  while(true) {
    //input for switching auton selection
    /*
    if(Controller1.ButtonRight.pressing()) {
      if(!rightPressing) {
        rightPressing = true;
        currentAutonSelection++;
      }
    }
    else {
      rightPressing = false;
    }
*/
    if(Controller1.ButtonLeft.pressing()) {
      if(!leftPressing) {
        leftPressing = true;
        currentAutonSelection++;
        Controller1.rumble(".");
      }
    }
    else {
      leftPressing = false;
    }


    //keeping the auton selection in range (3 different autons: 0, 1, and 2)
    if(currentAutonSelection > autonCount)
      currentAutonSelection=0;
    else if(currentAutonSelection < 0)
      currentAutonSelection = autonCount;

    //updating the controller screen
    Controller1.Screen.setCursor(1,1);

    if(currentAutonSelection == 0) {
      Controller1.Screen.print("Skills auton");
    }
    else if(currentAutonSelection == 1) {
      Controller1.Screen.print("Right side auton 1");
    }
    else if(currentAutonSelection == 2) {
      Controller1.Screen.print("Left side auton 1");
    }
    else if(currentAutonSelection == 3) {
      Controller1.Screen.print("No auton");
    }
    else if(currentAutonSelection == 4) {
      Controller1.Screen.print("Conveyor spin auton");
    }
    else if(currentAutonSelection == 5) {
      Controller1.Screen.print("Right side skills");
    }
    else if(currentAutonSelection == 6) {
      Controller1.Screen.print("Right side auton 2 (Experimental)");
    }

    wait(15, msec);
    Controller1.Screen.clearScreen();
  }
}

//align the arms and the tilter and set their positions as 0
void resetAll()
{
  bool armSet = false;
  currentRotation = 0;

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
double Kp = 0.35;
double Ki = 0.01;
double Kd = 0.15;

double turnKp = 0.40;
double turnKi = 0.00;
double turnKd = 0.10;

PID driveTrainPID(Kp, Ki, Kd, 20, 20, 10);
PID turnPID(turnKp, turnKi, turnKd);
PID moveTurnPID(0.3, 0, 0.6);
PID tilterPID(0.4, 0, 0.45);


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

    this_thread::sleep_for(20);
  } while(abs((int)turnPid.error) > 4);

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


void Clamp(int val) {
  ClamperL.set(val);
  ClamperR.set(val);
}

//MOVE METHODS------------------------------------------------------------------------------------------------------------------------------
void Move(PID& pid, PID& turnPID, int amount, double speed) {
  RightFront.setPosition(0, degrees);
  InertialSensor.setRotation(0, degrees);

  do {
    double pidSpeed = pid.calculate(RightFront.position(degrees), amount) * speed;
    double turnSpeed = turnPID.calculate(InertialSensor.rotation(degrees), 0);

    RightFront.setVelocity(pidSpeed - turnSpeed, percent);
    RightBack.setVelocity(pidSpeed - turnSpeed, percent);
    LeftBack.setVelocity(pidSpeed + turnSpeed, percent);
    LeftFront.setVelocity(pidSpeed + turnSpeed, percent);

    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);

    this_thread::sleep_for(20);
  } while(abs((int)pid.error) > 3);

  RightFront.stop();
  LeftFront.stop();
  RightBack.stop();
  LeftBack.stop();
}

//for moving and turning at the same time
void Move(PID& pid, PID& turnPID, int amount, double speed, double turningSpeed, int finishedAngle) {
  RightFront.setPosition(0, degrees);
  InertialSensor.setRotation(0, degrees);
  bool useTurn = true;

  do {
    double pidSpeed = pid.calculate(RightFront.position(degrees), amount);
    if(pidSpeed > 100) {
      pidSpeed = 100;
    }
    else {
      useTurn = false;
    }
    pidSpeed *= speed;

    double turnSpeed;

    if(useTurn)
      turnSpeed = turnPID.calculate(InertialSensor.rotation(degrees), finishedAngle) * turningSpeed;
    else
      turnSpeed = 0;

    RightFront.setVelocity(pidSpeed - turnSpeed, percent);
    RightBack.setVelocity(pidSpeed - turnSpeed, percent);
    LeftBack.setVelocity(pidSpeed + turnSpeed, percent);
    LeftFront.setVelocity(pidSpeed + turnSpeed, percent);

    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);

    this_thread::sleep_for(20);
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

    this_thread::sleep_for(20);
  } while(abs((int)pid.error) > 5);

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

void MoveAndPulseIntake(int amount, int speed) {
  RightFront.setPosition(0, degrees);

  while(abs((int)RightFront.position(degrees)) < amount) {
    Conveyor.setVelocity((sin(RightFront.position(degrees)) + 1) * 50, percent);
    Conveyor.spin(forward);

    RightFront.setVelocity(speed, percent);
    RightBack.setVelocity(speed, percent);
    LeftBack.setVelocity(speed, percent);
    LeftFront.setVelocity(speed, percent);

    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);
  }

  Conveyor.stop();

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
    Conveyor.setVelocity(65, percent);
    Conveyor.spin(forward);
  }
  else {
    Conveyor.stop();
  }
}


//seperate boolean used in the autons to control whether or not the clamper should clamp (stored in heap so that any auton function can access it)
bool clampUsingLidar = false;
bool stopClampThread = false; //for determining if the thread should be stopped

//seperate thread of clamping down whenever a lidar detects something in range
void lidarClampThread() {
  stopClampThread = false;

  while(!stopClampThread) {
    if((ClamperDistance.objectDistance(mm) < 25 && clampUsingLidar) && ClamperDistance.objectDistance(mm) != 0) {
      inClamp();
      wait(1, seconds);
    }
    else {
      outClamp();
    }

    wait(15, msec);
  }
}



//auton functions-----------------------------------------------------------------------------------------------------------------------
void SkillsAuton() {
  //resetting tilter position for consitancy when scoring rings into it
  resetAll();

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

void RightSideSkills() {
  //RIGHT SIDE AUTON

  //resetting tilter position for consitancy when scoring rings into it
  resetAll();

  thread t(lidarClampThread);

  //move forward and turn (start against the wall for maximum consistency)
  Move(75, 50);

  //turn towards mobile goal on the AWP line
  turnWithPID(turnPID, 50, 1);

  clampUsingLidar = true; //autoclamp when front lidar detects object

  //move forward to the mobile goal on the AWP line
  Move(driveTrainPID, 520, 0.4);

  //grab the mobile goal and move back (performed automatically throught the lidar thread)
  Move(driveTrainPID, -240, 0.6);

  //align the tilter to the conveyer and start the intake
  alignTilter(true);
  turnToRotation(turnPID, 180, 1);

  //move backwards while the intake is running to get some pringles on the mobile goal
  Move(200,-50);
  setIntake(true);
  Move(700, -30);
  //turnWithPID(turnPID, -15, 1);

  //move to the other side of the field
  Move(driveTrainPID, turnPID, -950, 0.9);

  //put down the mogoal and move to the middle mogoal
  alignTilter(false);
  setIntake(false);

  //turn off the automatic detection of the lidar thread (to pick up mogoals)
  clampUsingLidar = false;
  wait(0.5, seconds);

  //move backwards first before leaving so that the bot doesn't hit the mogoal it just worked so hard to get here in the first place
  Move(driveTrainPID, -240, 1);

  turnWithPID(turnPID, 41, 1);

  Move(driveTrainPID, turnPID, 1500, 0.5);

  //clamp down on the middle mogoal
  inClamp();

  Move(driveTrainPID, turnPID, 1400, 1); //move to the opposite side of the field

  //stop lidar thread
  stopClampThread = true;
}
//SKILLS ABOVE THIS LINE-----------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------


void RightSideOne() {
  //RIGHT SIDE AUTON

  //resetting tilter position for consitancy when scoring rings into it
  resetAll();

  thread t(lidarClampThread);

  //move forward and turn (start against the wall for maximum consistency)
  Move(75, 50);

  //turn towards mobile goal on the AWP line
  turnWithPID(turnPID, 50, 1);

  clampUsingLidar = true; //autoclamp when front lidar detects object

  //move forward to the mobile goal on the AWP line
  Move(driveTrainPID, 530, 0.4);

  //grab the mobile goal and move back (performed automatically throught the lidar thread)
  Move(driveTrainPID, -240, 0.6);

  //align the tilter to the conveyer and start the intake
  alignTilter(true);
  turnToRotation(turnPID, 180, 1);

  //move backwards while the intake is running to get some pringles on the mobile goal
  Move(100,-50);
  setIntake(true);
  Move(700, -20);

  Move(300, 100);
  wait(1, seconds);
  setIntake(false);

  stopClampThread = true;
  alignTilter(false);
}

void RightSideTwo() 
{
  thread t(lidarClampThread);

  //move forward to get the middle neutral mogoal

  clampUsingLidar = true;



  //move back, grab the mogoal on the AWP line and dispense preload rings into it

  stopClampThread = true;
}

void LeftSideOne() {
  //drive forward to pick up mobile goal
  thread t(lidarClampThread);
  clampUsingLidar = true; //autoclamp when front lidar detects object

  Move(driveTrainPID, moveTurnPID, 1300, 1, 10, 20);

  wait(0.01, seconds);

  //drive back
  turnWithPID(turnPID, -15, 1);
  Move(driveTrainPID, -1100, 0.7);

  wait(0.01, seconds);

  clampUsingLidar = false; //release clamp regardless of anything detected by the lidar
  stopClampThread = true;
}

void EmptyAuton() {

}

void ConveyorSpinAuton() {
  setIntake(true);

  while(true)
    wait(15, msec);
}

void autonomous(void) {
  //extend the clamp
  outClamp();

  //currentAutonSelection assigned in the pre-auton method
  if(currentAutonSelection == 0)
    SkillsAuton();
  else if(currentAutonSelection == 1)
    RightSideOne();
  else if(currentAutonSelection == 2)
    LeftSideOne();
  else if(currentAutonSelection == 3)
    EmptyAuton(); //do nothing (empty auton for when we don't run auton)
  else if(currentAutonSelection == 4)
    ConveyorSpinAuton();
  else if(currentAutonSelection == 5)
    RightSideSkills();
  else if(currentAutonSelection == 6)
    RightSideTwo();
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
    else if(Controller1.ButtonR2.pressing()) {
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
      speed = tilterPID.calculate(Tilter.position(degrees), perfectTilterPosition);
      Tilter.setVelocity(-speed, percent);
      Tilter.spin(reverse);
    }

    if(down) {
      speed = tilterPID.calculate(Tilter.position(degrees), -0);
      Tilter.setVelocity(speed * 0.5, percent);
      Tilter.spin(forward);
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