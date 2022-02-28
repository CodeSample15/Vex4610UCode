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
// FrontDistanceSensor  distance      20              
// LeftLineTracker      line          D               
// RightLineTracker     line          C               
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

    Brain.Screen.setCursor(9, 1);
    Brain.Screen.print("Debugging positions: ");

    Brain.Screen.setCursor(10, 1);
    Brain.Screen.print("Front lidar reading: %f", FrontDistanceSensor.objectDistance(mm));

    Brain.Screen.setCursor(11, 1);
    Brain.Screen.print("Right line trackers: %d", RightLineTracker.reflectivity());

    Brain.Screen.setCursor(12, 1);
    Brain.Screen.print("Inertial: %f", Inertial.rotation(degrees));

    wait(15, msec);

    Brain.Screen.clearScreen();
  }
}



//PUT ALL METHODS AND INSTANCE VARIABLES HERE FOR CONTROLLING THE BOT IN BOTH AUTON AND DRIVER
//BELOW THIS LINE

AutonSelector selector(0); //for auton selection

int currentRotation = 0;

//threshold of what is considered a white line when it comes to the line tracker sensors
float reflectiveThreshold = 10;
float rightReflectiveThreshold = 10;

//variables to keep track of certain speeds and whatnot
int conveyorSpeed = 80;
int tilterSpeed = 50;
int liftSpeed = 100;
//int maxTurningSpeed = 70;

//variables for preset motor positions
int tiltAmount = -320;
int FrontArmUpPosition = 900;
int BackArmUpPosition = 640;

int lidarDistance = 15; //how far a mogoal has to be from the distance sensor before it clamps //used to be 17
int frontLidarDistance = 40;

bool stopThreads = false; //stop threads at the end of autons so that they don't carry over to the teleop period

bool autonRunning = false;
bool armCodeRunning = false; //for keeping track of arm pids that are running during auton
bool stopArmThreads = false; //for stopping other threads that are running arm code during auton

bool autonRan = false; //killing loops that don't need to run after the auton executes

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
PID drivePID(0.09, 0.00, 0.07, 15); //used to be 0.08 for p
PID turnPID(0.55, 0.005, 0.40, 15, 20, 3);

PID armPID(0.75, 0, 0);
PID clampPID(0.23, 0.00, 0.015, 15); //when driving up to objects using the clamper data as input


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

  //reset positions
  BackLift.setPosition(0, degrees);
  FrontLift.setPosition(0, degrees);
  Tilter.setPosition(0, degrees);

  //set the stopping modes of the lifts and tilter
  BackLift.setStopping(hold);
  FrontLift.setStopping(hold);
  Tilter.setStopping(hold);

  //adding autons to the selector
  selector.add("Right side one", "(right neutral goal", "AWP line goal)");                  //0
  selector.add("Left side one", "(Left side neutral goal", "and middle neutral goal)");     //1
  selector.add("Left side two", "(JUST the left side neutral", "goal)");                    //2
  selector.add("DO NOT RUN", "FOR LUKE TO TEST", "AUTON STUFF ONLY");                       //3
  selector.add("Right side two", "(ONLY middle mogoal", "from right side)");                //4
  selector.add("SKILLS", "(main skills program)");                                          //5          

  closeFrontClamp();

  autonRan = false; //auton has not run yet

  //calibrate inertial sensor last
  Inertial.calibrate();
  wait(4.5, seconds);
  Inertial.setRotation(0, degrees);
  currentRotation = 0;


  //auton selection menu once everything is done resetting and calibrating
  bool pressing = false;
  selector.display_autons();

  while(!autonRan) {
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

//turn code using pid
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
  } while(abs((int)turnPid.error) >= 1);

  RightBack.setStopping(brake);
  RightFront.setStopping(brake);
  LeftBack.setStopping(brake);
  LeftFront.setStopping(brake);

  RightBack.stop();
  RightFront.stop();
  LeftBack.stop();
  LeftFront.stop();

  wait(50, msec);

  RightBack.setStopping(coast);
  RightFront.setStopping(coast);
  LeftBack.setStopping(coast);
  LeftFront.setStopping(coast);

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


//Tilter code
void tilterUp() 
{
  int startLocation = Tilter.position(degrees);

  while(abs((int)Tilter.position(degrees) - startLocation) < abs(tiltAmount)) {
    Tilter.spin(reverse);
  }

  Tilter.stop();
}

void tilterDown()
{
  int startLocation = Tilter.position(degrees);

  while(abs((int)Tilter.position(degrees) - startLocation) < abs(tiltAmount)) {
    Tilter.spin(forward);

    if(BackArmLimitSwitch.pressing())
      break;
  }

  Tilter.stop();
}



//MOVING ARMS: ----------------------------------------------------------------------------------------------------------------------------------------------
//stop preexisting arm threads
void manageArmThreads() {
  if(armCodeRunning) {
    stopArmThreads = true;
  }
  else {
    stopArmThreads = false;
  }

  armCodeRunning = false;
  wait(15, msec);
}


void frontArmUp() 
{
  manageArmThreads();
  armCodeRunning = true;

  do {
    double speed = armPID.calculate(FrontLift.position(degrees), FrontArmUpPosition);

    FrontLift.setVelocity(speed, percent);
    FrontLift.spin(forward);
  } while(abs((int)armPID.error) > 5 && !stopArmThreads);

  FrontLift.stop();

  //telling the rest of the program that the code is finished running
  stopArmThreads = false;
  armCodeRunning = false;
}

void frontArmDown() 
{
  manageArmThreads();
  armCodeRunning = true;

  do {
    double speed = armPID.calculate(FrontLift.position(degrees), 0);

    FrontLift.setVelocity(speed, percent);
    FrontLift.spin(forward);
  } while(abs((int)armPID.error) > 5 && !stopArmThreads);

  FrontLift.stop();

  //telling the rest of the program that the code is finished running
  stopArmThreads = false;
  armCodeRunning = false;
}

void backArmUp() 
{
  manageArmThreads();
  armCodeRunning = true;

  do {
    double speed = armPID.calculate(BackLift.position(degrees), BackArmUpPosition);

    BackLift.setVelocity(speed, percent);
    BackLift.spin(forward);
  } while(abs((int)armPID.error) > 5 && !stopArmThreads);

  BackLift.stop();

  //telling the rest of the program that the code is finished running
  stopArmThreads = false;
  armCodeRunning = false;
}

void backArmDown() 
{
  manageArmThreads();
  armCodeRunning = true;

  do {
    double speed = armPID.calculate(BackLift.position(degrees), 0);

    BackLift.setVelocity(speed, percent);
    BackLift.spin(forward);
  } while(abs((int)armPID.error) > 5 && !stopArmThreads);

  BackLift.stop();

  //telling the rest of the program that the code is finished running
  stopArmThreads = false;
  armCodeRunning = false;
}


//for only slightly moving the two arms in order to prevent the mogoals from rubbing against the ground
void smallFrontArmLift()
{
  FrontLift.spinToPosition(100, degrees);
}

void smallBackArmLift() 
{
  BackLift.spinToPosition(100, degrees);
}


//for fast stops
void hardDriveStop()
{
  RightBack.setStopping(brake);
  RightFront.setStopping(brake);
  LeftBack.setStopping(brake);
  LeftFront.setStopping(brake);

  RightBack.stop();
  RightFront.stop();
  LeftBack.stop();
  LeftFront.stop();

  while(abs((int)RightBack.velocity(percent)) > 3) {
    wait(15, msec);
  }

  RightBack.setStopping(coast);
  RightFront.setStopping(coast);
  LeftBack.setStopping(coast);
  LeftFront.setStopping(coast);
}
//MOVING ARMS -----------------------------------------------------------------------------------------------------------------------------------------------





//MOVE METHODS BELOW ======================================================================================================================================

void Move(int amount, int speed, bool hardstop) {
  RightFront.setPosition(0, degrees);

  if(amount < 0)
    speed *= -1;

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

  if(hardstop) 
    hardDriveStop();
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
  } while(abs((int)pid.error) > 2);

  RightFront.stop();
  LeftFront.stop();
  RightBack.stop();
  LeftBack.stop();
}




//returns the total distance traveled by the bot to help with positioning
int MoveUntilClamp(int speed, int maxMove) {
  RightFront.setPosition(0, degrees);
  int lidarReading = 0;
  int target = speed > 0 ? frontLidarDistance : lidarDistance;

  do {
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

    if(speed < 0) {
      //robot is moving reverse
      lidarReading = DistanceSensor.objectDistance(mm);
    }
    else { 
      //robot is moving forward
      lidarReading = FrontDistanceSensor.objectDistance(mm);
    }

  } while(lidarReading > target && lidarReading != 0);

  RightFront.stop();
  LeftFront.stop();
  RightBack.stop();
  LeftBack.stop();

  return abs((int)RightFront.position(degrees));
}

//PID edition of the method ebove
int MoveUntilClamp(PID& pid, float speed, int maxMove) {
  RightFront.setPosition(0, degrees);
  int lidarReading = 0;
  int target = (speed < 0 ? lidarDistance : frontLidarDistance);

  do {
    if(abs((int)RightFront.position(degrees)) > maxMove) {
      break;
    }

    if(speed < 0) {
      //robot is moving reverse
      lidarReading = DistanceSensor.objectDistance(mm);
    }
    else { 
      //robot is moving forward
      lidarReading = FrontDistanceSensor.objectDistance(mm);
    }

    int addition = (speed < 0 ? -10 : 10);

    double moveSpeed = (clampPID.calculate(lidarReading, target) * speed * -1) + addition;

    RightFront.setVelocity(moveSpeed, percent);
    RightBack.setVelocity(moveSpeed, percent);
    LeftBack.setVelocity(moveSpeed, percent);
    LeftFront.setVelocity(moveSpeed, percent);

    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);

    if(lidarReading < target + (RightFront.velocity(percent) / 2)) {
      closeFrontClamp();
      break;
    }
  } while(lidarReading > target && lidarReading != 0);

  //hardDriveStop(); //stop the bot quickly by setting the motors to brake

  RightFront.stop();
  RightBack.stop();
  LeftFront.stop();
  LeftBack.stop();

  return abs((int)RightFront.position(degrees));
}

//pid that locks rotation as well
void MoveUntilClamp(PID& pid, PID& turnLock, float speed, int maxMove) {
  RightFront.setPosition(0, degrees);
  Inertial.setRotation(0, degrees);

  int lidarReading = 0;
  int target = (speed < 0 ? lidarDistance : frontLidarDistance);

  do {
    if(abs((int)RightFront.position(degrees)) > maxMove) {
      break;
    }

    if(speed < 0) {
      //robot is moving reverse
      lidarReading = DistanceSensor.objectDistance(mm);
    }
    else { 
      //robot is moving forward
      lidarReading = FrontDistanceSensor.objectDistance(mm);
    }

    int addition = (speed < 0 ? -10 : 10);

    double moveSpeed = (pid.calculate(lidarReading, target) * speed * -1) + addition;
    double turnSpeed = turnLock.calculate(Inertial.rotation(degrees), 0);

    RightFront.setVelocity(moveSpeed - turnSpeed, percent);
    RightBack.setVelocity(moveSpeed - turnSpeed, percent);
    LeftBack.setVelocity(moveSpeed + turnSpeed, percent);
    LeftFront.setVelocity(moveSpeed + turnSpeed, percent);

    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);

    if(lidarReading < target+(RightFront.velocity(percent) / 2)) {
      closeFrontClamp();
      break;
    }
  } while(lidarReading > target && lidarReading != 0);

  hardDriveStop(); //stop the bot quickly by setting the motors to brake
}


//move back until a line is detected under the robot
void MoveUntilLine(int speed)
{
  RightFront.setVelocity(speed, percent);
  RightBack.setVelocity(speed, percent);
  LeftBack.setVelocity(speed, percent);
  LeftFront.setVelocity(speed, percent);

  while(LeftLineTracker.reflectivity() < reflectiveThreshold && RightLineTracker.reflectivity() < rightReflectiveThreshold) 
  {
    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);
  }

  hardDriveStop(); //stop the bot quickly by setting the motors to brake
}

//same method as the one above, but with a turn pid to lock rotation
void MoveUntilLine(PID& pid, int speed) 
{
  Inertial.setRotation(0, degrees);

  while(LeftLineTracker.reflectivity() < reflectiveThreshold && RightLineTracker.reflectivity() < rightReflectiveThreshold) 
  {
    double turnSpeed = pid.calculate(Inertial.rotation(degrees), 0);

    RightFront.setVelocity(speed - turnSpeed, percent);
    RightBack.setVelocity(speed - turnSpeed, percent);
    LeftBack.setVelocity(speed + turnSpeed, percent);
    LeftFront.setVelocity(speed + turnSpeed, percent);

    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);
  }

  hardDriveStop(); //stop the bot quickly by setting the motors to brake
}

//untested method to line the bot up to a line
void LineUpOnLine(int speed) //TODO: TEST THIS METHOD
{
  bool spinningLeft = true;
  bool spinningRight = true;

  while(spinningLeft || spinningRight)
  {
    RightFront.setVelocity(spinningRight ? speed : 0, percent);
    RightBack.setVelocity(spinningRight ? speed : 0, percent);
    LeftFront.setVelocity(spinningLeft ? speed : 0, percent);
    LeftBack.setVelocity(spinningLeft ? speed : 0, percent);

    RightFront.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    LeftBack.spin(forward);

    if(LeftLineTracker.reflectivity() > reflectiveThreshold)
      spinningLeft = false;
    if(RightLineTracker.reflectivity() > rightReflectiveThreshold)
      spinningRight = false;
  }

  RightFront.stop();
  LeftFront.stop();
  RightBack.stop();
  LeftBack.stop();
}


//"S turn"
void MoveAndTurn(PID& pid, PID& turnPID, int amount, double speed, double turningSpeed, int finishedAngle) {
  RightFront.setPosition(0, degrees);
  Inertial.setRotation(0, degrees);

  do {
    double pidSpeed = pid.calculate(RightFront.position(degrees), amount);
    pidSpeed *= speed;

    double turnSpeed;
    turnSpeed = turnPID.calculate(Inertial.rotation(degrees), finishedAngle) * turningSpeed;

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










//SKILLS
void SkillsAutonMain() 
{
  thread t(resetTilter);
  openFrontClamp();

  //pick up awp
  turnWithPID(turnPID, -120, 1);
  int disp = MoveUntilClamp(-50, 3000);
  closeBackClamp();
  wait(0.4, seconds);

  //move back
  Move(disp - 100, 30, true);
  tilterUp(); //tilt mogoal

  //turn to center
  turnToRotation(turnPID, 0, 1);

  //drive forward to neutral and clamp
  LineUpOnLine(20); //make sure the bot is lined up
  currentRotation = 0;

  MoveUntilClamp(40, 5000);
  closeFrontClamp();

  //drive forward to line
  Move(1000, 30, false);
  MoveUntilLine(30);

  //overshoot turn to platform and move forward a little bit
  turnWithPID(turnPID, -90, 1);
  Move(drivePID, turnPID, 1500, 1);

  //turn to platform
  turnWithPID(turnPID, 45, 1);

  //lift arms up
  frontArmUp();



  //score one mogoal
  Move(drivePID, turnPID, 1500, 0.8);
  openFrontClamp();
  wait(0.1, seconds);

  //back up and turn
  Move(drivePID, turnPID, -1500, 0.8);
  turnWithPID(turnPID, 180, 1);

  //lift up other arm
  tilterDown();
  backArmUp();

  //score other mogoal
  Move(drivePID, turnPID, -1500, 0.8);        //facing backwards
  openBackClamp();

  //move back
  Move(drivePID, turnPID, 1500, 0.8);



  //turn towards awp
  turnToRotation(turnPID, -90, 1); //face backwards

  //put arms down
  frontArmDown();
  backArmDown();

  //grab awp
  Move(drivePID, -3000, 1);
  MoveUntilClamp(-30, 5000);
  closeBackClamp();

  //move forward and turn towards neutral mogoal
  Move(drivePID, 1000, 1);
  turnToRotation(turnPID, 180, 1); //facing forward now

  //clone above code
}










//MATCH AUTONS:
void rightSideOne() 
{
  //make sure to reset arm positions just in case
  FrontLift.setPosition(0, degrees);
  BackLift.setPosition(0, degrees);

  //side match auton for right neutral goal and awp line goal
  thread t(resetTilter);
  openFrontClamp();

  //give the clamp time to open fully so it doesn't disrupt the lidar sensor
  Move(1000, 100, false);

  //run out to grab the right mogoal
  MoveUntilClamp(clampPID, turnPID, 1, 2000);
  closeFrontClamp();
  wait(0.1, seconds);
  
  //lift lift slightly
  smallFrontArmLift();

  //move back to the line to reposition
  MoveUntilLine(turnPID, -50);

  //move to desired location for grabbing awp line mogoal
  Move(drivePID, -550, 1);
  hardDriveStop();

  //turn towards the AWP mogoal
  turnWithPID(turnPID, -79, 1);

  //back up and grab awp line goal
  MoveUntilClamp(-30, 800);
  closeBackClamp();
  wait(0.8, seconds);
  tilterUp();

  turnWithPID(turnPID, -20, 1);

  //move back and dispense preloads
  Move(drivePID, 1000, 1);

  FrontLift.spinToPosition(200, degrees);

  setIntake(true);
}

void leftSideOne() 
{
  //left side match auton for left and center mogoals
  thread t(resetTilter);
  openFrontClamp();

  //give the clamp time to open fully so it doesn't disrupt the lidar sensor
  Move(1500, 100, false);

  //move forward and grab the left mogoal
  MoveUntilClamp(clampPID, turnPID, 1, 3500);
  closeFrontClamp();
  wait(0.7, seconds);

  //lift lift slightly
  smallFrontArmLift();

  //move back to white line and turn to next goal
  MoveUntilLine(turnPID, -50);
  turnWithPID(turnPID, -120, 1.2); //originally -120

  openFrontClamp(); //let go of the first mogoal the bot grabbed

  //PID& pid, PID& turnPID, int amount, double speed, double turningSpeed, int finishedAngle
  //MoveAndTurn(drivePID, turnPID, -1000, 0.5, 1, 90);

  //move towards and grab the next mogoal
  //MoveUntilClamp(clampPID, turnPID, -1, 5000);
  Move(drivePID, turnPID, -2300, 0.8);
  MoveUntilClamp(clampPID, turnPID, -0.8, 1000);
  closeBackClamp();
  wait(0.8, seconds);

  //tilt the tilter and dispense preloads
  tilterUp();
  setIntake(true);

  //reset rotation
  turnToRotation(turnPID, 0, 1);

  //move back to correct side of field and put down mogoal
  MoveUntilLine(turnPID, -50);
  tilterDown();
  openBackClamp();
}

void leftSideTwo()
{
  //left side match auton for just getting the left mogoal
  thread t(resetTilter);
  openFrontClamp();

  //give the clamp time to open fully so it doesn't disrupt the lidar sensor
  Move(1500, 100, false);

  //move forward and grab the left mogoal
  MoveUntilClamp(clampPID, turnPID, 1, 3500);
  closeFrontClamp();
  wait(0.15, seconds);

  //lift lift slightly
  smallFrontArmLift();

  //move back to white line and turn to next goal
  MoveUntilLine(turnPID, -50);
  Move(drivePID, -800, 1);
  turnWithPID(turnPID, -120, 1.2); //originally -120

  openFrontClamp(); //let go of the first mogoal the bot grabbed
  wait(0.5, seconds);

  //turn aound and dispense preloads into the mogoal
  Move(drivePID, turnPID, -1000, 1);
  turnWithPID(turnPID, 180, 1);
  Move(drivePID, turnPID, -1000, 1);
  frontArmUp(); //moving the arm out of the way for the intake to move
  setIntake(true);
}

void rightSideTwo() 
{
  thread t(resetTilter);
  openFrontClamp();

  //run to grab the center mogoal
  MoveAndTurn(drivePID, turnPID, 2000, 100, 0.3, -41);
  MoveUntilClamp(clampPID, turnPID, 1, 5000); //clamp the mogoal
  wait(0.2, seconds);

  //move back
  MoveUntilLine(-100);
  LineUpOnLine(-100);
}





//for testing new stuff only-------------------------------------------------------------------------------------
void testing() 
{
  //speed and max move
  openFrontClamp();
  Move(1000, 100, false);

  MoveUntilClamp(clampPID, turnPID, 1, 5000);

  wait(0.15, seconds);
  Move(-2000, 100, false);
  MoveUntilLine(-100);
}
//just for testing new stuff-------------------------------------------------------------------------------------




/*
  ACTUAL AUTON METHOD CALLS BELOW

  *Uses the auton selector class to determine which program to run
*/
void autonomous(void) {
  autonRunning = true; //DO NOT REMOVE (tells the haptic feedback on the controller to stop)
  autonRan = true;

  int selectedAuton = selector.getSelected();

  if(selectedAuton == 0)
    rightSideOne(); //right side match auton for right neutral goal and awp line goal
  else if(selectedAuton == 1)
    leftSideOne(); //left side match auton for left and center mogoals
  else if(selectedAuton == 2)
    leftSideTwo(); //left side match auton for just getting the left mogoal
  else if(selectedAuton == 3)
    testing(); //for testing new stuff only
  else if(selectedAuton == 4)
    rightSideTwo(); //grabbing the center mogoal ONLY. starts on the right side
  else if(selectedAuton == 5)
    SkillsAutonMain(); //main skills auton
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

void motorTempsToController()
{
  while(true) {
    //print average motor temps to the controller screen once the auton is done running (auton selection menu will be erased from the screen)
    if(autonRan) {
      double rightFrontTemp = RightFront.temperature(percent);
      double rightBackTemp = RightFront.temperature(percent);
      double leftFrontTemp = LeftFront.temperature(percent);
      double leftBackTemp = LeftBack.temperature(percent);

      double average = (rightFrontTemp + rightBackTemp + leftFrontTemp + leftBackTemp) / 4;
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("Average motor temp:");
      Controller1.Screen.setCursor(2, 12);
      Controller1.Screen.print("%d", (int)average);

      wait(20, msec);

      Controller1.Screen.clearScreen();
    }
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
      *ButtonX: outtake
      *ButtonY: toggle front clamp
      *ButtonA: Modifier for slowing down the drive
      *ButtonRight: togggle back clamp
  */

  //making sure all of the motors aren't set to brake stopping mode from auton 
  RightBack.setStopping(coast);
  RightFront.setStopping(coast);
  LeftBack.setStopping(coast);
  LeftFront.setStopping(coast);

  autonRunning = false;
  thread d(hapticFeedBack);
  thread f(motorTempsToController);

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

/*
    if(turnAmount > maxTurningSpeed) {
      turnAmount = maxTurningSpeed;
    }
*/

    //apply modifier for slow driving (X button)
    rightAmount = (Controller1.Axis3.position(percent) * (Controller1.ButtonA.pressing() ? .35 : 1) ) - turnAmount;
    leftAmount = (Controller1.Axis3.position(percent) * (Controller1.ButtonA.pressing() ? .35 : 1) ) + turnAmount;

    //set drive motors to hold when slow mode is enabled
    if(Controller1.ButtonA.pressing()) {
      RightFront.setStopping(hold);
      RightBack.setStopping(hold);
      LeftFront.setStopping(hold);
      LeftBack.setStopping(hold);
    }
    else { //default stopping mode is coast, makes for better driving feel
      RightFront.setStopping(coast);
      RightBack.setStopping(coast);
      LeftFront.setStopping(coast);
      LeftBack.setStopping(coast);
    }

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
    else if(Controller1.ButtonX.pressing()) {
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