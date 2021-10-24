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
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "PID.h"
#include <vector>
#include <stdio.h> // null
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

using namespace vex;

int randomInt(int min, int max) { 
  return ((rand() % (max - min)) + min);
}

int dir = 0;

void control(){
  while(true) {
    //getting user input through the controller
    if(Controller1.ButtonUp.pressing() && dir != 1) {
      dir = 3;
    }
    if(Controller1.ButtonRight.pressing() && dir != 2) {
      dir = 0;
    }
    if(Controller1.ButtonDown.pressing() && dir != 3) {
      dir = 1;
    }
    if(Controller1.ButtonLeft.pressing() && dir != 0) {
      dir = 2;
    }

    wait(1, msec);
  }
}

void snake()
{
  //initialize the random library with a random seed
  srand(time(NULL));
  thread con(control);

  int score = 0;

  std::vector<int> xlocations;
  std::vector<int> ylocations;

  int xLimit = 480;
  int yLimit = 240;
  int width = 10;
  int height = 10;

  //starting values of the snake
  int x = 20;
  int y = 20;
  int snakeLength = 5;

  //starting values of the apple
  int appleX = 100;
  int appleY = 20;

  bool alive = true;

  while(alive)
  {
    //text information
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Score: %d", score);

    //moving the snake
    if(dir == 0) {
      //right
      x += width;
    }
    else if(dir == 1) {
      //down
      y += height;
    }
    else if(dir == 2) {
      //left
      x -= width;
    }
    else {
      //up
      y -= height;
    }

    xlocations.push_back(x);
    ylocations.push_back(y);

    //erase the tail of the snake
    if(xlocations.size() > snakeLength){
      xlocations.erase(xlocations.begin());
      ylocations.erase(ylocations.begin());
    }

    //drawing the snake
    for(int i = 0; i < xlocations.size(); i++)
    {
      Brain.Screen.setFillColor(green);
      Brain.Screen.drawRectangle(xlocations[i], ylocations[i], width, height);
    }

    //draw the apple
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawRectangle(appleX, appleY, width, height);

    //check if the snake's head is in bounds
    if(x > xLimit - width)
      alive = false;
    if(x < 0)
      alive = false;

    if(y > yLimit - height)
      alive = false;
    if(y < 0)
      alive = false;

    //check if the player is still alive or not
    for(int i = 0; i < xlocations.size() -1; i++) {
      if(xlocations[i] == x && ylocations[i] == y) {
        alive = false;
      }
    }

    //check if the player has eaten an apple or not
    if(appleX == x && appleY == y) { 
      //player has eaten an apple, set a new x y position for the apple that isn't inside of the snake's body and make the snake longer
      appleX = randomInt(1, xLimit);
      appleY = randomInt(1, yLimit);

      //snap location to the grid
      appleX = (int)floor(appleX / width) * width;
      appleY = (int)floor(appleY / height) * height;

      //make the snake longer
      snakeLength++;

      //increase the score
      score += 100;
    }

    //wait 0.4 seconds before refreshing the screen
    wait(0.1, seconds);
    Brain.Screen.clearScreen();
  }

  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("You died!");
}

void clamp(double& variable, double bottom, double top)
{
  if(variable < bottom) {
    variable = bottom;
  }
  if(variable > top) {
    variable = top;
  }
}

//helper function to move to a certain position using a right and left pid
void moveTo(PID leftPID, PID rightPID, int location)
{
  //reset rotation
  Rotation.setRotation(0, degrees);

  do {
    double rightSpeed = rightPID.calculate(RightFront.position(degrees), location);
    double leftSpeed = leftPID.calculate(LeftFront.position(degrees), location);

    //clamp values and factor in rotation error
    clamp(rightSpeed, -100, 100);
    clamp(leftSpeed, -100, 100);
    rightSpeed += Rotation.rotation(degrees);
    leftSpeed -= Rotation.rotation(degrees);

    //set velocities and spin
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
  thread t(snake);

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
  //moveTo(leftPid, rightPid, 2000);

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
