/*
  *Written by Luke Crimi
  *BetterAuton.cpp
  *Date of creation: 11/21/2021
*/

#include "BetterAuton.h"
#include "PID.h"
#include "vex.h"

BetterAuton::BetterAuton() 
{
  //reset robot position
  BetterAuton::_x = 0;
  BetterAuton::_y = 0;
  BetterAuton::_rotation = 0;

  BetterAuton::_lastXTurn = 0;
  BetterAuton::_lastYTurn = 0;
  BetterAuton::_lastRotation = 0;

  BetterAuton::reachedLocation = false;
}

BetterAuton::BetterAuton(int offSetX, int offSetY, int rotationOffSet) 
{
  //set robot position to offsets
  BetterAuton::_x = offSetX;
  BetterAuton::_y = offSetY;
  BetterAuton::_rotation = rotationOffSet;

  BetterAuton::_lastXTurn = offSetX;
  BetterAuton::_lastYTurn = offSetY;

  BetterAuton::reachedLocation = false;
}

void BetterAuton::rotatePoint(int rotateX, int rotateY, double amount, int& x, int& y) 
{
  double degrees = ((amount) * (3.145926/180));

  int tempX = x;
  int tempY = y;

  x = round(cos(degrees) * (tempX - rotateX) - sin(degrees) * (tempY - rotateY) + rotateX);
  y = round(sin(degrees) * (tempX - rotateX) + cos(degrees) * (tempY - rotateY) + rotateY);
}

void BetterAuton::MoveTo(double& leftOutput, double& rightOutput, int x, int y, double speed)
{
  //the robot has not reached the requested location
  BetterAuton::reachedLocation = false;

  int xPos = BetterAuton::_x;
  int yPos = BetterAuton::_y;
  int rot = BetterAuton::_rotation;

  //calculate next point to move to 
  BetterAuton::rotatePoint(BetterAuton::_lastXTurn, BetterAuton::_lastYTurn, BetterAuton::_rotation, x, y);

  if(x > xPos) {

  } 
  else if(x < xPos) {

  }
}

void BetterAuton::Turn(int rotationPosition) 
{
  BetterAuton::_rotation += rotationPosition;
  BetterAuton::_lastXTurn = BetterAuton::_x;
  BetterAuton::_lastYTurn = BetterAuton::_y;

}