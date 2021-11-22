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
}

BetterAuton::BetterAuton(int offSetX, int offSetY, int rotationOffSet) 
{
  //set robot position to offsets
  BetterAuton::_x = offSetX;
  BetterAuton::_y = offSetY;
  BetterAuton::_rotation = rotationOffSet;

  BetterAuton::_lastXTurn = offSetX;
  BetterAuton::_lastYTurn = offSetY;
}

void BetterAuton::rotatePoint(int& x, int& y) 
{
  
}

void BetterAuton::MoveTo(double& leftOutput, double& rightOutput, int x, int y, double speed)
{
  int xPos = BetterAuton::_x;
  int yPos = BetterAuton::_y;
  int rot = BetterAuton::_rotation;
}

void BetterAuton::Turn(int rotationPosition) 
{
  BetterAuton::_rotation = BetterAuton::_rotation + rotationPosition;
  BetterAuton::_lastXTurn = BetterAuton::_x;
  BetterAuton::_lastYTurn = BetterAuton::_y;
}