/*
  *Written by Luke Crimi
  *BetterAuton.h
  *Date of creation: 11/21/2021
*/

#include "PID.h"
#include "vex.h"

class BetterAuton {
  public:
    BetterAuton();
    BetterAuton(int offSetX, int offSetY, int rotationOffSet);

    void MoveTo(double& leftOutput, double& rightOutput, int x, int y, double speed);
    void MoveTo(double& leftOutput, double& rightOutput, PID& left, PID& right, int x, int y, double speed);
    void MoveTo(double& leftOutput, double& rightOutput, PID& left, PID& right, int x, int y, double speed, double rotationSpeed);

    void Turn(int rotationPosition);
  private:
    void rotatePoint(int& x, int &y);

    //current position of the robot
    int _x;
    int _y;
    int _rotation;

    //last time the robot turned
    int _lastXTurn;
    int _lastYTurn;
};