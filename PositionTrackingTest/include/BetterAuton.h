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

    //determining if the robot has reached the desired location
    bool reachedLocation;
    
  private:
    void rotatePoint(int rotateX, int rotateY, double amount, int& x, int &y);

    //current position of the robot
    int _x;
    int _y;
    int _rotation;

    //last time the robot turned
    int _lastXTurn;
    int _lastYTurn;
    double _lastRotation; //last rotation value the robot had in order to keep track of how much its turning
};