/*
  Luke Crimi
  PID_Test
  9/19/2021
*/

#include "PID.h"
#include <cmath>

PID::PID()
{
  //no parameters, nothing gets initialized
}

//the robot will use the I term if the error is less than IMax, but greater than IMin
PID::PID(double Kp, double Ki, double Kd, double dt, double IMax, double IMin)
{
  PID::_Kp = Kp;
  PID::_Ki = Ki;
  PID::_Kd = Kd;
  PID::_dt = dt;
  PID::_IMin = IMin;
  PID::_IMax = IMax;
  
  PID::_integral = 0;
}

PID::PID(double Kp, double Ki, double Kd, double dt)
{
  PID::_Kp = Kp;
  PID::_Ki = Ki;
  PID::_Kd = Kd;
  PID::_dt = dt;
  PID::_IMin = 5;
  PID::_IMax = 20;
  
  PID::_integral = 0;
}

PID::PID(double Kp, double Ki, double Kd)
{
  PID::_Kp = Kp;
  PID::_Ki = Ki;
  PID::_Kd = Kd;
  PID::_dt = 20;
  PID::_IMin = 5;
  PID::_IMax = 20;
  
  PID::_integral = 0;
}

double PID::calculate(int currentPoint, int endPoint)
{
  PID::error = endPoint - currentPoint;

  //perform calculations

  //P
  double Pout = PID::error * PID::_Kp;

  //I
  if(std::abs(PID::error) > PID::_IMax || std::abs(PID::error) < PID::_IMin) {
    PID::_integral = 0;
  }
  else {
    PID::_integral += PID::error * PID::_dt;
  }
  double Iout = PID::_integral * PID::_Ki;

  //D
  double derivative = (PID::error - PID::_pre_error) / PID::_dt;
  double Dout = derivative * PID::_Kd;
  PID::_pre_error = PID::error;

  return Pout + Iout + Dout;
}