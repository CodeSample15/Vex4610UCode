/*
  Author: Luke Crimi
  File: AutonSelector.cpp
  Date Created: 1/30/2022
  
  Use this class to easily select autons in the preauton function
*/

#include <vector>
#include <cstring>

#include "AutonSelector.h"
#include "vex.h"

AutonSelector::AutonSelector() 
{
  AutonSelector::selected = 0;
  AutonSelector::numberOfAutons = 0;
}

AutonSelector::AutonSelector(int start)
{
  AutonSelector::selected = start;
  AutonSelector::numberOfAutons = 0;
}

AutonSelector::~AutonSelector() 
{
  //clear vectors
  AutonSelector::autons.clear();
  AutonSelector::descriptions.clear();
}

//return the selected auton
int AutonSelector::getSelected() 
{
  return AutonSelector::selected;
}

void AutonSelector::add(std::string name, std::string description, std::string description2)
{
  AutonSelector::numberOfAutons++;

  AutonSelector::autons.push_back(name);

  std::vector<std::string> stringVector;
  stringVector.push_back(description);
  stringVector.push_back(description2);
  AutonSelector::descriptions.push_back(stringVector);
}

//overloads if there isn't a long descriptions or no description at all:
void AutonSelector::add(std::string name, std::string description)
{
  AutonSelector::numberOfAutons++;

  AutonSelector::autons.push_back(name);

  std::vector<std::string> stringVector;
  stringVector.push_back(description);
  stringVector.push_back("");
  AutonSelector::descriptions.push_back(stringVector);
}

void AutonSelector::add(std::string name)
{
  AutonSelector::numberOfAutons++;

  AutonSelector::autons.push_back(name);

  std::vector<std::string> stringVector;
  stringVector.push_back("");
  stringVector.push_back("");
  AutonSelector::descriptions.push_back(stringVector);
}

//change the current selected auton and give haptic feedback to the driver
void AutonSelector::iterate() 
{
  AutonSelector::selected++;

  if(AutonSelector::selected >= AutonSelector::numberOfAutons)
    AutonSelector::selected = 0;

  Controller1.rumble(".");
}

//display the current auton on the remote screen
void AutonSelector::display_autons() 
{
  //get the lengths of each string
  int one = AutonSelector::autons[AutonSelector::selected].length();
  int two = AutonSelector::descriptions[AutonSelector::selected][0].length();
  int three = AutonSelector::descriptions[AutonSelector::selected][1].length();

  //creating the character arrays
  char oneArray[one + 1];
  char twoArray[two + 1];
  char threeArray[three + 1];

  //copying the strings to the char arrays
  std::strcpy(oneArray, AutonSelector::autons[AutonSelector::selected].c_str());
  std::strcpy(twoArray, AutonSelector::descriptions[AutonSelector::selected][0].c_str());
  std::strcpy(threeArray, AutonSelector::descriptions[AutonSelector::selected][1].c_str());

  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print(oneArray);
  Controller1.Screen.setCursor(2,1);
  Controller1.Screen.print(twoArray);
  Controller1.Screen.setCursor(3,1);
  Controller1.Screen.print(threeArray);

  wait(15, msec);

  Controller1.Screen.clearScreen();
}