/*
  Author: Luke Crimi
  File: AutonSelector.h
  Date Created: 1/30/2022
  
  Use this class to easily select autons in the preauton function
*/

#include <vector>
#include "vex.h"

using namespace vex;

class AutonSelector 
{
public:
  AutonSelector();

  void display_autons();
  void iterate(); //called when a user presses a button to change the auton selection by 1
  void add(std::string name, std::string description, std::string description2);

  int getSelected();

private:
  int selected;
  int numberOfAutons;

  std::vector<std::string> autons;
  std::vector<std::vector<std::string>> descriptions;
};