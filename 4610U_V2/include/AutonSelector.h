/*
  Author: Luke Crimi
  File: AutonSelector.h
  Date Created: 1/30/2022
  
  Use this class to easily select autons in the preauton function
*/

#include <vector>
#include <string>
#include "vex.h"

using namespace vex;

class AutonSelector 
{
public:
  AutonSelector();
  AutonSelector(int start);
  ~AutonSelector();

  void display_autons();
  void iterate(); //called when a user presses a button to change the auton selection by 1
  void add(std::string name, std::string description, std::string description2);
  void add(std::string name, std::string description);
  void add(std::string name);

  int getSelected();

private:
  int selected;
  int numberOfAutons;

  std::vector<std::string> autons;
  std::vector<std::vector<std::string>> descriptions;
};