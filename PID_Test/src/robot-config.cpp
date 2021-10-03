#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LeftFront = motor(PORT16, ratio18_1, true);
motor LeftBack = motor(PORT12, ratio18_1, true);
motor RightFront = motor(PORT10, ratio18_1, false);
motor RightBack = motor(PORT20, ratio18_1, false);
encoder Sensor = encoder(Brain.ThreeWirePort.A);
controller Controller1 = controller(primary);
inertial Rotation = inertial(PORT18);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}