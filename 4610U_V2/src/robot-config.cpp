#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor RightFront = motor(PORT3, ratio6_1, false);
motor LeftFront = motor(PORT2, ratio6_1, true);
motor RightBack = motor(PORT4, ratio6_1, false);
motor LeftBack = motor(PORT5, ratio6_1, true);
motor FrontLift = motor(PORT6, ratio36_1, false);
motor BackLift = motor(PORT7, ratio36_1, true);
motor Tilter = motor(PORT8, ratio36_1, false);
digital_out BackClamp = digital_out(Brain.ThreeWirePort.E);
limit BackArmLimitSwitch = limit(Brain.ThreeWirePort.F);
digital_out FrontClamp = digital_out(Brain.ThreeWirePort.G);
inertial Inertial = inertial(PORT9);
distance DistanceSensor = distance(PORT10);
motor Conveyor = motor(PORT18, ratio18_1, false);
distance FrontDistanceSensor = distance(PORT17);

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