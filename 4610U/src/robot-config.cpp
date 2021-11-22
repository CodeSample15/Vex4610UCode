#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor RightFront = motor(PORT9, ratio18_1, false);
motor RightBack = motor(PORT21, ratio18_1, false);
motor LeftFront = motor(PORT15, ratio18_1, true);
motor LeftBack = motor(PORT11, ratio18_1, true);
motor Lift = motor(PORT13, ratio36_1, false);
motor Conveyor = motor(PORT12, ratio18_1, false);
rotation RotationSensor = rotation(PORT14, false);
inertial InertialSensor = inertial(PORT10);
motor Tilter = motor(PORT17, ratio36_1, false);
digital_out ClamperL = digital_out(Brain.ThreeWirePort.G);
digital_out ClamperR = digital_out(Brain.ThreeWirePort.F);
bumper TilterButton = bumper(Brain.ThreeWirePort.E);
bumper ArmBumper = bumper(Brain.ThreeWirePort.H);
motor Lift2 = motor(PORT18, ratio36_1, true);

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