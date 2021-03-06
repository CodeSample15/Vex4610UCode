using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor RightFront;
extern motor LeftFront;
extern motor RightBack;
extern motor LeftBack;
extern motor FrontLift;
extern motor BackLift;
extern motor Tilter;
extern digital_out BackClamp;
extern limit BackArmLimitSwitch;
extern digital_out FrontClamp;
extern inertial Inertial;
extern distance DistanceSensor;
extern motor Conveyor;
extern distance FrontDistanceSensor;
extern line LeftLineTracker;
extern line RightLineTracker;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );