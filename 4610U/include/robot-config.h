using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor RightFront;
extern motor RightBack;
extern motor LeftFront;
extern motor LeftBack;
extern motor Lift;
extern motor Conveyor;
extern led IndicatorLight;
extern rotation RotationSensor;
extern inertial InertialSensor;
extern motor Tilter;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );