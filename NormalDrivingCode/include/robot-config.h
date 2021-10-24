using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor LeftBack;
extern motor LeftFront;
extern motor Conveyor;
extern motor RightBack;
extern motor RightFront;
extern controller Controller1;
extern motor Lift;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );