using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor ArmClaw;
extern motor_group Arm;
extern motor BackClaw;
extern rotation RightWheelEncoder;
extern rotation LeftWheelEncoder;
extern inertial InertialSensor;
extern motor FLW;
extern motor BLW;
extern motor FRW;
extern motor BRW;
extern rotation BackClawEncoder;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );