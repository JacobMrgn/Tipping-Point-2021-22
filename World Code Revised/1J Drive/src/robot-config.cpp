#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor ArmClaw = motor(PORT20, ratio36_1, true);
motor ArmMotorA = motor(PORT9, ratio36_1, true);
motor ArmMotorB = motor(PORT10, ratio36_1, false);
motor_group Arm = motor_group(ArmMotorA, ArmMotorB);
motor BackClaw = motor(PORT11, ratio36_1, true);
rotation RightWheelEncoder = rotation(PORT19, false);
rotation LeftWheelEncoder = rotation(PORT18, false);
inertial InertialSensor = inertial(PORT16);
motor FLW = motor(PORT3, ratio18_1, true);
motor BLW = motor(PORT2, ratio18_1, true);
motor FRW = motor(PORT7, ratio18_1, false);
motor BRW = motor(PORT4, ratio18_1, false);
rotation BackClawEncoder = rotation(PORT17, false);
limit BackClawLimit = limit(Brain.ThreeWirePort.A);

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