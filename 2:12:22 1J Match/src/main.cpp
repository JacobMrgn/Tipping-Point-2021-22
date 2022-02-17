///////////////////////////////////////////////////////////////////////////////
#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// ArmClaw              motor         20              
// Arm                  motor_group   9, 10           
// BackClaw             motor         11              
// RightWheelEncoder    rotation      19              
// LeftWheelEncoder     rotation      18              
// InertialSensor       inertial      16              
// FLW                  motor         3               
// BLW                  motor         2               
// FRW                  motor         7               
// BRW                  motor         4               
// BackClawEncoder      rotation      17              
// BackClawLimit        limit         A               
// ---- END VEXCODE CONFIGURED DEVICES ----
          
using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*                             Global Variables                              */
/*---------------------------------------------------------------------------*/
//double ArmLimit;
double inchesforward;
double rightMotorPower = 0.0;
double leftMotorPower = 0.0;
bool BackClawSet = true;
/*---------------------------------------------------------------------------*/
/*                                  PD Loop                                  */
/*---------------------------------------------------------------------------*/
// PD Loop
  //Tuning Variables
    double kP = 0.01;
    double kD = 0.04;
    double turnkP = 0.09;
    double turnkD = 0.2;

  //PD Variables
    int desiredValue = 360 * inchesforward/12.5663;
    int desiredTurnValue = 0;

    int error; 
    int prevError = 0;
    int derivative;
    int totalError = 0; 

    int turnError; 
    int turnPrevError = 0; 
    int turnDerivative; 
    int turnTotalError = 0; 

    bool resetDriveSensors = false;
    bool enableDrivePID = true;

  int drivePID(){
    while(enableDrivePID){
      if (resetDriveSensors) {
        resetDriveSensors = false;
        LeftWheelEncoder.setPosition(0,degrees);
        RightWheelEncoder.setPosition(0,degrees);
        InertialSensor.setHeading(0, degrees);
      }

      double turnDifference = InertialSensor.orientation(vex::yaw, degrees);
      int leftWheelPosition = LeftWheelEncoder.position(degrees);
      int rightWheelPosition = RightWheelEncoder.position(degrees);
      int averagePosition = (leftWheelPosition + rightWheelPosition)/2;



      //Lateral PD
        //P
        error = desiredValue - averagePosition;
        //D
        derivative = error - prevError;

        double lateralMotorPower = error * kP + derivative * kD;
 
      //Turning PD
        //P 
        turnError = desiredTurnValue - turnDifference;
        //D
        turnDerivative = turnError - turnPrevError;

        double turnMotorPower = turnError * turnkP + turnDerivative * turnkD;

      FLW.spin(forward, lateralMotorPower + turnMotorPower + leftMotorPower, voltageUnits::volt);
      BLW.spin(forward, lateralMotorPower + turnMotorPower + leftMotorPower, voltageUnits::volt);
      FRW.spin(forward, lateralMotorPower - turnMotorPower + rightMotorPower, voltageUnits::volt);
      BRW.spin(forward, lateralMotorPower - turnMotorPower + rightMotorPower, voltageUnits::volt);
      

      prevError = error;
      turnPrevError = turnError;
      vex::task::sleep(20);
    }
    return 1;
  }
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/
  void pre_auton(void) { vexcodeInit(); }
/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/
void autonomous(void) {
  // ..........................................................................
  vex::task hopecraphappens(drivePID);
  enableDrivePID = true;

  BackClawEncoder.setPosition(0.01, turns);
  int urmom = 100;

  FLW.setVelocity(urmom, percent);
  FRW.setVelocity(urmom, percent);
  BLW.setVelocity(urmom, percent);
  BRW.setVelocity(urmom, percent);

  BackClaw.setPosition(0, degrees);

  //Forward until it hits the first tower and grabs it   NOTE:: Back claw moves to downward position
  resetDriveSensors = true;
  desiredTurnValue = 0;
  desiredValue = 1574;
  wait(1000, msec);
  waitUntil(FRW.velocity(rpm) <= 10);

  ArmClaw.spin(forward, 6, voltageUnits::volt);
  wait(250, msec);

  //While holding tower lifts it and backs away
  resetDriveSensors = true;
  desiredValue = -1200;
  Arm.spin(forward, 6, voltageUnits::volt);
  wait(350, msec);
  Arm.stop(hold);
  wait(500, msec);
  waitUntil(FRW.velocity(rpm) <= 0);

  resetDriveSensors = true;
  desiredTurnValue = -100;
  wait(1000, msec);
  waitUntil(FRW.velocity(rpm) <= 5);

  resetDriveSensors = true;
  desiredTurnValue = 0;
  desiredValue = 300;
  waitUntil(FRW.velocity(rpm) <= 5);
  
  ArmClaw.spin(reverse, 6, voltageUnits::volt);
  wait(250, msec);
  ArmClaw.stop(brake);


/*
  wait(100, msec);

  //Turns to have back face tall neutural goal and drops goal its holding
  resetDriveSensors = true;
  desiredTurnValue = 150;
  wait(1000, msec);
  waitUntil(FRW.velocity(rpm) <= 5);

  ArmClaw.spin(reverse, 6, voltageUnits::volt);
  wait(250, msec);
  ArmClaw.stop(brake);

  //Goes back to line up with tower and grabs it
  resetDriveSensors = true;
  desiredValue = -1350;
  wait(1000, msec);
  waitUntil(FRW.velocity(rpm) <= 5);

  resetDriveSensors = true;
  desiredValue = 0;
  BackClawSet = false;

  wait(1000, msec);

  BackClawSet = false;

  BackClaw.spin(reverse, 6, voltageUnits::volt);
  waitUntil(BackClawLimit.pressing() == true);
  BackClaw.stop(hold);

  //Moves forward and turns scoring the tall neutral goal 
  leftMotorPower = -9;

  resetDriveSensors = true;
  desiredValue = 2300;
  wait(1000, msec);
  waitUntil(FRW.velocity(rpm) <= 10);

  FRW.stop(brake);
  FLW.stop(brake);
  BRW.stop(brake);
  BLW.stop(brake);

  Arm.spin(forward, 6, voltageUnits::volt);
  wait(1000, msec);
  Arm.stop(coast);
  wait(100, msec);
  Arm.stop(brake);

  BackClaw.spin(forward, 6, voltageUnits::volt);
  waitUntil(BackClawEncoder.position(turns) <= 1.2);
  BackClaw.stop();

  Arm.spin(reverse, 6, voltageUnits::volt);
  wait(1000, msec);
  Arm.stop(coast);
  wait(100, msec);
  Arm.stop(brake);

  leftMotorPower = 0.0;

  wait(500, msec);

  resetDriveSensors = true;
  desiredTurnValue = 95;
  wait(1000, msec);
  waitUntil(FRW.velocity(rpm) <= 5);

  resetDriveSensors = true;
  desiredValue = -500;
  wait(1000, msec);
  waitUntil(FRW.velocity(rpm) <= 10);
*/
  vex::task::sleep(1000);
  // ..........................................................................
}
/*---------------------------------------------------------------------------*/ 
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/
void usercontrol(void) {
  // Set Up
    enableDrivePID = false;

    double ClawSpeed = 6; //Volts
    double ArmSpeed = 9.6; //Volts

    FLW.setVelocity(100, percent);
    FRW.setVelocity(100, percent);
    BLW.setVelocity(100, percent);
    BRW.setVelocity(100, percent);

    Arm.setVelocity(100, percent);
    ArmClaw.setVelocity(100, percent);
    BackClaw.setVelocity(100, percent);

    Brain.Screen.setFillColor(red);
    Brain.Screen.setPenColor(red);

    Controller1.Screen.clearScreen();
    Brain.Screen.clearScreen();

    double TemperatureLimit = 55; //Degrees C
  while (true) {
    // ........................................................................
    //Stuff you don't wants running during driver
      enableDrivePID = false;
      resetDriveSensors = false;
      BackClawSet = false;
    //Driver Variables

      double straight = Controller1.Axis2.position(vex::percent) * 0.12;
      double yaw = (Controller1.Axis1.position(vex::percent)* 0.75) * 0.12;
    //Temperature Variables
      double ArmTemp = Arm.temperature(temperatureUnits::celsius);
      double FLWTemp = FLW.temperature(temperatureUnits::celsius);
      double FRWTemp = FRW.temperature(temperatureUnits::celsius);
      double BLWTemp = BLW.temperature(temperatureUnits::celsius);
      double BRWTemp = BRW.temperature(temperatureUnits::celsius);
      double ArmClawTemp = ArmClaw.temperature(temperatureUnits::celsius);
      double BackClawTemp = BackClaw.temperature(temperatureUnits::celsius);
    //Drive Control
      //One Joystick
      FRW.spin(vex::forward, straight - yaw, voltageUnits::volt);
      BRW.spin(vex::forward, straight - yaw, voltageUnits::volt);
      FLW.spin(vex::forward, straight + yaw, voltageUnits::volt);
      BLW.spin(vex::forward, straight + yaw, voltageUnits::volt);

      if (straight == 0 & yaw == 0) {

        FRW.stop(brake);
        BRW.stop(brake);
        FLW.stop(brake);
        BLW.stop(brake);
      }
    //Claw Control
      if (Controller1.ButtonL2.pressing()) {

        ArmClaw.spin(vex::forward, ClawSpeed, voltageUnits::volt);
      } 
      else if (Controller1.ButtonL1.pressing()) {

        ArmClaw.spin(vex::reverse, ClawSpeed, voltageUnits::volt);
      } 
      else {

        ArmClaw.stop(brake);
      }

    //Arm Control
      //&& ArmPosition <= ArmLimit
      if (Controller1.ButtonR1.pressing()) {

        Arm.spin(vex::forward, ArmSpeed, voltageUnits::volt);
      } 
      //&& ArmPosition <= ArmLimit
      else if (Controller1.ButtonR2.pressing()) {

        Arm.spin(vex::reverse, ArmSpeed, voltageUnits::volt);
      } 
      else {

        Arm.stop(hold);
      }

      /*if (ArmPosition > ArmLimit){

        Arm.spin(forward, ArmLimit - ArmPosition, percent);
      }*/

    //Back Claw Control
      if (Controller1.ButtonUp.pressing()){

        BackClaw.spin(vex::forward, 9, voltageUnits::volt);
      }
      else if (Controller1.ButtonLeft.pressing() && BackClawLimit.pressing() == false){

        BackClaw.spin(vex::reverse, 9, voltageUnits::volt);
      }
      else {

        BackClaw.stop(hold);
      }

    //Temperature Control 
      if (BackClawTemp > TemperatureLimit || ArmClawTemp > TemperatureLimit || ArmTemp > TemperatureLimit || FLWTemp > TemperatureLimit || FRWTemp > TemperatureLimit || BLWTemp > TemperatureLimit || BRWTemp > TemperatureLimit){

        Controller1.Screen.setCursor(1,1);
        Controller1.Screen.print("Toasty robot");
        
        Brain.Screen.drawRectangle(0, 0, 480, 240); 

        Controller1.rumble(rumbleShort);

        wait(25, msec);
        Controller1.Screen.clearScreen();
      }
    
    // ........................................................................
    wait(25, msec);
  }
}
/*---------------------------------------------------------------------------*/
/*                                 VEX Stuff                                 */
/*---------------------------------------------------------------------------*/
int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) {
    wait(100, msec);
  }
}
//////////////////////////////////////////////////////////////////////////////