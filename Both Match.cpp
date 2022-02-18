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
// Vision15             vision        15              
// ---- END VEXCODE CONFIGURED DEVICES ----
          
using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*                             Global Variables                              */
/*---------------------------------------------------------------------------*/
//double ArmLimit;
float inchesforward;
float rightMotorPower = 0.0;
float leftMotorPower = 0.0;
bool arrowPositionUp;
bool selectionVari = true;
bool TwoJoystickDrive;
bool preMatch = true;
bool limitVari = true;
bool BackClawSet = true;
/*---------------------------------------------------------------------------*/
/*                                  PD Loop                                  */
/*---------------------------------------------------------------------------*/ 
// PD Loop
  //Tuning Variables
    float kP = 0.01;
    float kD = 0.04;
    float turnkP = 0.09;
    float turnkD = 0.2;

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

      if (BackClawEncoder.position(turns) <= 1.2 && BackClawSet == true){

        BackClaw.spin(reverse, 6, voltageUnits::volt);
      }
      if (BackClawEncoder.position(turns) > 1.2 && BackClawSet == true){

        BackClaw.stop(hold);
      }

      float turnDifference = InertialSensor.orientation(vex::yaw, degrees);
      int leftWheelPosition = LeftWheelEncoder.position(degrees);
      int rightWheelPosition = RightWheelEncoder.position(degrees);
      int averagePosition = (leftWheelPosition + rightWheelPosition)/2;



      //Lateral PD
        //P
        error = desiredValue - averagePosition;
        //D
        derivative = error - prevError;

        float lateralMotorPower = error * kP + derivative * kD;
 
      //Turning PD
        //P 
        turnError = desiredTurnValue - turnDifference;
        //D
        turnDerivative = turnError - turnPrevError;

        float turnMotorPower = turnError * turnkP + turnDerivative * turnkD;

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
  desiredValue = -800;
  Arm.spin(forward, 6, voltageUnits::volt);
  wait(350, msec);
  Arm.stop(hold);
  wait(500, msec);
  waitUntil(FRW.velocity(rpm) <= 10);

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
  desiredValue = 2000;
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

  vex::task::sleep(1000);
  // ..........................................................................
}
/*---------------------------------------------------------------------------*/ 
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/
//Driver Functions
  void arrowUp (void){

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("One");
    Controller1.Screen.print(" <--");
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print("Two");
    
    arrowPositionUp = true;
  }
  void arrowDown (void){

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("One");
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print("Two");
    Controller1.Screen.print(" <--");

    arrowPositionUp = false;
  }

  void selectionFunc (void){

    selectionVari = false;

    if (arrowPositionUp == true && limitVari == true){

      TwoJoystickDrive = false;
    }

    else if (arrowPositionUp == false && limitVari == true){

      TwoJoystickDrive = true;
    }

    else{

      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("Error: Could not");
      Controller1.Screen.setCursor(2,1);
      Controller1.Screen.print("complete task");
    }
  }

void usercontrol(void) {
  // Set Up
    enableDrivePID = false;

    float ClawSpeed = 6; //Volts
    float ArmSpeed = 9.6; //Volts

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

    float TemperatureLimit = 55; //Degrees C
  //Joystick Select
    
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("One");
    Controller1.Screen.print(" <--");
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print("Two");

    while (selectionVari == true){
    Controller1.ButtonUp.pressed(arrowUp);
    Controller1.ButtonDown.pressed(arrowDown);

    Controller1.ButtonA.pressed(selectionFunc);
    }

    Controller1.Screen.clearScreen();

  while (true) {
    // ........................................................................
    //Stuff you don't wants running during driver
      enableDrivePID = false;
      resetDriveSensors = false;
      selectionVari = false;
      limitVari = false;
      BackClawSet = false;
    //Driver Variables
      float right = Controller1.Axis2.position(vex::percent) * 0.12;
      float left = Controller1.Axis3.position(vex::percent) * 0.12;
      float straight = Controller1.Axis2.position(vex::percent) * 0.12;
      float yaw = (Controller1.Axis1.position(vex::percent)* 0.75) * 0.12;
    //Temperature Variables
      float ArmTemp = Arm.temperature(temperatureUnits::celsius);
      float FLWTemp = FLW.temperature(temperatureUnits::celsius);
      float FRWTemp = FRW.temperature(temperatureUnits::celsius);
      float BLWTemp = BLW.temperature(temperatureUnits::celsius);
      float BRWTemp = BRW.temperature(temperatureUnits::celsius);
      float ArmClawTemp = ArmClaw.temperature(temperatureUnits::celsius);
      float BackClawTemp = BackClaw.temperature(temperatureUnits::celsius);
    //Drive Control
      if (TwoJoystickDrive == true){
        //Two Joystick
        FRW.spin(vex::forward, right, voltageUnits::volt);
        BRW.spin(vex::forward, right, voltageUnits::volt);
        FLW.spin(vex::forward, left, voltageUnits::volt);
        BLW.spin(vex::forward, left, voltageUnits::volt);

        if (right == 0) {

          FRW.stop(brake);
          BRW.stop(brake);
        }
        if (left == 0) {

          FLW.stop(brake);
          BLW.stop(brake);
        }
      }

      if (TwoJoystickDrive == false){
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
    //Battery Control
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("Battery: %d",Brain.Battery.capacity()); 
    // ........................................................................
    wait(25, msec);
    Controller1.Screen.clearScreen();
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
