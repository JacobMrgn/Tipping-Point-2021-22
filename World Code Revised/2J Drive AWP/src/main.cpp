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
// ---- END VEXCODE CONFIGURED DEVICES ---- 
          
using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*                             Global Variables                              */
/*---------------------------------------------------------------------------*/
  float inchesforward;
  float rightMotorPower = 0.0;
  float leftMotorPower = 0.0;
  float BackClawlimit = 28.0;
  bool BackClawSet = true;
  bool BackClawGrabTower = false;
  float timerVar = 0.0;
  bool enableTimer = true;
  bool HeatOverride = false;
  bool LowBatteryRumble = true;
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
      /*
      //BackClaw Limit
        if ((0 - BackClawEncoder.position(degrees)) <= 75 && BackClawSet ==true){

          BackClaw.spin(forward, 6, voltageUnits::volt);
        }

        if ((0 - BackClawEncoder.position(degrees)) >= 75 && BackClawSet ==true) {

          BackClaw.stop(hold);
        }

      //BackClaw Limit 2
        if ((0 - BackClawEncoder.position(degrees)) >= BackClawlimit && BackClawGrabTower ==true){

          BackClaw.spin(reverse, 4, voltageUnits::volt);
        }

        if ((0 - BackClawEncoder.position(degrees)) <= BackClawlimit && BackClawGrabTower ==true){

          BackClaw.stop(hold);
        }
        */

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

//Timer
  int customTimer(){
    timerVar = 0.0;
    while(enableTimer){

      timerVar = timerVar + 0.25;
      wait(250, msec);
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
  vex::task timergo(customTimer);
  enableDrivePID = true;
  enableTimer = true;

  BackClawEncoder.resetPosition();

  Arm.spin(forward, 7.5, voltageUnits::volt);
  wait(1000, msec);
  Arm.spin(reverse, 7.5, voltageUnits::volt);
  wait(1000, msec);

  resetDriveSensors = true;
  desiredValue = 1000;
  desiredTurnValue = 170;
  wait(1000, msec);
  waitUntil(FRW.velocity(rpm) <= 0);

  wait(500, msec);

  resetDriveSensors = true;
  desiredValue = 4500;
  wait(1000, msec);
  waitUntil(FRW.velocity(rpm) <= 0);

  ArmClaw.spin(reverse, 6, voltageUnits::volt);
  wait(250, msec);
  ArmClaw.stop(hold);

  Arm.spin(forward, 6, voltageUnits::volt);
  wait(1500, msec);
  Arm.stop(hold);

  resetDriveSensors = true;
  desiredValue = -1500;
  wait(1000, msec);
  waitUntil(FRW.velocity(rpm) <= 0);

  Arm.stop(coast);
  wait(1000, msec); 

  ArmClaw.spin(forward, 6, voltageUnits::volt);
  wait(250, msec);
  ArmClaw.stop(coast);

  vex::task::sleep(1000);
  // ..........................................................................
}
/*---------------------------------------------------------------------------*/ 
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/
void usercontrol(void) {
  // Set Up
      //Stuff you don't wants running during driver
    enableDrivePID = false;
    resetDriveSensors = false;  

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

    BackClawEncoder.resetPosition(); 

  while (true) {                                             
    // ........................................................................
    //Driver Variables
      double right = Controller1.Axis2.position(vex::percent) * 0.12;
      double left = Controller1.Axis3.position(vex::percent) * 0.12;

      double BackClawPosition = (0 - BackClawEncoder.position(degrees));

    //Temperature Variables
      double ArmTemp = Arm.temperature(temperatureUnits::celsius);
      double FLWTemp = FLW.temperature(temperatureUnits::celsius);
      double FRWTemp = FRW.temperature(temperatureUnits::celsius);
      double BLWTemp = BLW.temperature(temperatureUnits::celsius);
      double BRWTemp = BRW.temperature(temperatureUnits::celsius);
      double ArmClawTemp = ArmClaw.temperature(temperatureUnits::celsius);
      double BackClawTemp = BackClaw.temperature(temperatureUnits::celsius);

      double tempList[] = {ArmTemp,FLWTemp,FRWTemp,BLWTemp,BRWTemp,ArmClawTemp,BackClawTemp};
    
    //Battery Variables
      double BatteryPercent = Brain.Battery.capacity();

    //Drive Control
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
      if (Controller1.ButtonR1.pressing()) {

        Arm.spin(vex::forward, ArmSpeed, voltageUnits::volt);
      } 
      else if (Controller1.ButtonR2.pressing()) {

        Arm.spin(vex::reverse, ArmSpeed, voltageUnits::volt);
      } 
      else {

        Arm.stop(hold);
      }

    //Back Claw Control
      if (Controller1.ButtonUp.pressing()){

        BackClaw.spin(vex::forward, 9, voltageUnits::volt);
      }
      else if (Controller1.ButtonLeft.pressing() && BackClawPosition >= BackClawlimit){

        BackClaw.spin(vex::reverse, 6, voltageUnits::volt);
      }
      else {

        BackClaw.stop(hold);
      }

    //Battery Control
      if (BatteryPercent >= 10 && HeatOverride == false){

        Controller1.Screen.setCursor(1,1);
        Controller1.Screen.print(BatteryPercent);
      }
      if (BatteryPercent <= 10 && HeatOverride == false){

        Controller1.Screen.setCursor(1,1);
        Controller1.Screen.print(BatteryPercent);
        Controller1.Screen.setCursor(2,1);
        Controller1.Screen.print("Battery Low");
        if (LowBatteryRumble == true){

          Controller1.rumble(rumbleLong);
          LowBatteryRumble = false;
        }
      }

    //Temperature Control 
      for (double item : tempList){

        if (item > TemperatureLimit){

          HeatOverride = true;
          Controller1.Screen.setCursor(1,1);
          Controller1.Screen.print("Toasty robot");
          
          Brain.Screen.drawRectangle(0, 0, 480, 240); 

          Controller1.rumble(rumbleShort);

          wait(25, msec);
          Controller1.Screen.clearScreen();
        }
      }
    
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