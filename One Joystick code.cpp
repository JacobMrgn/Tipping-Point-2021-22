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
  while (true) {
    // ........................................................................
    //Driver Variables
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
    //Drive Control - One Joystick
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
    //Battery Control
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("Battery: %d",Brain.Battery.capacity()); 
    // ........................................................................
    wait(25, msec);
    Controller1.Screen.clearScreen();
  }
}
