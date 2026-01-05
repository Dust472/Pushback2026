/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       freeb                                                     */
/*    Created:      12/7/2025, 9:25:11 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;
using namespace std;

// A global instance of competition
competition Competition;

/*
Motors and other physical parts of the bot
Left drivetrain ports: 1 (reversed), 2 (reversed), 3
Right drivetrain ports: 18, 19, 20 (reversed)
*/ 
motor leftSide1 = motor(PORT1); // This motor doesn't have a boolean because the default value of the boolean is false
motor leftSide2 = motor(PORT2);
motor leftSide3 = motor(PORT3, true); 

motor rightSide1 = motor(PORT18, ratio6_1);
motor rightSide2 = motor(PORT19, ratio6_1);
motor rightSide3 = motor(PORT20, ratio6_1, true);

// Put both left side motors and right side motors in their respective motorgroups
motor_group rightSideDT = motor_group(rightSide1, rightSide2, rightSide3);
motor_group leftSideDT = motor_group(leftSide1, leftSide2, leftSide3);

motor firstStage = motor(PORT8);
motor secondStage = motor(PORT9);
motor thirdStage = motor(PORT10, true);

// Optical, Distance, Rotation, Inertia Sensors
optical colorSensor = optical(PORT5);
rotation horizontalOdom = rotation(PORT6);
rotation verticalOdom = rotation(PORT13);
inertial inertialSensor = inertial(PORT7);
distance distanceSensor = distance(PORT11);

// Pneumatics
brain botBrain;
digital_out doubleParkSolenoid = digital_out(botBrain.ThreeWirePort.A);
digital_out wingSolenoid = digital_out(botBrain.ThreeWirePort.B);
digital_out scraperSolenoid = digital_out(botBrain.ThreeWirePort.C);

controller botController;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // inertialSensor.calibrate();
  // while(inertialSensor.isCalibrating()) {
  //   wait(10, msec);
  // }
}

void autonThreadOne() {

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  leftSideDT.setStopping(brake);
  rightSideDT.setStopping(brake);
}





/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void driverControlThreadOne() {
  while(10) {
    double movement = botController.Axis3.position() * (11 / 100);
    double rotation = botController.Axis1.position() * (11 / 100);

    rightSideDT.spin(forward, movement - rotation, volt);
    leftSideDT.spin(forward, movement + rotation, volt);
  }
}

void whenL1Pressed() {
  
}

void driverControlThreadThree() {
  while(10) {

  }
}

void driverControlThreadFour() {
  while(10) {

  }
}

void usercontrol(void) {
  botController.ButtonL1.pressed(whenL1Pressed);
  thread moveFunctionality = thread(driverControlThreadOne);
  thread colorSortingFunct = thread(driverControlThreadThree);
  thread doubleParkingFunct = thread(driverControlThreadFour);
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
