/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       freeb                                                     */
/*    Created:      12/7/2025, 9:25:11 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <vector>

#include "vex.h"


using namespace vex;

// A global instance of competition
competition Competition;

/*
Motors and other physical parts of the bot
Left drivetrain ports: 1 (reversed), 2 (reversed), 3
Right drivetrain ports: 18, 19, 20 (reversed)
*/ 
motor leftSide1 = motor(PORT1, ratio6_1, true); // This motor doesn't have a boolean because the default value of the boolean is false
motor leftSide2 = motor(PORT2, ratio6_1, true);
motor leftSide3 = motor(PORT3, ratio6_1, true); 

motor rightSide1 = motor(PORT18, ratio6_1);
motor rightSide2 = motor(PORT19, ratio6_1);
motor rightSide3 = motor(PORT20, ratio6_1);

// Put both left side motors and right side motors in their respective motorgroups
motor_group rightSideDT = motor_group(rightSide1, rightSide2, rightSide3);
motor_group leftSideDT = motor_group(leftSide1, leftSide2, leftSide3);

motor firstStage = motor(PORT8);
motor secondStage = motor(PORT9);
motor thirdStage = motor(PORT10, true);

// Optical, Distance, Rotation, Inertia Sensors
optical colorSensor = optical(PORT5);
rotation horizontalOdom = rotation(PORT6);
inertial inertialSensor = inertial(PORT7);
distance disSensor = distance(PORT11);

// Pneumatics
brain botBrain;
digital_out doubleParkSolenoid = digital_out(botBrain.ThreeWirePort.C);
bool doubleParkMechExtended = false;
digital_out wingSolenoid = digital_out(botBrain.ThreeWirePort.B);
bool wingMech = false;
digital_out scraperSolenoid = digital_out(botBrain.ThreeWirePort.A);
bool scraperMech = true;

controller botController;

class odom {
  private:
    std::vector<double> position;
    double oldRightAvg;
    double oldLeftAvg;
    double totalRightDist;
    double totalLeftDist;
  public:
    odom() {
      position[0] = 0;
      position[1] = 0;
      position[2] = 0;
      oldRightAvg = 0;
      oldLeftAvg = 0;
      totalRightDist = 0;
      totalLeftDist = 0;
    }
    
    // This updates the robot's orientation and position
    void updatePos() {
      while(10) {
        std::vector<double> rightSideEncoderPositions = {rightSide1.position(degrees), rightSide2.position(degrees), rightSide3.position(degrees)};
        std::vector<double> leftSideEncoderPositions = {leftSide1.position(degrees), leftSide2.position(degrees), leftSide3.position(degrees)};

        double rightSideAverage = 0;
        double leftSideAverage = 0;

        double trackWidth = 12.5;
        double rightSum = 0;
        double leftSum = 0;

        for(int i = 0; i < 3; i++) {
          rightSum += rightSideEncoderPositions[i];
          leftSum += leftSideEncoderPositions[i];
        }
        rightSideAverage = (rightSum / 3) * M_PI / 180 * 3.25; 
        leftSideAverage = (leftSum / 3) * M_PI / 180 * 3.25; 

        double horizontalTWDistance = horizontalOdom.position(degrees) * M_PI / 180 * 2;

        double changeInLeft = rightSideAverage - oldRightAvg;
        double changeInRight = leftSideAverage - oldLeftAvg;
        
        oldRightAvg = rightSideAverage;
        oldLeftAvg = leftSideAverage;

        totalRightDist += changeInRight;
        totalLeftDist += changeInLeft;

        double absoluteOrientation = position[2] + (totalRightDist + totalLeftDist) / trackWidth;
        double changeInTheta = absoluteOrientation - position[2];
        double averageOrientation = position[2] + (changeInTheta / 2);
        position[2] = absoluteOrientation;

        double localXOffset = 0;
        double localYOffset = 0;
        if(changeInTheta == 0) {
          localXOffset = horizontalTWDistance;
          localYOffset = changeInRight;
        } else {
          localXOffset = 2 * sin(changeInTheta / 2) * (horizontalTWDistance / changeInTheta + trackWidth / 2);
          localYOffset = 2 * sin(changeInTheta / 2) * (changeInRight / changeInTheta + trackWidth / 2);
        }

        

        double globalXOffset = 0;
        double globalYOffset = 0;
        double localRadius = sqrt(localXOffset * localXOffset + localYOffset * localYOffset);
        double localAngle = atan2(localYOffset, localXOffset) - averageOrientation;

        globalXOffset = localRadius * cos(localAngle);
        globalYOffset = localRadius * sin(localAngle);

        position[0] += globalXOffset;
        position[1] += globalYOffset;
      }
    }

    void moveToPose(double x, double y, double xErr, double yErr) {
      
    }

};
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

void move(int param, int speed){
  if(param == 1){
    leftSideDT.spin(forward, speed, volt);
    rightSideDT.spin(forward, speed, volt);
  } else if(param == -1){
    leftSideDT.spin(reverse, speed, volt);
    rightSideDT.spin(reverse, speed, volt);
  } else if(param = 2){
    leftSideDT.spin(reverse, speed, volt);
    rightSideDT.spin(forward, speed, volt);
  } else if(param = -2){
    leftSideDT.spin(forward, speed, volt);
    rightSideDT.spin(reverse, speed, volt);
  } else if(param == 0){
    leftSideDT.spin(forward, 0, volt);
    rightSideDT.spin(forward, 0, volt);
  }
}

void autoTake(int dir){
  if(dir == 1){
    firstStage.spin(reverse, 5.5, volt);
    secondStage.spin(reverse, 11, volt);
    thirdStage.spin(reverse, 5.5, volt);
  } else if(dir == 2){
    firstStage.spin(forward, 5.5, volt);
    secondStage.spin(forward, 11, volt);
    thirdStage.spin(reverse, 5.5, volt);
  } else if(dir == 3){
    wingSolenoid.set(false);
    wingMech = false;
    firstStage.spin(forward, 5.5, volt);
    secondStage.spin(forward, 11, volt);
    thirdStage.spin(forward, 5.5, volt);
  }
}


void autonomous(void) {
  // odom odomPod;
  // odom* odomThing = &odomPod;
  
  // move(1, 6);
  // wait(700, msec);
  // move(0,0);
  // wait(100,msec);
  // move(2,6);
  // wait(300,msec);
  // move(-2,6);
  // wait(300,msec);



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
  printf("thread one \n");
  double movement = 0;
  double rotation = 0;
  while(true) {
    movement = botController.Axis3.position();
    rotation = botController.Axis1.position();

    rightSideDT.spin(forward, movement - rotation, volt);
    leftSideDT.spin(forward, movement + rotation, volt);
    wait(10, msec);
  }
}

void unjamCheck() {
  while(true) {
    // Get the voltage of each motor using the formula
    int firstStageVoltage = (firstStage.velocity(rpm) * 2 * M_PI * firstStage.torque()) / firstStage.current();
    int secondStageVoltage = (secondStage.velocity(rpm) * 2 * M_PI * secondStage.torque()) / secondStage.current();
    int thirdStageVoltage = (thirdStage.velocity(rpm) * 2 * M_PI * thirdStage.torque()) / thirdStage.current();

    // Check to see if the first stage is jammed
    if(firstStageVoltage < (11.0 / 10)) {
      // stop all other motors to prevent infractions
      secondStage.spin(forward, 0, volt);
      thirdStage.spin(forward, 0, volt);

      // Stop the jammed motor in case it is already spinning
      firstStage.spin(forward, 0, volt);
      firstStage.spin(reverse, 5.5, volt);
      wait(1, seconds);
      firstStage.spin(forward, 0, volt);
    }

    // Check to see if the first stage is jammed
    if(secondStageVoltage < (11.0 / 10)) {
      // stop all other motors to prevent infractions
      firstStage.spin(forward, 0, volt);
      thirdStage.spin(forward, 0, volt);
      
      // Stop the jammed motor in case it is already spinning
      secondStage.spin(forward, 0, volt);
      secondStage.spin(reverse, 5.5, volt);
      wait(1, seconds);
      secondStage.spin(forward, 0, volt);
    }

    // Check to see if the first stage is jammed
    if(thirdStageVoltage < (11.0 / 10)) {
      // stop all other motors to prevent infractions
      secondStage.spin(forward, 0, volt);
      firstStage.spin(forward, 0, volt);

      // Stop the jammed motor in case it is already spinning
      thirdStage.spin(forward, 0, volt);
      thirdStage.spin(reverse, 5.5, volt);
      wait(1, seconds);
      thirdStage.spin(forward, 0, volt);
    }
    wait(10, msec);
  }
}

void whenL1Pressed() {
  printf("L1 pressed \n");
  if(!wingMech){
    wingSolenoid.set(true);
    wingMech = true; 
  }
  doubleParkSolenoid.set(false);
  doubleParkMechExtended = false;
  firstStage.spin(forward, 5.5, volt);
  secondStage.spin(forward, 11, volt);
  thirdStage.spin(forward, 0, volt);
}


void whenL1Released(){
  printf("L1 released \n");
  firstStage.spin(forward, 0, volt);
  secondStage.spin(forward, 0, volt);
  thirdStage.spin(forward, 0, volt);
}

void whenR1Pressed(){
  printf("R1 released \n");
  firstStage.spin(reverse, 5.5, volt);
  secondStage.spin(reverse, 11, volt);
  thirdStage.spin(forward, 5.5, volt);
  wait(150,msec);
  doubleParkSolenoid.set(true);
  doubleParkMechExtended = true;
  firstStage.spin(forward, 5.5, volt);
  secondStage.spin(forward, 11, volt);
  thirdStage.spin(reverse, 5.5, volt);
}

void whenR1Released(){
  printf("R1 released \n");
  firstStage.spin(forward, 0, volt);
  secondStage.spin(forward, 0, volt);
  thirdStage.spin(reverse, 0, volt);
}

void whenL2Pressed() {
  printf("L2 pressed \n");
  firstStage.spin(reverse, 5.5, volt);
  secondStage.spin(reverse, 11, volt);
  thirdStage.spin(reverse, 5.5, volt);
}

void whenL2Released(){
  printf("L2 released \n");
  firstStage.spin(reverse, 0, volt);
  secondStage.spin(reverse, 0, volt);
  thirdStage.spin(forward, 0, volt);
}

void whenR2Pressed() {
  printf("R2 pressed \n");
  firstStage.spin(reverse, 5.5, volt);
  secondStage.spin(reverse, 11, volt);
  thirdStage.spin(reverse, 5.5, volt);
  wait(150,msec);
  doubleParkSolenoid.set(true);
  doubleParkMechExtended = true;
  if(wingMech == true){
    wingSolenoid.set(false);
    wingMech = false;
  }
  firstStage.spin(forward, 5.5, volt);
  secondStage.spin(forward, 11, volt);
  thirdStage.spin(forward, 5.5, volt);
}

void whenR2Released() {
  printf("R2 released \n");
  firstStage.spin(forward, 0, volt);
  secondStage.spin(forward, 0, volt);
  thirdStage.spin(forward, 0, volt);
}

void whenDownPressed() {
  if(!scraperMech) {
    scraperSolenoid.set(true);
    scraperMech = true;
  } else {
    scraperSolenoid.set(false);
    scraperMech = false;
  }
}

void whenLeftPressed() {
  if(!doubleParkMechExtended) {
    doubleParkSolenoid.set(true);
    doubleParkMechExtended = true;
  } else {
    doubleParkSolenoid.set(false);
    doubleParkMechExtended = false;
  }
}

void whenRightPressed() {
  if(!wingMech) {
    wingSolenoid.set(true);
    wingMech = true;
  } else {
    wingSolenoid.set(false);
    wingMech = false;
  }
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
  thread moveFunctionality = thread(driverControlThreadOne);
  thread unjam = thread(unjamCheck);
  botController.ButtonL1.pressed(whenL1Pressed);
  botController.ButtonL1.released(whenL1Released);
  botController.ButtonR1.pressed(whenR1Pressed);
  botController.ButtonR1.released(whenR1Released);
  botController.ButtonL2.pressed(whenL2Pressed);
  botController.ButtonL2.released(whenL2Released);
  botController.ButtonR2.pressed(whenR2Pressed);
  botController.ButtonR2.released(whenR2Released);
  botController.ButtonY.pressed(whenDownPressed);
  botController.ButtonLeft.pressed(whenLeftPressed);
  botController.ButtonB.pressed(whenRightPressed);
  // thread colorSortingFunct = thread(driverControlThreadThree);
  // thread doubleParkingFunct = thread(driverControlThreadFour);
  while(Competition.isDriverControl()) {
    wait(10, msec);
  }
  moveFunctionality.interrupt();
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
