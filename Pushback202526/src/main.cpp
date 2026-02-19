/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       freeb                                                     */
/*    Created:      12/7/2025, 9:25:11 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include <algorithm>
#include "vex.h"


using namespace vex;

// A global instance of competition
competition Competition;

/*
Motors and other physical parts of the bot
Left drivetrain ports: 1 (reversed), 2 (reversed), 3
Right drivetrain ports: 18, 19, 20 (reversed)
*/ 
motor leftSide1 = motor(PORT1, ratio6_1, true);
motor leftSide2 = motor(PORT2, ratio6_1, true);
motor leftSide3 = motor(PORT3, ratio6_1, true); 

motor rightSide1 = motor(PORT18, ratio6_1);
motor rightSide2 = motor(PORT19, ratio6_1);
motor rightSide3 = motor(PORT20, ratio6_1);

// Put both left side motors and right side motors in their respective motorgroups
motor_group rightSideDT = motor_group(rightSide1, rightSide2, rightSide3);
motor_group leftSideDT = motor_group(leftSide1, leftSide2, leftSide3);
// L1 - Intake
// R1 - Intake
motor motorIntakeOne = motor(PORT8, ratio6_1, true);
motor motorIntakeTwo = motor(PORT9, ratio6_1);

rotation horizontalOdom = rotation(PORT17);
inertial inertialSensor = inertial(PORT4);

// Pneumatics
brain botBrain;
digital_out intakePiston1 = digital_out(botBrain.ThreeWirePort.C);
bool piston1E = false;
digital_out intakePiston2 = digital_out(botBrain.ThreeWirePort.B);
bool piston2E = false;
digital_out wingSolenoid = digital_out(botBrain.ThreeWirePort.B);
bool wingMech = false;
digital_out scraperSolenoid = digital_out(botBrain.ThreeWirePort.A);
bool scraperMech = false;

// Pneumatics
controller botController;
brain botBrain;
// Wing/store
// digital_out wingSolenoid = digital_out(botBrain.ThreeWirePort.B);
// bool wingMech = false;
// digital_out scraperSolenoid = digital_out(botBrain.ThreeWirePort.A);
// bool scraperMech = false;

class odometry {
  private:
    double position[3];

    double motorSpeed;
    double turnSpeed; 
    double desiredAngle;
    double updatingError;
    bool angled;

    double x;
    double y;
    double xError;
    double yError;
    double percent;
    double originalDistance;
    bool stop;
    bool positioned;
    bool curving;
    bool tracking;
    double integral;
    bool on;
    double maxSpeed;
    bool turnSwitch;

    double oldLeftEncoderPos;
    double oldRightEncoderPos;
    double oldOdomPos;

    double distanceTraveled;


    void moveDriveTrain(double throttle, double steering) {
      if(0 < throttle - steering) {
        rightSideDT.spin(forward, 11 * (throttle - steering), volt);
      } else {
        rightSideDT.spin(reverse, 11 * (throttle - steering) * -1, volt);
      }

      if(0 < throttle + steering) {
        leftSideDT.spin(forward, 11 * (throttle + steering), volt);
      } else {
        leftSideDT.spin(reverse, 11 * (throttle + steering) * -1, volt);
      }
    }
  public:
    odometry() {
      position[0] = 0.0;
      position[1] = 0.0;
      position[2] = 0.0;

      motorSpeed = 0;
      turnSpeed = 0;
      desiredAngle = 0;
      updatingError = 0;
      angled = true;

      x = 0;
      y = 0;
      xError = 0;
      yError = 0;
      percent = 1;
      originalDistance = 1;
      stop = true;
      positioned = true;
      curving = false;
      tracking = false;
      integral = 0;
      on = true;
      maxSpeed = 1;
      turnSwitch = true;

      oldLeftEncoderPos = 0;
      oldRightEncoderPos = 0;
      oldOdomPos = 0;

      distanceTraveled = 0;
    }

    void updatePosition() {
      /*
      We need to average out the positions in our motor encoders
      Since our drivetrain will most likely never change from three motors, we can just
      input the values of our motor encoders manually
      Each motor encoder can only give degrees, but we want radians, so we use pi/180
      */
      double leftSideSum = leftSide1.position(degrees) + leftSide2.position(degrees) + leftSide3.position(degrees);
      double leftSideAvg = leftSideSum / 3;

      double rightSideSum = rightSide1.position(degrees) + rightSide2.position(degrees) + rightSide3.position(degrees);
      double rightSideAvg = rightSideSum / 3;

      double leftSideDist = leftSideAvg - oldLeftEncoderPos;
      double rightSideDist = rightSideAvg - oldRightEncoderPos;
      oldLeftEncoderPos = leftSideAvg;
      oldRightEncoderPos = rightSideAvg;

      leftSideDist = leftSideDist/360 * 3.25 * M_PI * 0.75;
      rightSideDist = rightSideDist/360 * 3.25 * M_PI * 0.75;

      double odomDistance = horizontalOdom.position(degrees) * M_PI / 180;
      double sideways = (odomDistance - oldOdomPos)/360 * 2 * M_PI;
      oldOdomPos = odomDistance;

      double theta = (leftSideDist - rightSideDist)/11.375;
      double totalDistance = 0;

      distanceTraveled += (rightSideDist + leftSideDist) / 2;

      if(abs(sideways) < 0.01) {
        sideways = 0.0;
      }

      if(theta != 0) {
        rightSideDist = 2*((rightSideDist)/theta + (11.375 / 2)) * sin(theta/2);
        totalDistance = rightSideDist;

        double angle = (180 - theta) / 2;
        if(angle > 0) {
          angle -= 90;
        } else if(angle < 0){
          angle += 90;
        }

        double sidewaysDistance = 2*((sideways)/theta+1.15) * sin(theta/2);

        position[0] += totalDistance * sin((angle+position[2])*M_PI/180) + sidewaysDistance * sin((angle+position[2]+90)*M_PI/180);
        position[1] += totalDistance * cos((angle+position[2])*M_PI/180) + sidewaysDistance * cos((angle+position[2]+90)*M_PI/180);
      } else {
        totalDistance = rightSideDist;

        if(abs(sideways) < 0.009) {
          sideways = 0;
        }

        position[0] += totalDistance * sin(position[2]*M_PI/180) + sideways * sin((position[2]+90)*M_PI/180);
        position[1] += totalDistance * cos(position[2]*M_PI/180) + sideways * cos((position[2]+90)*M_PI/180);
      }

      position[2] = inertialSensor.rotation();
      printf("x: %f y: %f theta: %f", position[0], position[1], position[2]);
      printf("\n");
      wait(10, msec);
    }

    void reset() {
      leftSideDT.resetPosition();
      rightSideDT.resetPosition();

      horizontalOdom.resetPosition();

      inertialSensor.setRotation(0, degrees);
      position[0] = 0.0;
      position[1] = 0.0;
      position[2] = 0.0;
    }

    void turnTo(double direction) {
      wait(10, msec);
      tracking = false;
      integral = 0;
      desiredAngle = direction;
    }

    void moveToPos(double x, double y, double waitTime = 0, double xError = 1, double yError = 1, bool stop = true, bool w = true) {
      this->x = x;
      this->y = y;
      this->xError = xError;
      this->yError = yError;
      this->stop = stop;
      percent = 0;

      double xDif = this->x - position[0];
      double yDif = this->y - position[1];
      originalDistance = sqrt(pow(xDif, 2) + pow(yDif, 2));
      wait(10, msec);
      tracking = true;
      positioned = false;

      if(waitTime != 0) {
        wait(waitTime, msec);
      } else {
        while(w) {
          if(positioned) {
            break;
          }
          wait(1, msec);
        }
      }

      originalDistance = 1;
      percent = 100;
      desiredAngle = position[2];
      tracking = false;
    }

    void changeSpeed(double newSpeed = 1) {
      maxSpeed = newSpeed;
    }

    void circle(double radius = 1, double degrees = 90, double speed = 0.2) {
      curving = true;
      distanceTraveled = 0;
      motorSpeed = speed;
      double totalDistance = 2*M_PI*radius * (abs(degrees)/360);
      double originalAngle = position[2];

      while(distanceTraveled < totalDistance) {
        double p = distanceTraveled/totalDistance;

        desiredAngle = originalAngle + degrees * p;
      }

      motorSpeed = 0;
      curving = false;
    }

    void turning() {
      double error = desiredAngle - position[2];
      // 0.06
      double kp = 0.06;
      double ki = 0.00001;
      double kd = 1.5;

      while(true) {
        if(tracking) {
          desiredAngle = position[2] + updatingError;
        }

        double lastError = error;
        error = desiredAngle - position[2];

        double proportional = (kp * error);
        integral += ki*error*(0.0001);
        double derivative = kd*(error - lastError);
        if(tracking) {
          derivative *= std::max(originalDistance/10, 1.0);
        }

        if(error < 0.4 && error > -0.4) {
          angled = true;
          integral = 0;
          turnSpeed = 0;
        } else {
          angled = false;
          double newSpeed = proportional + integral + derivative;
          if(abs(turnSpeed - newSpeed) > 0.5) {
            newSpeed = (newSpeed + turnSpeed)/2;
          }
          turnSpeed = newSpeed;
          if(tracking) {
            turnSpeed *= 0.95;
          }
        }

        wait(10, msec);
      }
    }

    void moveTo() {
      double error = 0;


      //0.15
      double kp = 0.14;
      double ki = 0.034;
      integral = 0;
      double kd = 5.1;

      double oldDistance = 0;

      double xDif = x - position[0];
      double yDif = y - position[1];
      oldDistance = originalDistance;

      int quadrant = 1;


      while(true) {
        if((0 <= position[2] && position[2] <= 90) || (-360 <= position[2] && position[2] <= -270)) {
          quadrant = 1;
        } else if((90 <= position[2] && position[2] <= 180) || (-270 <= position[2] && position[2] <= -180)) {
          quadrant = 2;
        } else if((180 <= position[2] && position[2] <= 270) || (-180 <= position[2] && position[2] <= -90)) {
          quadrant = 3;
        } else if((270 <= position[2] && position[2] <= 360) || (-90 <= position[2] && position[2] <= 0)) {
          quadrant = 4;
        }

        xDif = x - position[0];
        yDif = y - position[1];


        double driveDirection = -1;
        error = 0;


        if(abs(xDif) <= abs(yDif)) {
          if(quadrant == 1 || quadrant == 4) {
            if(yDif > 0) {
              driveDirection = -1;
            } else {
              driveDirection = 1;
            }
          } else if(quadrant == 2 || quadrant == 3) {
            if(yDif > 0) {
              driveDirection = 1;
            } else {
              driveDirection = -1;
            }
          }
        } else {
          if(quadrant == 1 || quadrant == 2) {
            if(xDif > 0) {
              driveDirection = -1;
            } else {
              driveDirection = 1;
            }
          } else if(quadrant == 4 || quadrant == 3) {
            if(xDif > 0) {
              driveDirection = 1;
            } else {
              driveDirection = -1;
            }
          }
        }

        if(yDif != 0) {
          error = atan(xDif/yDif)*180/M_PI - position[2];
          double errorTwo = 0;

          if(xDif >= 0 && yDif >= 0) {
            if(position[2] > 0) {
              errorTwo = error;
            } else {
              errorTwo = -360 + error;
            }
          } else if(xDif > 0 && yDif < 0) {
            if(position[2] > 0) {
              errorTwo = 180+error;
            } else {
              errorTwo = -180+error;
            }
          } else if(xDif < 0 && yDif < 0) {
            if(position[2] > 0) {
              errorTwo = 180+error;
            } else {
              errorTwo = -180+error;
            }
          } else if(xDif < 0 && yDif > 0) {
            if(position[2] < 0) {
              errorTwo = error;
            } else {
              errorTwo = 360+error;
            }
          }

          if(abs(position[2] - error) > abs(position[2] - errorTwo)) {
            error = errorTwo;
          }
        } else {
          error = 0;
        }


        double distance = sqrt(pow(xDif, 2) + pow(yDif, 2));


        double proportional = kp*distance;
        integral += ki*distance*(0.001);
        double derivative = kd*(distance - oldDistance)*(0.001)*-1;

        if(stop && positioned) {
          if(!curving) {
            motorSpeed = 0;
          }
          integral = 0;
        } else {
          percent = (distance/originalDistance);
          if(turnSwitch) {
            updatingError = error;
          } else {
            updatingError = 0;
          }

          if(tracking && !curving) {
            motorSpeed = proportional + integral + derivative - (std::min((error/10 * percent), 0.0) * int(turnSwitch));
            if(motorSpeed > maxSpeed) {
              motorSpeed = maxSpeed;
            }
            motorSpeed *= driveDirection * -1;
          }
        }


        if(abs(position[0] - x) < xError && abs(position[1] - y) < yError) {
          positioned = true;
        } else {
          positioned = false;
        }
        wait(1, msec);
      }
    }

    

    void drive() {
      while(true) {
        if(on) {
          moveDriveTrain(motorSpeed, turnSpeed);
        }
        wait(5, msec);
      }
    }

    void mode(bool on = true, bool turning = true) {
      this->on = on;
      turnSwitch = turning;
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
odometry odom;
bool autonStarted = false;
//1 = awp left, 2 = awp right, 3 = right side elim auton, 4 = left side elim auton, 5 = skills
int selectedAuton = 0;
void startDrive() {
  odom.drive();
}

void startMove() {
  odom.moveTo();
}

void startTurn() {
  odom.turning();
}

void startUpdating() {
  while(true) {
    odom.updatePosition();
  }
}

void whenBrainScreenPressed() {
  int pressedX = botBrain.Screen.xPosition();
  int pressedY = botBrain.Screen.yPosition();
  if(50 < pressedX && pressedX < 50 + 480 * 0.375 && 60 < pressedY && pressedY < 60 + 240 * 0.375) {
    selectedAuton = 2;
    botBrain.Screen.clearScreen(green);
  } else if(50 < pressedX && pressedX < 50 + 480 * 0.375 && 60 + 240 * 0.375 < pressedY && pressedY < 60 + 2 * 240 * 0.375) {
    selectedAuton = 1;
    botBrain.Screen.clearScreen(green);
  } else if(50 + 480 * 0.375 < pressedX && pressedX < 50 + 2 * 480 * 0.375 && 60 < pressedY && pressedY < 60 + 240 * 0.375) {
    selectedAuton = 3;
    botBrain.Screen.clearScreen(green);
  } else if(50 + 480 * 0.375 < pressedX && pressedX < 50 + 2 * 480 * 0.375 && 60 + 240 * 0.375 < pressedY && pressedY < 60 + 2 * 240 * 0.375) {
    selectedAuton = 4;
    botBrain.Screen.clearScreen(green);
  } else if(50 + 2 * 480 * 0.375 < pressedX && pressedX < 50 + 3 * 480 * 0.375 && 60 < pressedY && pressedY < 60 + 240 * 0.375) {
    selectedAuton = 5;
    botBrain.Screen.clearScreen(green);
  }
}



int pre_auton(void) {
  inertialSensor.calibrate();
  while(inertialSensor.isCalibrating()) {
    wait(10, msec);
  }
  return 0;
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

// void move(int param, int speed){
//   if(param == 1){
//     leftSideDT.spin(forward, speed, volt);
//     rightSideDT.spin(forward, speed, volt);
//   } else if(param == -1){
//     leftSideDT.spin(reverse, speed, volt);
//     rightSideDT.spin(reverse, speed, volt);
//   } else if(param = 2){
//     leftSideDT.spin(reverse, speed, volt);
//     rightSideDT.spin(forward, speed, volt);
//   } else if(param = -2){
//     leftSideDT.spin(forward, speed, volt);
//     rightSideDT.spin(reverse, speed, volt);
//   } else if(param == 0){
//     leftSideDT.spin(forward, 0, volt);
//     rightSideDT.spin(forward, 0, volt);
//   }
// }

// void autoTake(int dir){
//   if(dir == 1){
//     motorIntakeOne.spin(reverse, 5.5, volt);
//     motorIntakeTwo.spin(reverse, 11, volt);
    
//   } else if(dir == 2){
//     motorIntakeOne.spin(forward, 5.5, volt);
//     motorIntakeTwo.spin(forward, 11, volt);
    
//   } else if(dir == 3){
//     wingSolenoid.set(false);
//     wingMech = false;
//     motorIntakeOne.spin(forward, 5.5, volt);
//     motorIntakeTwo.spin(forward, 11, volt);
    
//   }
// }

void rightSideElims() {

}

void leftSideElims() {

}

void rightSideAWP() {

}

void leftSideAWP() {

}

void skills() {

}

void autonomous(void) {
  thread drivingThread(startDrive);
  thread movingThread(startMove);
  thread turningThread(startTurn);
  thread updatingThread(startUpdating);
  if(selectedAuton == 1) {
    leftSideAWP();
  } else if(selectedAuton == 2) {
    rightSideAWP();
  } else if(selectedAuton == 4) {
    leftSideElims();
  } else if(selectedAuton == 5) {
    skills();
  } else {
    rightSideElims();
  }

  while(Competition.isAutonomous()) {
    wait(10, msec);
  }
  drivingThread.interrupt();
  movingThread.interrupt();
  turningThread.interrupt();
  updatingThread.interrupt();
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

void whenL1Pressed() {
  printf("L1 pressed \n");
  if(piston1E == false){
    intakePiston1.set(false);
    piston1E = false;
  }
  if(piston2E == true){
    intakePiston2.set(false);
    piston2E = false;
  }
  motorIntakeOne.spin(forward, 11, volt);
  motorIntakeTwo.spin(forward, 11, volt);
}

void whenL1Released(){
  printf("L1 released \n");
  motorIntakeOne.spin(forward, 0, volt);
  motorIntakeTwo.spin(forward, 0, volt);
  if(piston1E != false || piston2E != false){
    intakePiston1.set(false);
    piston1E = false;
    intakePiston2.set(false);
    piston2E = false;
  }
}

void whenR2Pressed(){
  printf("R1 released \n");
  if(piston1E != false || piston2E != true){
    intakePiston1.set(false);
    piston1E = true;
    intakePiston2.set(true);
    piston2E = true;
  }
  motorIntakeOne.spin(forward, 11, volt);
  motorIntakeTwo.spin(forward, 11, volt);
}

void whenR2Released(){
  printf("R1 released \n");
  motorIntakeOne.spin(forward, 0, volt);
  motorIntakeTwo.spin(forward, 0, volt);
  if(piston1E != false || piston2E != false){
    intakePiston1.set(false);
    piston1E = false;
    intakePiston2.set(false);
    piston2E = false;
  }
}

void whenL2Pressed() {
  printf("L2 pressed \n");
  motorIntakeOne.spin(reverse, 11, volt);
  motorIntakeTwo.spin(reverse, 11, volt);
  
}

void whenL2Released(){
  printf("L2 released \n");
  motorIntakeOne.spin(reverse, 0, volt);
  motorIntakeTwo.spin(reverse, 0, volt);
}

void whenR1Pressed() { 
  if(piston1E != true || piston2E != false){
    intakePiston1.set(true);
    piston1E = true;
    intakePiston2.set(false);
    piston2E = false;
  }
  motorIntakeOne.spin(forward, 11, volt);
  motorIntakeTwo.spin(forward, 11, volt);
}

void whenR1Released() {
  printf("R2 released \n");
  motorIntakeOne.spin(forward, 0, volt);
  motorIntakeTwo.spin(forward, 0, volt);
  if(piston1E != false || piston2E != false){
    intakePiston1.set(false);
    piston1E = false;
    intakePiston2.set(false);
    piston2E = false;
  }
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
  if(!piston1E) {
    intakePiston1.set(true);
    piston1E = true;
  } else {
    intakePiston1.set(false);
    piston1E = false;
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
  //thread unjam = thread(unjamCheck);
  botController.ButtonL1.pressed(whenL1Pressed);
  botController.ButtonL1.released(whenL1Released);
  botController.ButtonR2.pressed(whenR2Pressed);
  botController.ButtonR2.released(whenR2Released);
  botController.ButtonL2.pressed(whenL2Pressed);
  botController.ButtonL2.released(whenL2Released);
  botController.ButtonR1.pressed(whenR1Pressed);
  botController.ButtonR1.released(whenR1Released);
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

  botBrain.Screen.setPenColor(yellow);
  botBrain.Screen.setPenWidth(20);
  botBrain.Screen.printAt(50, 40, "AWP Autons");
  // right auton = red, left = blue
  botBrain.Screen.drawRectangle(50, 60, 480 * 0.375, 240 * 0.375, red);
  botBrain.Screen.drawRectangle(50, 60 + 240 * 0.375, 480 * 0.375, 240 * 0.375, blue);

  botBrain.Screen.printAt(50 + 480 * 0.375, 40, "Elim Autons");

  botBrain.Screen.drawRectangle(50 + 480 * 0.375, 60, 480 * 0.375, 240 * 0.375, red);
  botBrain.Screen.drawRectangle(50 + 480 * 0.375, 60 + 240 * 0.375, 480 * 0.375, 240 * 0.375, blue);

  botBrain.Screen.printAt(50 + 2 * 480 * 0.375, 40, "Skills Auton");

  botBrain.Screen.drawRectangle(50 + 2 * 480 * 0.375, 60, 480 * 0.375, 240 * 0.375, purple);

  botBrain.Screen.pressed(whenBrainScreenPressed);

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
