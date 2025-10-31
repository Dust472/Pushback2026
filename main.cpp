#include "main.h"
#include "api.h"

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"

using namespace std;
using namespace pros;

#define DIGITAL_SENSOR_PORT 1
#define DIGITAL_SENSOR_PORT_2 2


Controller master(E_CONTROLLER_MASTER);
MotorGroup left_mg({1, -2, -3});
MotorGroup right_mg({11, -12, 13});
MotorGroup intake({-9, 10});
IMU inertialSensor(19);
bool state = LOW;
pros::adi::Pneumatics intakeHoodPiston1('A', true);
pros::adi::Pneumatics intakeHoodPiston2('B', false);


// poopooopooo
//676776767676767667676767
// class odom {
// 	private:
// 		MotorGroup* leftDT;
// 		MotorGroup* rightDT;
// 		double position[2];
// 		IMU* inertialSensor;
// 		double Kp;
// 	public:
// 		/*
// 		This is the constructor for our odom object. It takes in the sides of the drivetrains and sets the class attributes (above)
// 		to the respective sides.
// 		*/
// 		odom() {
// 			this->leftDT = &left_mg;
// 			this->rightDT = &right_mg;
// 			this->position[0] = 0;
// 			this->position[1] = 0;
// 		}
		
// 		/** 
// 		 * This function actually takes input (when we call this function in the autonomous function below) and moves the bot
// 		 * based on that input
// 		 * Parameters:
// 		 * x: the target x-coordinate for the bot
// 		 * y: the target y-coordinate for the bot
// 		 * xErr: the bounds allowed for overshoot (x-axis)
// 		 * yErr: the bounds allowed for overshoot (y-axis)
// 		 * */
// 		void changeBotPosition(double x, double y, double xErr, double yErr) {
// 			double targetRadius = sqrt((x * x) + (y * y));
// 			double targetAngle = atan2(y, x);
// 			double currentRadius;
			
// 			while(abs(this->position[0] - x) > xErr && abs(this->position[1] - y) > yErr){
// 				currentRadius = sqrt((this->position[0] * this->position[0]) + (this->position[1] * this->position[1]));
				
// 			}
// 			resetBotPosition();
// 		}

// 		/**
// 		 * This resets the position arrays values. The plan is to use this after we reach each target position. That way, we can get accurate
// 		 * feedback for when we check to see if the robot's position is 
// 		 */
// 		void resetBotPosition() {
// 			this->position[0] = 0;
// 			this->position[1] = 0;
// 		}
// };

// odom odomPod();

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	lcd::initialize();
	lcd::print(0, "code uploaded");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	inertialSensor.reset();
}

/*While the Odom is deactivated, this code will move the bot forward or backward
  depending on a value between -1 and 1 using move_voltage.
*/
void preciseMove(int moveDirection){
	if(moveDirection == 1) {
		right_mg.move_voltage(7000);
		left_mg.move_voltage(7000);
	}
	if(moveDirection == 0) {
		right_mg.move_voltage(0);
		left_mg.move_voltage(0);	
	}
	if(moveDirection == -1) {
		right_mg.move_voltage(-7000);
		left_mg.move_voltage(-7000);
	}
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {

}

/*
These variables are the main part of the logic of the drivetrain
Allows us to easily swap between two-stick and one stick
Brody prefers one-stick whereas Anthony prefers two-stick.
oneOrTwo variable: One = true, two = false
rightOrLeft variable: Right = true, left = false
*/
bool rightOrLeft = true;
bool oneOrTwo = false;

// Because rightOrLeft will not be constant, we cannot rely on it for when we switch back to the original.
// So, this variable is made to hold the original value
bool defaultRight = rightOrLeft;

/**
 * This function is its own independent event. It changes the global rightOrLeft value to true if and only if
 * the user has the oneStick option enabled
 */
void rightButtonPressed() {
	while(true) {
		if(oneOrTwo) {
			if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)) {
				//lcd::print(5, "rightButtonPressed");
				rightOrLeft = true;
			}
		}
		delay(1);
	}
}

//Same as the rightbuttonPressed() method except it changes the rightOrLeft variable to false
void leftButtonPressed() {
	while(true) {
		if(oneOrTwo) {	
			if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)) {
				//lcd::print(6, "Left Button Pressed");
				rightOrLeft = false;
			}
		}
		delay(1);
	}
}


// Also an independent event like the previous 2 methods
void bButtonPressed() {
	while(true) {
		if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
			//lcd::print(2, "B Button Pressed");
			oneOrTwo = !oneOrTwo;
			if(oneOrTwo == false) {
				rightOrLeft = defaultRight;
			}
			//lcd::print(4, "One stick active: %d", oneOrTwo);
		}
		delay(1);
	}
}

// This method is dependent upon the button input from the bButtonPressed(), leftButtonPressed(), and rightButtonPressed() methods
void moveMotors() {
	while(true) {
		int rightStickValueX = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
		int rightStickValueY = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);
		int leftStickValueX = master.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
		int leftStickValueY = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);

		// This switch statement handles all of the button logic : it moves our drivetrain based on this logic
		switch(oneOrTwo) {
			case true:
				if(rightOrLeft == true) {
					right_mg.move_velocity(rightStickValueY - rightStickValueX);
					left_mg.move_velocity(rightStickValueY + rightStickValueX);
				}
				else {
					right_mg.move_velocity(leftStickValueY - leftStickValueX);
					left_mg.move_velocity(leftStickValueY + leftStickValueX);
				}
				break;
			case false:
				right_mg.move_velocity(leftStickValueY - rightStickValueX);
				left_mg.move_velocity(leftStickValueY + rightStickValueX);
		}

		// To prevent the program from crashing
		pros::Task::delay(5);
	}
}

/**
 * These tasks are nested to allow our intake to be able to move at the same time that our drivetrain moves
 * (that is, the different tasks inside these functions are the drivetrain's logic)
 */
void moveDriveTrain() {
	Task bButtonPress(bButtonPressed);
	Task rightButtonPress(rightButtonPressed);
	Task leftButtonPress(leftButtonPressed);
	Task moveMotor(moveMotors);
}


bool isIntaking = false;
bool isExtaking = false;

/**
 * This allows moving the drivetrain to be asynchronous with moving the intake
 * by using two different tasks in the opcontrol() function
 */
void moveIntake() {
	while(true) {
		if(!isExtaking) {
			if(master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
				intake.move_velocity(200);
				isIntaking = true;
			}
			else {
				intake.move_velocity(0);
				isIntaking = false;
			}
		}
		pros::Task::delay(20);
	}
}

void extake() {
	while(true) {
		if(!isIntaking) {
			if(master.get_digital(E_CONTROLLER_DIGITAL_R1)) {
				intake.move_velocity(-200);
				isExtaking = true;
			}
			else {
				intake.move_velocity(0);
				isExtaking = false;
			}
		}
		pros::Task::delay(20);
	}
}

void hoodIntake1() {
	while(true) {
		if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)) {
			intakeHoodPiston1.toggle();
		}
		delay(5);
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() { 
	lcd::print(1, "Intake task working");
	Task movingDT(moveDriveTrain);
	Task movingIntake(moveIntake);
	Task extaking(extake);
	Task hoodIntake(hoodIntake1);
}

/*
movement type (boolean, true = one stick, false = two stick)
right stick or left stick (boolean, true = right stick, false = left stick)

if(b button new press) then
	one or two stick = opposite of its current value
if(one or two stick == one stick) then
	if(left button new press) then
		right stick or left stick = left
	if(right button new press) then 
		right stick or left stick = right
*/

// void opcontrol() {
// 	bool oneOrTwo = true;
// 	bool rightOrLeft = true;
// 	bool initOneOrTwo = oneOrTwo;
// 	while(true) {
// 		int turnRight = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
// 		int dirLeft = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
// 		int turnLeft = master.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
// 		int dirRight = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);

// 		if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
// 			oneOrTwo = !oneOrTwo;
// 			if(oneOrTwo == false) {
// 				rightOrLeft = initOneOrTwo;
// 			}
// 		}

// 		if(oneOrTwo) {
// 			if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)) {
// 				rightOrLeft = false;
// 			}
// 			if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)) {
// 				rightOrLeft = true;
// 			}
// 		}

// 		switch(oneOrTwo) {
// 			case true:
// 				if(rightOrLeft == true) {
// 					right_mg.move_velocity(dirRight - turnRight);
// 					left_mg.move_velocity(dirRight + turnRight);
// 				}
// 				else {
// 					right_mg.move_velocity(dirLeft - turnLeft);
// 					left_mg.move_velocity(dirLeft + turnLeft);
// 				}
// 				break;
// 			case false:
// 				right_mg.move_velocity(dirLeft - turnRight);
// 				left_mg.move_velocity(dirLeft + turnRight);

// 		}
// 		pros::Task::delay(5);
// 	}
// }







