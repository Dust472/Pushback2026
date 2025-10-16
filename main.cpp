#include "main.h"
#include "api.h"

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"

using namespace std;
using namespace pros;

Controller master(E_CONTROLLER_MASTER);
MotorGroup left_mg({1, -2, -3});
MotorGroup right_mg({11, -12, 13});
Motor intake(10);

class PID {
	
};

class odom {
	private:
		MotorGroup* leftDT;
		MotorGroup* rightDT;
		int quadrantTarget;
		bool forewardDriving;
		double position[2];
	public:
		/*
		This is the constructor for our odom object. It takes in the sides of the drivetrains and sets the class attributes (above)
		to the respective sides. The "&" symbol just gets the VALUE of the MotorGroup parameters
		(since Pointers are essentially just placeholders in memory for things)
		*/
		odom() {
			this->leftDT = &left_mg;
			this->rightDT = &right_mg;
			this->quadrantTarget = 0;
			this->forewardDriving = true;
		}
		
		/** 
		 * This function actually takes input (when we call this function in the autonomous function below) and moves the bot
		 * based on that input
		 * Parameters:
		 * x: the target x-coordinate for the bot
		 * y: the target y-coordinate for the bot
		 * xErr: The allowed difference for the x (greater or less than)
		 * this prevents the bot from wiggling (the plan is to use a while loop to keep the bot moving while it's not at its target position)
		 * yErr: same as x error, and needed for the same reason
		 * */
		void changeBotPosition(double x, double y, double xErr, double yErr) {
			// These nested if-statements check to see what quadrant the target position is based on the x and y
			// This will allow us to check and see if the bot needs to drive backward or foreward
			if(x < 0){
				if(y < 0) {
					this->quadrantTarget = 3;
				}
				else {
					this->quadrantTarget = 2;
				}
			}
			else{
				if(y < 0) {
					this->quadrantTarget = 4;
				}
				else{
					this->quadrantTarget = 1;
				}
			}
			
			// This is the actual logic to see if we drive backwards or not (using quadrantTarget that is set above)
			if(this->quadrantTarget == 3 or this->quadrantTarget == 4) {
				this->forewardDriving = false;
			}
			else{
				this->forewardDriving = true;
			}
			
			while(true) {

			}
		}

		void resetBotPosition() {
			this->position[0] = 0;
			this->position[1] = 0;
		}
};

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {

}


// One = true, two = false
//Right = true, left = false
bool rightOrLeft = true;
bool oneOrTwo = true;
bool defaultRight = rightOrLeft;

void rightButtonPressed() {
	while(true) {
		if(oneOrTwo) {
			if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)) {
				lcd::print(5, "rightButtonPressed");
				rightOrLeft = true;
			}
		}
		delay(1);
	}
}

void leftButtonPressed() {
	while(true) {
		if(oneOrTwo) {	
			if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)) {
				lcd::print(6, "Left Button Pressed");
				rightOrLeft = false;
			}
		}
		delay(1);
	}
}

void bButtonPressed() {
	while(true) {
		if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
			lcd::print(2, "B Button Pressed");
			oneOrTwo = !oneOrTwo;
			if(!oneOrTwo) {
				rightOrLeft = defaultRight;
			}
			lcd::print(4, "One stick active: %d", oneOrTwo);
		}
		delay(1);
	}
}

/**
 * Actual performs all of the button logic to move the motors
 */
void moveMotors() {
	// This runs the code to actually move the motors based on the buttons
	while(true) {
		// Get all needed controller analog stick values
		int rightStickValueX = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
		int rightStickValueY = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);
		int leftStickValueX = master.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
		int leftStickValueY = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);

		// This switch statement handles all of the button-pressing logic
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
 * This starts all of the tasks (which hold the logic for the drivetrain)
 */
void moveDriveTrain() {
	Task bButtonPress(bButtonPressed);
	Task rightButtonPress(rightButtonPressed);
	Task leftButtonPress(leftButtonPressed);
	Task moveMotor(moveMotors);
}

/**
 * This allows moving the drivetrain to be asynchronous with moving the intake
 */
void moveIntake() {
	while(true) {
		if(master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
			intake.move_velocity(200);  
		}
		else {
			intake.move_velocity(0);
		}
		pros::Task::delay(20);
	}
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	lcd::initialize();
	lcd::print(0, "sksks");
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
}
