#include "main.h"
#include "api.h"

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"

using namespace std;
using namespace pros;



Controller master(E_CONTROLLER_MASTER);
MotorGroup left_mg({-1, 2, 3});
MotorGroup right_mg({11, -12, 13});
Motor firstStageIntake(10);
Motor secondStageIntake(-9);
MotorGroup intake({firstStageIntake.get_port(), secondStageIntake.get_port()});
Rotation odomP(18);
IMU inertialSensor(15);
adi::Pneumatics intakeHoodPiston1('A', false);
adi::Pneumatics scraperMech1('B', false);
class odom {
	private:
		double position[3];
		bool odomOn;
		double leftAverage;
		double rightAverage;
		double odomPPosition;
	public:
		/*
		This is the constructor for our odom object. It takes in the sides of the drivetrains and sets the class attributes (above)
		to the respective sides.
		*/
		odom() {
			this->position[0] = 0;
			this->position[1] = 0;
			this->position[2] = 0;
			this->odomOn = false;
			this->leftAverage = 0;
			this->rightAverage = 0;
			this->odomPPosition = 0;
		}
		/**
		 * This resets the position arrays values. The plan is to use this after we reach each target position. That way, we can get accurate
		 * feedback for when we check to see if the robot's position is 
		 */
		void resetBotPosition() {
			this->position[0] = 0;
			this->position[1] = 0;
		}

		void resetBotOrientation() {
			this->position[2] = 0;
		}

		bool getOdomOn() {
			return this->odomOn;
		}

		void setOdomOn(bool newValue) {
			this->odomOn = newValue;
			if(newValue) {
				resetBotPosition();
				resetBotOrientation();
			}
		}

		double getPoseX() {
			return this->position[0];
		}

		double getPoseY() {
			return this->position[1];
		}

		double getOrientation() {
			return this->position[2];
		}

		void printPose() {
			lcd::print(5, "%d", this->position[0]);
			lcd::print(6, "%d", this->position[1]);
			lcd::print(7, "%d", this->position[2]);
		}

		int turnTo(double degrees) {
			if(!this->odomOn) {
				return 1;
			}
			while(0 > degrees) {
				degrees += 360;
			}
			while(degrees > 360){
				degrees -= 360;
			}
			degrees -= 180;
			double radians = degrees * (M_PI / 180);
			/*
			change in theta = change in left - change in right / track width. Change in left = - change in right
			so change in theta = -2* change in right / track width
			so change in right = (change in theta * track width) / -2
			or change in theta = 2 * change in left / track width
			so change in left = (change in theta * track width) / 2
			*/
			double currentOrientation = inertialSensor.get_yaw() * (M_PI / 180);
			
		}


		/** 
		 * This function updates the position of our odom pod
		 * */
		int changeOdomPosition() {
			if(!this->odomOn) {
				return 1;
			}

			vector<double> leftSideEncoders = left_mg.get_position_all();
			vector<double> rightSideEncoders = right_mg.get_position_all();

			// Get the average of the left side of the DT
			double leftSideCurrentAvg = 0;
			double leftSum = 0;
			for(int i = 0; i < leftSideEncoders.size(); i++) {
				leftSum += leftSideEncoders[i];
			}
			leftSideCurrentAvg = leftSum / leftSideEncoders.size();


			// Get the average of the right side of the DT
			double rightSideCurrentAvg = 0;
			double rightSum = 0;
			for(int i = 0; i < rightSideEncoders.size(); i++) {
				rightSum += rightSideEncoders[i];
			}
			rightSideCurrentAvg = rightSum / rightSideEncoders.size();

			// Get the change in the odom's position (divided by 100 since get_position() gives centidegrees which is just 100 times a degree)
			double changeInOdom = ((odomP.get_position() - this->odomPPosition) / 100) * 2 * (M_PI/180);

			// Get the actual differece in arc length of both the left and right sides averages compared to the last iteration
			double differenceLeft = abs(leftSideCurrentAvg - this->leftAverage) * 3.25 * (M_PI / 180);
			double differenceRight = abs(rightSideCurrentAvg - this->rightAverage) * 3.25 * (M_PI/180);

			// Using the formula phetal = phetar + deltaL - deltaR/sL + sR, we find our new orientation
			double currentAbsOrientation = this->position[2] + (differenceLeft - differenceRight)/15.5;

			// Get the actual change in orientation
			double changeInOrientation = currentAbsOrientation - this->position[2];

			/*
			This part checks to see if our orientation even changes
			if it does, then we use a derived formula
			If it doesn't then we just use one of the sides of the drivetrain to set to our local
			x and y (these are the x and y's calculated relative to the bot and not the field)
			*/
			double localX;
			double localY;
			if(changeInOrientation != 0) {
				localX = 2 * sin(changeInOrientation/2) * (changeInOdom/changeInOrientation);
				localY = 2 * sin(changeInOrientation/2) * (differenceRight/changeInOrientation + 6.75);
			} else {
				localX = changeInOdom;
				localY = differenceRight;
			}

			/*
			The rest of this code converts from local to global position
			First, we get the average orientation
			Then we convert to polar cords using trigonometry, subtracting the average orientation from the angle in order to get global poses
			Then convert back to cartesian
			We then add these to the position attributes of the class
			*/
			double averageOrientation = this->position[2] + changeInOrientation / 2;

			double polarRadius = sqrt(localX * localX + localY * localY);
			double polarAngle = atan2(localY, localX) - averageOrientation;

			while(0 > polarAngle){
				polarAngle += M_PI * 2;
			}

			while(polarAngle > (M_PI * 2)) {
				polarAngle -= M_PI * 2;
			}

			double globalX = polarRadius * cos(polarAngle);
			double globalY = polarRadius * sin(polarAngle);

			this->position[0] += globalX;
			this->position[1] += globalY;
			this->position[2] = currentAbsOrientation;

			// Update all of the variables to the current (was previous)
			this->leftAverage = leftSideCurrentAvg;
			this->rightAverage = rightSideCurrentAvg;
			this->odomPPosition = odomP.get_position();
			return 1;
		}

		int changeBotPosition(double x, double y, double xErr, double yErr) {
			if(!this->odomOn) {
				return 1;
			}
			double targetRadius = sqrt(x * x + y * y);
			double targetAngle = atan2(y, x);

			double minRadiusX = x - xErr;
			double maxRadiusX = x + xErr;
			double minRadiusY = y - yErr;
			double maxRadiusY = y + yErr;

			double minRadius = sqrt(minRadiusX * minRadiusX + minRadiusY * minRadiusY);
			double maxRadius = sqrt(maxRadiusX * maxRadiusX + maxRadiusY * maxRadiusY);

			return 1;
		}

		

		
		
};

odom odomPod;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	lcd::initialize();
	lcd::print(0, "code uploaded");
	right_mg.set_encoder_units_all(E_MOTOR_ENCODER_DEGREES);
	left_mg.set_encoder_units_all(E_MOTOR_ENCODER_DEGREES);
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
	odomP.reset();
	odomPod.setOdomOn(true);
}

/*While the Odom is deactivated, this code will move the bot forward or backward
  depending on a value between -1 and 1 using move_voltage.
*/
int preciseMove(int moveDirection){
	if(!odomPod.getOdomOn()) {
		return 1;
	}
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
	return 1;
}

void testing() {
	odomPod.setOdomOn(false);
	while(true) {
		preciseMove(1);
		printf("%d", odomPod.getPoseX());
		printf("%d", odomPod.getPoseY());
		delay(1000);
		preciseMove(0);
		delay(1000);
	}
	
}

void on_auton_1() {
	while(true) {
		odomPod.changeOdomPosition();
		delay(5);
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
	Task updateOdom(on_auton_1);
	Task test(testing);
	while(true) {
		odomPod.printPose();
		delay(20);
	}
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

/**
 * This allows moving the drivetrain to be asynchronous with moving the intake
 * by using two different tasks in the opcontrol() function
 */
void moveIntake() {
	while(true) {
		if(master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
			intake.move_velocity(200);
		} else if(master.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			intake.move_velocity(-200);
		} else if(master.get_digital(E_CONTROLLER_DIGITAL_L2)) {
			firstStageIntake.move_velocity(200);
		} else {
			intake.move_velocity(0);
		}
		pros::Task::delay(20);
	}
}

void hoodIntake1() {
	while(true) {
		if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)) {
			intakeHoodPiston1.toggle();
		}
		delay(5);
	}
}

void scraperMechF() {
	while(true) {
		if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
			scraperMech1.toggle();
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
	Task hoodIntake(hoodIntake1);
	Task scraperMech(scraperMechF);
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







