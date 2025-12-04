#include "main.h"
#include "api.h"

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"

#define DIGITAL_SENSOR_PORT 'A'

using namespace std;
using namespace pros;
using namespace competition;



Controller master(E_CONTROLLER_MASTER);
MotorGroup left_mg({-1, -2, 3});
MotorGroup right_mg({18, 19, -20});
Motor firstStageIntake(-8);
Motor secondStageIntake(9);
Motor thirdStageIntake(-10);
MotorGroup intake({firstStageIntake.get_port(), secondStageIntake.get_port()});
pros::adi::Pneumatics topPiston('B', true);
adi::Pneumatics scraperMech('A', true);
adi::Pneumatics wingPiston('C', false);
Rotation odomP(18);
IMU inertialSensor(4);

class odom {
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
		bool odomOn;
		double maxSpeed;
		bool turnSwitch;

		double oldL;
		double oldR;

		double distanceTraveled;
		static int turnCount;
		static int moveCount;
	public:
		/*
		This is the constructor for our odom object. It takes in the sides of the drivetrains and sets the class attributes (above)
		to the respective sides.
		*/
		odom() {
			this->position[0] = 0;
			this->position[1] = 0;
			this->position[2] = 0;

			this->motorSpeed = 0;
			this->turnSpeed = 0;
			this->desiredAngle = 0;
			this->updatingError = 0;
			this->angled = true;

			this->x = 0;
			this->y = 0;
			this->xError = 0;
			this->yError = 0;
			this->percent = 1;
			this->originalDistance = 1;
			this->stop = true;
			this->positioned = true;
			this->curving = false;
			this->tracking = false;
			this->integral = 0;
			this->odomOn = true;
			this->maxSpeed = 1;
			this->turnSwitch = true;

			this->oldL = 0;
			this->oldR = 0;
			

			this->distanceTraveled = 0;
			turnCount = 0;
			moveCount = 0;
		}
		



		
		int changeOdomPosition() {
			vector<double> leftSideEncoders = left_mg.get_position_all();
			vector<double> rightSideEncoders = right_mg.get_position_all();

			// Get the average of the left side of the DT
			double leftSideCurrentAvg = 0;
			double leftSum = 0;
			for(int i = 0; i < leftSideEncoders.size(); i++) {
				leftSum += leftSideEncoders[i];
			}
			leftSideCurrentAvg = leftSum / leftSideEncoders.size();
			double differenceLeft = leftSideCurrentAvg - this->oldL;
			this->oldL = leftSideCurrentAvg;


			differenceLeft = differenceLeft/360 * 3.25 * M_PI * 0.75;
			// Get the average of the right side of the DT
			double rightSideCurrentAvg = 0;
			double rightSum = 0;
			for(int i = 0; i < rightSideEncoders.size(); i++) {
				rightSum += rightSideEncoders[i];
			}
			rightSideCurrentAvg = rightSum / rightSideEncoders.size();
			double differenceRight = rightSideCurrentAvg - this->oldR;
			this->oldR = rightSideCurrentAvg;


			differenceRight = differenceRight/360 * 3.25 * M_PI * 0.75;
			// Get the change in the odom's position (divided by 100 since get_position() gives centidegrees which is just 100 times a degree)
			// double changeInOdom = 0.00000001 * 2 * (M_PI/180);

			// Get the actual differece in arc length of both the left and right sides averages compared to the last iteration
			

			// Using the formula phetal = phetar + deltaL - deltaR/sL + sR, we find our new orientation
			
			double theta = (differenceLeft - differenceRight) / 12.84;

			double displacement = (differenceRight + differenceLeft) / 2;
			this->distanceTraveled += displacement;

			if(theta != 0) {
				double arcDistance = 2 * (displacement / theta) * sin(theta / 2);
				double angle = this->position[2] + theta / 2 * 180 / M_PI;
				this->position[0] += arcDistance * sin(angle * M_PI / 180);
				this->position[1] += arcDistance * cos(angle * M_PI / 180);
			} else {
				this->position[0] += displacement * sin(this->position[2] * M_PI / 180);
				this->position[1] += displacement * cos(this->position[2] * M_PI / 180);
			}


			/*
			This part checks to see if our orientation even changes
			if it does, then we use a derived formula
			If it doesn't then we just use one of the sides of the drivetrain to set to our local
			x and y (these are the x and y's calculated relative to the bot and not the field)
			*/
			// double localX;
			// double localY;
			// if(changeInOrientation != 0) {
			// 	localX = 2 * sin(changeInOrientation/2) * (changeInOdom/changeInOrientation);
			// 	localY = 2 * sin(changeInOrientation/2) * (differenceRight/changeInOrientation + 6.75);
			// } else {
			// 	localX = changeInOdom;
			// 	localY = differenceRight;
			// }

			/*
			The rest of this code converts from local to global position
			First, we get the average orientation
			Then we convert to polar cords using trigonometry, subtracting the average orientation from the angle in order to get global poses
			Then convert back to cartesian
			We then add these to the position attributes of the class
			*/
			// double averageOrientation = this->position[2] + changeInOrientation / 2;

			// double polarRadius = sqrt(localX * localX + localY * localY);
			// double polarAngle = atan2(localY, localX) - averageOrientation;

			// while(0 > polarAngle){
			// 	polarAngle += M_PI * 2;
			// }

			// while(polarAngle > (M_PI * 2)) {
			// 	polarAngle -= M_PI * 2;
			// }

			this->position[2] = inertialSensor.get_rotation();
			return 1;
		}

		void resetOdom() {
			right_mg.set_zero_position_all(0);
			left_mg.set_zero_position_all(0);

			inertialSensor.set_rotation(0);

			this->position[0] = 0;
			this->position[1] = 0;
			this->position[2] = 0;
		}


		void turnTo(double direction) {
			delay(20);
			this->tracking = false;
			this->integral = 0;
			this->desiredAngle = direction;
		}

		int changeBotPosition(double x, double y, int waitTime = 0, double xError = 1.0, double yError = 1.0, bool stop = true, bool w = true) {
			this->x = x;
			this->y = y;
			this->xError = xError;
			this->yError = yError;
			this->stop = stop;
			this->percent = 0;

			double dx = this->x - this->position[0];
			double dy = this->y - this->position[1];
			this->originalDistance = sqrt(dx * dx + dy * dy);
			delay(20);
			this->tracking = true;
			this->positioned = false;

			if(waitTime != 0) {
				delay(waitTime);
			} else {
				while(w) {
					if(this->positioned) {
						break;
					}
					delay(1);
				}
			}

			this->originalDistance = 1;
			this->percent = 100;
			this->desiredAngle = this->position[2];
			this->tracking = false;
		}

		void changeSpeed(double newMax = 1) {
			this->maxSpeed = newMax;
		}

		void turning() {
			double error = this->desiredAngle - this->position[2];

			double kP = 0.06;
			double kI = 0.00001;
			if(turnCount == 0) {
				this->integral = 0;
			}
			double kD = 1.5;
			turnCount = 1;

			if(this->tracking) {
				this->desiredAngle = this->position[2] + this->updatingError;
			}
			double lastError = error;
			error = this->desiredAngle - this->position[2];

			double proportional = kP * error;
			this->integral += kI * error * 0.0001;
			double derivative = kD * (error - lastError);
			if(this->tracking) {
				derivative *= fmax(this->originalDistance/10, 1);
			}

			if(-0.4 < error && error < 0.4) {
				this->angled = true;
				this->integral = 0;
				this->turnSpeed = 0;
			} else {
				this->angled = false;
				double newSpeed = proportional + this->integral + derivative;
				if(abs(this->turnSpeed - newSpeed) > 0.5) {
					newSpeed = (newSpeed + this->turnSpeed)/2;
				}
				this->turnSpeed = newSpeed;
				if(this->tracking) {
					this->turnSpeed *= 0.95;
				}
			}
		}

		void moveTo() {
			double error = 0;
			double kP = 0.15;
			double kI = 0.034;
			if(moveCount == 0) {
				this->integral = 0;
			}
			double kD = 5.1;

			double oldDist = 0;

			double dx = this->x - this->position[0];
			double dy = this->y - this->position[1];
			oldDist = this->originalDistance;
			
			

			int quadrant = 1;
		
			if((0 <= this->position[2] && this->position[2] <= 90) || (-360 <= this->position[2] && this->position[2] <= -270)) {
				quadrant = 1;
			} else if ((90 <= this->position[2] && this->position[2] <= 180) || (-270 <= this->position[2] && this->position[2] <= -180)) {
				quadrant = 2;
			} else if ((180 <= this->position[2] && this->position[2] <= 270) || (-180 <= this->position[2] && this->position[2] <= -90)) {
				quadrant = 3;
			} else if((270 <= this->position[2] && this->position[2] <= 360) || (-90 <= this->position[2] && this->position[2] <= 0)) {
				quadrant = 4;
			}

			dx = this->x - this->position[0];
			dy = this->y - this->position[1];

			int driveDir = -1;
			error = 0;

			if(abs(dx) <= abs(dy)) {
				if(quadrant == 1 || quadrant == 4) {
					if (dy > 0){
						driveDir = -1;
					} else {
						driveDir = 1;
					}
				} else if (quadrant == 2 || quadrant == 3) {
					if (dy > 0) {
						driveDir = 1;
					} else {
						driveDir = -1;
					}
				}
			} else {
				if(quadrant == 1 || quadrant == 4) {
					if (dx > 0){
						driveDir = -1;
					} else {
						driveDir = 1;
					}
				} else if (quadrant == 2 || quadrant == 3) {
					if (dx > 0) {
						driveDir = 1;
					} else {
						driveDir = -1;
					}
				}
			}

			if(dy != 0) {
				error = atan(dx/dy)*180/M_PI - this->position[2];
				double errorTwo = 0;

				if(dx >= 0 && dy >= 0) {
					if(this->position[2] > 0) {
						errorTwo = error;
					} else {
						errorTwo = -360 + error;
					}
				} else if(dx > 0 && dy < 0) {
					if(this->position[2] > 0) {
						errorTwo = 180 + error;
					} else {
						errorTwo = -180 + error;
					}
				} else if(dx < 0 && dy < 0) {
					if(this->position[2] > 0) {
						errorTwo = 180 + error;
					} else {
						errorTwo = -180 + error;
					}
				} else if (dx < 0 && dy > 0) {
					if(this->position[2] < 0) {
						errorTwo = error;
					} else { 
						errorTwo = 360 + error;
					}
				}

				if(abs(this->position[2] - error) > abs(this->position[2] - errorTwo)) {
					error = errorTwo;
				}
			} else {
				error = 0;
			}

			double dist = sqrt(dx * dx + dy * dy);


			double proportional = kP * dist;
			this->integral += kI * dist * 0.001;
			double derivative = kD * (dist - oldDist) * 0.001 * -1;

			if(this->stop && this->positioned) {
				if(!this->curving) {
					this->motorSpeed = 0;
				}
				integral = 0;
			} else {
				this->percent = (dist/this->originalDistance);
				if(this->turnSwitch) {
					this->updatingError = error;
				} else {
					this->updatingError = 0;
				}

				if(this->tracking && !this->curving) {
					this->motorSpeed = proportional + integral + derivative - (fmin((error/10 * this->percent), 0) * int(this->turnSwitch));
					if(this->motorSpeed > this->maxSpeed) {
						this->motorSpeed = this->maxSpeed;
					}
					this->motorSpeed *= driveDir * -1;
				}
			}

			if(abs(this->position[0] - this->x) < this->xError && abs(this->position[1] - this->y) < this->yError) {
				this->positioned = true;
			} else {
				this->positioned = false;
			}
		}
		void drive() {
			if(this->odomOn) {
				right_mg.move_voltage(this->motorSpeed - this->turnSpeed);
				left_mg.move_voltage(this->motorSpeed + this->turnSpeed);
			}
		}
		void mode(bool on, bool turnSwitch = true) {
			this->odomOn = on;
			this->turnSwitch = turnSwitch;
		}
};


odom* odomPod = new odom();


void odom_task_fn() {
    while (true) {
		odomPod->changeOdomPosition();
		delay(10);
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
	lcd::print(0, "code");
	right_mg.set_encoder_units_all(E_MOTOR_ENCODER_DEGREES);
	left_mg.set_encoder_units_all(E_MOTOR_ENCODER_DEGREES);
	inertialSensor.reset();
	while(inertialSensor.is_calibrating()) {
		delay(10);
	}
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
void competition_initialize() {}

/*While the odom is off, this code will move the bot forward or backward
  depending on a value between -1 and 1 using the autonomous function 
  by setting the voltage of the motors to a constant speed.
*/
void passiveMove(int moveDirection){
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
	if(moveDirection == 2) {
		right_mg.move_voltage(-7000);
		left_mg.move_voltage(7000);
	}if(moveDirection == -2) {
		right_mg.move_voltage(7000);
		left_mg.move_voltage(-7000);
	}
	if(moveDirection == 3){
		right_mg.move_voltage(3000);
		left_mg.move_voltage(3000);
	}
	
}

/*while the odom is off, this code will move intake orbs or extake orbs
  depending on a value between -1 and 1 using the autonomous function 
  by setting the velocity of the intake to a constant speed.
*/
void passiveInExtake(int param){
	if(param == 1){
		intake.move_velocity(400);
	}
	if(param == 0){
		intake.move_velocity(0);
	}
	if(param == -1){
		intake.move_velocity(-400);
	}
}
void onAuton1() {
	right_mg.set_brake_mode_all(E_MOTOR_BRAKE_BRAKE);
	left_mg.set_brake_mode_all(E_MOTOR_BRAKE_BRAKE);
	delay(10);


	odomPod->resetOdom();
	odomPod->changeBotPosition(0, 10, 0, 1, 1, true, true);


	delay(1000);
	right_mg.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
	left_mg.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
}

void startDrive() {
	while(true) {
		odomPod->drive();
		delay(1);
	}
}

void startTurn() {
	while(true) {
		odomPod->turning();
		delay(1);
	}
}

void startMove() {
	while(true) {
		odomPod->moveTo();
		delay(1);
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
	Task odomTask(odom_task_fn);
	Task odomTask1(startDrive);
	Task odomTask2(startMove);
	Task odomTask3(startTurn);
	onAuton1();
	while(is_autonomous()) {
		delay(10);
	}
	odomTask.remove();
	odomTask1.remove();
	odomTask2.remove();
	odomTask3.remove();
}

void movePistons() {
	while(true) {
		if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
			scraperMech.toggle();
		}
		if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
			topPiston.toggle();
		}
		if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)){
			wingPiston.toggle();
		}
		delay(10);
	}
}


// This method is how our bot moves during driver control from the sticks
// Brody (backup driver) was used to one-stick, but after practice, he got used to and liked two stick better
void moveMotors() {
	while(true) {
		double rightStickValueX = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
		double rightStickValueY = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);
		double leftStickValueX = master.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
		double leftStickValueY = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);

		right_mg.move(leftStickValueY - rightStickValueX);
		left_mg.move(leftStickValueY + rightStickValueX);

		// To prevent the program from crashing
		pros::Task::delay(10);
	}
}



/**
 * This allows moving the drivetrain to be asynchronous with moving the intake
 * by using two different tasks in the opcontrol() function
 */
void moveIntake() {
	while(true) {
		if(master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
			intake.move_velocity(200);
		} else if(master.get_digital(E_CONTROLLER_DIGITAL_L2)) {
			intake.move_velocity(-200);
			thirdStageIntake.move_velocity(200);
		} else if(master.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			intake.move_velocity(200);
			topPiston.retract();
		} else if(master.get_digital(E_CONTROLLER_DIGITAL_R2)) {
			thirdStageIntake.move_velocity(-200);
			intake.move_velocity(200);
			topPiston.extend();
		} else {
			intake.move_velocity(0);
			thirdStageIntake.move_velocity(0);
			topPiston.extend();
		}
		pros::Task::delay(20);
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
	Task movingDT(moveMotors);
	Task movingIntake(moveIntake);
	Task pneuTask(movePistons);
}
