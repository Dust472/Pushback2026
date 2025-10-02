#include "main.h"

using namespace std;
using namespace pros;

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
		odom(MotorGroup& leftSide, MotorGroup& rightSide) {
			this->leftDT = &leftSide;
			this->rightDT = &rightSide;
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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	/*
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	*/
	
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
	Controller master(E_CONTROLLER_MASTER);
	MotorGroup left_mg({-1, 2, 3});
	MotorGroup right_mg({11, 12, 13});
	odom odomPod(left_mg, right_mg);
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
	Controller master(E_CONTROLLER_MASTER);
	MotorGroup left_mg({-1, 2, 3});
	MotorGroup right_mg({11, 12, 13});  
	
	while (true) {
		/*
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs
		*/
		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    
		int turn = master.get_analog(ANALOG_RIGHT_X);  
		left_mg.move_velocity(-1*(dir + turn));                      
		right_mg.move_velocity(dir + turn);                     
		delay(20);                               
	}
}