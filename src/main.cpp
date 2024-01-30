#include "main.h"

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
	Motor leftFront(leftFrontPort, leftFrontGearset, leftFrontReversed, leftFrontEncoder);
    Motor leftMid(leftMidPort, leftMidGearset, leftMidReversed, leftMidEncoder);
    Motor leftBack(leftBackPort, leftBackGearset, leftBackReversed, leftBackEncoder);
    Motor rightFront(rightFrontPort, rightFrontGearset, rightFrontReversed, rightFrontEncoder);
    Motor rightMid(rightMidPort, rightMidGearset, rightMidReversed, rightMidEncoder);
    Motor rightBack(rightBackPort, rightBackGearset, rightBackReversed, rightBackEncoder);
	Motor intake(intakePort, MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);
	Motor cata(cataPort, MOTOR_GEAR_GREEN, false, MOTOR_ENCODER_DEGREES);
	ADIDigitalOut wingLeft(wingLeftPort, false);
	ADIDigitalOut wingRight(wingRightPort, false);
    Imu inertial(inertialPort);
	Controller master(CONTROLLER_MASTER);

	Task cataPIDTask(cataPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Cata PID Task");
	inertial.reset();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	//matchload()
	ADIDigitalOut wingRight(wingRightPort, false);
	wingRight.set_value(true);

	// //balls()
}

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
	matchload();
	// balls();
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
	Motor leftFront(leftFrontPort, leftFrontReversed);
    Motor leftMid(leftMidPort, leftMidReversed);
    Motor leftBack(leftBackPort, leftBackReversed);
    Motor rightFront(rightFrontPort, rightFrontReversed);
    Motor rightMid(rightMidPort, rightMidReversed);
    Motor rightBack(rightBackPort, rightBackReversed);
	Motor intake(intakePort, false);
	Motor cata(cataPort, false);
	ADIDigitalOut wingLeft(wingLeftPort, false);
	ADIDigitalOut wingRight(wingRightPort, false);
    Imu inertial(inertialPort);
	Controller master(CONTROLLER_MASTER);

	bool invert = true, wingLeftState = false, wingRightState = false;
	double left, right;

	leftFront.set_brake_mode(MOTOR_BRAKE_HOLD);
	leftMid.set_brake_mode(MOTOR_BRAKE_HOLD);
	leftBack.set_brake_mode(MOTOR_BRAKE_HOLD);
	rightFront.set_brake_mode(MOTOR_BRAKE_HOLD);
	rightMid.set_brake_mode(MOTOR_BRAKE_HOLD);
	rightBack.set_brake_mode(MOTOR_BRAKE_HOLD);

    while (true) {
		left = 1.27*master.get_analog(ANALOG_LEFT_Y);
		right = 1.27*master.get_analog(ANALOG_RIGHT_Y);

		if(invert){
			leftFront.move(-right);
			leftMid.move(-right);
			leftBack.move(-right);
			rightFront.move(-left);
			rightMid.move(-left);
			rightBack.move(-left);
		} else {
			leftFront.move(left);
			leftMid.move(left);
			leftBack.move(left);
			rightFront.move(right);
			rightMid.move(right);
			rightBack.move(right);
		}

		if(master.get_digital(DIGITAL_L1)){
			intake.move(-127);
		} else if(master.get_digital(DIGITAL_R1)){
			intake.move(127);
		} else {
			intake.move(0);
		}

		//wing buttons are swapped because the wings are backwards
		if(master.get_digital_new_press(DIGITAL_R2)){
			wingLeftState = !wingLeftState;
			wingLeft.set_value(wingLeftState);
		}

		if(master.get_digital_new_press(DIGITAL_L2)){
			wingRightState = !wingRightState;
			wingRight.set_value(wingRightState);
		}

		if(master.get_digital_new_press(DIGITAL_X)){
			shoot();
		}

		if(master.get_digital_new_press(DIGITAL_UP)){
			invert = !invert;
		}
        delay(20);
    }
}
