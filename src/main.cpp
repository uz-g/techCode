#include "techLib.h"

pros::Controller master(CONTROLLER_MASTER);

pros::Motor lF(-LEFT_MTR_F, pros::E_MOTOR_GEARSET_06); // left front motor. reversed
pros::Motor lM(-LEFT_MTR_M, pros::E_MOTOR_GEARSET_06); // left middle motor. reversed
pros::Motor lB(-LEFT_MTR_B, pros::E_MOTOR_GEARSET_06); // left back motor. reversed
pros::Motor rF(RIGHT_MTR_F, pros::E_MOTOR_GEARSET_06); // right front motor
pros::Motor rM(RIGHT_MTR_M, pros::E_MOTOR_GEARSET_06); // right middle motor
pros::Motor rB(RIGHT_MTR_B, pros::E_MOTOR_GEARSET_06); // right back motor

pros::Motor cataR(-CATA_R, pros::E_MOTOR_GEARSET_36);	  // cata motor
pros::Motor cataF(CATA_F, pros::E_MOTOR_GEARSET_36); // catapult motor

pros::MotorGroup leftMotors({lF, lM, lB});	// left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group

pros::MotorGroup catapult({cataF, cataR}); // catapult motor group

// sensors
pros::Imu inertial(INERTIAL);				// inertial sensor
pros::Rotation rotationV(ROTATION_V);		// left rotation sensor
pros::Rotation rotationH(ROTATION_H, true); // middle rotation sensor, reversed

// pneumatics
pros::ADIDigitalOut leftWing('A', false);  // piston
pros::ADIDigitalOut rightWing('H', false); // piston

lemlib::TrackingWheel verticalWheel(&rotationV, 2.75, 4.3, 1);

lemlib::TrackingWheel horizontalWheel(&rotationH, 2.75, 4.3, 1);

// drivetrain
lemlib::Drivetrain_t drivetrain{
	&leftMotors,  // left drivetrain motors
	&rightMotors, // right drivetrain motors
	12.33,		  // track width
	3.25,		  // wheel diameter
	360			  // wheel rpm
};

// sensors for odometry
lemlib::OdomSensors_t sensors{
	&verticalWheel,
	nullptr,
	&horizontalWheel,
	nullptr,
	&inertial};

// forward/backward PID
lemlib::ChassisController_t lateralController{
	8,	 // kP
	30,	 // kD
	1,	 // smallErrorRange
	100, // smallErrorTimeout
	3,	 // largeErrorRange
	500, // largeErrorTimeout
	5	 // slew rate
};

// turning PID
lemlib::ChassisController_t angularController{
	4,	 // kP
	40,	 // kD
	1,	 // smallErrorRange
	100, // smallErrorTimeout
	3,	 // largeErrorRange
	500, // largeErrorTimeout
	40	 // slew rate
};

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

//drive curves
int forwardCurve = 14.6; // Curve strength

// Function to calculate the curve for tank drive
int curveTankDrive(bool red, int input, double t) {
    if (red) {
        // Red curve calculation
        // Modify this calculation as needed
        return (std::exp(-t / 10) + std::exp((std::abs(input) - 127) / 10) * (1 - std::exp(-t / 10))) * input; // Default behavior if red curve is specified
    } else {
        // Blue curve calculation
        return std::exp(((std::abs(input) - 127) * t) / 1000) * input;
    }
}		

// Function to drive the robot using tank drive with curves
void driveTankWithCurve(int leftInput, int rightInput) {
    int leftVal = curveTankDrive(true, leftInput, forwardCurve);
    int rightVal = curveTankDrive(true, rightInput, forwardCurve);

    // Apply the calculated values to the left and right motors
    leftMotors.move(leftVal);
    rightMotors.move(rightVal);
}

void driveTankWithCurveReverse(int leftInput, int rightInput) {
	int leftVal = curveTankDrive(true, leftInput, forwardCurve);
    int rightVal = curveTankDrive(true, rightInput, forwardCurve);

    // Apply the calculated values to the left and right motors
    leftMotors.move(-rightVal);
    rightMotors.move(-leftVal);
}
//auto selection [the state the the auton will be in]
enum class autonState
{
	off,
	closeSide, // close side is the side where the opponents goal / offensive zone is
	farSide,   // far side is the side where the ally / team goal / offensive zone is
	skills
};
autonState autonSelection = autonState::off;

static const char *btnmMap[] = {"far side", "close side", "skills", ""}; // button matrix map for auton selection

static lv_res_t autonBtnmAction(lv_obj_t *btnm, const char *txt) // button matrix action for auton selection
{
	if (lv_obj_get_free_num(btnm) == 100)
	{ // reds
		if (txt == "far side")
		{
			master.rumble(". _");

			autonSelection = autonState::closeSide;
		}
		else if (txt == "close side")
		{
			master.rumble(".. _");

			autonSelection = autonState::farSide;
		}
		else if (txt == "skills")
		{
			master.rumble("_._");

			autonSelection = autonState::skills;
		}
	}

	return LV_RES_OK; // return OK because the button matrix is not deleted
}

// make a function to set the brake mode of the motors by requesting the name of motors and the mode of the brake,
//  when requesting this stuff use strings and if statements
void setBrakeModeOf(std::string motorName, std::string brakeMode)
{
	if (motorName == "left")
	{
		if (brakeMode == "coast")
		{
			lF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			lM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			lB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
		else if (brakeMode == "brake")
		{
			lF.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			lM.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			lB.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		}
		else if (brakeMode == "hold")
		{
			lF.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			lM.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			lB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		}
	}
	else if (motorName == "right")
	{
		if (brakeMode == "coast")
		{
			rF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			rM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			rB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
		else if (brakeMode == "brake")
		{
			rF.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			rM.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			rB.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		}
		else if (brakeMode == "hold")
		{
			rF.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			rM.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			rB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		}
	}
	else if (motorName == "chassis")
	{
		if (brakeMode == "coast")
		{
			lF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			lM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			lB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			rF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			rM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			rB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
		else if (brakeMode == "brake")
		{
			lF.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			lM.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			lB.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			rF.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			rM.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			rB.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		}
		else if (brakeMode == "hold")
		{
			lF.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			lM.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			lB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			rF.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			rM.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			rB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		}
	}
	else if (motorName == "cataR")
	{
		if (brakeMode == "coast")
		{
			cataR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
		else if (brakeMode == "brake")
		{
			cataR.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		}
		else if (brakeMode == "hold")
		{
			cataR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		}
	}
	else if (motorName == "cataF")
	{
		if (brakeMode == "coast")
		{
			cataF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
		else if (brakeMode == "brake")
		{
			cataF.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		}
		else if (brakeMode == "hold")
		{
			cataF.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		}
	} else if (motorName == "catapult") {
		if (brakeMode == "coast")
		{
			cataF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			cataR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
		else if (brakeMode == "brake")
		{
			cataF.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			cataR.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		}
		else if (brakeMode == "hold")
		{
			cataF.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			cataR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		}
	}
	
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize()
{
	chassis.calibrate();

	// djmango korvex gui stuff

	lv_theme_t *th = lv_theme_alien_init(300, NULL); // Set a HUE value and keep font default MAGENTA
	lv_theme_set_current(th);

	// create a tab view object
	std::cout << pros::millis() << ": creating gui..." << std::endl;
	lv_obj_t *tabview = lv_tabview_create(lv_scr_act(), NULL);

	// add 4 tabs (the tabs are page (lv_page) and can be scrolled
	lv_obj_t *mainTab = lv_tabview_add_tab(tabview, "Autons");

	// main tab
	lv_obj_t *mainBtnm = lv_btnm_create(mainTab, NULL);
	lv_btnm_set_map(mainBtnm, btnmMap);
	lv_btnm_set_action(mainBtnm, autonBtnmAction);
	lv_obj_set_size(mainBtnm, 450, 50);
	lv_btnm_set_toggle(mainBtnm, true, 3);
	lv_obj_set_pos(mainBtnm, 0, 100);
	lv_obj_align(mainBtnm, NULL, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_free_num(mainBtnm, 100);
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
	leftMotors.move_velocity(0);
	rightMotors.move_velocity(0);
	cataR.move_velocity(0);
	cataF.move_velocity(0);
	leftWing.set_value(false);
	rightWing.set_value(false);
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
void autonomous()
{
	setBrakeModeOf("chassis", "hold");
	setBrakeModeOf("catapult", "hold");
	auto startTime = std::chrono::high_resolution_clock::now();
	

	if (autonSelection == autonState::off)
		autonSelection = autonState::skills; // use skills [the main focus] if we havent selected an auton just in case

	switch (autonSelection)
	{
	case autonState::closeSide:
		// opponent goalside auton

		chassis.setPose(-48, -72, 0); //starting position
		chassis.moveTo(-48, -50, 1000, 127); //drive forward
		chassis.turnTo(-67, -32, 1000, 50); //turn to the left
		chassis.moveTo(-67, -32, 2000, 127); //drive to the left and push ball in goal
		chassis.turnTo(-67, -26, 1000, true, 50); //go and turn backwards
		chassis.moveTo(-67, -26, 1000, 70);
		chassis.moveTo(-67, -32, 1000, 127); //push ball in goal forwards
		chassis.turnTo(-27, -67, 1000, 50); //turn to the right
		chassis.moveTo(-27, -67, 2000, 70);
		chassis.turnTo(-15, -67, 1000, 127);
		chassis.moveTo(-15, -67, 1000, 127); //touch horizontal bar

		break;

	case autonState::farSide:
		// ally goalside auton
		chassis.setPose(0, 0, 0); //starting position
		
		chassis.moveTo(0, 10, 1000, 127); //lateral controller pid tuning command

		// chassis.turnTo(0, 30, 1000, 127); //angular controller pid tuning command

		break;

	case autonState::skills:

		auto markTime = std::chrono::high_resolution_clock::now();


		// Loop for 40 seconds from the mark
		while (std::chrono::duration_cast<std::chrono::seconds>(
				   std::chrono::high_resolution_clock::now() - markTime)
				   .count() < 35)
		{
			catapult.move_velocity(100); // Start the catapult for time
		}

		// Stop the catapult
		catapult.move_velocity(0);



		break;
	}

	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - startTime);
	std::cout << "Elapsed time: " << elapsedTime.count() << " milliseconds\n";
	// chassis->stop();
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
void opcontrol()
{
	setBrakeModeOf("chassis", "coast");
	setBrakeModeOf("catapult", "hold");
	master.rumble(".");

	bool reversed = false;

	bool leftWingState = false;

	bool rightWingState = false;

	bool intakeToggled = false;

	bool launcherToggle = false;

	while (true)
	{

		if (master.get_digital_new_press(DIGITAL_X))
		{
			reversed = !reversed;
		}

		double leftStick = master.get_analog(ANALOG_LEFT_Y);
		double rightStick = master.get_analog(ANALOG_RIGHT_Y);

		if (reversed)
		{
			driveTankWithCurveReverse(leftStick, rightStick);
		}
		else if (!reversed)
		{
			driveTankWithCurve(leftStick, rightStick);
		}

		// wing controls
		if (master.get_digital_new_press(DIGITAL_L1))
		{ // Left wing
			leftWingState = !leftWingState;
			leftWing.set_value(leftWingState);
		}
		else if (master.get_digital_new_press(DIGITAL_R1))
		{ // Right wing
			rightWingState = !rightWingState;
			rightWing.set_value(rightWingState);
		}

		else if (master.get_digital_new_press(DIGITAL_L2))
		{ // Both wings
			leftWingState = !leftWingState;
			rightWingState = leftWingState;
			leftWing.set_value(leftWingState);
			rightWing.set_value(rightWingState);
		}

		// // intake controls
		// if (master.get_digital(DIGITAL_R2))
		// {
		// 	intake.move_velocity(200);
		// }
		// else if (master.get_digital(DIGITAL_DOWN))
		// {
		// 	intake.move_velocity(-200);
		// }
		// else if (master.get_digital_new_press(DIGITAL_UP))
		// {
		// 	intakeToggled = !intakeToggled;
		// }
		// else if (intakeToggled)
		// {
		// 	intake.move_velocity(200);
		// }
		// else if (!intakeToggled)
		// {
		// 	intake.move_velocity(0);
		// }

		// catapult controls
		if (master.get_digital(DIGITAL_A))
		{
			setBrakeModeOf("catapult", "hold");

			catapult.move_velocity(60);
		}
		else if (master.get_digital_new_press(DIGITAL_Y))
		{
			launcherToggle = !launcherToggle;
		}
		else if (launcherToggle)
		{
			setBrakeModeOf("catapult", "hold");
			catapult.move_velocity(60);
		}
		else if (!launcherToggle)
		{
			catapult.move_velocity(0);
		}

		if (master.get_digital_new_press(DIGITAL_B))
		{
			setBrakeModeOf("catapult", "coast");
		}

		pros::delay(10); // Run for 20 ms then update
	}
}