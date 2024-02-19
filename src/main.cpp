#include "techLib.h"
#include "lemlib/api.hpp"

pros::Controller master(CONTROLLER_MASTER);

pros::Motor lF(-LEFT_MTR_F, pros::E_MOTOR_GEARSET_06); // left front motor. reversed
pros::Motor lM(-LEFT_MTR_M, pros::E_MOTOR_GEARSET_06); // left middle motor. reversed
pros::Motor lB(-LEFT_MTR_B, pros::E_MOTOR_GEARSET_06); // left back motor. reversed
pros::Motor rF(RIGHT_MTR_F, pros::E_MOTOR_GEARSET_06); // right front motor
pros::Motor rM(RIGHT_MTR_M, pros::E_MOTOR_GEARSET_06); // right middle motor
pros::Motor rB(RIGHT_MTR_B, pros::E_MOTOR_GEARSET_06); // right back motor

pros::Motor intake(INTAKE, pros::E_MOTOR_GEARSET_18);	  // intake motor
pros::Motor catapult(CATAPULT, pros::E_MOTOR_GEARSET_36); // catapult motor

pros::MotorGroup leftMotors({lF, lM, lB});	// left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group

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

enum class autonState
{
	off,
	closeSide, // close side is the side where the opponents goal / offensive zone is
	farSide,   // far side is the side where the ally / team goal / offensive zone is
	skills
};
autonState autonSelection = autonState::off;

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
	else if (motorName == "intake")
	{
		if (brakeMode == "coast")
		{
			intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
		else if (brakeMode == "brake")
		{
			intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		}
		else if (brakeMode == "hold")
		{
			intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		}
	}
	else if (motorName == "catapult")
	{
		if (brakeMode == "coast")
		{
			catapult.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
		else if (brakeMode == "brake")
		{
			catapult.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		}
		else if (brakeMode == "hold")
		{
			catapult.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
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
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
	// chassis->stop();
	// intake.moveVelocity(0);
	// catapult.moveVelocity(0);
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
	setBrakeModeOf("intake", "coast");
	setBrakeModeOf("catapult", "hold");
	auto startTime = std::chrono::high_resolution_clock::now();

	if (autonSelection == autonState::off)
		autonSelection = autonState::skills; // use testing if we havent selected an auton

	switch (autonSelection)
	{
	case autonState::closeSide:
		// opponent goalside auton

		break;

	case autonState::farSide:
		// ally goalside auton

		break;

	case autonState::skills:

		auto markTime = std::chrono::high_resolution_clock::now();

		// Launch the catapult and measure time
		// catapult.moveVelocity(100);

		// Loop for 40 seconds from the mark
		while (std::chrono::duration_cast<std::chrono::seconds>(
				   std::chrono::high_resolution_clock::now() - markTime)
				   .count() < 35)
		{
		}

		// Stop the catapult
		// catapult.moveVelocity(0);

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

	bool wingToggle = false;

	bool leftWingState = false;

	bool rightWingState = false;

	while (true)
	{
		setBrakeModeOf("chassis", "coast");
		leftMotors.move(master.get_analog(ANALOG_LEFT_Y));
		rightMotors.move(master.get_analog(ANALOG_RIGHT_Y));

		if (master.get_digital_new_press(DIGITAL_L1))
		{ // Left wing
			leftWingState = !leftWingState;
			wingToggle = false; // Reset wingToggle if an individual wing is toggled
		}
		else if (master.get_digital_new_press(DIGITAL_R1))
		{ // Right wing
			rightWingState = !rightWingState;
			wingToggle = false;

		}

		if (master.get_digital_new_press(DIGITAL_L2))
		{ // Both wings
			wingToggle = !wingToggle;
		}

		if (wingToggle)
		{
			leftWingState = true;
			rightWingState = true;
		} 
		else if (!wingToggle && !leftWingState && !rightWingState)
		{
			leftWingState = false;
			rightWingState = false;
		}

		leftWing.set_value(leftWingState);
		rightWing.set_value(rightWingState);

		pros::delay(10); // Run for 20 ms then update
	}
}