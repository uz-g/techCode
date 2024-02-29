#include "techLib.h"

using namespace lemlib;

pros::Controller master(CONTROLLER_MASTER);

pros::Motor lF(-LEFT_MTR_F,
               pros::E_MOTOR_GEARSET_06); // left front motor. reversed
pros::Motor lM(-LEFT_MTR_M,
               pros::E_MOTOR_GEARSET_06); // left middle motor. reversed
pros::Motor lB(-LEFT_MTR_B,
               pros::E_MOTOR_GEARSET_06); // left back motor. reversed
pros::Motor rF(RIGHT_MTR_F, pros::E_MOTOR_GEARSET_06); // right front motor
pros::Motor rM(RIGHT_MTR_M, pros::E_MOTOR_GEARSET_06); // right middle motor
pros::Motor rB(RIGHT_MTR_B, pros::E_MOTOR_GEARSET_06); // right back motor



pros::Motor intake(-INTAKE, pros::E_MOTOR_GEARSET_36);
pros::Motor hang(HANG, pros::E_MOTOR_GEARSET_06);

pros::MotorGroup leftMotors({lF, lM, lB});  // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group


// sensors
pros::Imu inertial(INERTIAL);               // inertial sensor
pros::Rotation rotationV(ROTATION_V);       // left rotation sensor
pros::Rotation rotationH(ROTATION_H, true); // middle rotation sensor, reversed

// pneumatics
pros::ADIDigitalOut wings('H', false);
pros::ADIDigitalOut hangP('A', false);

lemlib::TrackingWheel verticalWheel(&rotationV, 2.75, 4.3, 1);

lemlib::TrackingWheel horizontalWheel(&rotationH, 2.75, 4.3, 1);

// drivetrain
lemlib::Drivetrain drivetrain{
    &leftMotors,  // left drivetrain motors
    &rightMotors, // right drivetrain motors
    9.9,          // track width
    3.25,         // wheel diameter
    360,          // wheel rpm
    0.5,          // max chase

};

// sensors for odometry
lemlib::OdomSensors sensors{nullptr,    // vertical tracking wheel 1
                            nullptr,    // vertical tracking wheel 2
                            nullptr,    // horizontal tracking wheel
                            nullptr,    // horizontal tracking wheel 2
                            &inertial}; // inertial sensor [imu]

// conntrollerSettings requiremtns:
/*
@param
                 * @param kP proportional constant for the chassis controller
                 * @param kI integral constant for the chassis controller
                 * @param kD derivative constant for the chassis controller
                 * @param antiWindup
                 * @param smallError the error at which the chassis controller
will switch to a slower control loop
                 * @param smallErrorTimeout the time the chassis controller will
wait before switching to a slower control loop
                 * @param largeError the error at which the chassis controller
will switch to a faster control loop
                 * @param largeErrorTimeout the time the chassis controller will
wait before switching to a faster control loop
                 * @param slew the maximum acceleration of the chassis
controller
*/
// forward/backward PID
lemlib::ControllerSettings lateralController{
    8.000,  // kP
    0.000,  // kI
    30.000, // kD
    0.000,  // antiWindup
    1,      // smallErrorRange
    100,    // smallErrorTimeout
    3,      // largeErrorRange
    500,    // largeErrorTimeout
    5       // slew rate
};

// turning PID
lemlib::ControllerSettings angularController{
    8.000,  // kP
    0.000,  // kI
    30.000, // kD
    0.000,  // antiWindup
    1,      // smallErrorRange
    100,    // smallErrorTimeout
    3,      // largeErrorRange
    500,    // largeErrorTimeout
    5       // slew rate
};

lemlib::Chassis chassis(drivetrain, lateralController, angularController,
                        sensors);

// auto selection [the state the the auton will be in]
enum class autonState {
  off,
  closeSide, // close side is the side where the opponents goal / offensive zone
             // is
  farSide,   // far side is the side where the ally / team goal / offensive zone
             // is
  skills
};
autonState autonSelection = autonState::off;

static const char *btnmMap[] = {"far side", "close side", "skills",
                                ""}; // button matrix map for auton selection

static lv_res_t
autonBtnmAction(lv_obj_t *btnm,
                const char *txt) // button matrix action for auton selection
{
  if (lv_obj_get_free_num(btnm) == 100) {
    if (std::string(txt) == "far side") {
      master.rumble("._");

      autonSelection = autonState::farSide;
    } else if (std::string(txt) == "close side") { // original working code was
                                                   // : txt == "close side"
      master.rumble(".. _");

      autonSelection = autonState::closeSide;
    } else if (std::string(txt) == "skills") {
      master.rumble("_._");

      autonSelection = autonState::skills;
    }
  }

  return LV_RES_OK; // return OK because the button matrix is not deleted
}

// make a function to set the brake mode of the motors by requesting the name of
// motors and the mode of the brake,
//  when requesting this stuff use strings and if statements
void setBrakeModeOf(std::string motorName, std::string brakeMode) {
  if (motorName == "left") {
    if (brakeMode == "coast") {
      lF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      lM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      lB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    } else if (brakeMode == "brake") {
      lF.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      lM.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      lB.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    } else if (brakeMode == "hold") {
      lF.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      lM.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      lB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }
  } else if (motorName == "right") {
    if (brakeMode == "coast") {
      rF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      rM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      rB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    } else if (brakeMode == "brake") {
      rF.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      rM.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      rB.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    } else if (brakeMode == "hold") {
      rF.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      rM.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      rB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }
  } else if (motorName == "chassis") {
    if (brakeMode == "coast") {
      lF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      lM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      lB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      rF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      rM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      rB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    } else if (brakeMode == "brake") {
      lF.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      lM.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      lB.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      rF.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      rM.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      rB.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    } else if (brakeMode == "hold") {
      lF.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      lM.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      lB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      rF.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      rM.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      rB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
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

void initialize() {

  hangP.set_value(true);

  // djmango korvex gui stuff

  lv_theme_t *th = lv_theme_alien_init(
      300, NULL); // Set a HUE value and keep font default MAGENTA
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

  chassis.calibrate(); // calibrate the chassis and imu stuff
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  leftMotors.move_velocity(0);
  rightMotors.move_velocity(0);
  intake.move_velocity(0);
  hang.move_velocity(0);
  wings.set_value(false);
  hangP.set_value(true);
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
  setBrakeModeOf("chassis", "hold");
  setBrakeModeOf("catapult", "hold");
  auto startTime = std::chrono::high_resolution_clock::now();

  if (autonSelection == autonState::off)
    autonSelection =
        autonState::skills; // use skills [the main focus] if we havent selected
                            // an auton just in case

  switch (autonSelection) {
  case autonState::closeSide:
    // opponent goalside auton

    chassis.setPose(0, 0, 0); // starting position

    chassis.moveToPoint(0, 10, 1000,
                        127); // lateral controller pid tuning command

    // chassis.turnTo(0, 30, 1000, 127); //angular controller pid tuning command

    break;

  case autonState::farSide:
    // ally goalside auton
    chassis.setPose(20, -67, 90);
    chassis.moveToPose(11, -67, 90, 2000);
    intake.move_relative(720, 600);
    chassis.moveToPose(33, -65, 90, 2000);
    chassis.turnTo(54, -26, 2000, false);
    chassis.moveToPose(52, -35, 180, 700);
    chassis.moveToPose(50, -30, 0, 700);
    chassis.moveToPose(52, -33, 0, 700);
    intake.move_velocity(-600);
    chassis.moveToPose(52, -35, 0, 700);
    intake.move_velocity(0);
    chassis.moveToPose(52, -26, 90, 700);
    chassis.turnTo(13, -37, 1000);
    intake.move_velocity(600);
    chassis.moveToPoint(13, -37, 1000);
    intake.move_velocity(0);



    break;

  case autonState::off:
    break;

  case autonState::skills:

    chassis.setPose(-48, -72, 0); // starting position

    chassis.moveToPose(-42, -40, 0, 1000); // drive forward

    chassis.turnTo(-70, -28, 1000, true, 75); // turn to the left and backwards

    chassis.moveToPose(-70, -28, 0, 1000); // drive forward

    wings.set_value(true);             // hold on to matchload bar
    setBrakeModeOf("chassis", "hold"); // hold the catapult
    chassis.turnTo(28, -24, 1000, 75); // turn to the left

    auto markTime = std::chrono::high_resolution_clock::now();

    // Loop for 40 seconds from the mark
    while (std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::high_resolution_clock::now() - markTime)
               .count() < 35) {
      intake.move_velocity(100); // Start the catapult for time
    }

    // Stop the catapult
    intake.move_velocity(0);

    wings.set_value(false); // release matchload bar

    chassis.turnTo(-28, -55, 3000, true, 75);
    chassis.moveToPoint(-28, -55, 3000, true, 100);
    chassis.turnTo(32, -51, 3000, true, 75);
    chassis.moveToPoint(32, -51, 3000, true, 75);
    chassis.turnTo(50, -36, 3000, true, 75);
    chassis.moveToPoint(50, -36, 3000, true, 100);
    chassis.moveToPoint(50, -28, 3000, true, 100);
    chassis.turnTo(7, -34, 3000, false, 75);
    chassis.moveToPoint(7, -34, 3000, false, 100);
    chassis.turnTo(36, -9, 3000, true, 75);
    wings.set_value(true);
    chassis.moveToPoint(36, -9, 3000, true, 100);
    wings.set_value(false);
    chassis.turnTo(3, -4, 3000, false, 75);
    chassis.moveToPoint(3, -4, 3000, false, 100);
    chassis.turnTo(40, 15, 3000, true, 75);
    wings.set_value(true);
    chassis.moveToPoint(41, 15, 3000, false, 100);
    wings.set_value(false);
    chassis.moveToPoint(24, 19, 3000, true, 100);

    chassis.turnTo(39, 55, 3000, true, 75);
    chassis.moveToPoint(39, 55, 3000, true, 100);

    chassis.turnTo(48, 32, 3000, true, 75);
    chassis.moveToPoint(48, 32, 3000, true, 100);

    chassis.turnTo(9, 41, 3000, true, 75);
    chassis.moveToPoint(9, 41, 3000, true, 100);

    break;
  }

  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - startTime);

  std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
                   .count()
            << ": auton took " << elapsedTime.count() << " milliseconds"
            << std::endl;
  disabled();
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
  setBrakeModeOf("chassis", "coast");

  hang.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  master.rumble(".");
  // new timer
  auto startTime = std::chrono::high_resolution_clock::now();
  bool reversed = false;

  bool WingState = false;

  bool intakeToggled = false;

  bool launcherToggle = false;

  bool hangState = false;

  while (true) {

    // chassis controls
    chassis.tank(ANALOG_LEFT_Y, ANALOG_RIGHT_Y, 10);

    // wing controls
    if (master.get_digital_new_press(DIGITAL_A)) {
      WingState = !WingState;
      wings.set_value(WingState);
    }
    // intake controls
    if (master.get_digital(DIGITAL_R2)) {
      intake.move_velocity(600);
    } else if (master.get_digital(DIGITAL_DOWN)) {
      intake.move_velocity(-600);
    } else if (master.get_digital_new_press(DIGITAL_UP)) {
      intakeToggled = !intakeToggled;
    } else if (intakeToggled) {
      intake.move_velocity(600);
    } else if (!intakeToggled) {
      intake.move_velocity(0);
    }

    // hang controls
    if (master.get_digital(DIGITAL_L1)) {
      hang.move_velocity(100);
    } else if (master.get_digital(DIGITAL_L2)) {
      hang.move_velocity(-100);
    } else {
      hang.move_velocity(0);
    }

    if (master.get_digital_new_press(DIGITAL_B)) {
      hangState = !hangState;
      hangP.set_value(hangState);
    }

    // // catapult controls
    // if (master.get_digital(DIGITAL_A)) {
    //   setBrakeModeOf("catapult", "hold");

    //   catapult.move_velocity(47);
    // } else if (master.get_digital_new_press(DIGITAL_Y)) {
    //   launcherToggle = !launcherToggle;
    // } else if (launcherToggle) {
    //   setBrakeModeOf("catapult", "hold");
    //   setBrakeModeOf("chassis",
    //                  "hold"); // hold the chassis im place when matchloading
    //                  to
    //                           // prevent shock and to prevent pushing
    //   catapult.move_velocity(47);
    // } else if (!launcherToggle) {
    //   catapult.move_velocity(0);
    //   setBrakeModeOf("chassis", "coast");
    //   setBrakeModeOf("catapult", "coast");
    // }

    // if (master.get_digital_new_press(DIGITAL_B)) {
    //   setBrakeModeOf("catapult", "coast");
    // }

    // timer stuff

    if (std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::high_resolution_clock::now() - startTime)
            .count() >= 75) {
      master.set_text(0, 0, "30s left");
    } else if (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - startTime)
                   .count() >= 105) {
      master.set_text(0, 0, "15s left");
    }

    pros::delay(10); // run this stuff than wait 20 ms
  }
}