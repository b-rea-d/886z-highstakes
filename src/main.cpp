#include "main.h"
#include <algorithm>
#include <cstddef>
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

Motor Lf(-12); // number (06) is the cartridge
Motor Lf2 (14);
Motor Rf2 (-20);
Motor Lb(-17);
Motor Rf(18);
Motor Rb(19);
Motor conveyer(6);

Motor intake(11);
Motor armR(14);
Motor armL(13);
Controller master(E_CONTROLLER_MASTER);
adi::Pneumatics Hang ('2', true);

// sensor ports
Rotation odomVerticalPod(18);
Rotation odomHorizontalPod(19);
Imu imu(16);
Optical distance(20);
adi::Pneumatics intup('4', true);

// motor groups
pros::MotorGroup left_side_motors({
    -12,
    -17,
    14,
});
pros::MotorGroup right_side_motors({
    18,
    19,
    -20, 
});

// Lemlib drivetrain  struct

lemlib::Drivetrain drivetrain{
    &left_side_motors,  // left drrivetrain motors
    &right_side_motors, // right train motors
    10,                 // track width in INCHES
    3.25,               // wheel diameter
    360,                // wheel rpm
    8                   // tune this value later : )

};

// define odom pod
lemlib::TrackingWheel vertical_tracking_wheel(&odomVerticalPod, 2.75, 4.3, 1);
lemlib::TrackingWheel horizontal_tracking_wheel(&odomHorizontalPod, 2.75, 2.5,
                                                1);
// odom struct
lemlib::OdomSensors sensors{&vertical_tracking_wheel, nullptr,
                            &horizontal_tracking_wheel, nullptr, &imu

};

// forward/backward PID
lemlib::ControllerSettings lateralController{
    10,  // kP
    0,   // KI
    55,   // kD
    0,   // windup
    1,   // smallErrorRange
    100, // smallErrorTimeout
    3,   // largeErrorRange
    250, // largeErrorTimeout
    5    // slew rate
};

// turning PID
lemlib::ControllerSettings angularController{

    1.5, // kP (1.6)
    0,
    0.5, // kD (1)
    0,
    2,   // smallErrorRange
    100, // smallErrorTimeout
    3,   // largeErrorRange
    250, // largeErrorTimeout
    5    // slew rate
};

// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController,
                        sensors);
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
void autonomous() {}

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
  bool wingPosition = false;
  bool HangPosition = false;
  bool AWPPosition = false;
  bool hangState = false;
  bool mogoState = false;
  bool intupState = false;
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(25);
    



    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      armR.move(-127);
      armL.move(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      armR.move(127);
      armL.move(-127);
    } else {
      armR.move(0);
      armR.move(0);
      armL.move(0);
    }
    // armR.move(-master.get_analog(ANALOG_RIGHT_Y));

    // armL.move(master.get_analog(ANALOG_RIGHT_Y));

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake.move(-127);
      conveyer.move(-100);

    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake.move(127);
      conveyer.move(100);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      distance.get_proximity();

      if (distance.get_proximity() < 230){ 
        intake.move(-70); 
        }
      else {
        intake.move(-0);
      }

    }

    else {
      intake.move(0);
      conveyer.move(0);
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      mogoState = !mogoState;
      mogo.set_value(mogoState);
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      intupState = !intupState;
      intup.set_value(intupState);
    }

    if (master.get_digital_new_press (pros::E_CONTROLLER_DIGITAL_X)){
        hangState = !hangState;
        hang.set_value(hangState);
		}
}
}

