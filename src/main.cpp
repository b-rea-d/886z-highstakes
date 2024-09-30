#include "main.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

Motor Lf(14); // number (06) is the cartridge
Motor Lf2 (-13);
Motor Rf2 (-16);
Motor Lb(15);
Motor Rf(-19);
Motor Rb(18);
Motor arm (3);
Motor intake2 (9);
adi::DigitalOut awp (3);
adi::DigitalOut armc (8);
Motor intake(1);
Motor armR(14);
Motor armL(13);
Controller master(E_CONTROLLER_MASTER);
adi::DigitalOut hang (2);
adi::DigitalOut mogo (1);
// sensor ports
Rotation odomVerticalPod(20);
Rotation odomHorizontalPod(5);
Imu imu(10);
Optical distance(20);


// motor groups
pros::MotorGroup left_side_motors({
    14,
    15,
    -13,
});
pros::MotorGroup right_side_motors({
    18,
    -19, 
    -16, 

});

// Lemlib drivetrain  struct

lemlib::Drivetrain drivetrain{
    &left_side_motors,  // left drrivetrain motors
    &right_side_motors, // right train motors
    10,                 // track width in INCHES
    2.75,               // wheel diameter
    450,                // wheel rpm
    8                   // tune this value later : )

};

// define odom pod
lemlib::TrackingWheel vertical_tracking_wheel(&odomVerticalPod, 2.75, 0, 1);
lemlib::TrackingWheel horizontal_tracking_wheel(&odomHorizontalPod, 2, 4,
                                                1);
// odom struct
lemlib::OdomSensors sensors{&vertical_tracking_wheel, nullptr,
                            &horizontal_tracking_wheel, nullptr, &imu

};

// forward/backward PID
lemlib::ControllerSettings lateralController{
    13,  // kP
    0,   // KI
    60,   // kD
    0,   // windup
    1,   // smallErrorRange
    100, // smallErrorTimeout
    3,   // largeErrorRange
    250, // largeErrorTimeout
    5    // slew rate
};

// turning PID
lemlib::ControllerSettings angularController{

    1.09, // kP (1.6)
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

    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
	chassis.setPose(0, 0, 0); //starting position
    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
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
 // awp red :)
 
//  chassis.setPose(-38, -47, 207); 
// chassis.moveToPose(-26, -24, 207, 1000, {false, 8,0, 60, 40}); //move to get goal
//   chassis.waitUntil(23);
//   mogo.set_value(true);
//   chassis.waitUntilDone();
//   chassis.turnToHeading(315, 1000);//turn to face rings
//   intake.move (127);
//   intake2.move (-127);//turn on intake
// chassis.moveToPoint (-35, -31, 1000, {true, 70, 40});//go forward towards rings
// chassis.turnToHeading(280, 1000,{.maxSpeed = 60, .minSpeed = 30});//turn to face stake
// arm.move (-50);
// chassis.moveToPose (-59, -21.5, 272, 5000, {true,8, .1, 60, 20});//move forward to intake
//   chassis.waitUntil(50);
//   intake.move (0);//stop intaking
//   chassis.waitUntilDone();
//   intake.move(127);
//   delay (700);
//   arm.move (50);
//    intake.move (0);
//   intake2.move (0);
//   delay (300);
//   armc.set_value(true);
//   chassis.moveToPoint(-40, -23, 1000, {false}); //move back
//   arm.move (0); 
//   intake.move (127); 
//   intake2.move (-127);
//   arm.move (0);
//   chassis.turnToHeading(193, 700, {.maxSpeed = 100, .minSpeed = 20});//turn
//   chassis.moveToPoint(-48, -20, 1000, {.maxSpeed = 80, .minSpeed = 20, });//move forward to intake rings
//   arm.move (-50); 
//   chassis.turnToHeading(73, 700);//turn to face ladder
//   chassis.moveToPoint(-44, -21, 700);//move forward to set up jar arm
//   delay (1000);
//   arm.move (50);
//   delay (1000);
//   arm.move (0);

//awp blue
  chassis.setPose(-38, 47, -27); 
chassis.moveToPose(-26, 24, -27, 1000, {false, 8,0, 60, 40}); //move to get goal
  chassis.waitUntil(23);
  mogo.set_value(true);
  chassis.waitUntilDone();
  chassis.turnToHeading(-134, 1000);//turn to face rings
  intake.move (127);
  intake2.move (-127);//turn on intake
chassis.moveToPoint(-36, 29, 1000, {true, 80, 20});//go forward
chassis.turnToHeading(-104, 700, {.maxSpeed = 80, .minSpeed = 30});//turn
chassis.moveToPoint(-44, 21, 1000 ,{true, 60, 30});//go forward to intake rings, -36, 23, 104
chassis.turnToHeading(2, 500, {.maxSpeed = 80, .minSpeed = 40});//turn 2
chassis.moveToPoint(-52, 25, 700);//go forward a little to intake the ring
chassis.turnToHeading(-152, 700, {.maxSpeed = 80, .minSpeed = 60});//turn to face stake -150
arm.move (-50); //arm up
chassis.moveToPoint(-57, 28, 1000, {true, 60, 40});//move forward to stake
intake.move (0);
intake2.move (0);
chassis.waitUntilDone();
arm.move (50);
delay (300);
armc.set_value(true);
// chassis.moveToPoint(-54, 30, 700);
// chassis.turnToHeading(90, 700, {.maxSpeed = 80, .minSpeed = 40}); //turn
// arm.move (0);

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
  bool hangState = false;
  bool mogoState = false;
  bool awpState = false;
  bool armcState = false;
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
      arm.move(-70);

    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      arm.move(70);

    } else {
      arm.move(0);
     }


    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake.move(127);


    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake.move(-127);
    } 
      else {
      intake.move(0);
     }
     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake2.move(-127);


    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake2.move(127);
    } 
      else {
      intake2.move(0);
     }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      mogoState = !mogoState;
      mogo.set_value(mogoState);
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      awpState = !awpState;
      awp.set_value(awpState);
    }

    if (master.get_digital_new_press (pros::E_CONTROLLER_DIGITAL_Y)){
        hangState = !hangState;
        hang.set_value(hangState);
    }

    if (master.get_digital_new_press (pros::E_CONTROLLER_DIGITAL_B)){
        armcState = !armcState;
        armc.set_value(armcState);
    }
}
}



