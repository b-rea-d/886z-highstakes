#include "main.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"

// Motor Lf(11); // number (06) is the cartridge
// Motor Lf2 (-12);
// Motor Rf2 (19);
// Motor Lb(13);
// Motor Rf(-18);
// Motor Rb(-20);
Motor arm (3);
Motor hooks (20);
Motor ladybrown (5);
adi::DigitalOut awp (2);
adi::DigitalOut armc (4);
adi::DigitalOut jararm (7);
Motor intake(16);
Motor armR(10);
//Motor armL(13);
Controller master(E_CONTROLLER_MASTER);
adi::DigitalOut hang (2);
adi::DigitalOut mogo (1);
// sensor ports
Rotation odomVerticalPod(1);
Rotation odomHorizontalPod(16);
Imu imu(10);
Optical distance(20);
Rotation brown (7);

//motor groups
pros::MotorGroup left_side_motors({
    3,
    -19,
    15,
});
pros::MotorGroup right_side_motors({
    -4,
    -11, 
    12, 

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
lemlib::TrackingWheel vertical_tracking_wheel(&odomVerticalPod, 2, 4, 1);
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
    70,   // kD
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



//  int brownState =2;
// void armthingidk(){
//   int target;
//   int error= 0;
//   brownEnc.set_position(0);
//   while(true){
//       if(brownState == 1){
//           target = 2950; //loading height
//       }
//       else if (brownState == 2){
//           target = 0; //chill height
//       }


//       else if (brownState == 3){
//           target = 15000; //score height
//       }
//       else if (brownState == 4){
//           target = 12000; //auto height
//       }

//       error = target - brownEnc.get_position();
//       ladybrown.move(error*0.02);


//       delay(10);
//   }


//  }
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

//safe awp red
//set pose
//go to goal
//wait until
//clamp
//delay
//intake on
//turn to face double ring 
//delay
//go to right ring
//delay
//move back to have space to get left ring
//delay
//turn to face left ring
//delay
//go for left ring
//delay
//move back as to not hit the new ring
//delay
//turn to new ring
//delay
//go for new ring
//delay
//turn to face ladder
//delay
//go touch ladder

//safe awp blue
//set pose
//go to goal
//wait until
//clamp
//delay
//intake on
//turn to face double ring 
//delay
//go to right ring
//delay
//move back to have space to get left ring
//delay
//turn to face left ring
//delay
//go for left ring
//delay
//move back as to not hit the new ring
//delay
//turn to new ring
//delay
//go for new ring
//delay
//turn to face ladder
//delay
//go touch ladder

//nsolo awp red
//set pose
//wall stake mech up
//delay (a litte longer)
//wall stake mech down
//delay (a little longer)
//go for mogo 
//wait until
//clamp
//delay
//intake on
//turn to face double ring 
//delay
//go to right ring
//delay
//move back to have space to get left ring
//delay
//turn to face left ring
//delay
//go for left ring
//delay
//move back as to not hit the new ring
//delay
//turn to new ring
//delay
//go for new ring
//delay
//go for last ring aligned with the wallstake mech
//delay
//turn to face ladder
//delay
//go to ladder


//nsolo awp blue
//set pose
//wall stake mech up
//delay (a litte longer)
//wall stake mech down
//delay (a little longer)
//go for mogo 
//wait until
//clamp
//delay
//intake on
//turn to face double ring 
//delay
//go to right ring
//delay
//move back to have space to get left ring
//delay
//turn to face left ring
//delay
//go for left ring
//delay
//move back as to not hit the new ring
//delay
//turn to new ring
//delay
//go for new ring
//delay
//go for last ring aligned with the wallstake mech
//delay
//turn to face ladder
//delay
//go to ladder

//full goal red
//setPose
//go to goal
//wait until
//clamp
//delay
//intake on
//turn to face right ring
//delay
//go for right ring
//delay
//turn so it's a straight line
//delay
//go forward to intake the left one too
//delay
//turn to face the new ring
//delay
//go for new ring
//delay
//go forwa



















































//skills auto

// intake.move (127);
// intake2.move (-127);//outtake to put ring on the wall stake
// delay (470);
// intake.move (0);
// intake2.move (0);
// chassis.moveToPoint(-5, -63, 1000, {false, 127, 100});//go back
// delay (480);
// chassis.moveToPoint(12, -63, 700, {true, 70, 10});//go forward a little bit
// chassis.turnToHeading(0, 700);//turn around so back is facing goal
// chassis.moveToPoint(20, -86, 1000, {false, 70, 40});//go for goal
// delay (1000);
// mogo.set_value(true);//clamp
// intake.move (127);
// intake2.move (-127);//turn on intake, 
// delay (100);
// chassis.turnToHeading(103, 700, {.maxSpeed = 75, .minSpeed = 30});//turn around so intake faces rings (94)
// chassis.moveToPoint(30, -79, 1000, {.maxSpeed = 75, .minSpeed = 30});//move forward to intake ring
// chassis.turnToHeading(180, 700, {.maxSpeed = 60, .minSpeed = 30});//turn 90 degrees
// chassis.moveToPoint(44, -97, 3000, {true, 70, 40});//44, -97
// chassis.turnToHeading(269, 700, {.maxSpeed = 60, .minSpeed = 30});//turn 90 degrees (269)
// chassis.moveToPoint(22, -110, 3000, {true, 70, 40});//move it forward (22, -107)
// chassis.moveToPoint(41, -103, 1000, {.forwards = false, .maxSpeed = 75, .minSpeed = 30});//move back (41, -103)
// chassis.turnToHeading(212, 700, {.maxSpeed = 75, .minSpeed = 30});//turn (212)
// chassis.moveToPoint(26, -112, 1000, { .maxSpeed = 75, .minSpeed = 30});//intake last ring 
// chassis.moveToPoint(32, -105, 1000, {.forwards = false, .maxSpeed = 75, .minSpeed = 40});//move back
// chassis.turnToHeading(57, 1000, {.maxSpeed = 75, .minSpeed = 30});//turn to face corner
// chassis.moveToPoint(0.334, -134, 2000, {.forwards = false, .maxSpeed = 75, .minSpeed = 40});//go back to corner
// delay (1000);
// mogo.set_value(false); //unclamp
// chassis.moveToPoint(9, -128, 1000, {.forwards = true, .maxSpeed = 60, .minSpeed = 30});//move back a little
// delay (100);
// chassis.turnToHeading(180, 1000, {.maxSpeed = 60, .minSpeed = 30});//turn to face goal
// chassis.moveToPoint(18, -20, 5000, {false,70, 40});//go to goal
// delay (2500);
// mogo.set_value(true);//clamp
// intake.move (127);
// intake2.move (-127);//turn on intake
// chassis.turnToHeading(94, 1000, {.maxSpeed = 60, .minSpeed = 30});///turn so intakppe faces ring 
// chassis.moveToPoint(30.6, -48.6, 1000, {.maxSpeed = 75, .minSpeed = 30});//move forward to intake ring//30.6, -48.6
// chassis.turnToHeading(52.9, 700, {.maxSpeed = 60, .minSpeed = 30});//turn to (52.9
// chassis.moveToPoint(55, -30, 1200, {.maxSpeed = 70, .minSpeed = 30});//55, -29 get ring
// chassis.turnToHeading(-109, 1000, {.maxSpeed = 60, .minSpeed = 30});//one more turn -109
// chassis.moveToPoint(42, -28, 1000, {true, 70, 30 });//move forward 42, -28
// chassis.turnToHeading(-90, 700, {.maxSpeed = 75, .minSpeed = 30});//turn to face the 3 rings
// delay (100);
// chassis.moveToPoint(17.9, -27.5, 3000, {true, 70, 40});//intaake those 3 rings and move forward
// delay (200);
// chassis.moveToPoint(45, -24.8, 1000, {false, 60, 40});//move back (45, -24.8)
// chassis.turnToHeading(-68.1, 700, {.maxSpeed = 75, .minSpeed = 30});//turn to intake that one ring
// chassis.moveToPoint(28.7, -24.3, 1000, {true, 70, 40});//go forward and intake ring
// chassis.turnToHeading(-227.4, 700, {.maxSpeed = 75, .minSpeed = 30});//turn to face corner
// chassis.moveToPoint(-1.5, 2.9, 1000, {false, 75, 40});////drop off goal
// delay (1300);
// mogo.set_value(false);//unclamp
// chassis.moveToPoint(14, -16, 700, {true, 75, 40});//forwward slightly (19.5, -13) 14-16
// chassis.turnToHeading(90, 1000, {.maxSpeed = 75, .minSpeed = 30});//turn to face rings(-267) 90
// intake.move (50);
// intake2.move (-50);
// chassis.moveToPoint(73, -14, 3000, {true, 75, 40});//go across field for rings
// delay (100);
// chassis.turnToHeading(138, 1000, {.maxSpeed = 60, .minSpeed = 30});//turn to face wall 138
// chassis.moveToPoint(117, -48, 2000, {true, 70, 30});//go to wall
// chassis.turnToHeading(177, 1000, {.maxSpeed = 60, .minSpeed = 30});//turn to blue goal
// chassis.moveToPoint(122, -37.6, 1000, {false, 70, 30});//go to blue goal 122, -37.6
// intake.move(0);
// intake2.move (0);
// delay (100);//delay
// chassis.moveToPoint(126.7, 0.4, 2000, {false, 70, 30});//go to corner
// chassis.moveToPoint(123, -35.2, 1500, {true, 70, 30});//go forward to try to goal
// chassis.turnToHeading(31, 1000, {.maxSpeed = 60, .minSpeed = 30});//turn to align with goal
// chassis.moveToPoint(107, -69, 1300, {false, 70, 30});// go to goal
// delay (1000);
// mogo.set_value(true);//clamp
// delay (100);
// chassis.turnToHeading(-134, 1000, {.maxSpeed = 70, .minSpeed = 30});//turn to face ring 220
// delay (100);
// intake.move (127);
// intake2.move (-127);//turn on intake
// chassis.moveToPoint(98, -76, 1000, {true, 70, 30});//go to ring 105.9 -72.6
// chassis.turnToHeading(-180, 1000, {.maxSpeed = 60, .minSpeed = 30});//turn to face wall and ring
// chassis.moveToPoint(92, -97, 1500, {true, 70, 30});//go to ring
// delay (100);
// chassis.turnToHeading(-242, 1000, {.maxSpeed = 60, .minSpeed = 40});//turn to face ring
// chassis.moveToPoint(96, -108, 1000, {true, 70, 30});//go for ring
// delay (100);
// chassis.moveToPoint(83, -108, 1000, {false, 70, 30});//back up to original spot
// delay (100);
// chassis.turnToHeading(-242, 1000, {.maxSpeed = 60, .minSpeed = 30});//turn to 45 degrees
// chassis.moveToPoint(95, -110, 1000, {true, 70, 30});//go to ring 96, -106
// delay (100);
// chassis.turnToHeading(-109, 1000, {.maxSpeed = 60, .minSpeed = 30});//turn to face ring near wall stake -109
// chassis.moveToPoint(84, -121, 1500, {true, 70, 30});//go to that ring93, -120
// delay (100);//delay
// chassis.turnToHeading(-78, 800, {.maxSpeed = 60, .minSpeed = 30});//turn -81
// delay (800);
// mogo.set_value(false);//release goal
// chassis.moveToPoint(132, -137, 3000, {false, 127, 127});//back up into corner 133, -130
// intake.move(0);
// intake2.move (0);
// delay (100);
// chassis.moveToPoint(126, -129, 1000, {true, 100, 80});//move forward 126 -129
// hang.set_value(true);//piston hang
// chassis.turnToHeading(-37, 1000, {.maxSpeed = 60, .minSpeed = 30});//turn to face ladder -37
// chassis.moveToPoint(81, -82, 2000, {true, 80, 30});//go forward to hang 81, -82
// delay (100);//delayy
// chassis.moveToPoint(10000, -85, 1000, {false, 127, 100});//84, -85

//skills auto pt.2 
// chassis.setPose(0, -69, 360);
// intake.move (127);
// intake2.move (-127);//outtake
// delay (500);
// chassis.moveToPoint(0, -72, 1000, {false, 70, 30}); //push
//  delay (100);
//  intake.move (0);
//  intake2.move (0);
// chassis.moveToPoint(.09, -57, 1000, {true, 70, 30});//go forward .09 -55.3
// delay (100);//delay
// chassis.turnToHeading(273, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face goal 273
// delay (100);//delay
// chassis.moveToPoint(17.9, -57, 1500, {false, 60, 30});//go for goal 17.9, -56.7
// chassis.waitUntil(25);//wait until
// mogo.set_value(true);//clamp
// delay (100);//delay
// chassis.turnToHeading(369, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face ring 369
// intake.move (127);
// intake2.move (-127);//intake on
// delay (100);//delay
// chassis.moveToPoint(21.6, -40.9, 1500, {true, 70, 30}); //go for ring
// delay (100);//delay (100)
// chassis.turnToHeading(411, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to farthest ring
// delay (100);//delay
// chassis.moveToPoint(56.3, -11.7, 1500, {true, 70, 30});// go to farthest ring
// delay (100);//delay
// chassis.turnToHeading(539.7, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face three ring 539.7
// delay (100);//delay
// chassis.moveToPoint(51.1, -65.8, 4000, {true, 60, 20});//go for three ring SLOWER 49.3, -65.8
// delay (100);//delay
// chassis.moveToPoint(51.2, -43.7, 1000, {false, 70, 30});//go back so it can get ring 51.2, -43.7
// delay (100);//delay
// chassis.turnToHeading(505, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face last ring 90 degrees 505
// delay (100);//delay
// chassis.moveToPoint(52.8, -54.6, 1500, {true, 70, 30});//go for last ring 58.7, -51
// delay (100);//delay
// chassis.moveToPoint(54.8, -42.8, 1000, {false, 70, 30});//move back to get room to turn 54.8 -42.8
// delay (100);//delay
// chassis.turnToHeading(338, 1600, {.maxSpeed = 60, .minSpeed = 30});//turn to face corner 348
// delay (100);//delay
// chassis.moveToPoint(64, -70, 1300, {false, 90, 30});//go back to corner 60.8 -68.3 338
// chassis.waitUntil(10);//wait until..DROP IT OFF
// mogo.set_value(false);//drop it off
// intake.move (0);
// intake2.move (0);
// delay (100);
// chassis.moveToPoint(57, -60, 1000, {true, 70,  30});//go to middle point 57, -57.9, 451
// delay (100);//delay
// chassis.turnToHeading(85, 1500, {.maxSpeed = 50 ,.minSpeed = 30});//turn to align with second goal 109
// delay (100);//delay
// chassis.moveToPoint(-17.9, -57, 7500, {false, 85, 30});//go for goal 17.9, -56.7
// chassis.waitUntil(89);//wait until
// mogo.set_value(true);//clamp
// delay (100);//delay
// chassis.turnToHeading(-369, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face ring 369
// intake.move (127);
// intake2.move (-127);//intake on
// delay (100);//delay
// chassis.moveToPoint(-21.6, -40.9, 1500, {true, 70, 30}); //go for ring
// delay (100);//delay (100)
// chassis.turnToHeading(685, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to farthest ring 678
// delay (100);//delay
// chassis.moveToPoint(-50, -9.9, 1500, {true, 70, 30});// go to farthest ring -53.5 -11.9
// delay (100);//delay
// chassis.turnToHeading(534, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face three ring 539.7 538
// delay (100);//delay
// chassis.moveToPoint(-45, -67.8, 4000, {true, 60, 20});//go for three ring SLOWER 49.3, -65.8
// delay (100);//delay
// chassis.moveToPoint(-51.2, -43.7, 1000, {false, 70, 30});//go back so it can get ring 51.2, -43.7
// delay (100);//delay
// chassis.turnToHeading(-505, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face last ring 90 degrees 505
// delay (100);//delay
// chassis.moveToPoint(-52.8, -54.6, 1500, {true, 70, 30});//go for last ring 58.7, -51
// delay (100);//delay
// chassis.moveToPoint(-54.8, -42.8, 1000, {false, 70, 30});//move back to get room to turn 54.8 -42.8
// delay (100);//delay
// chassis.turnToHeading(-338, 1300, {.maxSpeed = 60, .minSpeed = 30});//turn to face corner 348
// delay (100);//delay
// chassis.moveToPoint(-64, -70, 1000, {false, 90, 30});//go back to corner 60.8 -68.3 338
// chassis.waitUntil(10);//wait until..DROP IT OFF
// mogo.set_value(false);//drop it off
// intake.move (0);
// intake2.move (0);
// delay (100);
// chassis.turnToHeading(407, 1000, {.maxSpeed = 50, .minSpeed  =30});//407
// delay (100);
// chassis.moveToPoint(-45, -52.4, 1000, {true, 70, 30});//go to line -49.5, -52.4
// delay (100);//delay
// chassis.turnToHeading(0, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to go to other side
// delay (100);//delay
// chassis.moveToPoint(-45, 10.5, 5000, {true, 90, 30});//go to other side 10.5
// intake.move (70);
// intake2.move (-70);
// delay (100);//delay
// chassis.turnToHeading(92, 1000, {.maxSpeed = 50, .minSpeed =30});//turn 90 degrees to face ring
// delay (100);//delay

// //chassis.setPose (-45, 10.5, 92);intake.move (60);
// chassis.moveToPoint(-24, 9.1, 1000, {true, 70, 30});//go to second ring -42 9.1
// delay (100);//delay
// intake.move (0);
// intake2.move (0);
// chassis.turnToHeading(177, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face goal 177
// delay (100);//delay
// chassis.moveToPoint(-24, 54, 1500, {false, 70, 30});//push back -23, 46.2
// chassis.waitUntil(40);//chassis.wait until
// mogo.set_value(true);//clamp
// delay (100);//delay
// chassis.turnToHeading(102, 900, {.maxSpeed = 50, .minSpeed = 30});//turn so back faces corner better
// delay (100);
// chassis.moveToPoint(-56.5, 54, 2000, {false, 100, 30});//push to corner -56.5, 54.7
// chassis.waitUntil(2);//wait until
// mogo.set_value(false);//unclamp
// delay (100);//delay
// chassis.moveToPoint(-16, 54, 3000, {true, 70, 30});//ove forward so it has room to turn -22.4, 49.2
// delay (100);//delay
// chassis.turnToHeading(-48, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face middle line so it aligns with goal -39
// delay (100);
// chassis.moveToPoint(0, 38, 2000, {false, 70, 30});//go to goal -4.1, 34
// chassis.waitUntil(89);//wait until
// mogo.set_value(true);//clamp
// delay (100);//delay
// intake.move (127);//intake on
// intake2.move (-127);
// chassis.moveToPoint(-6, 44, 1000, {true, 70, 30});//move back -6 38
// delay (100);//delay
// chassis.turnToHeading(125, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to align with ring 125
// delay (100);
// chassis.moveToPoint(24.5, 19, 1200,{true, 70, 30});//go for ring 24.5 22.4
// delay (100);//delay
// chassis.turnToHeading(100, 1000, {.maxSpeed = 50, .minSpeed = 30});//align with second ring
// delay (100);//delay
// chassis.moveToPoint(48, 19, 1500, {true, 70, 30});//go for second ring 48 24.3
// delay (100);//delay
// chassis.turnToHeading(17, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to left ring 17
// delay (100);//delay
// chassis.moveToPoint(46, 35, 1200, {true, 60, 30});// 46 36 go for left ring
// delay (100);//delay
// chassis.moveToPoint(46.8, 33.3, 1000, {false, 70, 30});//move back 46.8, 33.3
// delay (100);//delay
// chassis.turnToHeading(44, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to get right one 57
// delay (100);//delay
// chassis.moveToPoint(53.2, 35, 1200, {true, 70, 30});//go for right one 54.5 41.9 53.2 35.9
// delay (100);// delay (100);
// chassis.turnToHeading(222, 1600, {.maxSpeed = 70, .minSpeed = 30});//turn so back faces corner209
// delay (100);
// chassis.moveToPoint(61, 56, 1200, {false, 127, 127});//go to corner 57.6, 49.
// chassis.waitUntil(1);
// mogo.set_value(false);//release
// delay (100);//delay
// intake.move (0);
// intake2.move (0);//intake off
// hang.set_value(true);//hang up
// chassis.turnToHeading(222, 800, {.maxSpeed = 100, .minSpeed = 30});//turn 222
// delay (100);
// chassis.moveToPoint(12, 0, 1000, {true, 127, 127});//go to hang 14 10
// chassis.moveToPoint(12, 20, 2000, {false, 127, 30});//go back a little





//blue goal rush
// chassis.setPose(-53, -27, -24);
// chassis.moveToPose(-33, -68, -24, 2000, {false, 8, 0, 100, 80});//go to goal (-35, -69)
// delay (1200);
// mogo.set_value(true); //clamp
// chassis.turnToHeading(7, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to  face rings 0
// intake.move(127);
// intake2.move (-127);
// chassis.moveToPoint(-34, -47, 1000, {true, 70, 40});//move to intake 1 blue ring -39, -62
// delay (3000);
// mogo.set_value(false);//let go
// intake.move (0);
// intake2.move (0);
// chassis.turnToHeading(60, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face other goal //59
// chassis.moveToPoint(-60, -48, 2000, {false, 70, 40});//go for goal -57, -48
// delay (1800);
// mogo.set_value(true);
// chassis.turnToHeading(14, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn 14
// chassis.moveToPoint(-46, -30, 1000, {true, 70, 40});//go for ring -43, -36
// intake.move (127);
// intake2.move (-127);
// chassis.turnToHeading(63, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn t corner 62
// chassis.moveToPoint(-24, -16, 4000, {true, 70, 40});//move to corner -28, -29
// awp.set_value(true);//arm down
// intake.move (0);
// intake2.move (0);//stp[ intkae]
// chassis.turnToHeading(233, 4000, {.maxSpeed = 80, .minSpeed = 40});//turn 140 to clear
// intake.move (127);
// intake2.move (-127);
// chassis.turnToHeading(220, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to 220
// intake.move (0);
// intake2.move (0);//stop intake
// arm.move(-50);//raise arm
// chassis.moveToPoint(-47, -52, 7000, {true, 70, 40});//-47, -52
// delay (2000);
// arm.move (50);
// delay (1000);
// arm.move (0);



//solo awp red (9pts)
// chassis.setPose(-38.5, -50, 196);
// chassis.moveToPoint(-35.6, -30.2, 1700, {false, 60, 20});//go to goal
// chassis.waitUntil(27);
// mogo.set_value(true);//clamp
// delay (100);
// chassis.turnToHeading(349, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face rings
// intake.move (127);
// sintake.move (-127);
// delay (100);
// chassis.moveToPoint (-49, -1, 1500, {true, 70, 30});//go to rings
// delay (100);
// chassis.turnToHeading(300, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face right rings
// delay (100);
// chassis.moveToPoint (-62, 3, 2000, {true, 60, 30});//go for ring
// delay (100);
// chassis.turnToHeading(204.6, 900, {.maxSpeed = 50, .minSpeed = 30});//turn for middle ring
// delay (100);
// chassis.moveToPoint(-57, -8, 1200, {true, 60, 30});//go for middle ring
// delay (100);
// chassis.turnToHeading(132, 1500, {.maxSpeed = 50, .minSpeed = 30});//turn to  face double ring 477 482
// delay (100);
// chassis.moveToPoint(-32, -36.7, 3000, {true, 70, 30});//go to double ring 20, -34
// intake.move (0);
// sintake.move (0);
// chassis.waitUntil(6);
// awp.set_value(true);
// intake.move (0);
// sintake.move (0);
// delay (100);
// chassis.turnToHeading(213, 1000, {.maxSpeed = 70, .minSpeed = 30});//turn to sweep 298
// delay (100);
// chassis.waitUntil(35);
// awp.set_value (false); //arm back up
// intake.move (127);
// sintake.move (-127);
// chassis.moveToPoint (-39.5, -60, 1000, {true, 70, 30});//go forward to intake it 6, -20
// delay (100);
// chassis.moveToPoint(-34, -26, 1000, {false, 70, 30});//go back to touch ladder 21, -27
//  chassis.turnToHeading(133, 1000, {.maxSpeed = 70, .minSpeed = 30});//ace ladder
//  awp.set_value(true);
//  delay (100);
//   intake.move (0);
// sintake.move (0);
// chassis.turnToHeading(143, 1000, {.maxSpeed = 60, .minSpeed = 30});//354


//solo awp blue (9pts)
// chassis.setPose(38.5, -50, 196);
// chassis.moveToPoint(43, -28, 1700, {false, 60, 20});//go to goal
// chassis.waitUntil(26);
// mogo.set_value(true);//clamp
// delay (100);
// chassis.turnToHeading(66.96, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face rings
// intake.move (127);
// sintake.move (-127);
// delay (100);
// chassis.moveToPoint (55, -13, 1500, {true, 70, 30});//go to rings
// delay (100);
// chassis.turnToHeading(95, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face right rings
// delay (100);
// chassis.moveToPoint (67, -14, 2000, {true, 60, 30});//go for ring
// delay (100);
// chassis.turnToHeading(206, 900, {.maxSpeed = 50, .minSpeed = 30});//turn for middle ring
// delay (100);
// chassis.moveToPoint(67, -29, 1200, {true, 60, 30});//go for middle ring
// delay (100);
// chassis.turnToHeading(260, 1500, {.maxSpeed = 50, .minSpeed = 30});//turn to  face double ring 477 482
// delay (100);
// chassis.moveToPoint(22, -34, 2500, {true, 70, 30});//go to double ring 20, -34
// chassis.waitUntil(6);
// awp.set_value(true);
// intake.move (0);
// sintake.move (0);
// delay (100);
// chassis.turnToHeading(305, 1000, {.maxSpeed = 75, .minSpeed = 30});//turn to sweep 298
// delay (100);
// chassis.waitUntil(7);
// awp.set_value (false); //arm back up
// intake.move (127);
// sintake.move (-127);
// chassis.moveToPoint (4, -23, 1000, {true, 70, 30});//go forward to intake it 6, -20
// delay (100);
// chassis.moveToPoint(21, -27, 1000, {false, 70, 30});//go back to touch ladder 21, -27
//  chassis.turnToHeading(328, 1000, {.maxSpeed = 70, .minSpeed = 30});//ace ladder
//  awp.set_value(true);
//  delay (100);
//   intake.move (0);
// sintake.move (0);
// chassis.moveToPoint(18, -23, 1000, {true, 70, 30});//18, -23
// chassis.turnToHeading(354, 1000, {.maxSpeed = 60, .minSpeed = 30});//354





//b rush
// chassis.setPose(-38.5, -50, -116);
// chassis.moveToPoint(-27.5, -28, 1500, {false, 70, 30});//go to goal
// chassis.waitUntil(22);
// mogo.set_value(true);//clamp
// delay (100);
// chassis.turnToHeading(-90, 900, {.maxSpeed = 50, .minSpeed = 30});//turn to face rings
// intake.move (127);
// intake2.move (-127);
// delay (100);
// chassis.moveToPoint (-40, -26, 1000, {true, 70, 30});//go to rings
// delay (100);//delay
// chassis.turnToHeading(90, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn so back facing ladder 90
// hang.set_value(true);//raise hang
// delay (100);//delay
// mogo.set_value(false);
// chassis.moveToPoint(-12, -12, 3000, {false, 70, 30});//go to ladder -12, -12

//goal rush red
// chassis.setPose(30.5, -52, 24);
// chassis.moveToPoint(46, -17.5, 2000, {true, 90, 40});//go to goal 45, -15
// delay (100);
// chassis.turnToHeading(106, 2000, {.maxSpeed = 80, .minSpeed = 40});//turn to swing it 90
// chassis.waitUntil(0.5);
// awp.set_value(true);//awp arm down
// chassis.waitUntil(100);//wait until 
// awp.set_value(false);//move arm up
// delay (100);
// chassis.turnToHeading(246, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face goal -100
// delay (500);//delay
// chassis.moveToPoint(60, -16, 1000, {false, 70, 30});//go to goal 58, -11
// chassis.waitUntil(40);//wait until
// mogo.set_value(true);//clamp
// delay (100);
// intake.move (127);
// sintake.move (-127);
// delay (500);
// chassis.turnToHeading(169, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face ring //-224
// delay (100);
// chassis.moveToPoint(63, -23, 1000, {true, 70, 30});// go for ring
// delay (100);
// chassis.turnToHeading(89, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face second goal -270
// chassis.waitUntil(10);
// mogo.set_value(false);
// delay (100);//delay
// intake.move (0);
// sintake.move (0);
// chassis.moveToPoint(40, -22, 2000, {false, 70, 30});//go to goal 31, -24
// chassis.waitUntil(30);
// mogo.set_value(true);
// chassis.turnToHeading(119, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face corner -214
// delay (100);
// chassis.moveToPoint(64, -38, 3000, {true, 70, 30});//38, -46 GO TO CORNER
// delay (100);
// chassis.turnToHeading(151, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face stack
// delay (100);
// chassis.moveToPoint(70, -56, 1000, {true, 70, 30});//go to stack 62, -58
// chassis.waitUntil(1);//wait until
// awp.set_value(true);//awp arm sown
// delay (100);// delay (100);
// chassis.turnToHeading(230, 2000, {.maxSpeed = 75, .minSpeed = 30});//turn to sweep
// delay (100);//delay
// chassis.waitUntil(1);
// awp.set_value(false);//arm up
// delay (100);//delay
// chassis.turnToHeading(150, 1000, {.maxSpeed = 60, .minSpeed = 30});//turn back to red ring 132
// delay (100);//delay
// intake.move (127);
// sintake.move (-127);
// chassis.moveToPoint(73, -58, 4000, {true, 70, 30});//go intake
// delay (1000);
// chassis.moveToPoint(63, -47, 1000, {false, 70, 30});// go back 63, -47 to give space
// delay (100);
// chassis.turnToHeading(-28, 1000, {.maxSpeed = 70, .minSpeed = 30});//-28 turn to face ladder
// delay (100);
// chassis.moveToPoint(33, -15, 6000, {true, 100, 70});//33 -15,  
// awp.set_value(true);
// chassis.turnToHeading(-1, 1000, {.maxSpeed = 100, .minSpeed = 30});//-1
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
  bool jararmState = false;
  bool intup = false;
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(25);




    // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    //   arm.move(-70);

    // } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    //   arm.move(70);

    // } else {
    //   arm.move(0);
    //  }

     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      ladybrown.move(127);


    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      ladybrown.move(-127);
    } 
      else {
      ladybrown.move(0);
     }
  
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        jararmState = jararmState;
        jararm.set_value(jararmState);
    }
     if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        jararmState = jararmState;
        jararm.set_value(!jararmState);
    }



    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake.move(127);


    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake.move(-127);
    } 
      else {
      intake.move(0);
     }
     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      hooks.move(127);


    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      hooks.move(-127);
    } 
      else {
      hooks.move(0);
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

    if (master.get_digital_new_press (pros::E_CONTROLLER_DIGITAL_UP)){
        armcState = !armcState;
        armc.set_value(armcState);
    }
}
}



