#include "main.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

// Motor Lf(11); // number (06) is the cartridge
// Motor Lf2 (-12);
// Motor Rf2 (19);
// Motor Lb(13);
// Motor Rf(-18);
// Motor Rb(-20);
Motor arm (3);
Motor hooks (20);
Motor ladybrown (5);
adi::DigitalOut awp (6);
adi::DigitalOut armc (4);
adi::DigitalOut jararm (7);
Motor intake(15);
Motor armR(10);
//Motor armL(13);
Controller master(E_CONTROLLER_MASTER);
adi::DigitalOut intlift (7);
adi::DigitalOut mogo (8);
// sensor ports
Rotation odomVerticalPod(6); //change 
Rotation odomHorizontalPod(-16);
Imu imu(10);
Optical color(21);
Rotation brown (13);


//motor groups
pros::MotorGroup left_side_motors({
    3,
    -19,
    14,
},pros::MotorGearset::blue);
pros::MotorGroup right_side_motors({
    -4,
    -11, 
    12, 

},pros::MotorGearset::blue);

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

const int numStates = 3;
//make sure these are in centidegrees (1 degree = 100 centidegrees)
int states[numStates] = {0, 2100, 14500,};
int currState = 0;
int loadState = 0;
int autoState = 0;
int restState = 0;
int target = 0;

void nextState() {
    currState += 1;
    if (currState == 3) {
        currState = 1;
    }
    else if (currState == 0) {
        currState = 1;
    }
    
    target = states[currState];
}

void rest() {
    restState = 0;
    if (restState > 0) {
        restState = 0;
    }
    target = states[restState];
}

void Astake() {
    autoState = 3;
    if (autoState < 3) {
        autoState = 3;
    }
    target = states[autoState];
}

void Aload() {
    loadState = 1;
    if (loadState > 0) {
        loadState = 1;
    }
    target = states[loadState];
}

void liftControl() {
    double kp = 0.027;
    double error = target - brown.get_position();
    double velocity = kp * error;
    ladybrown.move(velocity);
}
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
// bool coloring = true; //blue = false; red = true
// bool  checkState = true;
// void color_sort(){

//    while(true){
//             if(color.get_hue() < 20){ //if blue or nothing
//             hooks.move(24);
//             delay(100); 
//             hooks.move(-100);
//         }

//        else if(color.get_hue() > 200){
//             hooks.move(24);
//             delay(100); 
//             hooks.move(-100);
//         }
    
//     delay(10);
//    }

// }

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
        // pros::Task colorTask (color_sort);

    pros::Task liftControlTask([]{
        while (true) {
            liftControl();
            pros::delay(10);
        }
    });
    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "hueValue: %f", color.get_hue());
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



//  int brownState = 2;
// void armthingidk(){
//   int target;
//   int error= 0;
//   brown.set_position(0);
//   while(true){
//       if(brownState == 1){
//           target = 1730; //loading height
//       }
//       else if (brownState == 2){
//           target = 0; //chill height
//       }
//       else if (brownState == 3){
//           target = 16000; //score height
//       }
//       // else if (brownState == 4){
//       //     target = 12000; //auto height
//       // }

//       error = target - brown.get_position();
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

//b pos high stake
// chassis.setPose (35.5,-52.5,0);
// intake.move(127);//only first stage on
// chassis.moveToPoint(35.5, -18.9, 2000, {true, 90, 60});//go forward to have room to turn for left ring
// delay (100);
// chassis.turnToHeading(22.5, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face left ring
// delay (100);
// chassis.moveToPoint(39.18, -8.6, 1000, {true, 80, 50});//get left ring
// delay (100);
// chassis.turnToHeading(45.7, 1000, {.maxSpeed = 50, .minSpeed = 30}); //turn back to face goal
// delay (100);
// chassis.moveToPoint(28.8, -16.1, 1500, {false, 80, 50});//go back to goal
// chassis.waitUntil(14);
// mogo.set_value(true);//clamp
// delay (100);
// chassis.turnToHeading(86, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face double middle ring
// hooks.move (-127);
// delay (100);
// chassis.moveToPoint(46.8, -16.2, 1500, {true, 80, 50});//go intake middle ring
// Aload();
// delay (100);
// chassis.turnToHeading(58, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to align with the lady brown position
// delay (100);
// chassis.moveToPoint(60, -4.2, 1500, {true, 80, 50});//go to ladybrown position
// chassis.waitUntil(100000);
// hooks.move (0); //stop second stage
// Astake();//ladybrown up
// delay (1000);
// rest (); //ladybrown down
// delay (100);
// chassis.moveToPoint(37.6, -25, 1000, {false, 100, 80}); //move back to the middle ring
// hooks.move (-127);
// delay (100);
// chassis.turnToHeading(123, 1000, {.maxSpeed = 50, .minSpeed = 30}); //turn to corner
// delay (100);
// chassis.moveToPoint(71.1, -54.2, 1000, {true, 100, 80});//go to intake corner ring
// hooks.move (0);
// delay (800);
// chassis.moveToPoint(71.1, -60, 500, {true, 80, 50});//push a little
// chassis.moveToPoint(60.4, -52.8, 1000, {false, 80, 50});//move back to align with single ring
// delay (100);
// chassis.turnToHeading(88.72, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn so back align with single ring
// delay (100);
// chassis.moveToPoint(-2.4, -52.8, 2000, {false, 100, 80});//sweep the ring out of the way
// mogo.set_value(false);
// delay (100);
// chassis.moveToPoint(7, -54.2, 1000, {true, 80, 50});//go forward to align with alliance stake
// delay (100);
// chassis.turnToHeading(-1.41, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn back to face alliance stake
// delay (100);
// chassis.moveToPoint(7, -65, 1000, {false, 80, 50}); //go back to intake onto stake
// intake.move (127);
// hooks.move (-127);//outtake
// delay (700);
// intake.move (0);
// hooks.move (0);
// chassis.moveToPoint(3, -36, 1000, {true, 100, 80});//go to touch pole

//b solo solo awp
// chassis.setPose(-24, -53, 180);
// chassis.moveToPoint(0, 0, 1000, {false, 80, 50});//go back to goal
// delay (100);
// chassis.moveToPoint(0, 0, 1000, {false, 80, 50});//clamp to goal
// mogo.set_value(true);
// delay (100);
// chassis.turnToHeading(0, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face left middle rings
// delay (100);
// hooks.move(-127);
// intake.move(127);
// chassis.moveToPoint(0, 0, 1000, {true, 90, 60});//go intake that blue ring
// delay (100);
// chassis.turnToHeading(0, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to fave the white line on the right side
// delay (100);
// chassis.moveToPoint(0, 0, 3000, {true, 90, 60});//go to the right side and intake double ring
// mogo.set_value(false);
// delay (100);
// chassis.turnToHeading(0, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to push all rings out of the way like 90 degrees
// delay (100);
// chassis.moveToPoint(0, 0, 1000, {true, 90, 60});//push eveyrthing out of the way
// delay (100);
// chassis.moveToPoint(0, 0, 1000, {false, 90, 60});//move back to aligin with stake
// delay (100);
// chassis.moveToPoint(float x, float y, int timeout);

//b goal rush
// chassis.setPose(-61, -49, 20.7);//setpose -57, 51.5, 16
// intake.move (127);//first stage on
// chassis.moveToPoint(-49, -15, 1500, {true, 110, 90});//rush goal
// awp.set_value(true);
// delay (100);
// chassis.moveToPoint(-57, -37, 1000, {false, 80, 50});//go back to bring goal back -57, -37
// intake.move (0);
// delay (100);//delay
// chassis.turnToHeading(181, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn so back faces rushed goal 181
// awp.set_value(false);//awp arm up
// delay (100);//delay
// chassis.moveToPoint(-47, -19, 1000, {false, 80, 50});//go back to clamp -47, -21
// chassis.waitUntil(18);//waituntil
// mogo.set_value(true);//clamp
// delay (100);//delay
// chassis.turnToHeading(130, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn far away 130
// hooks.move (-127);//hooks run
// delay (100);//delay
// chassis.moveToPoint(-9, -49, 1000, {true, 80, 60});//-9, -49 go fR AWAY
// chassis.waitUntil(20);
// mogo.set_value(false);//drop off
// delay (100);//delay
// chassis.turnToHeading(158, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face second goal 158
// delay (100);//delay
// chassis.moveToPoint(-17, -20, 1000, {false, 80, 50});//go to second goal -17, -20
// delay (100);
// chassis.moveToPoint(-17, -18, 1000, {false, 80, 50});
// mogo.set_value(true);//clamp
// delay (100);//delay
// chassis.moveToPoint(-13, -49, 1000, {true, 80, 50});//gp to wall -13, -49
// delay (100);//delay
// chassis.turnToHeading(253, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to corner 253
// delay (100);//delay
// chassis.moveToPoint(-48, -64, 2000, {true, 70, 30});//go forward to rings -53, -70
// intake.move(127);
// awp.set_value(true);//awp arm down
// chassis.turnToHeading(300, 1000, {.maxSpeed = 100, .minSpeed = 80});//sweep tun 312
// delay (500);//delay
// chassis.turnToHeading(252, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn back 241
// awp.set_value(false);//arm back up
// delay (100);//delay
// chassis.moveToPoint(-50, -65, 1000, {true, 70, 30});//move forwrard to intake -54, -71
// delay (1000);//delay
// chassis.moveToPoint(-17, -56, 1000, {false, 80, 50});//move back to have room to turn to ladder -41, -54
// delay (100);//delay
// chassis.turnToHeading(380, 800, {.maxSpeed = 70, .minSpeed = 30});//turn to ladder 402
// hooks.move (0);
// intake.move (0);
// chassis.moveToPoint(-16, -15, 3000, {true, 70, 30});//go to touch ladder -19, -24
// mogo.set_value(false);//drop off goal

//r goal rush
// chassis.setPose(-61, -49, 20.7);//setpose 
// intake.move (127);//first stage on
// chassis.moveToPoint(-49, -15, 1500, {true, 110, 90});//rush goal
// awp.set_value(true);
// delay (100);
// chassis.moveToPoint(-57, -37, 1000, {false, 80, 50});//go back to bring goal back -57, -37
// intake.move (0);
// delay (100);//delay
// chassis.turnToHeading(181, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn so back faces rushed goal 181
// awp.set_value(false);//awp arm up
// delay (100);//delay
// chassis.moveToPoint(-47, -19, 1000, {false, 80, 50});//go back to clamp -47, -21
// chassis.waitUntil(18);//waituntil
// mogo.set_value(true);//clamp
// delay (100);//delay
// chassis.turnToHeading(130, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn far away 130
// hooks.move (-127);//hooks run
// delay (100);//delay
// chassis.moveToPoint(-9, -49, 1000, {true, 80, 60});//-9, -49 go fR AWAY
// chassis.waitUntil(20);
// mogo.set_value(false);//drop off
// delay (100);//delay
// chassis.turnToHeading(158, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face second goal 158
// delay (100);//delay
// chassis.moveToPoint(-17, -20, 1000, {false, 80, 50});//go to second goal -17, -20
// delay (100);
// chassis.moveToPoint(-17, -18, 1000, {false, 80, 50});
// mogo.set_value(true);//clamp
// delay (100);//delay
// chassis.moveToPoint(-13, -49, 1000, {true, 80, 50});//gp to wall -13, -49
// delay (100);//delay
// chassis.turnToHeading(253, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to corner 253
// delay (100);//delay
// chassis.moveToPoint(-48, -64, 2000, {true, 70, 30});//go forward to rings -53, -70
// intake.move(127);
// awp.set_value(true);//awp arm down
// chassis.turnToHeading(300, 1000, {.maxSpeed = 100, .minSpeed = 80});//sweep tun 312
// delay (500);//delay
// chassis.turnToHeading(252, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn back 241
// awp.set_value(false);//arm back up
// delay (100);//delay
// chassis.moveToPoint(-50, -65, 1000, {true, 70, 30});//move forwrard to intake -54, -71
// delay (1000);//delay
// chassis.moveToPoint(-17, -56, 1000, {false, 80, 50});//move back to have room to turn to ladder -41, -54
// delay (100);//delay
// chassis.turnToHeading(380, 800, {.maxSpeed = 70, .minSpeed = 30});//turn to ladder 402
// hooks.move (0);
// intake.move (0);
// chassis.moveToPoint(-16, -15, 3000, {true, 70, 30});//go to touch ladder -19, -24
// mogo.set_value(false);//drop off goal



//b SOLO AWP
// chassis.setPose(10, -61, -108);
// Astake(); //up
// delay (550);
// rest(); //down
// delay (100);
// chassis.moveToPoint(22, -56, 1000, {false, 120, 100});//go back 32, -56
// delay (100);//delay
// chassis.turnToHeading(-176, 1000, {.maxSpeed = 60, .minSpeed = 40});//turn to face right goal -176
// delay (100);//delay
// chassis.moveToPoint(27, -18, 1500, {false, 90, 60});//go back to clamp on goal 24, -17
// delay (100);
// chassis.moveToPoint(27, -16, 1000, {false, 90, 60});//go back to clamo
// mogo.set_value(true);
// delay (100);//delay
// chassis.turnToHeading(-252, 1000, {.maxSpeed = 60, .minSpeed =40});//turn to face middle right ring -252
// delay (100);//delay
// chassis.moveToPoint(39, -15, 1000, {true, 120, 110});//go intake double middle ring 39, -15
// intake.move (127);
// hooks.move (-127);
// delay (100);//delay
// chassis.moveToPoint(32, -13, 1000, {false, 110, 90});//move back 32, -13
// delay (100);//delay
// chassis.turnToHeading(-120, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to go other side -110
// delay (100);//delay
// chassis.moveToPoint(-36, -56, 3000, {true, 120, 90});//go to other side -42, -57
// chassis.waitUntil(55);//wait until
// mogo.set_value(false);//drop goal
// delay (100);//delay
// chassis.turnToHeading(-145, 1000, {.maxSpeed = 60, .minSpeed = 40});//turn to face second goal -145
// delay (100);//delay
// chassis.moveToPoint(-15, -17, 1000, {false, 90, 60});//-21 -16 to goal
// hooks.move (0);
// delay (100);
// chassis.moveToPoint(-17, -15, 1000, {false, 90, 60});//-21 -16 to goal
// mogo.set_value(true);
// delay (100);//delay
// chassis.turnToHeading(-108, 1000, {.maxSpeed = 60, .minSpeed = 40});//turn to face middle left rings -25, -16 108
// hooks.move (127);
// delay (100);//delay
// chassis.moveToPoint(-39, -21, 1000, {true, 110, 90});//go intake those rings -41 -21
// hooks.move (-127);
// delay (500);//delay
// chassis.moveToPoint(-28, -19, 800, {false, 120, 80});//go back -28, -16
// delay (100);//delay
// chassis.turnToHeading(-165, 1000, {.maxSpeed = 60, .minSpeed = 0});//turn to face wall -166
// delay (100);//delay
// chassis.moveToPoint(-29, -48, 800, {true, 120, 80});//go forward -35, -49
// delay (100);//delay
// chassis.turnToHeading(-118, 1000, {.maxSpeed = 60, .minSpeed = 40});//turn to face vorner -104
// delay (100);//delay
// chassis.moveToPoint(-54, -59, 1000, {true, 120, 90});//move to corner -55 -56
// delay (100);//delay
// awp.set_value(true);
// chassis.turnToHeading(-64, 800, {.maxSpeed = 100, .minSpeed = 80});//turn to sweep
// intake.move (0);
// delay (100);//delay
// chassis.turnToHeading(-94, 1000, {.maxSpeed = 60, .minSpeed = 40});//turn it back to intake ring -94
// awp.set_value(false);
// delay (100);//delay
// chassis.moveToPoint(-60, -59, 1000, {true, 100, 80});//move forward to intake -60 -59
// intake.move (127);
// delay (700);//delay
// chassis.moveToPoint(-50, -58, 1000, {false, 100, 80});//go back -50, -58
// delay (100);
// chassis.turnToHeading(50, 1000, {.maxSpeed = 60, .minSpeed = 40});//turn -50
// delay (100);//delay
// chassis.moveToPoint(-26, -6, 1000, {true, 100, 80});//-26, -8 go touch pole
// chassis.waitUntil(10);//drop goal
// mogo.set_value(false);
// chassis.waitUntil(10);//drop goal

//hard skills
// chassis.setPose(0, -69, 360);
// intake.move (127);
// hooks.move (-127);//outtake
// delay (700);
//  intake.move (0);
//  hooks.move (0);
// chassis.moveToPoint(.09, -58.5, 1000, {true, 70, 30});//go forward .09 -55.3
// delay (100);//delay
// chassis.turnToHeading(270, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face goal 273
// delay (100);//delay
// chassis.moveToPoint(23.5, -58.5, 1500, {false, 80, 50});//go for goal 17.9, -56.7
// chassis.waitUntil(22);//wait until
// mogo.set_value(true);//clamp
// delay (100);//delay
// chassis.turnToHeading(369, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face ring 369
// delay (100);//delay
// chassis.moveToPoint(20, -36, 1500, {true, 90, 65}); //go for ring
// intake.move (110);
// hooks.move (-127);//intake on
// delay (100);//delay
// chassis.turnToHeading(392, 800, {.maxSpeed = 50, .minSpeed = 30});//turn to align with farthest ring
// delay (100);//delay
// chassis.moveToPoint(40, -4, 1000, {true, 90, 65});//go to the alignment position
// delay (100);//delay
// chassis.turnToHeading(382, 900, {.maxSpeed = 50, .minSpeed = 30});//turn to align with farther ring
// delay (100);//delay
// chassis.moveToPoint(47, 17, 2000, {true, 90, 60});//go to farther ring
// Aload();//ladybrown to setting postition
// delay (1000);//delay
// chassis.moveToPoint(40, -4, 1000, {false, 90, 60});//go back to orginal position
// delay (100);//delay
// chassis.turnToHeading(450, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face stake
// hooks.move (0);
// delay (100);//delay
// chassis.moveToPoint(67, 0, 1000, {true,80, 50});//go forward to stake and intake one ring in the process
// chassis.waitUntil(100);
// delay (100);//delay
// Astake();//ladybrown up and score!
// delay(1000);//delay
// chassis.moveToPoint(50, 0, 2000, {false, 110, 90});//move back to align with 3 rings
// delay (500);//delay
// rest ();//ladybrown down
// delay (500);//delay
// chassis.turnToHeading(537, 800, {.maxSpeed = 50, .minSpeed = 30});//turn to 3 rings
// hooks.move (-110);
// delay (100);
// chassis.moveToPoint(58, -61, 3500, {true, 70, 40});//go forward and intake 3 rings
// chassis.waitUntil(2);
// intake.move (110);
// delay (900);//delay
// intake.move (127);
// chassis.moveToPoint(59, -39, 1000, {false, 90, 60});//go back so it can get last ring
// delay (100);//delay
// chassis.turnToHeading(509, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face last ring 9
// delay (100);//delay
// chassis.moveToPoint(69, -50, 1500, {true, 90, 60});//go for last ring 58.7, -51
// delay (100);//delay
// chassis.turnToHeading(331, 1600, {.maxSpeed = 60, .minSpeed = 30});//turn to face corner 348
// delay (100);//delay
// chassis.moveToPoint(63, -64, 1000, {false, 90, 60});//move bacl into corner 63, -64
// delay (100);//delay
// chassis.moveToPoint(50, -52, 3000, {true, 80, 50});//go forward to double line in order to align with left double ring
// intake.move (-127);
// hooks.move (127);
// mogo.set_value(false);//drop it off
// delay (100);//delay
// chassis.turnToHeading(352, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to align with double ring

// // // chassis.setPose (51, -48, 360);
// chassis.moveToPoint(50, 30, 7000, {true, 127, 127});//go intake far left double ring KEEP IN INTAKE
// intake.move (127);
// hooks.move (0);
// delay (100);//delay
// chassis.turnToHeading(431.5, 950, {.maxSpeed = 60, .minSpeed = 40});//turn so back faces the mogo
// delay (100);//delay
// chassis.moveToPoint(1, 42, 2000, {false, 80, 60});//go to goal to push it a bit farhter out of the way...slowly
// delay (100);//delay
// chassis.turnToHeading(441, 800, {.maxSpeed = 60, .minSpeed = 40});//turn so intake faces right mogo with blue ring on it
// delay (100);//delay
// intake.move (0);
// chassis.moveToPoint(55, 57, 2000, {.maxSpeed = 127, .minSpeed = 127});//push it in corner
// delay (100);//delay
// chassis.moveToPoint(10, 51, 3000, {false, 100, 70});//back up to align with blue alliance stake
// delay (100);//delay
// chassis.turnToHeading(528, 1000, {.maxSpeed = 40, .minSpeed = 30});//turn so back aligns with stake
// delay (100);
// chassis.moveToPoint(12, 58, 1000, {false, 80, 50});//go back
// delay (100);
// hooks.move (-127);
// delay (570); //outtake onto stake that one red ring
// chassis.moveToPoint(14, 39, 1000, {true, 80, 50});//go forward to align with mogo we pushed
// delay (100);//delay
// chassis.turnToHeading(490, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn so back faces mogo
// delay (100);//delay
// chassis.moveToPoint(-6, 40, 1500, {false, 80, 50});//go to mogo
// chassis.waitUntil(10);//wait until
// mogo.set_value(true);//clamp
// delay (100);//delay
// intake.move (-127);
// hooks.move (127);
// chassis.moveToPoint(18, 35, 1000, {.maxSpeed = 100, .minSpeed = 80});//go forward to have room to turn
// delay (100);//delay
// chassis.turnToHeading(485, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to intake that one ring
// intake.move (127);
// hooks.move (-127);
// delay (100);//delay
// chassis.moveToPoint(33, 22, 1000, {true, 100, 80});//go to intake that one ring we missed
// delay (100);//delay
// intake.move (127);
// chassis.turnToHeading(583, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to go to middle
// delay (100);//delay
// chassis.moveToPoint(-32, -66, 6000, {true, 70, 40});//go for middle ring under ladder AND EVERYTHING THAT IS IN THAT HORIZONTAL PATH...so intake 3 rings
// chassis.waitUntil(1);
// hooks.move(0);
// delay (100);//delay
// chassis.waitUntil(50);
// hooks.move(-110);//hooks back on//intake bottom ring
// chassis.turnToHeading(552, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to intake bottom ring 552
// delay (100);//delay
// chassis.moveToPoint(-32, -71, 1000, {true, 90, 60});//go to intake bottom ring -32, -71
// delay (100);//delay
// chassis.moveToPoint(-26, -50, 1000, {false, 90, 60});//go back to intake side ring
// delay (100);//delay
// chassis.turnToHeading(570, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to side ring
// delay (100);//delay
// chassis.moveToPoint(-40, -61, 1000, {true, 90, 60});//go to intake side ring
// delay (100);//delay
// chassis.turnToHeading(742, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn so back faces corner
// delay (100);//delay
// chassis.moveToPoint(-53, -71, 800, {false,90, 60 });//go back and drop it off
// intake.move(-127);
// hooks.move(127);
// delay (100);//delay
// chassis.moveToPoint(-45, -39, 1000, {true, 100, 80});//go to intake that ring -38, -38
// mogo.set_value(false);//drop off goalfalse
// chassis.waitUntil(10);
// intake.move (-127);
// hooks.move (127);
// Aload();//ladybrown load
// delay (100);//delay
// intake.move (127);
// hooks.move (-127);
// chassis.turnToHeading(684, 1000, {.maxSpeed = 60, .minSpeed = 40});//turn so back faces goal
// delay (100);//delay
// chassis.moveToPoint(-19, -62, 1500, {false, 100,80});//get goal
// chassis.waitUntil (26);
// mogo.set_value (true);
// delay (100);
// chassis.moveToPoint(-38, -18, 2000, {true, 100, 80}); //go for double line
// delay (100);
// chassis.turnToHeading(627, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face stake
// hooks.move (0);
// delay (100);//delay
// chassis.moveToPoint(-53, -16, 1000, {true,80, 50});//go forward to stake and intake one ring in the process
// chassis.waitUntil(100);
// delay (100);//delay
// Astake();//ladybrown up and score!
// delay(500);//delay
// intake.move (0);
// chassis.moveToPoint(-40, -18, 1000, {false, 110, 90});//back off to align with solo red rings nearest
// delay (700);//delay
// rest ();//ladybrown down
// delay (700);//delay
// chassis.turnToHeading(732, 800, {.maxSpeed = 50, .minSpeed = 30});//turn 90 degrees to face red ring
// delay (100);//delay
// chassis.moveToPoint(-47.2, 0.9, 1000, {true, 100, 80});//go to intake that one solo red ring
// intake.move (127);
// hooks.move (-127);
// delay (100);//delay
// chassis.turnToHeading(790, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn 90 degrees and intake the other one
// delay (100);//delay
// chassis.moveToPoint (-16, 21, 1000, {true, 100, 80}); //go to intake the other one
// delay (100);
// chassis.turnToHeading(673, 800, {.maxSpeed = 50, .minSpeed = 30});//turn to intake the double rings that are vertical
// delay (100);//delay
// chassis.moveToPoint(-42, 32, 1000, {true, 90, 60});//go to double ring
// delay (100);//delay
// chassis.moveToPoint(-31, 20, 1000, {false, 80, 50});//go back to make space for the other one
// delay (100);//delay
// chassis.turnToHeading(694, 800, {.maxSpeed = 50, .minSpeed = 30});//turn to face 2nd ring
// delay (100);//delay
// chassis.moveToPoint(-47.3, 42.1, 1000, {true, 90, 60});//go to 2nd ring
// delay (100);//delay
// chassis.moveToPoint(-26.5, 27.3, 1000, {false, 100, 80});//go back to make space for the third one
// delay (100);//delay
// chassis.turnToHeading (633, 1000, {.maxSpeed = 50, .minSpeed = 30});
// delay (100);
// chassis.moveToPoint(-52, 29.9, 1000, {true, 90, 60});//go to 3rd ring
// delay (100);//delay
// chassis.turnToHeading(509, 1000, {.maxSpeed = 60, .minSpeed =40});//turn so back faces corner
// chassis.moveToPoint(-44.3, 33.8, 800, {true, 127, 127});//go forward a bit
// mogo.set_value(false);//drop off goal
// chassis.turnToHeading(612, 800, {.maxSpeed = 127, .minSpeed = 127});//turnso front faces goal
// // delay (100);//delay
// chassis.moveToPoint(-61, 43.1,800, {true, 127, 127});//push goal in with intake
// // delay (100);
// chassis.moveToPoint(-44.3, 33.8, 800, {false , 127, 127});

//safe skills 
// chassis.setPose(0, -69, 360);
// intake.move (127);
// hooks.move (-127);//outtake
// delay (700);
//  intake.move (0);
//  hooks.move (0);
// chassis.moveToPoint(.09, -57, 1000, {true, 70, 30});//go forward .09 -55.3
// delay (100);//delay
// chassis.turnToHeading(270, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face goal 273
// delay (100);//delay
// chassis.moveToPoint(23.5, -58, 1500, {false, 60, 30});//go for goal 17.9, -56.7
// chassis.waitUntil(25);//wait until
// mogo.set_value(true);//clamp
// delay (100);//delay
// chassis.turnToHeading(369, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face ring 369
// intake.move (127);
// hooks.move (-127);//intake on
// delay (100);//delay
// chassis.moveToPoint(20, -36, 1500, {true, 70, 30}); //go for ring
// delay (100);//delay (100)
// chassis.turnToHeading(410, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to farthest ring
// delay (100);//delay
// chassis.moveToPoint(52, -7, 1500, {true, 70, 30});// go to farthest ring
// delay (100);//delay

// chassis.turnToHeading(555, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face three ring 539.7
// delay (100);//delay
// chassis.moveToPoint(58, -22, 1000, {true, 60, 30});//go for first ring
// delay (100);
// chassis.turnToHeading (540, 1000, {.maxSpeed = 50, .minSpeed = 30}); //turn to face 2 last rings
// delay (100);

// chassis.moveToPoint(57, -63, 4000, {true, 60, 20});//go for three ring SLOWER 49.3, -65.8
// delay (100);//delay
// chassis.moveToPoint(57, -35, 1000, {false, 70, 30});//go back so it can get ring 51.2, -43.7
// delay (100);//delay
// chassis.turnToHeading(505, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face last ring 90 degrees 505
// delay (100);//delay
// chassis.moveToPoint(64, -51, 1500, {true, 70, 30});//go for last ring 58.7, -51
// delay (100);//delay
// chassis.turnToHeading(338, 1600, {.maxSpeed = 60, .minSpeed = 30});//turn to face corner 348
// delay (100);//delay
// chassis.moveToPoint(63, -66, 1300, {false, 90, 30});//go back to corner 60.8 -68.3 338
// chassis.waitUntil(10);//wait until..DROP IT OFF
// intake.move (-127);
// hooks.move (127);
// chassis.moveToPoint(57, -57, 1000, {true, 70,  30});//go to middle point 57, -57.9, 451
// mogo.set_value(false);//drop it off
// delay (100);
// chassis.turnToHeading(805, 1500, {.maxSpeed = 50 ,.minSpeed = 30});//turn to align with second goal 109
// intake.move (-127);
// hooks.move (127);
// delay (100);//delay
// chassis.moveToPoint(-14, -56, 7500, {false, 85, 30});//go for goal 17.9, -56.7
// chassis.waitUntil(65);//wait until
// mogo.set_value(true);//clamp
// delay (100);//delay
// chassis.waitUntil(4);
// intake.move (127);
// hooks.move (-127);//intake on
// chassis.turnToHeading(376, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face ring 369
// intake.move (127);
// hooks.move (-127);//intake on
// delay (100);//delay
// chassis.moveToPoint(-20, -37, 1500, {true, 70, 30}); //go for ring
// delay (100);//delay (100)
// chassis.turnToHeading(309, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to farthest ring
// delay (100);//delay
// chassis.moveToPoint(-51, -20, 1600, {true, 70, 30});// go to farthest ring
// delay (100);//delay
// chassis.turnToHeading(518, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face three ring 539.7
// delay (100);//delay
// chassis.moveToPoint(-35, -33, 1000, {true, 70, 30});//go for first ring -35 -33
// delay (100);//delay
// chassis.turnToHeading(537, 1000, {.maxSpeed = 50, .minSpeed  = 30});//turn for last 2 

// delay (100);//delay
// chassis.moveToPoint(-31, -73, 4500, {true, 60, 20});//go for three ring SLOWER 49.3, -65.8
// delay (100);//delay
// chassis.moveToPoint(-34, -37, 1000, {false, 70, 30});//go back so it can get ring 51.2, -43.7
// delay (100);//delay
// chassis.turnToHeading(200, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face last ring 90 degrees 505
// delay (100);//delay
// chassis.moveToPoint(-41, -60, 1500, {true, 70, 30});//go for last ring 58.7, -51
// delay (100);//delay
// chassis.turnToHeading(396, 1600, {.maxSpeed = 60, .minSpeed = 30});//turn to face corner 348
// delay (100);//delay
// chassis.moveToPoint(-54, -69, 1500, {false, 90, 30});//go back to corner 60.8 -68.3 338
// chassis.waitUntil(10);//wait until..DROP IT OFF
// mogo.set_value(false);//drop it off
// delay (100);
// chassis.moveToPoint(-43, -39, 1000, {true, 70, 30});//go to line -49.5, -52.4

// delay (100);//delay
// chassis.turnToHeading(0, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to go to other side
// delay (100);//delay
// intake.move (0);
// hooks.move (0);
// chassis.moveToPoint(-43, 4, 5000, {true, 90, 30});//go to other side 10.5
// intake.move (70);
// delay (100);//delay
// chassis.turnToHeading(47, 1000, {.maxSpeed = 50, .minSpeed =30});//turn to face goal
// intake.move (0);
// delay (100);//delay
// chassis.moveToPoint(-11, 40, 1500, {true, 70, 30});//go to goal -23, 46.2
// chassis.turnToHeading(-69, 900, {.maxSpeed = 50, .minSpeed = 30});//turn so front faces corner better
// delay (100);
// chassis.moveToPoint(-46, 43, 2000, {true, 100, 30});//push to corner -56.5, 54.7
// delay (100);//delay
// chassis.moveToPoint(-26, 44, 3000, {false, 70, 30});//ove back so it has room to turn -22.4, 49.2
// delay (100);//delay
// chassis.turnToHeading(-76, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face middle line so it aligns with goal -39
// delay (100);
// chassis.moveToPoint(15, 32, 2000, {false, 70, 30});//go to goal -4.1, 34
// chassis.waitUntil(89);//wait until
// mogo.set_value(true);//clamp
// delay (100);//delay
// chassis.moveToPoint(17, 38, 1000, {true, 70, 30});//move back -6 38
// delay (100);//delay
// intake.move (127);//intake on
// hooks.move (-127);
// chassis.turnToHeading(114, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to align with ring 125
// delay (100);
// chassis.moveToPoint(31, 16, 1200,{true, 70, 30});//go for ring 24.5 22.4
// delay (100);//delay
// chassis.turnToHeading(-265, 1000, {.maxSpeed = 50, .minSpeed = 30});//align with second ring
// delay (100);//delay
// chassis.moveToPoint(52, 14, 1500, {true, 70, 30});//go for second ring 48 24.3
// delay (400);//delay
// chassis.turnToHeading(-356, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to left ring 17
// delay (400);//delay
// chassis.moveToPoint(50, 28, 1500, {true, 60, 30});// 46 36 go for left ring
// delay (100);//delay
// chassis.moveToPoint(50, 18, 1000, {false, 70, 30});//move back 46.8, 33.3
// delay (100);//delay
// chassis.turnToHeading(-323, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to get right one 57
// delay (100);//delay
// chassis.moveToPoint(56, 32, 1500, {true, 70, 30});//go for right one 54.5 41.9 53.2 35.9
// delay (100);// delay (100);
// chassis.turnToHeading(-149, 1600, {.maxSpeed = 70, .minSpeed = 30});//turn so back faces corner209
// delay (100);
// chassis.moveToPoint(58, 17, 1200, {true, 127, 127});//go forward 57.6, 49.
// intake.move (-127);
// hooks.move(127);
// mogo.set_value(false);
// chassis.turnToHeading(-684, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn around 320
// chassis.moveToPoint(65, 49, 2000, {true, 90, 30});//go to corner 60, 43
// intake.move (0);
// hooks.move(0);
// delay (300);
// chassis.turnToHeading(-670, 1000, {.maxSpeed = 100, .minSpeed = 80});//turn a little
// delay (100);
// chassis.moveToPoint(58, 17, 1200, {false, 127, 127});//go forward 57.6, 49.

//r doinker rush
chassis.setPose (-28.3, -52.5 ,-20.5);
awp.set_value(true);
intake.move (127);
chassis.moveToPoint(-44.2, -13.8, 1500, {true, 100, 50});//go to double rings
delay (1500);
chassis.moveToPoint(-30.6, -38.7,1800, {false, 60, 40}); //move back to align awp ring with middle ring
delay (100);
intake.move (0);
chassis.turnToHeading(-136, 1000, {.maxSpeed = 40, .minSpeed = 20});//turn so back faces goal
awp.set_value(false); //arm back up
delay (100);
chassis.moveToPoint(-18, -22, 700, {false, 70, 30}); //go to goal
chassis.waitUntil(27);
mogo.set_value(true);//clamp
chassis.turnToHeading(-106, 800, {.maxSpeed = 70, .minSpeed = 40});//turn 90 to face 2 rings i dropped off
delay (100);
chassis.waitUntil(4);
intake.move (127);
chassis.waitUntil(1);//run hooks
hooks.move (-127);
chassis.moveToPoint(-52, -29, 1700, {true, 75, 40});//go intake those 2 first rings
chassis.waitUntil(29);
delay (100);
chassis.moveToPoint(-44, -27, 800, {false, 80, 40});//go back -44, -27
intake.move (127);
hooks.move (-127);
delay (100);
chassis.turnToHeading(-234, 800, {.maxSpeed = 55, .minSpeed = 35});//turn to face oreload -235
delay (100);//delay
chassis.moveToPoint(-14, -38, 1000, {true, 110, 80});//go to preload -17, -36
chassis.waitUntil(100);
awp.set_value(true);
delay (200);
chassis.moveToPoint(-19, -35, 1500, {false, 70, 50});//move back slowly -17, -33
chassis.waitUntil(100);
awp.set_value(false);//up
delay (100);//delay
chassis.turnToHeading(-254, 800, {.maxSpeed = 50, .minSpeed = 30});//turn to get it -270
delay (100);
chassis.moveToPoint(-5, -34, 1000, {true, 80, 30});//go forward -7, -33 intake
delay (100);
hooks.move (0);
mogo.set_value(false);
chassis.turnToHeading(-226, 900, {.maxSpeed = 65, .minSpeed = 40});//turn to face b ring
delay (100);//delay
chassis.moveToPoint(15, -50, 770, {true, 120, 80});//push everything to other side
intake.move (-127);
mogo.set_value(false);
delay (100);
chassis.moveToPoint(0, -43, 1000, {false, 100, 80});//move back to get room 4, -45
delay (100);
chassis.turnToHeading(-361, 800, {.maxSpeed = 45, .minSpeed = 25});//turn to -361
delay (100);
chassis.moveToPoint(0, -65, 1000, {false, 70, 30});//go back to put it wall stake
delay (100);
hooks.move (-127);
delay (570); //outtake onto stake
chassis.moveToPoint(-2, -35, 1000, {true, 90, 80});//go forward to touch ladder
intake.move (0);
hooks.move (0);
Astake ();


//blue ring rush
// chassis.setPose (28.3, -52.5 ,20.5);
// intake.move (127);
// chassis.moveToPoint(44.2, -13.8, 2000, {true, 100, 50});//go to double rings
// delay (1500);
// chassis.moveToPoint(30.6, -38.7,1250, {false, 55, 30}); //move back to align awp ring with middle ring
// delay (100);
// intake.move (0);
// chassis.turnToHeading(143, 1000, {.maxSpeed = 40, .minSpeed = 20});//turn so back faces goal
// delay (100);
// chassis.moveToPoint(33, -25, 700, {false, 70, 30}); //go to goal
// chassis.waitUntil(27);
// mogo.set_value(true);//clamp
// chassis.turnToHeading(90, 800, {.maxSpeed = 70, .minSpeed = 40});//turn 90 to face 2 rings i dropped off
// delay (100);
// chassis.waitUntil(4);
// intake.move (127);
// chassis.waitUntil(1);//run hooks
// hooks.move (-127);
// chassis.moveToPoint(52, -21, 1700, {true, 75, 40});//go intake those 2 first rings
// chassis.waitUntil(29);
// delay (100);
// chassis.moveToPoint(44, -21, 800, {false, 80, 40});//go back -44, -27
// delay (100);//delay
// chassis.turnToHeading(146, 800, {.maxSpeed = 50, .minSpeed = 30});//turn to face corner 146
// intake.move (127);
// hooks.move (-127);
// delay (100);//delay
// chassis.moveToPoint(68, -58, 1000, {true, 80, 20});//go to corner 68, -58
// delay (1000);//delay
// chassis.moveToPoint(70, -61, 500, {true, 80, 20});//extra push
// delay (2000);
// chassis.moveToPoint(59, -46, 2000, {false, 40, 30});//move back slowly 59, -46
// delay (100);//delay
// chassis.turnToHeading(314, 1000, {.maxSpeed = 60, .minSpeed = 30});//turn to touch ladder 314
// delay (100);
// chassis.moveToPoint(19, -24, 2000, {true, 70, 30});//go touch ladder, 19, -24

// b pos alliance stake
// chassis.setPose(-10, -61, 108);
// Astake(); //up
// delay (550);
// rest(); //down
// delay (100);
// chassis.moveToPoint(-24, -56, 1000, {false, 70, 30});//move back 24, -56
// delay (100);//delay
// chassis.turnToHeading(175, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face goal 176
// delay (100);//delay
// chassis.moveToPoint(-18, -26, 1000, {false, 70, 30});//go to goal -19, -33
// chassis.waitUntil (27);
// mogo.set_value (true);
// chassis.turnToHeading(265, 1000, {.maxSpeed = 60, .minSpeed = 40});//turn to face middle ring 264
// delay (100);//delay
// intake.move (127);
// hooks.move (-127);//intake on
// chassis.moveToPoint(-43, -35, 2000, {true, 70, 30});//go to middle ring -43, -36
// delay (100);//delay
// chassis.moveToPoint(-30, -29, 1500, {false, 70, 30});//go back to have room for corner -26, -34
// delay (100);//delay
// chassis.turnToHeading(183, 1000, {.maxSpeed = 60, .minSpeed = 40});//turn to face wall 191
// delay (100);//delay
// chassis.moveToPoint(-27, -55, 1300, {true, 70, 30});//move forward to wall -27, 57
// delay (100);//delay
// chassis.turnToHeading(245, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face rings 249
// delay (100);//delay
// chassis.moveToPoint(-49, -69, 2000, {true, 100, 80});//go forward to rings -53, -70
// awp.set_value(true);//awp arm down
// chassis.turnToHeading(312, 2000, {.maxSpeed = 70, .minSpeed = 30});//sweep tun 312
// delay (500);//delay
// chassis.turnToHeading(241, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn back 241
// awp.set_value(false);//arm back up
// delay (100);//delay
// chassis.moveToPoint(-54, -71, 900, {true, 70, 30});//move forwrard to intake -54, -71
// delay (100);//delay
// chassis.moveToPoint(-41, -54, 1000, {false, 70, 30});//move back to have room to turn to ladder -41, -54
// delay (100);//delay
// chassis.turnToHeading(48, 800, {.maxSpeed = 70, .minSpeed = 30});//turn to ladder 402
// chassis.moveToPoint(-15, -19, 3000, {true, 127, 100});//go to touch ladder -19, -24
// Astake();
// mogo.set_value(false);//drop off goal

// red pos alliance stake
// chassis.setPose(10, -61, -108);
// Astake(); //up
// delay (550);
// rest(); //down
// delay (100);
// chassis.moveToPoint(24, -55, 1000, {false, 70, 30});//move back 24, -56
// delay (100);//delay
// chassis.turnToHeading(-181, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face goal 176
// delay (100);//delay
// chassis.moveToPoint(23, -21, 1000, {false, 70, 30});//go to goal -19, -33
// chassis.waitUntil (25);
// mogo.set_value (true);
// delay(100);
// chassis.turnToHeading(-270, 1000, {.maxSpeed = 60, .minSpeed = 40});//turn to face middle ring 264
// delay (100);//delay
// intake.move (127);
// hooks.move (-127);//intake onintake.move (127);
// chassis.moveToPoint(37, -14, 1200, {true, 70, 30});//go to middle ring -43, -36
// delay (700);//delay
// chassis.waitUntil(20);
// delay (100);
// chassis.moveToPoint(32, -12, 1000, {false, 70, 30});//move back 32, -12
// delay (100);
// chassis.turnToHeading(-226, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face wall 249
// delay (100);//delay
// chassis.moveToPoint(52, -29, 2000, {true, 70, 30});//go to wall 52, -29
// intake.move (0);
// delay (100);//delay
// chassis.turnToHeading(-194, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face rings -194
// delay (100);//delay
// chassis.moveToPoint(60, -49, 2000, {true, 70, 30});//go forward to rings -53, -70
// awp.set_value(true);//awp arm down
// chassis.turnToHeading(-142, 2000, {.maxSpeed = 100, .minSpeed = 80});//sweep tun 312
// delay (500);//delay
// chassis.turnToHeading(-198, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn back 241
// awp.set_value(false);//arm back up
// intake.move (127);
// delay (100);//delay
// chassis.moveToPoint(60, -63, 1000, {true, 70, 30});//move forwrard to intake -54, -71
// delay (100);//delay
// chassis.moveToPoint(56, -44, 1000, {false, 70, 30});//move back to have room to turn to ladder -41, -54
// delay (100);//delay
// chassis.turnToHeading(-48, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to ladder 402
// // hooks.move(0);
// chassis.moveToPoint(15, -17.5, 3000, {true, 70, 30});//go to touch ladder -19, -24
// Astake(); //up
// mogo.set_value(false);//drop off goal

};





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
  bool intliftState = false;
  bool mogoState = false;
  bool awpState = false;
  bool armcState = false;
  bool intup = false;
//   bool colorside = false ; //false is blue, true is red
  right_side_motors.set_brake_mode(E_MOTOR_BRAKE_COAST);
  ladybrown.set_brake_mode(E_MOTOR_BRAKE_HOLD);
//   color.set_led_pwm(100);
//   coloring = true;
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(25);

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
            nextState();
        }
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
            rest();
        }
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
        //coloring = !coloring; 
    }

    // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    //   arm.move(-70);

    // } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    //   arm.move(70);

    // } else {
    //   arm.move(0);
    //  }

    //  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    //   ladybrown.move(-127);


    // } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    //   ladybrown.move(127);
    // } 
    //   else {
    //   ladybrown.move(0);
    //  }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {

      intake.move(-127);
       hooks.move(110);


    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake.move(127);
       hooks.move(-110);
    } 
      else {
      intake.move(0);
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
        intliftState = !intliftState;
        intlift.set_value(intliftState);
    }

    if (master.get_digital_new_press (pros::E_CONTROLLER_DIGITAL_UP)){
        armcState = !armcState;
        armc.set_value(armcState);
    }


    }
}
