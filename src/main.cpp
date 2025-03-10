
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
Motor hooks (8);
Motor ladybrown (5);
adi::DigitalOut awp (6);
adi::DigitalOut armc (4);
adi::DigitalOut jararm (7);
Motor intake(7);
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
adi::DigitalOut tipper (5);

//motor groups
pros::MotorGroup left_side_motors({
    2,
    -19,
    14,
},pros::MotorGearset::blue);
pros::MotorGroup right_side_motors({
    -4,
    -18, 
    17, 

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
lemlib::TrackingWheel vertical_tracking_wheel(&odomVerticalPod, 2, -1, 1);
lemlib::TrackingWheel horizontal_tracking_wheel(&odomHorizontalPod, 2, 3,
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

    2.06, // kP (1.6) 1.09
    0,
    10, // kD (1)
    0,
    0,   // smallErrorRange
    0, // smallErrorTimeout
    0,   // largeErrorRange
    0, // largeErrorTimeout
    0    // slew rate
};

const int numStates = 3;
//make sure these are in centidegrees (1 degree = 100 centidegrees)
int states[numStates] = {0, 2100, 14500,}; //14500
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
right_side_motors.set_brake_mode(E_MOTOR_BRAKE_HOLD);
left_side_motors.set_brake_mode(E_MOTOR_BRAKE_HOLD);

//red neg good
// chassis.setPose(-10, -61, 108);
// Astake(); //up
// delay (550);
// rest(); //down
// delay (100);
// chassis.moveToPoint(-15, -59, 1000, {false});//move back 24, -56
// delay (100);//delay
// chassis.turnToHeading(166, 500);//turn to face goal 176
// delay (100);//delay
// chassis.moveToPoint(-24, -21, 1000, {false, 80});//go to goal -19, -33
// chassis.waitUntil (27);
// mogo.set_value (true);
// delay(100);
// chassis.turnToHeading(308, 500);//turn to face double rings
// intake.move (127);
// hooks.move (110);
// chassis.moveToPose(-40, -10, 265, 1300, {.forwards = true, .lead = 0.3, .maxSpeed = 100});//turn and intake one
// chassis.moveToPoint(-47, -9, 800, {true});//go to intake the second one
// chassis.moveToPose(-24, -21, 308, 1000, {.forwards = false, .lead = 0.3});//go to allignment position
// chassis.turnToHeading(264, 500);//turn to face middle ring
// chassis.moveToPoint(-41, -25, 1000, {true});//go to middle ring
// chassis.turnToHeading(217, 450);//turn to face corner
// chassis.moveToPose(-60, -62, 180, 2000, {.forwards = true, .lead = 0.25, .maxSpeed = 127});//go to corner
// chassis.moveToPoint(-60, -60, 200, {true, 127, 127});//push to intake
// delay (50);
// chassis.moveToPoint(-80, -80, 600, {true, 127, 127});//second push
// chassis.moveToPose(-48, -46, 205, 800, {.forwards = false, .lead = 0.2, .maxSpeed = 100});//move back to align with double alliance rings
// chassis.turnToHeading(95, 500);//turn to face double alliance rings
// intlift.set_value(true);//intake up
// chassis.moveToPoint(-6, -48, 1700, {true, 100});//go to double alliance rings
// chassis.waitUntil(1000);
// intlift.set_value(false);
// chassis.moveToPoint(-28, -45, 500, {false});//go back to have room to turn
// chassis.turnToHeading(41, 450);//turn 41
// chassis.moveToPoint(-12, -24, 1000, {true});//-12, -22 go to ladder

//blue neg good
// chassis.setPose(10, -61, -108);
// // Astake(); //up
// // delay (550);
// // rest(); //down
// // delay (100);
// chassis.moveToPoint(15, -59, 1000, {false});//move back 24, -56
// delay (100);//delay
// chassis.turnToHeading(-166, 500);//turn to face goal 176
// delay (100);//delay
// chassis.moveToPoint(22, -21, 1000, {false, 80});//go to goal -19, -33
// chassis.waitUntil (28);
// mogo.set_value (true);
// delay(100);
// chassis.turnToHeading(-308, 500);//turn to face double rings
// intake.move (127);
// hooks.move (110);
// chassis.moveToPose(40, -10, -265, 1300, {.forwards = true, .lead = 0.3, .maxSpeed = 100});//turn and intake one
// chassis.moveToPoint(47, -9, 800, {true});//go to intake the second one
// chassis.moveToPose(24, -21, -308, 1000, {.forwards = false, .lead = 0.3});//go to allignment position
// chassis.turnToHeading(-264, 500);//turn to face middle ring
// chassis.moveToPoint(41, -25, 1000, {true});//go to middle ring
// chassis.turnToHeading(-217, 450);//turn to face corner
// chassis.moveToPose(60, -62, -180, 2000, {.forwards = true, .lead = 0.25, .maxSpeed = 127});//go to corner
// chassis.moveToPoint(60, -60, 200, {true, 127, 127});//push to intake
// delay (50);
// chassis.moveToPoint(80, -80, 600, {true, 127, 127});//second push
// chassis.moveToPose(48, -46, -205, 800, {.forwards = false, .lead = 0.2, .maxSpeed = 100});//move back to align with double alliance rings
// chassis.turnToHeading(-95, 500);//turn to face double alliance rings
// intlift.set_value(true);//intake up
// chassis.moveToPoint(4, -48, 1700, {true, 100});//go to double alliance rings
// chassis.waitUntil(1000);
// intlift.set_value(false);
// chassis.moveToPoint(28, -45, 500, {false});//go back to have room to turn
// chassis.turnToHeading(-41, 450);//turn 41
// chassis.moveToPoint(14, -26, 1000, {true});//-12, -22 go to ladder

// r sig solo awp (start positive)
// chassis.setPose(10, -61, -108);
// Astake(); //up
// delay (550);
// rest(); //down
// delay (100);
// chassis.moveToPoint(15, -59, 1000, {false});//move back 24, -56
// delay (100);//delay
// chassis.turnToHeading(-166, 500);//turn to face goal 176
// delay (100);//delay
// chassis.moveToPoint(22, -21, 1000, {false, 80});//go to goal -19, -33
// chassis.waitUntil (28);
// mogo.set_value (true);
// chassis.turnToHeading(-268, 450);//turn to face middle pos ring
// intake.move (127);
// hooks.move (110);
// chassis.moveToPoint(38, -26, 1000, {true});//go intake middle pos ring
// chassis.turnToHeading (-151, 450);//turn to have enough room to swing
// chassis.moveToPose(11, -52, -76, 2000, {.forwards = true, .lead = 0.35, .maxSpeed = 100});//move to pose swing to alliance stake doube ring
// chassis.waitUntil(30.5);
// mogo.set_value(false);//drop goal
// chassis.moveToPose(-48, -29, -44, 2500, {.forwards = true, .lead = 0.3, .maxSpeed = 100});//move to pose to get blue one out and get the pos middle
// chassis.waitUntil(52);//wait until
// hooks.move (0);//stop hooks
// chassis.turnToHeading(-91, 500);//turn so back faces goal
// chassis.moveToPoint(-27, -24, 1000, {false, 80});//go back to goal
// chassis.waitUntil(15);//wait until
// mogo.set_value(true);//clamp
// delay (100);
// chassis.turnToHeading(-54, 500);//turn so back has room to swing -54
// hooks.move (110);//turn on hooks
// chassis.moveToPoint(9, -47, 1000, {false});//move back to pos corner 9, -47
// chassis.turnToHeading(0, 450);//turn to face ladder 0
// chassis.moveToPoint(14, -15, 3000, {true});//go to ladder 14, -22

//b sig solo awp
// chassis.setPose(-10, -61, 108);
// Astake(); //up
// delay (550);
// rest(); //down
// delay (100);
// chassis.moveToPoint(-15, -59, 1000, {false});//move back 24, -56
// delay (100);//delay
// chassis.turnToHeading(166, 500);//turn to face goal 176
// delay (100);//delay
// chassis.moveToPoint(-22, -21, 1000, {false, 80});//go to goal -19, -33
// chassis.waitUntil (28);
// mogo.set_value (true);
// chassis.turnToHeading(268, 450);//turn to face middle pos ring
// intake.move (127);
// hooks.move (110);
// chassis.moveToPoint(-38, -26, 1000, {true});//go intake middle pos ring
// chassis.turnToHeading (151, 450);//turn to have enough room to swing
// chassis.moveToPose(-11, -52, 76, 2000, {.forwards = true, .lead = 0.35, .maxSpeed = 100});//move to pose swing to alliance stake doube ring
// chassis.waitUntil(30.5);
// mogo.set_value(false);//drop goal
// chassis.moveToPose(48, -29, 44, 2500, {.forwards = true, .lead = 0.3, .maxSpeed = 100});//move to pose to get blue one out and get the pos middle
// chassis.waitUntil(52);//wait until
// hooks.move (0);//stop hooks
// chassis.turnToHeading(91, 500);//turn so back faces goal
// chassis.moveToPoint(27, -24, 1000, {false, 80});//go back to goal
// chassis.waitUntil(15);//wait until
// mogo.set_value(true);//clamp
// delay (100);
// chassis.turnToHeading(54, 500);//turn so back has room to swing -54
// hooks.move (110);//turn on hooks
// chassis.moveToPoint(-9, -37, 1000, {false});//move back to pos corner 9, -47
// chassis.turnToHeading(0, 450);//turn to face ladder 0
// chassis.moveToPoint(-14, -15, 3000, {true});//go to ladder 14, -22

//b pos no rush
// chassis.setPose(-10, -61, 108);
// Astake(); //up
// delay (550);
// rest(); //down
// delay (100);
// chassis.moveToPoint(-15, -59, 1000, {false});//move back 24, -56
// delay (100);//delay
// chassis.turnToHeading(166, 500);//turn to face goal 176
// delay (100);//delay
// chassis.moveToPoint(-22, -21, 1000, {false, 80});//go to goal -19, -33
// chassis.waitUntil (28);
// // hooks.move (-127);
// mogo.set_value (true);
// chassis.turnToHeading(268, 450);//turn to face middle pos ring
// intake.move (127);
// hooks.move (110);
// chassis.moveToPoint(-38, -26, 1000, {true});//go intake middle pos ring
// chassis.moveToPoint(-36, -15, 1000, {false});//move back -36,  -15
// // intake.move (-127);//outtake
// chassis.waitUntil(10);
// mogo.set_value(false);
// chassis.moveToPoint(-45, -24, 1000, {true});//move forward -45, -24
// chassis.turnToHeading(176, 500);//turn to face goal 176
// chassis.moveToPoint(-46, -10, 1000, {false, 70});//move back to goal SLOWLY -46, -10
// chassis.waitUntil(13);
// mogo.set_value(true);
// hooks.move (-127);
// chassis.moveToPose(-20, -59, 152, 1700, {.forwards = true, .lead = .3});//move to pose to go to wall -25, -47, 152
// chassis.turnToHeading(250, 500);//turn to face corner 250
// hooks.move (127);
// chassis.moveToPoint(-53, -70, 2000, {.forwards = true, .maxSpeed = 127});//go to corner
// chassis.moveToPoint(-60, -70, 200, {true, 127, 127});//push to intake
// delay (50);
// chassis.moveToPoint(-80, -80, 600, {true, 127, 127});//second push
// chassis.moveToPoint(-39, -53, 800, {.forwards = false, .maxSpeed = 100});//move back to align with double alliance rings
// chassis.turnToHeading(396, 500);//turn to face double alliance rings
// chassis.moveToPoint(-16, -21, 1700, {true, 100});//go to double alliance rings
// chassis.waitUntil(10);
// mogo.set_value(false);
// chassis.waitUntil(1000);
// intlift.set_value(false);
// chassis.moveToPoint(-28, -45, 500, {false});//go back to have room to turn
// chassis.turnToHeading(41, 450);//turn 41
// chassis.moveToPoint(-12, -21, 1000, {true});//-12, -22 go to ladder



//b goal rush
// chassis.setPose(-61, -53, 30);//setpose -57, 51.5, 16
// intake.move (127);//first stage on
// chassis.moveToPoint(-44, -20, 1500, {true, 110});//rush goal
// awp.set_value(true);
// chassis.moveToPoint(-46, -38, 1000, {false, 60});//go back to bring goal back -57, -37
// intake.move (0);
// delay (100);
// chassis.turnToHeading(150, 1000 );//turn so back faces rushed goal 181
// awp.set_value(false);//awp arm up
// delay (100);//delay
// chassis.moveToPoint(-49, -19, 1000, {false, 70});//go back to clamp -47, -21
// chassis.waitUntil(20);//waituntil
// mogo.set_value(true);//clamp
// delay (100);//delay
// chassis.turnToHeading(77, 600);//turn so back faces the left wall
// chassis.moveToPoint(-60, -31, 1000, {false});//-9, -49 go to position
// hooks.move (127);//hooks run
// chassis.turnToHeading(62, 600);//turn to face second goal 158
// mogo.set_value(false);//drop off
// delay (100);//delay
// chassis.moveToPoint(-29, -28, 2000, {false, 70});//go to second goal -17, -20
// delay (100);
// chassis.waitUntil(30);
// mogo.set_value(true);//clamp
// delay (100);//delay
// chassis.turnToHeading(-170, 1000);//turn to wall
// chassis.moveToPoint(-25, -57, 1000, {true, 80});//gp to wall -13, -49
// chassis.turnToHeading(-109, 1000);//turn to corner 253
// chassis.moveToPoint(-56, -62, 2000, {true});//go forward to rings -53, -70
// intake.move(127);
// awp.set_value(true);//awp arm down
// chassis.turnToHeading(-59, 1000, {.maxSpeed = 100, .minSpeed = 80});//sweep tun 312
// delay (500);//delay
// chassis.turnToHeading(-91, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn back 241
// awp.set_value(false);//arm back up
// delay (100);//delay
// chassis.moveToPoint(-62, -63, 1000, {true, 70, 30});//move forwrard to intake -54, -71
// delay (1000);//delay
// chassis.moveToPoint(-53, -64, 1000, {false, 80, 50});//move back to have room to turn to ladder -41, -54
// delay (100);//delay
// hooks.move (0);
// intake.move (0);
// chassis.turnToHeading(39, 800, {.maxSpeed = 70, .minSpeed = 30});//turn to ladder 402
// chassis.moveToPoint(-16, -24, 3000, {true, 70, 30});//go to touch ladder -19, -24
// mogo.set_value(false);//drop off goal

//b tip rush
// chassis.setPose(-61, -53, 20);//setpose -57, 51.5, 16
// intake.move (127);//first stage on
// chassis.moveToPoint(-48, -16, 1000, {true, 127});//rush goal
// tipper.set_value(true);
// chassis.waitUntil(1500);
// tipper.set_value(false);
// chassis.moveToPoint(-55, -35, 1000, {false, 127});//go back to bring goal back -57, -37
// intake.move (0);
// delay (100);
// chassis.turnToHeading(-95, 1000 );//turn so back faces stand goal
// delay (100);//delay
// chassis.moveToPoint(-20, -29, 1000, {false, 70});//go back to clamp -47, -21
// chassis.waitUntil(30);//waituntil
// mogo.set_value(true);//clamp
// delay (100);//delay
// chassis.turnToHeading(-142, 1000);//turn to wall
// chassis.moveToPoint(-39, -62, 1000, {true, 80});//gp to wall -13, -49
// hooks.move (110);
// chassis.turnToHeading(-97, 1000);//turn to corner 253
// chassis.moveToPoint(-57, -63, 2000, {true});//go forward to rings -53, -70
// intake.move(127);
// awp.set_value(true);//awp arm down
// chassis.turnToHeading(-57, 1000, {.maxSpeed = 100, .minSpeed = 80});//sweep tun 312
// delay (500);//delay
// chassis.turnToHeading(-80, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn back 241
// awp.set_value(false);//arm back up
// delay (100);//delay
// chassis.moveToPoint(-60, -61, 1000, {true, 70, 30});//move forwrard to intake -54, -71
// delay (1000);//delay
// intake.move (0);
// chassis.turnToHeading(45, 800, {.maxSpeed = 70, .minSpeed = 30});//turn to ladder 402
// chassis.moveToPoint(-19, -19, 3000, {true, 70, 30});//go to touch ladder -19, -24
// mogo.set_value(false);//drop off goal

//r tip rush
chassis.setPose(61, -53, -20);//setpose -57, 51.5, 16
intake.move (127);//first stage on
chassis.moveToPoint(48, -16, 1000, {true, 127});//rush goal
tipper.set_value(true);
chassis.waitUntil(1500);
tipper.set_value(false);
chassis.moveToPoint(55, -35, 1000, {false, 127});//go back to bring goal back -57, -37
// intake.move (0);
delay (100);
chassis.turnToHeading(95, 1000 );//turn so back faces stand goal
delay (100);//delay
chassis.moveToPoint(20, -29, 1000, {false, 70});//go back to clamp -47, -21
chassis.waitUntil(30);//waituntil
mogo.set_value(true);//clamp
delay (100);//delay
chassis.moveToPoint(55, -34, 1000, {true, 80});//gp to wall right wall
hooks.move (110);
chassis.turnToHeading(156, 1000);//turn to corner 253
chassis.moveToPoint(63, -56, 2000, {true});//go forward to rings -53, -70
intake.move(127);
awp.set_value(true);//awp arm down
chassis.turnToHeading(203, 1000, {.maxSpeed = 100, .minSpeed = 80});//sweep tun 312
delay (500);//delay
chassis.turnToHeading(175, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn back 241
awp.set_value(false);//arm back up
delay (100);//delay
chassis.moveToPoint(63, -58, 1000, {true, 70, 30});//move forwrard to intake -54, -71
delay (1000);//delay
chassis.moveToPoint(62, -52, 500, {false});//62, -52
intake.move (0);
chassis.turnToHeading(313, 800, {.maxSpeed = 70, .minSpeed = 30});//turn to ladder 402
chassis.moveToPose(48, -24, 0, 3000, {.forwards = true, .lead = .3});//go to touch ladder -19, -24
mogo.set_value(false);//drop off goal

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
// chassis.setPose(0, -69, 0);
// intake.move (127);
// hooks.move (127);//outtake
// delay (700);
//  intake.move (0);
//  hooks.move (0);
// chassis.moveToPoint(0, -56, 1000, {true});//go forward .09 -55.3
// // delay (100);//delay
// chassis.turnToHeading(-90, 550);//turn to face goal 273
// // delay (100);//delay
// chassis.moveToPoint(23.5, -54, 1500, {false, 70});//go for goal 17.9, -56.7
// chassis.waitUntil(13);//wait until
// mogo.set_value(true);//clamp
// delay (100);//delay
// chassis.turnToHeading(0, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face ring 369
// // delay (100);//delay
// chassis.moveToPoint(20, -36, 1300, {true}); //go for ring
// intake.move (127);
// hooks.move (110);//intake on
// delay (100);//delay
// chassis.turnToHeading(32, 700);//turn to align with farthest ring
// // delay (100);//delay
// chassis.moveToPoint(41, -4, 800, {true});//go to the alignment position
// // delay (100);//delay
// chassis.turnToHeading(29, 500);//turn to align with farther ring
// // delay (100);//delay
// chassis.moveToPoint(44, 17, 2000, {true});//go to farther ring
// Aload();//ladybrown to setting postition
// delay (1000);//delay
// chassis.moveToPoint(43, -6, 1000, {false});//go back to orginal position
// delay (100);//delay
// chassis.turnToHeading(90, 600);//turn to face stake
// hooks.move (0);
// delay (100);//delay
// chassis.moveToPoint(67, -6, 1000, {true, 50});//go forward to stake and intake one ring in the process
// chassis.waitUntil(100);
// delay (100);//delay
// Astake();//ladybrown up and score!
// delay(1000);//delay
// chassis.moveToPoint(45, -6, 2000, {false});//move back to align with 3 rings
// delay (500);//delay
// rest ();//ladybrown down
// delay (500);//delay
// chassis.turnToHeading(180, 600);//turn to 3 rings
// hooks.move (110);
// // delay (100);
// chassis.moveToPoint(45, -64, 3500, {true, 65});//go forward and intake 3 rings
// // delay (100);
// chassis.turnToHeading(48, 800);//turn to face last ring 9
// // delay (100);//delay
// chassis.moveToPoint(56, -54, 1500, {true});//go for last ring 58.7, -51
// // delay (100);//delay
// chassis.turnToHeading(-32, 800);//turn to face corner 348
// // delay (100);//delay
// chassis.moveToPoint(61, -63, 1000, {false, 90, 60});//move bacl into corner 63, -64
// // delay (100);//delay
// chassis.moveToPoint(53, -46, 3000, {true, 80, 50});//go forward to double line in order to align with left double ring
// intake.move (-127);
// hooks.move (-127);
// mogo.set_value(false);//drop it off
// delay (100);//delay
// chassis.turnToHeading(0, 350); //turn to align with double ring

// // // chassis.setPose (53, -46, 0);
// chassis.moveToPoint(54, 15, 2000, {true});//go forward to align with blue goal
// hooks.move (0);
// // delay (500);//delay
// chassis.turnToHeading(-54, 450);//turn to blue goal
// // delay (100);//delay
// chassis.moveToPoint(26, 38, 1800);//go to blue goal
// // delay (100);//delay
// chassis.turnToHeading(27, 450);//turn so intake faces right mogo with blue ring on it
// // delay (100);//delay
// intake.move (0);
// chassis.moveToPoint(26, 49, 800);//push it to wall to have room to turn
// // delay (100);
// chassis.turnToHeading(68, 500);//turn goal faces corner
// // delay (100);//delay
// chassis.moveToPoint(58, 55, 1400, {true, 127});//go to corner
// // delay (100);
// // chassis.turnToHeading (60, 500);//52
// // delay (100);
// chassis.moveToPose(31, 42, 60, 1300, {.forwards = false, .lead = .2, .maxSpeed = 100});//move it back to align with empty goal 31 41
// // delay (100);//delay
// chassis.turnToHeading(90, 550);//turn so back faces goal 90
// // delay (100);//delay
// chassis.moveToPoint(7, 40, 2000, {false, 65});//move to goal 3 38
// chassis.waitUntil(19);//wait until
// mogo.set_value(true);//clamp
// delay (100);//delay
// chassis.turnToHeading(129, 500);//turn to intake that one ring
// // delay (100);//delay
// chassis.moveToPoint(27, 22, 1000, {true, 100});//go to intake that one ring we missed
// intake.move (127);
// hooks.move (110);
// delay (100);//delay
// intake.move (127);
// chassis.turnToHeading(221, 700);//turn to go to middle
// delay (100);//delay
// chassis.moveToPoint(-36, -52, 2900, {true, 70});//go for middle ring under ladder AND EVERYTHING THAT IS IN THAT HORIZONTAL PATH...so intake 3 rings
// chassis.waitUntil(1);
// hooks.move(0);
// delay (100);//delay
// chassis.waitUntil(50);
// hooks.move(110);//hooks back on//intake bottom ring
// chassis.turnToHeading(201, 1000);//turn to intake bottom ring 552
// // delay (100);//delay
// chassis.moveToPoint(-36, -64, 1000, {true});//go to intake bottom ring -32, -71
// // delay (100);//delay
// chassis.turnToHeading(304, 1000);//turn to side ring
// // delay (100);//delay
// chassis.moveToPoint(-49, -58, 1000, {true, 90, 60});//go to intake side ring
// // delay (100);//delay
// chassis.turnToHeading(22, 1000);//turn so back faces corner
// // delay (100);//delay
// chassis.moveToPoint(-50, -65, 800, {false });//go back and drop it off
// intake.move(-127);
// hooks.move(-127);
// delay (100);//delay
// chassis.moveToPoint(-39, -33, 1000, {true});//go to intake that ring -38, -38

// // chassis.setPose(-36, -33, 22);
// mogo.set_value(false);//drop off goalfalse
// chassis.waitUntil(10);
// intake.move (-127);
// hooks.move (-127);
// Aload();//ladybrown load
// delay (100);//delay
// intake.move (127);
// hooks.move (110);
// chassis.turnToHeading(-29, 600);//turn so back faces goal
// // delay (100);//delay
// chassis.moveToPoint(-13, -58, 1500, {false, 70});//get goal
// chassis.waitUntil (26);
// mogo.set_value (true);
// delay (100);
// chassis.turnToHeading(-21, 450);//turn to face double line -21
// // delay (100);
// chassis.moveToPoint(-33, -9, 2000, {true, 100}); //go for double line
// // delay (100);
// chassis.turnToHeading(-90, 700);//turn to face stake
// hooks.move (0);
// // delay (100);//delay
// chassis.moveToPoint(-57, -9, 1000, {true,50});//go forward to stake and intake one ring in the process
// chassis.waitUntil(100);
// delay (100);//delay
// Astake();//ladybrown up and score!
// delay(500);//delay
// intake.move (0);
// chassis.moveToPoint(-42, -9, 700, {false});//back off to align with solo red rings nearest
// delay (700);//delay
// rest ();//ladybrown down
// delay (700);//delay
// chassis.turnToHeading(0, 800);//turn 90 degrees to face red ring
// // delay (100);//delay
// chassis.moveToPoint(-42, 12, 1000, {true});//go to intake that one solo red ring
// intake.move (127);
// hooks.move (110);
// // delay (100);//delay
// chassis.turnToHeading(90, 700, {.maxSpeed = 80});//turn 90 degrees and intake the other one
// // delay (100);//delay
// chassis.moveToPoint (-22, 16, 1000, {true, 100, 80}); //go to intake the other one
// // delay (100);
// chassis.turnToHeading(300, 800, {.maxSpeed = 70});//turn to intake the double rings that are vertical
// delay (100);//delay
// chassis.moveToPoint(-45, 36, 1000, {true, 100});//go to double ring
// delay (100);//delay
// chassis.turnToHeading(297, 800, {.maxSpeed = 80});//turn to face side ring
// // delay (100);//delay
// chassis.moveToPoint(-56, 37, 1000, {true, 100});//go to sidering
// // delay (100);//delay
// chassis.turnToHeading (390, 1000, {.maxSpeed = 80}); //turn to 3rd ring
// // delay (100);
// chassis.moveToPoint(-47, 47, 1000, {true, 100});//go to 3rd ring
// // delay (100);//delay
// chassis.turnToHeading(131, 700);//turn so back faces corner
// // delay (100);
// chassis.moveToPoint(-37, 35, 700, {true});//go forward a bit
// chassis.waitUntil(2);
// mogo.set_value(false);
// delay (100);
// intake.move (0);
// hooks.move(0);
// chassis.turnToHeading(300, 800);//turnso front faces goal
// chassis.moveToPoint(-59, 46,900, {true, 127, 127});//push goal in with intake
// intake.move (-127);
// chassis.moveToPoint(0, 0, 5000, {false, 110});
// Astake();//go hang

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

// //r doinker rush
// chassis.setPose (-28.3, -52.5 ,-20.5);
// awp.set_value(true);
// intake.move (127);
// chassis.moveToPoint(-44.2, -13.8, 1500, {true, 100, 50});//go to double rings
// delay (1500);
// chassis.moveToPoint(-30.6, -38.7,1800, {false, 60, 40}); //move back to align awp ring with middle ring
// delay (100);
// intake.move (0);
// chassis.turnToHeading(-136, 1000, {.maxSpeed = 40, .minSpeed = 20});//turn so back faces goal
// awp.set_value(false); //arm back up
// delay (100);
// chassis.moveToPoint(-18, -22, 700, {false, 70, 30}); //go to goal
// chassis.waitUntil(27);
// mogo.set_value(true);//clamp
// chassis.turnToHeading(-106, 800, {.maxSpeed = 70, .minSpeed = 40});//turn 90 to face 2 rings i dropped off
// delay (100);
// chassis.waitUntil(4);
// intake.move (127);
// chassis.waitUntil(1);//run hooks
// hooks.move (-127);
// chassis.moveToPoint(-52, -29, 1700, {true, 75, 40});//go intake those 2 first rings
// chassis.waitUntil(29);
// delay (100);
// chassis.moveToPoint(-44, -27, 800, {false, 80, 40});//go back -44, -27
// intake.move (127);
// hooks.move (-127);
// delay (100);
// chassis.turnToHeading(-234, 800, {.maxSpeed = 55, .minSpeed = 35});//turn to face oreload -235
// delay (100);//delay
// chassis.moveToPoint(-14, -38, 1000, {true, 110, 80});//go to preload -17, -36
// chassis.waitUntil(100);
// awp.set_value(true);
// delay (200);
// chassis.moveToPoint(-19, -35, 1500, {false, 70, 50});//move back slowly -17, -33
// chassis.waitUntil(100);
// awp.set_value(false);//up
// delay (100);//delay
// chassis.turnToHeading(-254, 800, {.maxSpeed = 50, .minSpeed = 30});//turn to get it -270
// delay (100);
// chassis.moveToPoint(-5, -34, 1000, {true, 80, 30});//go forward -7, -33 intake
// delay (100);
// hooks.move (0);
// mogo.set_value(false);
// chassis.turnToHeading(-226, 900, {.maxSpeed = 65, .minSpeed = 40});//turn to face b ring
// delay (100);//delay
// chassis.moveToPoint(15, -50, 770, {true, 120, 80});//push everything to other side
// intake.move (-127);
// mogo.set_value(false);
// delay (100);
// chassis.moveToPoint(0, -43, 1000, {false, 100, 80});//move back to get room 4, -45
// delay (100);
// chassis.turnToHeading(-361, 800, {.maxSpeed = 45, .minSpeed = 25});//turn to -361
// delay (100);
// chassis.moveToPoint(0, -65, 1000, {false, 70, 30});//go back to put it wall stake
// delay (100);
// hooks.move (-127);
// delay (570); //outtake onto stake
// chassis.moveToPoint(-2, -35, 1000, {true, 90, 80});//go forward to touch ladder
// intake.move (0);
// hooks.move (0);
// Astake ();


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
// hooks.move (127);//intake on
// chassis.moveToPoint(-43, -35, 2000, {true, 70, 30});//go to middle ring -43, -36
// delay (100);//delay
// chassis.moveToPoint(-30, -29, 1500, {false, 70, 30});//go back to have room for corner -26, -34
// delay (100);//delay
// chassis.turnToHeading(183, 1000, {.maxSpeed = 60, .minSpeed = 40});//turn to face wall 191
// delay (100);//delay
// chassis.moveToPoint(-27, -59, 1300, {true, 70, 30});//move forward to wall -27, 57
// delay (100);//delay
// chassis.turnToHeading(245, 1000, {.maxSpeed = 50, .minSpeed = 30});//turn to face rings 249
// delay (100);//delay
// chassis.moveToPoint(-49, -74, 2000, {true, 100, 80});//go forward to rings -53, -70
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
// // chassis.moveToPoint(-20, -19, 3000, {true, 80, 60});//go to touch ladder -19, -24
// // Astake();
// // mogo.set_value(false);//drop off goal
// // //elims
// chassis.moveToPoint(-40, -25, 3000, {true, 80, 60});
// mogo.set_value(false);
// chassis.turnToHeading(180, 800, {.maxSpeed = 70, .minSpeed = 30});

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
// chassis.moveToPoint(25, -21, 1000, {false, 70, 30});//go to goal -19, -33
// chassis.waitUntil (25);
// mogo.set_value (true);
// delay(100);
// chassis.turnToHeading(-265, 1000, {.maxSpeed = 60, .minSpeed = 40});//turn to face middle ring 264
// delay (100);//delay
// intake.move (127);
// hooks.move (127);//intake onintake.move (127);
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
// // //elims
// chassis.moveToPoint(40, -25, 3000, {true, 70, 30});
// mogo.set_value(false);
// chassis.turnToHeading(-180, 1000, {.maxSpeed = 50, .minSpeed = 30});

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
  bool tipperState = false;
//   bool colorside = false ; //false is blue, true is red
  right_side_motors.set_brake_mode(E_MOTOR_BRAKE_COAST);
  left_side_motors.set_brake_mode (E_MOTOR_BRAKE_COAST);
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
       hooks.move(-110);


    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake.move(127);
       hooks.move(110);
    } 
      else {
      intake.move(0);
       hooks.move(0);
     }


    if (master.get_digital_new_press (pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    tipperState = !tipperState;
    tipper.set_value(tipperState);
    }


    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      mogoState = !mogoState;
      mogo.set_value(mogoState);
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      awpState = !awpState;
      awp.set_value(awpState);
    }

    if (master.get_digital_new_press (pros::E_CONTROLLER_DIGITAL_B)){
        intliftState = !intliftState;
        intlift.set_value(intliftState);
    }

    if (master.get_digital_new_press (pros::E_CONTROLLER_DIGITAL_UP)){
        armcState = !armcState;
        armc.set_value(armcState);
    }


    }
}