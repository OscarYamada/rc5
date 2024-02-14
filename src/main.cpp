#include "main.h"
#include "lemlib/api.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drive motors
// middle is the bottom motor, back is the top motor
pros::Motor lF(-1, pros::E_MOTOR_GEAR_BLUE); // left front motor. port 12, reversed
pros::Motor lM(-11, pros::E_MOTOR_GEAR_BLUE); // left middle motor. port 11, reversed
pros::Motor lB(12, pros::E_MOTOR_GEAR_BLUE); // left back motor. port 1, reversed
pros::Motor rF(10, pros::E_MOTOR_GEAR_BLUE); // right front motor. port 2
pros::Motor rM(20, pros::E_MOTOR_GEAR_BLUE); // right middle motor. port 11
pros::Motor rB(-19, pros::E_MOTOR_GEAR_BLUE); // right back motor. port 13

// motor groups
pros::MotorGroup leftMotors({lF, lM, lB}); // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group

// intake and cata motors
pros::Motor cata(13, pros::E_MOTOR_GEAR_RED); // cata motor, port 10
pros::Motor intake(2, pros::E_MOTOR_GEAR_BLUE); // intake motor, port 19

// pneumatics
pros::ADIDigitalOut pto('C'); // PTO pneumatic, port C
pros::ADIDigitalOut backWings('A'); // PTO pneumatic, port A
pros::ADIDigitalOut frontWings('G'); // PTO pneumatic, port G
pros::ADIDigitalOut ratchet('F'); // PTO pneumatic, port F

// Inertial Sensor on port 18
pros::Imu imu(18);

// tracking wheels
// // horizontal tracking wheel encoder. Rotation sensor, port 15, reversed (negative signs don't work due to a pros bug)
// pros::Rotation horizontalEnc(15, true);
// // horizontal tracking wheel. 2.75" diameter, 3.7" offset, back of the robot (negative)
// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -3.7);

// drivetrain settings
lemlib::Drivetrain drivetrain
(
&leftMotors, // left motor group
&rightMotors, // right motor group
12, // 10 inch track width
lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
300, // drivetrain rpm is 300
8 // chase power is 2. If we had traction wheels, it would have been 8
);

// just remember kP + 2, then kD + 5 until its accurate then repeat
// lateral motion controller
// lateral motion controller
lemlib::ControllerSettings linearController(18, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            30, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             10, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             300, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            55, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             10, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             300, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)

);

// sensors for odometry
// note that in this example we use internal motor encoders (IMEs), so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(
nullptr, // vertical tracking wheel 1, set to null
nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
nullptr, // horizontal tracking wheel 1
nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
&imu // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);


/**
 * Here is all the void functions that normally would be outside the file
 *
 * It is reccomended to keep these functions isolated, and only run
 * once per time.
 */

 void PIDTune(){
    chassis.setPose(0, 0, 0);
    chassis.turnTo(0, -10, 5000);
    chassis.moveToPose(0, -10, 180, 5000);
 }


 void FarSideAuton(){
    // far side auton

    chassis.setPose(32, -54, 0);

    // move to the middle triball (eg. triball in the middle of the field, on our side)
    chassis.moveToPose(10, -5, 305, 2000);
    frontWings.set_value(true);
    intake.move(127);
    chassis.waitUntil(3);
    frontWings.set_value(false);

    chassis.waitUntilDone();
    intake.brake();

    // go score 2 of the triballs
    chassis.waitUntil(4);
    frontWings.set_value(true);
    chassis.moveToPose(45, -5, 90, 1300);
    intake.move(-127);

    // turn to bottom middle triball and go
    chassis.waitUntilDone();
    frontWings.set_value(false);
    intake.move(127);
    chassis.moveToPose(7, -23, 230, 1000);

    // go to bottom
    chassis.moveToPose(42, -45, 135, 1000);
    chassis.waitUntil(8);
    intake.move(-127);

    // sweep below hang
    chassis.moveToPose(43, -54, 270, 1000);
    chassis.waitUntilDone();
    intake.move(127);
    chassis.turnTo(12, -57, 800);
    chassis.moveToPose(12, -57, 270, 2000);

    // go backwards a bit and then turn 180 degrees
    chassis.moveToPose(25, -57, 270, 1000, {.forwards = false});
    chassis.turnTo(40, -57, 600);
    chassis.waitUntilDone();
    intake.move(-127);

    // go curve to the goal
    chassis.moveToPose(55, -57, 90, 1000);
    chassis.moveToPose(64, -20, 0, 2000);

    chassis.moveToPose(66, -38, 340, 1000, {.forwards = false});
    chassis.moveToPose(66, -15, 0, 1000, {.forwards = false});
 }


 void CloseSideAuton(){
    // close side auton
    chassis.setPose(50, 55, -45);
    backWings.set_value(true);
    pros::delay(100);
    rightMotors.move(127);
    pros::delay(1500);
    backWings.set_value(false);
    pros::delay(1000);
    chassis.turnTo(60, 22, 2000, false);
    chassis.moveToPose(60, 22, 0, 2000, {.forwards = false});
    chassis.moveToPose(60, 30, 0, 1000);
    chassis.moveToPose(60, 22, 0, 1000, {.forwards = false});
 }

// SKILLS AUTON AND MATCH AUTON BARRIER

// SKILLS AUTON AND MATCH AUTON BARRIER

// SKILLS AUTON AND MATCH AUTON BARRIER

// SKILLS AUTON AND MATCH AUTON BARRIER

// SKILLS AUTON AND MATCH AUTON BARRIER

// SKILLS AUTON AND MATCH AUTON BARRIER

// SKILLS AUTON AND MATCH AUTON BARRIER

// SKILLS AUTON AND MATCH AUTON BARRIER

// SKILLS AUTON AND MATCH AUTON BARRIER

// SKILLS AUTON AND MATCH AUTON BARRIER

// SKILLS AUTON AND MATCH AUTON BARRIER

// SKILLS AUTON AND MATCH AUTON BARRIER

// SKILLS AUTON AND MATCH AUTON BARRIER

// SKILLS AUTON AND MATCH AUTON BARRIER

// SKILLS AUTON AND MATCH AUTON BARRIER

// SKILLS AUTON AND MATCH AUTON BARRIER

void SkillsAuton(){
    // skills:
    // go into net
    chassis.setPose(-37, -56, -45);
    chassis.moveToPose(-56, -27, 0, 1400, {.lead = 0.2});

    // go to cata launch, and launch cata 30s
    chassis.moveToPose(-64.5, -44.2, 65, 3000, {.forwards = false});

    // launch the cata
    chassis.waitUntilDone();
    intake.brake();

    pros::millis();
    leftMotors.move(-1);
    rightMotors.move(-1);
    // cata.move(127);
    pros::delay(3000);

    //go to corner middle triball red side, on the way/in the middle
    // cata.brake();
    chassis.moveToPose(-35, -28, 90, 1200, {.forwards = false});

    //go to corner middle triball red side, in the corner
    chassis.moveToPose(-15.5, -30, 90, 1100, {.forwards = false});
    chassis.waitUntil(35);
    backWings.set_value(true);

    //midpoint at the middle bar
    chassis.moveToPose(-15.5, 20, 0, 1300, {.forwards = false});
    //move across middle bar
    chassis.moveToPose(-15.5, 43, 0, 1200, {.forwards = false, .maxSpeed = 70});

    //chassis go to in front of the blue hang, at matchload side
    
    chassis.moveToPose(-60, 38, 90, 2000, {.forwards = false});
    chassis.waitUntil(10);
    backWings.set_value(false);

    //chassis go under blue bar, pushing all triballs
    chassis.waitUntilDone();
    chassis.moveToPose(-25, 58, 270, 1600, {.forwards = false});

    //chassis go on way to push to the side
    chassis.waitUntilDone();
    intake.move(-127);
    chassis.moveToPose(53, 58, 270, 1500,{.forwards = false});

    //chassis go push to side and then go back
    chassis.moveToPose(60, 40, 135, 700,{.forwards = false, .lead = 0.2 });
    chassis.moveToPose(60, 24, 180, 700,{.forwards = false, .lead = 0.2 });

    //chassis go up a bit, so there's enough space to turn to not get stuck under the goal
    chassis.moveToPose(63, 26, 0, 1000);
    // ends up at the goal side

    //chassis turn to corner triballs at red side blue hang
    chassis.moveToPose(21, 27, -90, 3000, {.forwards = false});
    chassis.waitUntil(10);
    backWings.set_value(true);
    intake.brake();

    //chassis score middle
    chassis.moveToPose(43,0,130,5000,{.forwards = false});

    //chassis back up for second push in middle (chassis go to middle with back fwd)
    chassis.waitUntilDone();
    backWings.set_value(false);
    chassis.moveToPose(25,5,100,5000, {.forwards = false});

    //chassis to down from score side
    chassis.moveToPose(25, -35, 180, 5000, {.forwards = false});
    chassis.waitUntil(4);
    backWings.set_value(true);

    //chassis score 2nd time middle
    chassis.moveToPose(43, -5, -115, 5000, {.forwards = false});
}



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    imu.tare();
    chassis.calibrate(); // calibrate sensors
    chassis.setPose(0,0,0);

    cata.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    leftMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging


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
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
// ASSET(closeside1_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    // PIDTune();
    // FarSideAuton(); //this is the one that scores in the net, the 5 ball
    // CloseSideAuton(); //this is the one that doesn't score, the winpoint.
    SkillsAuton();
}

/**
 * Runs in driver control
 */

void opcontrol() {
    bool toggleFrontWings = false;
    bool toggleBackWings = false;
    bool togglePTO = false;
    bool toggleRatchet = false;
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the chassis with curvature drive
        chassis.tank(leftY, rightX);

		// move the catapult/lift
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			cata.move(127);
		}
		else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			cata.move(-127);
		}
		else{
			cata.brake();
		}

		// move the intake
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intake.move(127);
		}
		else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			intake.move(-127);
        }
		else{
			intake.brake();
		}   

        //shift back wings
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            toggleBackWings = !toggleBackWings;
            backWings.set_value(toggleBackWings);
            pros::delay(500);
        }

        //shift wings
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            toggleFrontWings = !toggleFrontWings;
            frontWings.set_value(toggleFrontWings);
            pros::delay(500);
        }

        //shift pto
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            togglePTO = !togglePTO;
            pto.set_value(togglePTO);
            pros::delay(500);
        }

        //shift ratchet
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
            toggleRatchet = !toggleRatchet;
            ratchet.set_value(toggleRatchet);
            pros::delay(500);
        }
        
        // delay to save resources
        pros::delay(10);
    }
}
