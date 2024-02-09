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

    chassis.moveToPose(10, -16.5, 315, 2000);
    intake.move(127);
    chassis.waitUntilDone();
    intake.brake();
    chassis.turnTo(43, -20, 700);
    chassis.waitUntilDone();
    chassis.moveToPose(43, -20, 90, 1300);
    intake.move(-127);

    // turn to L shape bottom triball and go
    chassis.waitUntilDone();
    intake.move(127);
    chassis.moveToPose(11, -31, 225, 1500);

    // turn to preload
    chassis.moveToPose(40, -40, 135, 800);
    chassis.waitUntil(8);
    intake.move(-127);

    // turn preload/bottom, and sweep
    chassis.moveToPose(28, -53, 270, 900);
    chassis.waitUntilDone();
    intake.move(127);
    chassis.turnTo(15, -53, 800);
    chassis.moveToPose(15, -53, 270, 2000);

    // go backwards to preload
    chassis.moveToPose(25, -53, 270, 1000, {.forwards = false});
    chassis.turnTo(40, -53, 1000);
    chassis.waitUntilDone();
    intake.move(-127);
    chassis.moveToPose(55, -53, 90, 1000);
    chassis.moveToPose(60, -20, 0, 2000);

    chassis.moveToPose(60, -40, 180, 1000, {.forwards = false});
    chassis.turnTo(60, -20, 1000);
    chassis.moveToPose(60, -15, 180, 1000, {.forwards = false});
 }

 void CloseSideAuton(){
    // close side auton
    backWings.set_value(true);
    pros::delay(100);
    rightMotors.move(127);
    pros::delay(1500);
 }

void SkillsAuton(){
    // skills:
    // go into net
    chassis.setPose(-46.74323, -49.84659, -17.973);
    intake.move(-127);
    chassis.moveToPose(-60,-22,0, 800, {.lead = 0.1});

    // go to cata launch, and launch cata 30s
    chassis.moveToPose(-60, -45.5, 60, 1500, {.forwards = false});
    chassis.waitUntilDone();
    intake.brake();
    // cata.move(127);
    // pros::delay(30000);

    //go to corner middle triball red side, on the way/in the middle
    chassis.moveToPose(-35, -17, 90, 1500, {.forwards = false});

    //go to corner middle triball red side, in the corner
    chassis.moveToPose(-16, -17, 90, 9000, {.forwards = false});
    chassis.waitUntil(35);
    backWings.set_value(true);

    //move across middle beam, matchloading side
    chassis.moveToPose(-16, 46, 0, 9000, {.forwards = false});

    //chassis go to in front of the blue hang, at matchload side
    chassis.moveToPose(-40, 52, 90, 9000, {.forwards = false});
    chassis.waitUntil(10);
    backWings.set_value(false);
    frontWings.set_value(true);

    //chassis go under blue bar, pushing all triballs with wings out
    chassis.waitUntilDone();
    frontWings.set_value(false); // set front wings out
    chassis.moveToPose(0, 65, 90, 9000);
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
    // FarSideAuton();
    // CloseSideAuton();
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
