#include "main.h"
#include "lemlib/api.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER); // controller, name: controller

// drive motors
// middle is the bottom motor, back is the top motor
pros::Motor lF(-1, pros::E_MOTOR_GEAR_BLUE); // left front motor. port 12, reversed
pros::Motor lM(2, pros::E_MOTOR_GEAR_BLUE); // left middle motor. port 11, reversed
pros::Motor lB(-3, pros::E_MOTOR_GEAR_BLUE); // left back motor. port 1, reversed
pros::Motor rF(10, pros::E_MOTOR_GEAR_BLUE); // right front motor. port 2
pros::Motor rM(-9, pros::E_MOTOR_GEAR_BLUE); // right middle motor. port 11
pros::Motor rB(8, pros::E_MOTOR_GEAR_BLUE); // right back motor. port 13

// motor groups
pros::MotorGroup leftMotors({lF, lM, lB}); // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group

// intake and cata motors
pros::Motor cata(4, pros::E_MOTOR_GEAR_RED); // cata motor, port 10
pros::Motor intake(5, pros::E_MOTOR_GEAR_BLUE); // intake motor, port 19

// pneumatics
pros::ADIDigitalOut pto('C'); // PTO pneumatic, port C
pros::ADIDigitalOut backWingsL('B'); // PTO pneumatic, port A
pros::ADIDigitalOut backWingsR('E'); // PTO pneumatic, port B
pros::ADIDigitalOut frontWingsL('A'); // PTO pneumatic, port G
pros::ADIDigitalOut frontWingsR('F'); // PTO pneumatic, port E
pros::ADIDigitalOut ratchet('D'); // PTO pneumatic, port F

// Inertial Sensor on port 18
pros::Imu imu(11);

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
lemlib::ControllerSettings linearController(18, // proportional gain (kP) 24
                                            0, // integral gain (kI)
                                            15, // derivative gain (kD) 20
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             75, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             200, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(4, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            25, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             75, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             200, // large error range timeout, in milliseconds
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
    // first cycle
    chassis.setPose(0, 0, 0);
    chassis.turnTo(0, -30, 9000);
    chassis.moveToPose(0, -30, 180, 9000);

    // 2nd cycle
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.turnTo(0, -30, 9000);
    chassis.moveToPose(0, -30, 180, 9000);
 }


 void FarSideAuton(){
    // far side auton
    chassis.setPose(32, -54, 0);

    // move to the middle triball (eg. triball in the middle of the field, on our side)
    frontWingsR.set_value(true);
    intake.move(127);
    chassis.moveToPose(6, -3, 305, 3000);

    // front wings in
    chassis.waitUntil(10);
    frontWingsR.set_value(false);

    // stop intake
    chassis.waitUntilDone();
    intake.brake();

    // front wings out
    chassis.waitUntil(2);
    frontWingsL.set_value(true);
    frontWingsR.set_value(true);

    //score in net, 2 triballs, up in middle
    chassis.moveToPose(45, -3, 90, 3000);
    intake.move(-127);

    // front wings in
    chassis.waitUntilDone();
    frontWingsL.set_value(false);
    frontWingsR.set_value(false);

    // go to bottom middle triball
    intake.move(127);
    chassis.moveToPose(8, -20, 230, 3000);

    // go to bottom
    chassis.moveToPose(42, -42, 135, 3000);

    // intake out the triball in middle
    chassis.waitUntil(8);
    intake.move(-127);

    // intake in the bottom triball from the bottom, but not under bar.
    chassis.waitUntilDone();
    intake.move(127);

    // go under bar
    chassis.moveToPose(7, -52, 270, 2000);

    // go backwards a bit 
    chassis.moveToPose(35, -53, 270, 1000, {.forwards = false});

    // turn 180 degrees, and intake out after
    chassis.turnTo(40, -53, 600);
    chassis.waitUntilDone();
    intake.move(-127);

    // go curve to the goal
    chassis.moveToPose(46, -53, 90, 9000);

    // go curve to matchload out position
    chassis.moveToPose(63, -40, 45, 9000);

    // get matchload out
    chassis.turnTo(63, -30, 9000);
    backWingsR.set_value(true);

    // turn back to get the triballs in range to score
    chassis.turnTo(68, -35, 9000, false);

    // score the 2-3 triballs in
    chassis.moveToPose(66, -20, 0, 9000, {.forwards = false});

    // go score
    // chassis.moveToPose(64, -20, 0, 2000);

    // // go back a bit
    // chassis.moveToPose(66, -50, 340, 1000, {.forwards = false});

    // // score again
    // chassis.moveToPose(66, -20, 0, 1000, {.forwards = false});
 }


 void CloseSideAuton(){
    // close side auton
    chassis.setPose(50, 55, -45);
    backWingsL.set_value(true);
    backWingsR.set_value(true);
    pros::delay(100);
    rightMotors.move(127);
    pros::delay(1500);
    backWingsL.set_value(false);
    backWingsR.set_value(false);
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
    chassis.moveToPose(-64.5, -44.2, 70, 1000, {.forwards = false});

    // launch the cata
    chassis.waitUntilDone();
    intake.brake();
    pros::millis();
    leftMotors.move(-1);
    rightMotors.move(-1);
    cata.move(127);
    pros::delay(30000);

    //go to corner middle triball red side, on the way/in the middle
    cata.brake();
    chassis.moveToPose(-60, -40, 65, 300);
    chassis.moveToPose(-35, -28, 90, 1200, {.forwards = false});

    //go to corner middle triball red side, in the corner
    chassis.moveToPose(-15.5, -30, 90, 1100, {.forwards = false});
    chassis.waitUntil(35);
    backWingsL.set_value(true);
    backWingsR.set_value(true);

    //midpoint at the middle bar
    chassis.moveToPose(-16, 25, 0, 1500, {.forwards = false});
    //move across middle bar
    chassis.moveToPose(-16, 43, 0, 1450, {.forwards = false, .maxSpeed = 70});

    //chassis go to in front of the blue hang, at matchload side
    
    chassis.moveToPose(-60, 38, 90, 1700, {.forwards = false});
    chassis.waitUntil(10);
    backWingsL.set_value(false);
    backWingsR.set_value(false);

    //chassis go under blue bar, pushing all triballs
    chassis.waitUntilDone();
    chassis.moveToPose(-25, 60, 270, 1600, {.forwards = false});

    //chassis go on way to push to the side
    chassis.waitUntilDone();
    intake.move(-127);
    chassis.moveToPose(53, 59, 270, 1450,{.forwards = false});

    //chassis go push to side and then go back
    chassis.moveToPose(60, 40, 135, 750,{.forwards = false});
    chassis.moveToPose(60, 24, 180, 700,{.forwards = false, .lead = 0.2 });

    //chassis go up a bit, so there's enough space to turn to not get stuck under the goal
    chassis.moveToPose(60, 35, 0, 600);
    // ends up at the goal side

    //chassis turn to corner triballs at red side blue hang
    chassis.moveToPose(10, 27, 180, 3000);
    chassis.waitUntil(10);
    frontWingsL.set_value(true);
    intake.brake();

    //chassis go a bit to the middle for grouping
    chassis.moveToPose(12, 3, 180, 3000);
    frontWingsR.set_value(true);

    //chassis score middle
    chassis.moveToPose(34, 1, -90,5000);

    //chassis back up for second push in middle (chassis go to middle with back fwd)
    chassis.waitUntilDone();
    frontWingsL.set_value(false);
    frontWingsR.set_value(false);
    backWingsL.set_value(true);
    chassis.moveToPose(10,5,100,5000, {.forwards = false});

    //chassis to down from score side
    chassis.moveToPose(10, -29, 180, 5000, {.forwards = false});
    backWingsR.set_value(true);

    //chassis swing turn (at least as close as i cam get it) to push all the triballs (tommy skills)
    chassis.waitUntilDone();
    pros::millis();
    leftMotors.move(-127);
    pros::delay(700);

    //chassis score 2nd time middle
    chassis.moveToPose(34, -2, -90, 5000, {.forwards = false});

    //chassis move up a bit
    chassis.moveToPose(10, -2, -90, 5000);
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
    FarSideAuton(); //this is the one that scores in the net, the 5 ball
    // CloseSideAuton(); //this is the one that doesn't score, the winpoint.
    // SkillsAuton();
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
            backWingsL.set_value(toggleBackWings);
            backWingsR.set_value(toggleBackWings);
            pros::delay(500);
        }

        //shift wings
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            toggleFrontWings = !toggleFrontWings;
            frontWingsL.set_value(toggleFrontWings);
            frontWingsR.set_value(toggleFrontWings);
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
