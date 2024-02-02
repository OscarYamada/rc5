#include "main.h"
#include "lemlib/api.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

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
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    //near side auton, the one that has the goal on the other side
    // void NearSide()

    //far side auton, the one that has the goal on the same side
    // void FarSide();
}

/**
 * Runs in driver control
 */

void opcontrol() {
    bool toggleWings = false;
    bool togglePTO = false;
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the chassis with curvature drive
        chassis.tank(leftY, rightX);

		// move the catapult/lift
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			cata.move(127);
		}
		else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
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

        //shift wings
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            toggleWings = !toggleWings;
            leftBackWings.set_value(toggleWings);
            rightBackWings.set_value(toggleWings);
            pros::delay(200);
        }

        //shift pto
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
            togglePTO = !togglePTO;
            leftBackWings.set_value(togglePTO);
            rightBackWings.set_value(togglePTO);
            pros::delay(200);
        }
        
        // delay to save resources
        pros::delay(10);
    }
}