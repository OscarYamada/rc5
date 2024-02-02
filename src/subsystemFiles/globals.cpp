#include "main.h"
#include "lemlib/api.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drive motors
// middle is thxe bottom motor
pros::Motor lF(-20, pros::E_MOTOR_GEAR_BLUE); // left front motor. port 12, reversed
pros::Motor lM(-17, pros::E_MOTOR_GEAR_BLUE); // left middle motor. port 11, reversed
pros::Motor lB(18, pros::E_MOTOR_GEAR_BLUE); // left back motor. port 1, reversed
pros::Motor rF(11, pros::E_MOTOR_GEAR_BLUE); // right front motor. port 2
pros::Motor rM(13, pros::E_MOTOR_GEAR_BLUE); // right middle motor. port 11
pros::Motor rB(-12, pros::E_MOTOR_GEAR_BLUE); // right back motor. port 13

// motor groups
pros::MotorGroup leftMotors({lF, lM, lB}); // left motor group
pros::MotorGroup rightMotors({	rF, rM, rB}); // right motor group

// intake and cata motors
pros::Motor cata(10, pros::E_MOTOR_GEAR_RED); // cata motor, port 10
pros::Motor intake(19, pros::E_MOTOR_GEAR_BLUE); // intake motor, port 19

// pneumatics
pros::ADIDigitalOut pto('A'); // PTO pneumatic, port A
pros::ADIDigitalOut leftBackWings('B'); // PTO pneumatic, port B
pros::ADIDigitalOut rightBackWings('C'); // PTO pneumatic, port C

// Inertial Sensor on port 21
pros::Imu imu(21);

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

// lateral motion controller
lemlib::ControllerSettings linearController
(
10, // proportional gain (kP)
0, // integral gain (kI)
3, // derivative gain (kD)
3, // anti windup
1, // small error range, in inches
100, // small error range timeout, in milliseconds
3, // large error range, in inches
500, // large error range timeout, in milliseconds
20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController
(
2, // proportional gain (kP)
0, // integral gain (kI)
10, // derivative gain (kD)
3, // anti windup
1, // small error range, in degrees
100, // small error range timeout, in milliseconds
3, // large error range, in degrees
500, // large error range timeout, in milliseconds
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
