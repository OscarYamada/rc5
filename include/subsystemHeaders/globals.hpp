#include "main.h"
#include "lemlib/api.hpp"
//recursion in this context is not a problem. ignore the error


// controller header
extern pros::Controller controller;

// drive motors headers
// middle is thxe bottom motor
extern pros::Motor lF; // left front motor. port 12, reversed
extern pros::Motor lM; // left middle motor. port 11, reversed
extern pros::Motor lB; // left back motor. p.ort 1, reversed
extern pros::Motor rF; // right front motor. port 2
extern pros::Motor rM; // right middle motor. port 11
extern pros::Motor rB; // right back motor. port 13

// motor groups headers
extern pros::MotorGroup leftMotors; // left motor group
extern pros::MotorGroup rightMotors; // right motor group

// intake and cata motors
extern pros::Motor cata;
extern pros::Motor intake;

// pneumatics
extern pros::ADIDigitalOut pto; // PTO pneumatic, port A
extern pros::ADIDigitalOut leftBackWings; // PTO pneumatic, port B
extern pros::ADIDigitalOut rightBackWings; // PTO pneumatic, port C

// Inertial Sensor on port 21
extern pros::Imu imu;

// tracking wheels
// // horizontal tracking wheel encoder. Rotation sensor, port 15, reversed (negative signs don't work due to a pros bug)
// pros::Rotation horizontalEnc(15, true);
// // horizontal tracking wheel. 2.75" diameter, 3.7" offset, back of the robot (negative)
// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -3.7);

// drivetrain settings
extern lemlib::Drivetrain drivetrain;

// lateral motion controller
extern lemlib::ControllerSettings linearController;

// angular motion controller
extern lemlib::ControllerSettings angularController;

// sensors for odometry
// note that in this example we use internal motor encoders (IMEs), so we don't pass vertical tracking wheels
extern lemlib::OdomSensors sensors;

// create the chassis
extern lemlib::Chassis chassis;