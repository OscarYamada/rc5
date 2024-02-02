#include "main.h"
#include "lemlib/api.hpp"

//near side auton, the one that has the goal on the other side
void NearSide(){

}

//far side auton, the one that has the goal on the same side
void FarSide(){

}


void DriveStraight(int dis, int spd, int dir) {
    int target = dis; // Target position based on the specified distance
    int integralL;
    int integralR;
    int prevErrorL;
    int prevErrorR;

    leftMotors.tare_position();
    rightMotors.tare_position();

    // PID loop
    while(true) {
        int lPos = (lF.get_position() + lM.get_position() + lB.get_position()) / 3;
        int rPos = (rF.get_position() + rM.get_position() + rB.get_position()) / 3;

        int errorL = target - lPos;
        int errorR = target - rPos;

        // Calculate the correction factor based on the difference in motor positions
        int correction = lPos - rPos;

        // Integral is the sum of errors over time
        integralL = integralL + errorL;
        integralR = integralR + errorR;

        // Derivative is the change in error from the last loop iteration
        int derivativeL = errorL - prevErrorL;
        int derivativeR = errorR - prevErrorR;

        // Calculate the motor speed using the PID constants
        int speedL = (errorR * linearController.kP) + (integralL * linearController.kI) + (derivativeL * linearController.kD);
        int speedR = (errorL * linearController.kP) + (integralR * linearController.kI) + (derivativeR * linearController.kD);

        // Set the motor speeds with the correction factor
        leftMotors.move(speedL * dir);
        rightMotors.move(speedR * dir);

        // Update the previous error
        prevErrorL = errorL;
        prevErrorR = errorR;

        // Break the loop if the error is close to zero
        if(abs(errorL) < 50 && abs(errorR) < 50) {
            break;
        }

        // Delay to simulate a 'time' parameter
        pros::delay(10);
    }
}

void PointTurn(int angle, int spd, int left) {
    int target = angle; // Target position based on the specified angle
    int integralL;
    int integralR;
    int prevErrorL;
    int prevErrorR;

    // PID loop
    while(true) {

        int lPos = (lF.get_position() + lM.get_position() + lB.get_position()) / 3;
        int rPos = (rF.get_position() + rM.get_position() + rB.get_position()) / 3;

        int errorL = target - lPos;
        int errorR = target - rPos;

        // Integral is the sum of errors over time
        integralL = integralL + errorL;
        integralR = integralR + errorR;

        // Derivative is the change in error from the last loop iteration
        int derivativeL = errorL - prevErrorL;
        int derivativeR = errorR - prevErrorR;

        // Calculate the motor speed using the PID constants
        int speedL = (errorR * linearController.kP) + (integralL * linearController.kI) + (derivativeL * linearController.kD);
        int speedR = (errorL * linearController.kP) + (integralR * linearController.kI) + (derivativeR * linearController.kD);

        // Set the motor speeds with the correction factor
        leftMotors.move(speedL * left);
        rightMotors.move(-speedR * left);

        // Update the previous error
        prevErrorL = errorL;
        prevErrorR = errorR;

        // Break the loop if the error is close to zero
        if(abs(errorL) < 50 && abs(errorR) < 50) {
            break;
        }

        // Delay to simulate a 'time' parameter
        pros::delay(10);
    }
}


void SwingTurn(int angle, int spd, bool left) {
    int target = angle; // Target position based on the specified angle
    int integral;
    int prevError;

    // PID loop
    while(true) {
        int pos = lF.get_position();

        int error = target - pos;

        // Integral is the sum of errors over time
        integral = integral + error;

        // Derivative is the change in error from the last loop iteration
        int derivative = error - prevError;

        // Calculate the motor speed using the PID constants
        int speed = (error * angularController.kP) + (integral * angularController.kI) + (derivative * angularController.kD);

        // Set the motor speeds with the correction factor
        if(left) {
            leftMotors.move(speed);
            rightMotors.move(0);
        } else {
            leftMotors.move(0);
            rightMotors.move(speed);
        }

        // Update the previous error
        prevError = error;

        // Break the loop if the error is close to zero
        if(abs(error) < 5) {
            break;
        }

        // Delay to simulate a 'time' parameter
        pros::delay(10);
    }
}