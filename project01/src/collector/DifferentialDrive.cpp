#include "DifferentialDrive.h"
#include <math.h>

float DifferentialDrive::posX;
float DifferentialDrive::posY;
float DifferentialDrive::angle;
int16_t DifferentialDrive::leftSpeed;
int16_t DifferentialDrive::rightSpeed;

void DifferentialDrive::reset(float x, float y, float a) {
    posX = x;
    posY = y;
    angle = a;
}

void DifferentialDrive::setLeftSpeed(int16_t speed) {
    Zumo32U4Motors::setLeftSpeed(speed);
    leftSpeed = speed;
}

void DifferentialDrive::setRightSpeed(int16_t speed) {
    Zumo32U4Motors::setRightSpeed(speed);
    rightSpeed = speed;
}

void DifferentialDrive::flipLeftMotor(bool flip) {
    Zumo32U4Motors::flipLeftMotor(flip);
    leftSpeed *= -1;
}

void DifferentialDrive::flipRightMotor(bool flip) {
    Zumo32U4Motors::flipRightMotor(flip);
    rightSpeed *= -1;
}

void DifferentialDrive::drive() {
    float z = (WHEEL_RADIUS * (leftSpeed + rightSpeed) / 2);
    float x_dot = z * cos(angle);
    float y_dot = z * sin(angle);
    float angle_dot = (WHEEL_RADIUS * (leftSpeed - rightSpeed) / WHEEL_DISTANCE);

    posX += x_dot;
    posY += y_dot;
    angle += angle_dot;

    /* TODO
     * this is not working, needs to know time since last call
     * such that concrete differntial equation is correct */
}

DifferentialDrive::DifferentialDrive() = default;

