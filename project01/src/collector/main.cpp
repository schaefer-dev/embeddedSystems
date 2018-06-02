#include <Arduino.h>
#include "DifferentialDrive.h"

DifferentialDrive motor = DifferentialDrive(10, 10, 0, 0, 0);
int16_t leftSpeed = 50;
int16_t rightSpeed = -50;

void drive() {
    motor.drive(leftSpeed, rightSpeed);
}

void setup() {
    DifferentialDrive::setLeftSpeed(leftSpeed);
    DifferentialDrive::setRightSpeed(rightSpeed);
}

void loop() {
    drive();
    delay(1000);
}


