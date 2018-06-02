#include <Arduino.h>
#include "DifferentialDrive.h"

DifferentialDrive motor = DifferentialDrive(10, 10, 0, 0, 0);
int16_t leftSpeed = 50;
int16_t rightSpeed = -50;
int ByteReceived;


void drive() {
    motor.drive(leftSpeed, rightSpeed);
}

void setup() {
    DifferentialDrive::setLeftSpeed(leftSpeed);
    DifferentialDrive::setRightSpeed(rightSpeed);

    Serial1.begin(9600);
    Serial1.println("--- Start Serial Monitor ---");
    Serial1.println("(Decimal)(Hex)(Character)");
    Serial1.println();
}

void loop() {
    drive();

    if (Serial1.available() > 0)
    {
        ByteReceived = Serial1.read();

        Serial1.print(ByteReceived);
        Serial1.print("        ");
        Serial1.print(ByteReceived, HEX);
        Serial1.print("       ");
        Serial1.print(char(ByteReceived));
        Serial1.println();
    }
}


