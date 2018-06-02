#include <Arduino.h>
#include "DifferentialDrive.h"
#include <math.h>

const int16_t baseSpeed = 50;
int16_t leftSpeed = baseSpeed;
int16_t rightSpeed = -baseSpeed;

float destinationX;
float destinationY;

int ByteReceived;

float getAngle(float currentX, float currentY) {
    float vecX = destinationY - currentX;
    float vecY = destinationY - currentY;
    float angle = atan(vecY / vecX);
    if (vecX < 0) angle += 180;
    if (angle > 180) angle = angle - 360;
    if (angle < - 180) angle = 360 - angle;
    return angle;
}

void thetaCorrection() {
    float currentX = DifferentialDrive::posX;
    float currentY = DifferentialDrive::posY;

    // check if destination reached
    if (currentX == destinationX && currentY == destinationY) {
        DifferentialDrive::setRightSpeed(0);
        DifferentialDrive::setLeftSpeed(0);
        return;
    }

    // turn towards destination
    float angle = getAngle(currentX, currentY);
    float deltaAngle = angle - DifferentialDrive::angle;
    if (deltaAngle > 0) {
        // turn left
        DifferentialDrive::setRightSpeed(baseSpeed);
        DifferentialDrive::setRightSpeed(-baseSpeed);
    } else {
        // turn right
        DifferentialDrive::setRightSpeed(-baseSpeed);
        DifferentialDrive::setRightSpeed(baseSpeed);
    }
}

void setup() {
    // initialize differential drive
    DifferentialDrive::reset(0,0,0);
    DifferentialDrive::setLeftSpeed(leftSpeed);
    DifferentialDrive::setRightSpeed(rightSpeed);

    // initialize destination
    destinationX = 50;
    destinationY = 50;

    // initialize serial connection
    Serial1.begin(9600);
    Serial1.println("--- Start Serial Monitor ---");
    Serial1.println("(Decimal)(Hex)(Character)");
    Serial1.println();
}

void loop() {
    // drive to destination
    thetaCorrection();
    DifferentialDrive::drive();

    // read communication
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


