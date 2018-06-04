#include <stddef.h>
#include "Collector_state.h"
#include "DifferentialDrive.h"
#include <math.h>

//constructor
Collector_state::Collector_state(){
    current_x = 0.0f;
    current_y = 0.0f;
    current_angle = 0.0f;

    destination_x = 0.0f;
    destination_y = 0.0f;

    left_speed = 0.0f;
    right_speed = 0.0f;
}

float Collector_state::getAngle() {
    float vecX = destination_y - current_x;
    float vecY = destination_y - current_y;
    float angle = atan(vecY / vecX);
    if (vecX < 0) angle += 180;
    if (angle > 180) angle = angle - 360;
    if (angle < - 180) angle = 360 - angle;
    return angle;
}


void Collector_state::thetaCorrection() {
    float currentX = DifferentialDrive::posX;
    float currentY = DifferentialDrive::posY;

    // check if destination reached
    if (currentX == destination_x && currentY == destination_y) {
        DifferentialDrive::setRightSpeed(0);
        DifferentialDrive::setLeftSpeed(0);
        return;
    }

    // turn towards destination
    float angle = getAngle();
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