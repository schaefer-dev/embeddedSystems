#include <stddef.h>
#include "Collector_state.h"
#include "DifferentialDrive.h"
#include <math.h>
#include <stdlib.h>
#include <Arduino.h>


const float theta_rotation_threshhold = 4.0f;
const float destination_reached_threshhold = 7.0f;

//constructor
Collector_state::Collector_state(){
    current_x = 0.0f;
    current_y = 0.0f;
    current_angle = 0.0f;

    destination_x = 0.0f;
    destination_y = 0.0f;

    left_speed = 0;
    right_speed = 0;
}

float Collector_state::getAngle() {
    float vecX = destination_x - current_x;
    float vecY = destination_y - current_y;
    float angle = atan(vecY / vecX);
    if (vecX < 0) angle += 180;
    if (angle > 180) angle = angle - 360;
    if (angle < - 180) angle = 360 - angle;

    /* for some reason this blows up program space on roboter */
    //String text = "Required angle: ";
    //text += String(angle);
    //Serial1.println(text);

    return angle;
}


void Collector_state::thetaCorrection() {

    Serial1.print("current angle: ");
    Serial1.println(current_angle);
    Serial1.print("current position: (");
    Serial1.print(current_x);
    Serial1.print(", ");
    Serial1.print(current_y);
    Serial1.println(")");


    // check if destination reached
    if (abs(current_x - destination_x) < destination_reached_threshhold
        && abs(current_y - destination_y) < destination_reached_threshhold) {
        DifferentialDrive::setRightSpeed(0);
        DifferentialDrive::setLeftSpeed(0);
        Serial1.println("Destination Reached!");
        return;
    }

    // turn towards destination
    float angle = getAngle();
    float deltaAngle = angle - current_angle;
    int help_angle = ((180 / M_PI) * deltaAngle);
    int deltaDegrees = help_angle % 360;



    if ((deltaDegrees < theta_rotation_threshhold) || (deltaDegrees > (360 - theta_rotation_threshhold))) {
        DifferentialDrive::setRightSpeed(baseSpeed);
        DifferentialDrive::setLeftSpeed(baseSpeed);
        Serial1.println("straight ahead!");
        return;
    }

    if (deltaAngle > 0) {
        // turn left
        DifferentialDrive::setRightSpeed(baseSpeed);
        DifferentialDrive::setLeftSpeed(-baseSpeed);
        Serial1.println("turning left!");
    } else {
        // turn right
        DifferentialDrive::setRightSpeed(-baseSpeed);
        DifferentialDrive::setLeftSpeed(baseSpeed);
        Serial1.println("turning right!");
    }
}