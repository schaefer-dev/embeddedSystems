#include <stddef.h>
#include "Collector_state.h"
#include "Zumo32U4Motors.h"
#include <math.h>
#include <stdlib.h>
#include <Arduino.h>

// 0,0 is top left corner
// degrees grow in clockwise rotation

const float theta_rotation_threshhold = 10.0f;
const float destination_reached_threshhold = 2.0f;
const float timefactor = 0.10f;
const float rotation_imprecision = 0.7f;

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
    float angle = atan2(vecY, vecX);


    return angle;
}


void Collector_state::thetaCorrection() {
    // check if destination reached
    if (abs(current_x - destination_x) < destination_reached_threshhold
        && abs(current_y - destination_y) < destination_reached_threshhold) {
        setRightSpeed(0);
        setLeftSpeed(0);
        Serial1.println("Destination Reached!");
        return;
    }

    // turn towards destination
    float angle = getAngle();
    float deltaAngle = angle - current_angle;
    int help_angle = ((180 / M_PI) * deltaAngle);

    float currentAnglePrint = current_angle;

    while (currentAnglePrint < 0) currentAnglePrint += 2 * M_PI;
    while (help_angle < 0) help_angle += 360;
    int deltaDegrees = help_angle % 360;


    Serial1.print("current angle: ");
    Serial1.println(((180 / M_PI) * currentAnglePrint));
    Serial1.print("soll angle: ");
    Serial1.println(angle);
    Serial1.print("deltaAngle angle: ");
    Serial1.println(deltaAngle);
    Serial1.print("current position: (");
    Serial1.print(current_x);
    Serial1.print(", ");
    Serial1.print(current_y);
    Serial1.println(")");

    if ((deltaDegrees < theta_rotation_threshhold) || (deltaDegrees > (360 - theta_rotation_threshhold))) {
        setRightSpeed(baseSpeed);
        setLeftSpeed(baseSpeed);
        Serial1.println("straight ahead!");
        return;
    }

    if (deltaAngle < 0) {
        // turn left
        setRightSpeed(1.5 * baseSpeed);
        setLeftSpeed(0);
        Serial1.println("turning left!");
    } else {
        // turn right
        setRightSpeed(0);
        setLeftSpeed(1.5 * baseSpeed);
        Serial1.println("turning right!");
    }
}


void Collector_state::diff_drive_reset(float x, float y, float a) {
    current_x = x;
    current_y = y;
    current_angle = a;
}

void Collector_state::setLeftSpeed(int speed) {
    Zumo32U4Motors::setLeftSpeed(speed);
    left_speed = speed;
}

void Collector_state::setRightSpeed(int speed) {
    Zumo32U4Motors::setRightSpeed(speed);
    right_speed = speed;
}

void Collector_state::drive() {
    float leftSpeedScaled = (50.0f / WHEEL_RADIUS) * (left_speed / 255.0f);
    float rightSpeedScaled = (50.0f / WHEEL_RADIUS) * (right_speed / 255.0f);


    float z = (WHEEL_RADIUS * (leftSpeedScaled + rightSpeedScaled) / 2);
    float x_dot = z * cos(current_angle);
    float y_dot = z * sin(current_angle);
    float angle_dot = (WHEEL_RADIUS * (leftSpeedScaled - rightSpeedScaled) / WHEEL_DISTANCE);

    current_x += x_dot * timefactor;
    current_y += y_dot * timefactor;
    current_angle += angle_dot * timefactor * rotation_imprecision;

    /* TODO
     * this is not working, needs to know time since last call
     * such that concrete differntial equation is correct */
}