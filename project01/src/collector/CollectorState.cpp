#include <stddef.h>
#include "CollectorState.h"
#include "Zumo32U4Motors.h"
#include <math.h>
#include <stdlib.h>
#include <Arduino.h>

// 0,0 is top left corner
// degrees grow in clockwise rotation

const float theta_rotation_threshhold = 10.0f;
const float destination_reached_threshhold = 3.0f;

//constructor
CollectorState::CollectorState(){
    currentX = 0.0f;
    currentY = 0.0f;
    currentAngle = 0.0f;
    destinationX = 0.0f;
    destinationY = 0.0f;
    leftSpeed = 0;
    rightSpeed = 0;
    lastDiffDriveCall = 0;
    destinationReached = true;
}

/*
 * Returns the angle of the vector from the current position to the
 * destination. Positive angle means clockwise turn. Angle is in
 * radians and can be > 2pi or < 0
 */
float CollectorState::getAngle() {
    double vecX = destinationX - currentX;
    double vecY = destinationY - currentY;
    float angle = atan2(vecY, vecX);
    return angle;
}


/* sets motor values to updateRoboterPositionAndAngles/turn towards the specified destination */
bool CollectorState::navigateToDestination() {
    // check if destination reached
    if (abs(currentX - destinationX) < destination_reached_threshhold
        && abs(currentY - destinationY) < destination_reached_threshhold) {
        setSpeeds(0,0);
        Serial1.println("Destination Reached!");
        destinationReached = true;
        return true;
    }

    // turn towards destination
    float angle = getAngle();                               // angle of vec from current pos to destination
    float deltaAngle = angle - currentAngle;                // angle we have to turn
    double deltaAngleDeg = ((180 / M_PI) * deltaAngle);     // deltaAngle in degrees

    float currentAnglePrint = currentAngle;

    while (currentAnglePrint < 0) currentAnglePrint += 2 * M_PI;
    while (deltaAngleDeg < 0) deltaAngleDeg += 360;
    int deltaDegrees = (int)deltaAngleDeg % 360;

    /*
    Serial1.print("current angle: ");
    Serial1.println(((180 / M_PI) * currentAnglePrint));
    Serial1.print("soll angle: ");
    Serial1.println(angle);
    Serial1.print("deltaAngle angle: ");
    Serial1.println(deltaAngle);
    Serial1.print("current position: (");
    Serial1.print(currentX);
    Serial1.print(", ");
    Serial1.print(currentY);
    Serial1.println(")");
     */

    if ((deltaDegrees < theta_rotation_threshhold) || (deltaDegrees > (360 - theta_rotation_threshhold))) {
        setSpeeds(baseSpeed, baseSpeed);
        //Serial1.println("straight ahead!");
        return false;
    }

    if (deltaAngle < 0) {
        // turn left
        setSpeeds(-0.5 * baseSpeed, 0.5 * baseSpeed);
        //Serial1.println("turning left!");
    } else {
        // turn right
        setSpeeds(0.5 * baseSpeed, -0.5 * baseSpeed);
        //Serial1.println("turning right!");
    }
    return false;
}


void CollectorState::resetDifferentialDrive(float x, float y, float a) {
    currentX = x;
    currentY = y;
    currentAngle = a;
}

void CollectorState::setSpeeds(int newLeftSpeed, int newRightSpeed) {
    if (rightSpeed == newRightSpeed && leftSpeed == newLeftSpeed)
        return;
    updateRoboterPositionAndAngles();
    Zumo32U4Motors::setSpeeds(newLeftSpeed, newRightSpeed);
    leftSpeed = newLeftSpeed;
    rightSpeed = newRightSpeed;
}

/* this function should be called BEFORE any changes to motors!
 * TODO: might be worth to call this function at the start of setLeft/Right speed to avoid any imprecisions,
 * if implemented properly using millis */
void CollectorState::updateRoboterPositionAndAngles() {
    float leftSpeedScaled = (50.0f / WHEEL_RADIUS) * (leftSpeed / 255.0f);
    float rightSpeedScaled = (50.0f / WHEEL_RADIUS) * (rightSpeed / 255.0f);


    float z = (WHEEL_RADIUS * (leftSpeedScaled + rightSpeedScaled) / 2);
    float x_dot = z * cos((double)currentAngle);
    float y_dot = z * sin((double)currentAngle);
    float angle_dot = (WHEEL_RADIUS * (leftSpeedScaled - rightSpeedScaled) / WHEEL_DISTANCE);

    long lastDiffDriveTime = lastDiffDriveCall;
    lastDiffDriveCall = millis();
    long msSinceLastUpdate = lastDiffDriveCall - lastDiffDriveTime;

    float timefactor = msSinceLastUpdate / 1000.0f;

    /* TODO use millis here to calculate exact timedifference since last call */
    /* should be (msSinceLastUpdate / 1000) as factor */
    currentX += x_dot * timefactor * straightImprecision;
    currentY += y_dot * timefactor * straightImprecision;
    currentAngle += angle_dot * timefactor * rotationImprecision;

    /* TODO
     * this is not working, needs to know time since last call
     * such that concrete differntial equation is correct */
}