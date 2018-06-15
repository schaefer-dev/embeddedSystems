#include "ScoutState.h"
#include "OrangutanMotors.h"
#include "OrangutanTime.h"
#include <OrangutanSerial.h>
#include <math.h>
#include <stdlib.h>
#include "main.h"

// 0,0 is top left corner
// degrees grow in clockwise rotation

const float theta_rotation_threshhold = 12.0f;      // for turningSpeed = 100, value > 11 prevents quick direction changes
const float destination_reached_threshhold = 3.0f;  // for forwardSpeed = 200, value > 2.5 prevents overshoot

//constructor
ScoutState::ScoutState() {
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
float ScoutState::getAngle() {
    double vecX = destinationX - currentX;
    double vecY = destinationY - currentY;
    float angle = atan2(vecY, vecX);
    return angle;
}


/* sets motor values to updateRoboterPositionAndAngles/turn towards the specified destination */
bool ScoutState::navigateToDestination() {
#ifdef DEBUG
    //Serial1.print("POS: (");
    //Serial1.print(currentX);
    //Serial1.print(", ");
    //Serial1.print(currentY);
    //Serial1.print(")");
#endif

    // check if destination reached
    if (abs(currentX - destinationX) < destination_reached_threshhold
        && abs(currentY - destinationY) < destination_reached_threshhold) {
        setSpeeds(0, 0);
#ifdef DEBUG
        serial_send_blocking("\nDestination Reached!\n", 22);
#endif
        destinationReached = true;
        return true;
    }

    // turn towards destination
    float angle = getAngle();                               // angle of vec from current pos to destination
    float deltaAngle = angle - currentAngle;                // angle we have to turn

    /* BEGIN variables only for debug purpose */
    float currentAnglePrint = currentAngle;
    float anglePrint = angle;
    while (currentAnglePrint < 0) currentAnglePrint += 2 * M_PI;
    while (currentAnglePrint > 2 * M_PI) currentAnglePrint -= 2 * M_PI;
    while (anglePrint < 0) anglePrint += 2 * M_PI;
    while (anglePrint > 2 * M_PI) anglePrint -= 2 * M_PI;
    /* END */

    // make sure he always turns in the shortest direction
    while (deltaAngle > M_PI) deltaAngle -= 2 * M_PI;
    while (deltaAngle < -M_PI) deltaAngle += 2 * M_PI;

    // calculate current delta in positive degrees
    double deltaAngleDeg = ((180 / M_PI) * deltaAngle);     // deltaAngle in degrees
    while (deltaAngleDeg < 0) deltaAngleDeg += 360;
    double deltaDegrees = deltaAngleDeg;


    if ((deltaDegrees < theta_rotation_threshhold) || (deltaDegrees > (360 - theta_rotation_threshhold))) {
        setSpeeds(forwardSpeed, forwardSpeed);
        //Serial1.println("straight ahead!");
        return false;
    }

    if (deltaAngle < 0) {
        // turn left
        setSpeeds(-turningSpeed, turningSpeed);
        //Serial1.println("turning left!");
    } else {
        // turn right
        setSpeeds(turningSpeed, -turningSpeed);
        //Serial1.println("turning right!");
    }
    return false;
}


void ScoutState::resetDifferentialDrive(float x, float y, float a) {
    currentX = x;
    currentY = y;
    currentAngle = a;
}

void ScoutState::setSpeeds(int newLeftSpeed, int newRightSpeed) {
    if (rightSpeed == newRightSpeed && leftSpeed == newLeftSpeed)
        return;
    updateRoboterPositionAndAngles();
    OrangutanMotors::setSpeeds(newLeftSpeed, newRightSpeed);
    leftSpeed = newLeftSpeed;
    rightSpeed = newRightSpeed;
}



void ScoutState::updateRoboterPositionAndAngles() {
    float leftSpeedScaled = (50.0f / WHEEL_RADIUS) * (leftSpeed / 255.0f);
    float rightSpeedScaled = (50.0f / WHEEL_RADIUS) * (rightSpeed / 255.0f);


    float z = (WHEEL_RADIUS * (leftSpeedScaled + rightSpeedScaled) / 2);
    float x_dot = z * cos((double) currentAngle);
    float y_dot = z * sin((double) currentAngle);
    float angle_dot = (WHEEL_RADIUS * (leftSpeedScaled - rightSpeedScaled) / WHEEL_DISTANCE);

    long lastDiffDriveTime = lastDiffDriveCall;
    lastDiffDriveCall = millis();
    long msSinceLastUpdate = lastDiffDriveCall - lastDiffDriveTime;

    float timeFactor = msSinceLastUpdate / 1000.0f;
    currentX += x_dot * timeFactor * straightImprecision;
    currentY += y_dot * timeFactor * straightImprecision;
    currentAngle += angle_dot * timeFactor * rotationImprecision;
}