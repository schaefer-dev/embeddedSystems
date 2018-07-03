#include "ScoutState.h"
#include "OrangutanMotors.h"
#include "OrangutanTime.h"
#include <OrangutanSerial.h>
#include <math.h>
#include <stdlib.h>
#include "main.h"

// 0,0 is top left corner
// degrees grow in clockwise rotation

const float theta_rotation_threshhold = 15.0f;      // for turningSpeed = 100, value > 11 prevents quick direction changes
const float destination_reached_threshhold = 3.0f;  // for forwardSpeed = 200, value > 2.5 prevents overshoot

//constructor
ScoutState::ScoutState() {
    currentX = 0.0f;
    currentY = 0.0f;
    currentAngle = 0.0f;
    destinationX = NO_DESTINATION;
    destinationY = NO_DESTINATION;
    leftSpeed = 0;
    rightSpeed = 0;
    lastDiffDriveCall = 0;
    destinationReached = true;
    photoSensorBack = 0;
    photoSensorFront = 0;
    photoSensorLeft = 0;
    photoSensorRight = 0;
    lastPhotoSensorUpdate = 0;
    outOfBounds = false;
    outOfBoundsTime = 0;
    drivingDisabled = true;
    earliestNextRotationTime = millis();
    navigationStep = NAV_NONE;
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


/* drives towards current destination */
void ScoutState::navigate(CoordinateQueue *coordinateQueue){

    if (drivingDisabled) {
        setSpeeds(0,0);
        ScoutSerial::serialWrite("driving disabled\n",17);
        return;
    }

    /* read new destination if no destination currently */
    if (destinationReached == true){
        /* take new destination from queue */
        if (coordinateQueue->isEmpty()) {
            setSpeeds(0,0);
            ScoutSerial::serialWrite("no new destination in queue\n",28);
            return;
        }

        struct CoordinateQueue::CoordinateNode *node = coordinateQueue->pop(currentX, currentY);

        ScoutSerial::serialWrite("set new destination from queue\n",31);
        ScoutSerial::serialWrite("X: ",2);
        ScoutSerial::serialWriteInt(node->x);
        ScoutSerial::serialWrite("Y: ",3);
        ScoutSerial::serialWriteInt(node->y);
        ScoutSerial::serialWrite("\n",1);

        destinationX = node->x;
        destinationY = node->y;
        destinationReached = false;
    }


    // check if destination reached
    if (abs(currentX - destinationX) < destination_reached_threshhold
        && abs(currentY - destinationY) < destination_reached_threshhold) {
        setSpeeds(0, 0);
#ifdef DEBUG
        serial_send_blocking("\nDestination Reached!\n", 22);
#endif
        destinationX = NO_DESTINATION;
        destinationY = NO_DESTINATION;
        destinationReached = true;
        return;
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


    /* drive straight if turned to right angle in near future */
    if (millis() < earliestNextRotationTime) {
        setSpeeds(forwardSpeed, forwardSpeed);
        navigationStep = NAV_DRIVING_STRAIGHT;
        ScoutSerial::serialWrite("straight ahead!\n", 16);
        return;
    }

    /* drive straight if angle still correct */
    if ((deltaDegrees < theta_rotation_threshhold) || (deltaDegrees > (360 - theta_rotation_threshhold))) {
        /* if turned in previous step make sure to turn for at least the defined ms now */
        if (navigationStep == NAV_TURNING_LEFT || navigationStep == NAV_TURNING_RIGHT){
            earliestNextRotationTime = millis() + DO_NOT_ROTATE_AGAIN_MS;
        }
        navigationStep = NAV_DRIVING_STRAIGHT;
        setSpeeds(forwardSpeed, forwardSpeed);
        ScoutSerial::serialWrite("straight ahead!\n", 16);
        return;
    }


    if (deltaAngle < 0) {
        // turn left
        navigationStep = NAV_TURNING_LEFT;
        setSpeeds(-turningSpeed, turningSpeed);
        ScoutSerial::serialWrite("turning left!\n", 14);
    } else {
        // turn right
        navigationStep = NAV_TURNING_RIGHT;
        setSpeeds(turningSpeed, -turningSpeed);
        ScoutSerial::serialWrite("turning right!\n", 15);
    }

}

void ScoutState::resetDifferentialDrive(float x, float y, float a) {
    currentX = x;
    currentY = y;
    currentAngle = a;
    lastDiffDriveCall = millis();
}

void ScoutState::setSpeeds(int newLeftSpeed, int newRightSpeed) {
    if (drivingDisabled) {
        OrangutanMotors::setSpeeds(0, 0);
        return;
    }
    if (outOfBounds){
        if (millis() - outOfBoundsTime >= OOB_PUNISH_TIME_MS){
            outOfBounds = false;
            ScoutSerial::serialWrite("OOB Punish over\n",16);
        } else {
            return;
        }
    }
    if (rightSpeed == newRightSpeed && leftSpeed == newLeftSpeed)
        return;
    updateRoboterPositionAndAngles();
#ifndef ROBOT_SIMULATOR
    OrangutanMotors::setSpeeds(newLeftSpeed, newRightSpeed);
#endif
#ifdef ROBOT_SIMULATOR
    ScoutSerial::serialWrite("Speed left: ", 12);
    ScoutSerial::serialWrite8Bit(newLeftSpeed);
    ScoutSerial::serialWrite(" right: ", 8);
    ScoutSerial::serialWrite8Bit(newRightSpeed);
    ScoutSerial::serialWrite("\n", 1);
#endif
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

/* write Readings of all Photosensors to DEV port serial */
void ScoutState::updatePhotoSensorReadings() {

    lastPhotoSensorUpdate = millis();

    int adcout11 = ScoutSPI::readADC(0);
    ScoutSPI::ADCConversionWait();

    photoSensorFront = ScoutSPI::readADC(1);
    ScoutSPI::ADCConversionWait();

    photoSensorRight = ScoutSPI::readADC(2);
    ScoutSPI::ADCConversionWait();

    photoSensorBack = ScoutSPI::readADC(3);
    ScoutSPI::ADCConversionWait();

    photoSensorLeft = ScoutSPI::readADC(11);
    ScoutSPI::ADCConversionWait();
}


/* notifies the robot, that it drove out of bounds, disable all motor activity for 30s */
void ScoutState::outOfBoundsMessage() {
    setSpeeds(0,0);
    outOfBounds = true;
    ScoutSerial::serialWrite("OOB Punish start\n",17);
    outOfBoundsTime = millis();
}