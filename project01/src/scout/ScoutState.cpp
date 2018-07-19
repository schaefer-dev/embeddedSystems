#include "ScoutState.h"
#include "OrangutanMotors.h"
#include "OrangutanTime.h"
#include <OrangutanSerial.h>
#include "../utils/Utility.h"
#include <math.h>
#include <stdlib.h>
#include "main.h"
#include "ScoutRF.h"
#include "adc.h"


// 0,0 is top left corner
// degrees grow in clockwise rotation

const float theta_rotation_threshhold = 15.0f;      // for turningSpeed = 100, value > 11 prevents quick direction changes
const float destination_reached_threshhold = 3.0f;  // for forwardSpeed = 200, value > 2.5 prevents overshoot

//constructor
ScoutState::ScoutState() {
    currentX = 0.0f;
    currentY = ARENA_SIZE_Y;
    currentAngle = 0.0f;
    destinationX = 0;
    destinationY = 0;
    leftSpeed = 0;
    rightSpeed = 0;
    lastDiffDriveCall = 0;
    destinationReached = true;
    photoSensorBack = 0;
    photoSensorFront = 0;
    photoSensorLeft = 0;
    photoSensorRight = 0;
    lastPhotoSensorUpdate = 0;
    photoSensorTimer = 0;
    photoSensorCurrentMax = 0;
    photoX = 0;
    photoY = 0;
    outOfBounds = false;
    outOfBoundsTime = 0;
    drivingDisabled = true;
    earliestNextRotationTime = millis();
    navigationStep = NAV_NONE;
    unhandledCollisionFlag = false;
    configurationReceived = false;
    gameStarted = false;
    driveBackwardsUntil = millis();
    collectorX = 0.0f;
    collectorY = 0.0f;
    collectorAngle = 0.0f;
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
void ScoutState::navigate(){

    if (drivingDisabled) {
        setSpeeds(0,0);
#ifdef DEBUG
        ScoutSerial::serialWrite("driving disabled\n",17);
#endif
        return;
    }

    /* read new destination if no destination currently */
    if (destinationReached == true){
        setSpeeds(0,0);
        generateDestination();
        return;
    }

    /*  check if collision has just happened  */
    if (unhandledCollisionFlag){
        driveBackwardsUntil = millis() + DRIVE_BACKWARDS_TIME;
        setSpeeds(backwardsSpeed, backwardsSpeed);
        unhandledCollisionFlag = false;
        return;
    }

    /*  check if we still have to drive backwards */
    if (millis() <= driveBackwardsUntil){
        setSpeeds(backwardsSpeed, backwardsSpeed);
        return;
    }


    // check if destination reached
    if (abs(currentX - destinationX) < destination_reached_threshhold
        && abs(currentY - destinationY) < destination_reached_threshhold) {
        setSpeeds(0, 0);
#ifdef DEBUG
        ScoutSerial::serialWrite("\nDestination Reached!\n", 22);
#endif
        destinationX = 0;
        destinationY = 0;
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
#ifdef DEBUG
        ScoutSerial::serialWrite("straight ahead!\n", 16);
#endif
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
#ifdef DEBUG
        ScoutSerial::serialWrite("straight ahead!\n", 16);
#endif
        return;
    }


    if (deltaAngle < 0) {
        // turn left
        navigationStep = NAV_TURNING_LEFT;
        setSpeeds(-turningSpeed, turningSpeed);
#ifdef DEBUG
        ScoutSerial::serialWrite("turning left!\n", 14);
#endif
    } else {
        // turn right
        navigationStep = NAV_TURNING_RIGHT;
        setSpeeds(turningSpeed, -turningSpeed);
#ifdef DEBUG
       //ScoutSerial::serialWrite("turning right!\n", 15);
#endif
    }

}

/**
 * Generates and sets a new destination. Only called when the current destination is reached
 */
void ScoutState::generateDestination() {
    destinationX = (random() % (ARENA_SIZE_X - 10)) + 5;
    destinationY = (random() % (ARENA_SIZE_Y - 10)) + 5;

#ifdef DEBUG
    ScoutSerial::serialWrite("New Random Goal:\nX: ", 20);
    ScoutSerial::serialWriteInt(destinationX);
    ScoutSerial::serialWriteInt(destinationY);
    ScoutSerial::serialWrite("Y: ", 3);
#endif

    destinationReached = false;
    drivingDisabled = false;
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
            OrangutanMotors::setSpeeds(0, 0);
            return;
        }
    }
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

/* write Readings of all Photosensors to DEV port serial */
void ScoutState::updatePhotoSensorReadings() {

    lastPhotoSensorUpdate = millis();

    int adcout11 = adc_communicate(0);
    delay_us(ADC_COMPUTE_DELAY);

    photoSensorFront = adc_communicate(1);
    delay_us(ADC_COMPUTE_DELAY);

    photoSensorRight = adc_communicate(2);
    delay_us(ADC_COMPUTE_DELAY);

    photoSensorBack = adc_communicate(3);
    delay_us(ADC_COMPUTE_DELAY);

    photoSensorLeft = adc_communicate(11);
    delay_us(ADC_COMPUTE_DELAY);
}


/* notifies the robot, that it drove out of bounds, disable all motor activity for 30s */
void ScoutState::outOfBoundsMessage() {
    setSpeeds(0,0);
    outOfBounds = true;
    ScoutSerial::serialWrite("OOB Punish start\n",17);
    outOfBoundsTime = millis();
}

/* communicate robot position to teammate */
void ScoutState::sendPosToTeammate(){
    uint8_t ownPosition[7];

    int ownAngle = (int) currentAngle;
    int ownXPos = (int) currentX;
    int ownYPos = (int) currentY;

    ownPosition[0] = 0x0030;

    ownPosition[1] = ownAngle / 256;
    ownPosition[2] = ownAngle % 256;
    ownPosition[3] = ownXPos / 256;
    ownPosition[4] = ownXPos % 256;
    ownPosition[5] = ownYPos / 256;
    ownPosition[6] = ownYPos % 256;

    ScoutRF::sendMessageTo(ScoutRF::collectorAdress, ownPosition, 7);

}


/* handle high photosensor readings by notifying Collector;
 * Send Light Intensity, X position, Y position like pos update */
void ScoutState::handleHighPhotoReadings() {
    ScoutState::updatePhotoSensorReadings();
    int readingsMax = 0;
    readingsMax = Utility::maximum(Utility::maximum(photoSensorFront, photoSensorBack),
                                   Utility::maximum(photoSensorLeft, photoSensorRight));

    if (photoSensorTimer == 0){

        if (readingsMax < PHOTOSENSOR_TRESHOLD){
            return;
        }

        photoSensorTimer = millis() + MSToCheckPhotosensorsBeforeMaximumSent;
        photoSensorCurrentMax = readingsMax;
        photoX = currentX;
        photoY = currentY;
        return;

    } else if (photoSensorTimer > millis()){

        if (readingsMax > photoSensorCurrentMax){
            photoSensorCurrentMax = readingsMax;
            photoX = currentX;
            photoY = currentY;
        }
        return;
    }

    uint8_t harvestUpdate[7];

    harvestUpdate[0] = 0x69;
    harvestUpdate[1] = 0;
    harvestUpdate[2] = photoSensorCurrentMax;

    harvestUpdate[3] = ( (int) (photoX * 10) ) / 256;
    harvestUpdate[4] = ( (int) (photoX * 10) ) % 256;

    harvestUpdate[5] = ( (int) (photoY * 10) ) / 256;
    harvestUpdate[6] = ( (int) (photoY * 10) ) % 256;

    ScoutRF::sendMessageTo(ScoutRF::collectorAdress, harvestUpdate, 7);

    // TESTING CODE: Send OOB message 5s after sending harvesting position to Collector. Update to: x:10 y:10 angle:0
    /*delay(3000);

    uint8_t payload[7];
    payload[0] = 0x61;
    payload[1] = 0;
    payload[2] = 0;
    payload[3] = 0;
    payload[4] = 100;
    payload[5] = 0;
    payload[6] = 100;
    ScoutRF::sendMessageTo(ScoutRF::collectorAdress, payload, 7);
     */


#ifdef DEBUG
    ScoutSerial::serialWrite("Sent ", 5);
    ScoutSerial::serialWriteInt(photoSensorCurrentMax);
    ScoutSerial::serialWrite("at ", 3);
    ScoutSerial::serialWriteInt(photoX);
    ScoutSerial::serialWrite(",", 1);
    ScoutSerial::serialWriteInt(photoY);
    ScoutSerial::serialWrite(" ", 1);
    ScoutSerial::serialWriteInt(currentX);
    ScoutSerial::serialWrite(" ", 1);
    ScoutSerial::serialWriteInt(currentY);
#endif


    photoSensorTimer = 0;
    photoSensorCurrentMax = 0;
}

