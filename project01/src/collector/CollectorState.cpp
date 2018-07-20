#include "CollectorState.h"
#include "Zumo32U4Motors.h"
#include <math.h>
#include <stdlib.h>
#include <Arduino.h>
#include "main.h"
#include "CollectorRF.h"

// 0,0 is top left corner
// degrees grow in clockwise rotation

const float theta_rotation_threshhold = 6.0f;      // for turningSpeed = 100, value > 11 prevents quick direction changes
const float destination_reached_threshhold = 3.0f;  // for forwardSpeed = 200, value > 2.5 prevents overshoot

//constructor
CollectorState::CollectorState() {
    currentX = 0.0f;
    currentY = 0.0f;
    currentAngle = 0.0f;
    destinationX = 0.0f;
    destinationY = 0.0f;
    leftSpeed = 0;
    rightSpeed = 0;
    lastDiffDriveCall = 0;
    destinationReached = true;
    isHarvestDestination = false;

    outOfBounds = false;
    outOfBoundsTime = 0;
    drivingDisabled = true;
    earliestNextRotationTime = millis();
    navigationStep = NAV_NONE;

    configurationReceived = false;
    gameStarted = false;

    scoutPosX = 0;
    scoutPosY = 0;
    scoutAngle = 0;

    unhandledCollisionFlag = false;
    driveBackwardsUntil = millis();
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

/* drives towards current destination */
void CollectorState::navigate() {

    unsigned long timeSinceLastHarvestingReached = millis() - harvestPositionReachedAtTime;

    if (timeSinceLastHarvestingReached < 5000 && millis() > 5000) {
#ifdef COLLECTOR_DEBUG
        Serial1.println("harvesting...");
#endif
        isHarvestDestination = false;
        setSpeeds(0, 0);
        return;
    }

    harvestPositionReachedAtTime = 0;

    if (drivingDisabled) {
        setSpeeds(0, 0);
        return;
    }

    /* read new destination if no destination currently */
    if (destinationReached) {
        if (!isHarvestDestination) {
            // if this was a random destination, continue driving randomly
            generateDestination();
#ifdef COLLECTOR_DEBUG
            Serial1.print("Generate random destination");
#endif
            return;
        } else {
            // if this was a harvest position, do X
            Serial1.print("Harvest reached");
            harvestPositionReachedAtTime = millis();
            return;
        }
    }

    /*  check if collision has just happened  */
    if (unhandledCollisionFlag) {
        driveBackwardsUntil = millis() + DRIVE_BACKWARDS_TIME;
        setSpeeds(backwardsSpeed, backwardsSpeed);
        unhandledCollisionFlag = false;
        return;
    }

    /*  check if we still have to drive backwards */
    if (millis() <= driveBackwardsUntil) {
        setSpeeds(backwardsSpeed, backwardsSpeed);
        return;
    }

    // check if destination reached
    if (abs(currentX - destinationX) < destination_reached_threshhold
        && abs(currentY - destinationY) < destination_reached_threshhold) {
        setSpeeds(0, 0);
#ifdef COLLECTOR_DEBUG
        Serial1.println("\nDestination Reached!");
#endif

        destinationX = 0;
        destinationY = 0;
        destinationReached = true;
        //drivingDisabled = true;
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
#ifdef COLLECTOR_DEBUG
        Serial1.println("straight...");
#endif
        return;
    }

    /* drive straight if angle still correct */
    if ((deltaDegrees < theta_rotation_threshhold) || (deltaDegrees > (360 - theta_rotation_threshhold))) {
        /* if turned in previous step make sure to turn for at least the defined ms now */
        if (navigationStep == NAV_TURNING_LEFT || navigationStep == NAV_TURNING_RIGHT) {
            earliestNextRotationTime = millis() + DO_NOT_ROTATE_AGAIN_MS;
        }
        navigationStep = NAV_DRIVING_STRAIGHT;
        setSpeeds(forwardSpeed, forwardSpeed);
#ifdef COLLECTOR_DEBUG
        Serial1.println("straight");
#endif
        return;
    }

    if (deltaAngle < 0) {
        // turn left
        navigationStep = NAV_TURNING_LEFT;
        setSpeeds(-turningSpeed, turningSpeed);
#ifdef COLLECTOR_DEBUG
        Serial1.println("turning left!");
#endif
    } else {
        // turn right
        navigationStep = NAV_TURNING_RIGHT;
        setSpeeds(turningSpeed, -turningSpeed);
#ifdef COLLECTOR_DEBUG
        Serial1.println("turning right!");
#endif
    }

}

void CollectorState::resetDifferentialDrive(float x, float y, float a) {
    currentX = x;
    currentY = y;
    currentAngle = a;
    lastDiffDriveCall = millis();
}

void CollectorState::setSpeeds(int newLeftSpeed, int newRightSpeed) {
    if (newLeftSpeed > 300 || newRightSpeed > 300) {
        Serial1.println("ERROR: never allowed to drive faster than 300!");
        return;
    }
    if (drivingDisabled) {
        Zumo32U4Motors::setSpeeds(0, 0);
        return;
    }
    if (outOfBounds) {
        if (millis() - outOfBoundsTime >= OOB_PUNISH_TIME_MS) {
            outOfBounds = false;
#ifdef COLLECTOR_DEBUG
            Serial1.println("OOB Punish over");
            Serial1.flush();
#endif
        } else {
            Zumo32U4Motors::setSpeeds(0, 0);
            return;
        }
    }
    if (rightSpeed == newRightSpeed && leftSpeed == newLeftSpeed)
        return;
    updateRoboterPositionAndAngles();
    Zumo32U4Motors::setSpeeds(newLeftSpeed, newRightSpeed);
    leftSpeed = newLeftSpeed;
    rightSpeed = newRightSpeed;
}

void CollectorState::updateRoboterPositionAndAngles() {
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

/* notifies the robot, that it drove out of bounds, disable all motor activity for 30s */
void CollectorState::outOfBoundsMessage() {
    setSpeeds(0, 0);
    outOfBounds = true;
#ifdef COLLECTOR_DEBUG
    Serial1.println("OOB Punish start");
    Serial1.flush();
#endif
    outOfBoundsTime = millis();
}

/**
 * Can be called anytime upon receiving a harvest position over RF and will immediately update the
 * @param x
 * @param y
 */
void CollectorState::harvestPositionMessage(int value, int x, int y) {
    destinationX = x;
    destinationY = y;

    destinationReached = false;
    drivingDisabled = false;
    isHarvestDestination = true;
}

/**
 * Generates and sets a new destination. Only called when the current destination is reached
 */
void CollectorState::generateDestination() {
    destinationX = random(20, ARENA_SIZE_X - 20);
    destinationY = random(20, ARENA_SIZE_Y - 20);

    destinationReached = false;
    drivingDisabled = false;

    isHarvestDestination = false;

    /*uint8_t payload[2];
    payload[0] = 0x81;
    payload[1] = (uint8_t) 'g';
    CollectorRF::sendMessageTo(CollectorRF::scoutAdress, payload, 2);*/
}

void CollectorState::scoutPositionMessage(float angle, float x, float y) {
    scoutAngle = angle;
    scoutPosY = x;
    scoutPosY = y;
}

void CollectorState::sendPositionUpdate() {

    uint8_t payloadArray[7];
    payloadArray[0] = 0x30;

    payloadArray[1] = (int) (currentAngle) / 256;
    payloadArray[2] = (int) (currentAngle) % 256;
    payloadArray[3] = (int) (currentX) / 256;
    payloadArray[4] = (int) (currentX) % 256;
    payloadArray[5] = (int) (currentY) / 256;
    payloadArray[6] = (int) (currentY) % 256;

    CollectorRF::sendMessageTo(CollectorRF::scoutAdress, payloadArray, 7);
}

void CollectorState::danceBlocking() {
    unsigned long danceUntil = millis() + 3000;
    int speed = 50;
    while (millis() < danceUntil) {
        speed = (speed + 1) % 300;
        setSpeeds(speed, -speed);

        delay(10);
    }
    setSpeeds(0,0);
    drivingDisabled = true;
}

