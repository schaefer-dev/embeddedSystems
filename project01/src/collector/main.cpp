#include <Arduino.h>
#include <Zumo32U4ProximitySensors.h>
#include "CollectorState.h"
#include "Coordinates.h"
#include "main.h"

CoordinateQueue *coordinateQueue;
CollectorState *collectorState;
Zumo32U4ProximitySensors *proximitySensors;

void setup() {

    proximitySensors = new Zumo32U4ProximitySensors();

    /* initialization of Data structures */
    collectorState = new CollectorState();
    coordinateQueue = new CoordinateQueue();

    // initialize differential updateRoboterPositionAndAngles
    collectorState->setSpeeds(0, 0);
    collectorState->resetDifferentialDrive(0, 0, 0);

    // initialize serial connection
    Serial1.flush();
    Serial1.begin(9600);
    Serial1.flush();
    Serial1.println("--- Start Serial Monitor ---");
    Serial1.println();

    proximitySensors->initThreeSensors();
    //proximitySensors->initFrontSensor();

    collectorState->lastDiffDriveCall = millis();
}

void loop() {

    /* Default roboter code */
    //readNewDestinations();
    //if (driveToDestination()) {
    //    performRotation();
    //}
    //collectorState->updateRoboterPositionAndAngles();

    /* Testing code */
    huntObject();
}


/* ------------------------------------------------------------*/
/* ----------------- HELPER FUNCTIONS -------------------------*/
/* ------------------------------------------------------------*/


void huntObject(){
    proximitySensors->read();

    delay(5);

    uint8_t frontLeftSensorValue = proximitySensors->countsFrontWithLeftLeds();

    uint8_t frontRightSensorValue = proximitySensors->countsFrontWithRightLeds();

    uint8_t leftSensorValue = proximitySensors->countsLeftWithLeftLeds();

    uint8_t rightSensorValue = proximitySensors->countsRightWithRightLeds();

    float averageFrontSensorValue = (frontLeftSensorValue + frontRightSensorValue) / 2.0f;

    Serial1.print(leftSensorValue);
    Serial1.print(", ");
    Serial1.print(frontLeftSensorValue);
    Serial1.print(", ");
    Serial1.print(frontRightSensorValue);
    Serial1.print(", ");
    Serial1.print(rightSensorValue);
    Serial1.println("");

    if (frontLeftSensorValue > 5 && frontRightSensorValue > 5){
        collectorState->setSpeeds(0.5 * collectorState->forwardSpeed, 0.5 * collectorState->forwardSpeed);
        return;
    }

    if (frontLeftSensorValue > 5){
        collectorState->setSpeeds(-0.7 * collectorState->turningSpeed, 0.7 * collectorState->turningSpeed);
    }

    if (frontRightSensorValue > 5){
        collectorState->setSpeeds(0.7 * collectorState->turningSpeed, -0.7 * collectorState->turningSpeed);
    }

    if (leftSensorValue > 4 && averageFrontSensorValue < leftSensorValue){
        collectorState->setSpeeds(-0.7 * collectorState->turningSpeed, 0.7 * collectorState->turningSpeed);
    }

    if (rightSensorValue > 4 && averageFrontSensorValue < rightSensorValue){
        collectorState->setSpeeds(0.7 * collectorState->turningSpeed, -0.7 * collectorState->turningSpeed);
    }


}


/**
 * When the current destination is reached, calls @readNewDestinations.
 * Drives to the current destination.
 * @return True iff the last destination is reached.
 */
bool driveToDestination() {
    if (collectorState->destinationReached) {
        struct CoordinateQueue::CoordinateNode *node = coordinateQueue->pop();
        if (node == nullptr) {
            collectorState->setSpeeds(0, 0);
            return false;
        }

        collectorState->destinationX = node->x;
        collectorState->destinationY = node->y;
        collectorState->destinationReached = false;
    }
    return collectorState->navigateToDestination();
}

/**
 * Reads a new destination from the serial stream and adds it to the queue.
 * New destination must be given as to integers separated by some non-numeric character
 */
void readNewDestinations() {
    // read new destination entry
    if (Serial1.available() > 2) {
        int xDestination = 0;
        int yDestination = 0;

        xDestination = Serial1.parseInt();
        yDestination = Serial1.parseInt();

        Serial1.print("NEW DESTINATION IN QUEUE: (");
        Serial1.print(xDestination);
        Serial1.print(", ");
        Serial1.print(yDestination);
        Serial1.println(")");
        Serial1.flush();

        coordinateQueue->append(xDestination, yDestination);
    }
}

/**
 * Performs the celebration rotation
 */
void performRotation() {
    float startAngle = collectorState->currentAngle;
    bool loopCondition = true;

    while (loopCondition) {

        // turn right
        collectorState->setSpeeds(collectorState->turningSpeed, -collectorState->turningSpeed);
        collectorState->updateRoboterPositionAndAngles();

        /* rotation completed condition */
        if (collectorState->currentAngle > startAngle + 2 * M_PI ||
            collectorState->currentAngle < startAngle - 2 * M_PI) {
            loopCondition = false;
            Serial1.println("One rotation performed!");
            collectorState->setSpeeds(0, 0);

        }
    }
}

void performStraightDrive(int cmLength) {
    float startX = collectorState->currentX;
    float targetX = startX + cmLength;
    bool loopCondition = true;

    while (loopCondition) {

        // drive straight
        collectorState->setSpeeds(collectorState->forwardSpeed, collectorState->forwardSpeed);

        delay(10);

        collectorState->updateRoboterPositionAndAngles();

        /* rotation completed condition */
        if (collectorState->currentX > targetX) {
            loopCondition = false;
            Serial1.println("Driving performed!");
            collectorState->setSpeeds(0, 0);

        } else {
            Serial1.println(collectorState->currentX);
        }
    }
}


