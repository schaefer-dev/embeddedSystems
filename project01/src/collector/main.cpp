#include <Arduino.h>
#include <Zumo32U4ProximitySensors.h>
#include "CollectorState.h"
#include "Coordinates.h"
#include "main.h"
#include <math.h>

CoordinateQueue *coordinateQueue;
CollectorState *collectorState;
Zumo32U4ProximitySensors *proximitySensors;

const uint8_t PROXIMITY_THRESHOLD = 6;   // (10-6) * 5cm = 20cm

/* used for demonstration and testing of individual functionalities */
const uint8_t MODE_DESTINATION = 1;
const uint8_t MODE_HUNT_OBJECT = 2;
const uint8_t MODE_COMMUNICATION = 3;
uint8_t modeOfOperation = MODE_DESTINATION;

boolean terminate = false;

void setup() {

    proximitySensors = new Zumo32U4ProximitySensors();
    generateBrightnessLevels();
    proximitySensors->initThreeSensors();

    /* initialization of Data structures */
    collectorState = new CollectorState();
    coordinateQueue = new CoordinateQueue();

    // TODO test if priority queue based on euclid distance works
    coordinateQueue->append(20, 0);     // #2
    coordinateQueue->append(10, 10);    // #1
    coordinateQueue->append(-21, 0);    // #3

    // initialize differential updateRoboterPositionAndAngles
    collectorState->setSpeeds(0, 0);
    collectorState->resetDifferentialDrive(0, 0, 0);
    collectorState->lastDiffDriveCall = millis();

    // initialize serial connection
#ifdef DEBUG
    Serial1.begin(9600);
    Serial1.println("--- Start Serial Monitor ---");
    Serial1.println();
#endif
}

void loop() {

    /* Default roboter code */
    if (modeOfOperation == MODE_DESTINATION) {
        readNewDestinations();

        if (terminate) {
            performRotation();
        }
        else {
            if (driveToDestination()) {
                performRotation();
            }
            collectorState->updateRoboterPositionAndAngles();
        }
    }
    else if (modeOfOperation == MODE_HUNT_OBJECT) {
        huntObject();
    }
    else if (modeOfOperation == MODE_COMMUNICATION) {
        // TODO RF MODULE TESTS
    }
}


/* ------------------------------------------------------------*/
/* ----------------- HELPER FUNCTIONS -------------------------*/
/* ------------------------------------------------------------*/


void huntObject(){
    proximitySensors->read();
    // delay(5);

    uint8_t frontLeftSensorValue = proximitySensors->countsFrontWithLeftLeds();
    uint8_t frontRightSensorValue = proximitySensors->countsFrontWithRightLeds();
    uint8_t leftSensorValue = proximitySensors->countsLeftWithLeftLeds();
    uint8_t rightSensorValue = proximitySensors->countsRightWithRightLeds();

    float averageFrontSensorValue = (frontLeftSensorValue + frontRightSensorValue) / 2.0f;

#ifdef DEBUG
    Serial1.print(leftSensorValue);
    Serial1.print(", ");
    Serial1.print(frontLeftSensorValue);
    Serial1.print(", ");
    Serial1.print(frontRightSensorValue);
    Serial1.print(", ");
    Serial1.print(rightSensorValue);
    Serial1.println("");
#endif

    if (frontLeftSensorValue > PROXIMITY_THRESHOLD && frontRightSensorValue > PROXIMITY_THRESHOLD){
        collectorState->setSpeeds(0.5 * collectorState->forwardSpeed, 0.5 * collectorState->forwardSpeed);
        return;
    }

    if (frontLeftSensorValue > PROXIMITY_THRESHOLD){
        collectorState->setSpeeds(-0.7 * collectorState->turningSpeed, 0.7 * collectorState->turningSpeed);
        return;
    }

    if (frontRightSensorValue > PROXIMITY_THRESHOLD){
        collectorState->setSpeeds(0.7 * collectorState->turningSpeed, -0.7 * collectorState->turningSpeed);
        return;
    }

    if (leftSensorValue > PROXIMITY_THRESHOLD && averageFrontSensorValue < leftSensorValue){
        collectorState->setSpeeds(-0.7 * collectorState->turningSpeed, 0.7 * collectorState->turningSpeed);
        return;
    }

    if (rightSensorValue > PROXIMITY_THRESHOLD && averageFrontSensorValue < rightSensorValue){
        collectorState->setSpeeds(0.7 * collectorState->turningSpeed, -0.7 * collectorState->turningSpeed);
        return;
    }
}

/**
 * When the current destination is reached, calls @readNewDestinations.
 * Drives to the current destination.
 * @return True iff the last destination is reached.
 */
bool driveToDestination() {
    if (collectorState->destinationReached) {
        struct CoordinateQueue::CoordinateNode *node = coordinateQueue->pop(collectorState->currentX, collectorState->currentY);
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
#ifdef DEBUG
/* IMPORTANT:
 * Reading serial is what blows memory up, around 30% - should be disabled once we get close to 100% */
    if (Serial1.available() > 2) {
        int xDestination = 0;
        int yDestination = 0;

        bool leftFull = false;
        String inStringLeft = "";   // string to hold input
        String inStringRight = "";  // string to hold input
        String command;
        while(Serial1.available() > 0) {
            int inChar = Serial.read();
            command += (char)inChar;
            if (!isDigit(inChar)) {
                leftFull = true;
                continue;
            }
            if (isDigit(inChar)) {
                // convert the incoming byte to a char and add it to the string:
                if (!leftFull)
                    inStringLeft += (char)inChar;
                else
                    inStringRight += (char)inChar;
            }
        }
        command.toUpperCase();


        if (command.equals("END")) {
            terminate = true;
        }
        else if (inStringLeft.length() > 0 && inStringRight.length() > 0){
            xDestination = (int)inStringLeft.toInt();
            yDestination = (int)inStringRight.toInt();
            Serial1.print("NEW DESTINATION IN QUEUE: (");
            Serial1.print(xDestination);
            Serial1.print(", ");
            Serial1.print(yDestination);
            Serial1.println(")");
            Serial1.flush();
            coordinateQueue->append(xDestination, yDestination);
        }
    }
#endif
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
#ifdef DEBUG
            Serial1.println("One rotation performed!");
#endif
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
#ifdef DEBUG
            Serial1.println("Driving performed!");
#endif
            collectorState->setSpeeds(0, 0);

        } else {
#ifdef DEBUG
            Serial1.println(collectorState->currentX);
#endif
        }
    }
}

void generateBrightnessLevels() {
    //uint16_t defaultBrightnessLevels[] = { 5, 15, 32, 55, 85, 120 };
    const uint16_t numBrightnessLevels = 10;
    uint16_t defaultBrightnessLevels[numBrightnessLevels] = {};

    /* generate 10 brightness values that will scale linearly for
     * proximities from 0cm to 50cm. Anything over 50cm will return
     * proximity 0. 25cm will return proximity of 5 etc. */
    for (uint16_t i = 0; i < numBrightnessLevels; ++i) {
        double magic = (2.236 + 1.0975 * (i / 2.0f));
        defaultBrightnessLevels[i] = static_cast<uint16_t>(magic * magic * 1/4.0f);
    }
    proximitySensors->setBrightnessLevels(defaultBrightnessLevels, numBrightnessLevels);
}