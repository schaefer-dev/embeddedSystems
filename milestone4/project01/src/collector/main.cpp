#include <Arduino.h>
#include <Zumo32U4ProximitySensors.h>
#include "CollectorState.h"
#include "Coordinates.h"
#include "main.h"
#include <math.h>
#include "avr/io.h"
#include "avr/interrupt.h"
#include "../scout/main.h"

CoordinateQueue *coordinateQueue;
CollectorState *collectorState;
Zumo32U4ProximitySensors *proximitySensors;

const uint8_t PROXIMITY_THRESHOLD = 7;   // (10-7) * 5cm = 15cm

/* used for demonstration and testing of individual functionalities */
const uint8_t MODE_DESTINATION = 1;
const uint8_t MODE_HUNT_OBJECT = 2;
const uint8_t MODE_TIMER = 3;
uint8_t modeOfOperation = MODE_DESTINATION;

boolean terminate = false;
uint16_t numInterrups = 0;
uint16_t numInterrupsPrevCycle = 0;
const uint8_t TIMER_DURATION = 16;      // in ms, must be at least 16

void setup() {

    proximitySensors = new Zumo32U4ProximitySensors();
    generateBrightnessLevels();
    proximitySensors->initThreeSensors();

    /* initialization of Data structures */
    collectorState = new CollectorState();
    coordinateQueue = new CoordinateQueue();

    /* Test goals to test pathfinding
    coordinateQueue->append(20, 0);     // #2
    coordinateQueue->append(10, 10);    // #1
    coordinateQueue->append(-21, 0);    // #3
     */

    // initialize differential updateRoboterPositionAndAngles
    collectorState->setSpeeds(0, 0);
    collectorState->resetDifferentialDrive(0, 0, 0);
    collectorState->lastDiffDriveCall = millis();

    // initialize timer
    setTimer(TIMER_DURATION);

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
    else if (modeOfOperation == MODE_TIMER) {
        if (numInterrups > numInterrupsPrevCycle) {
            numInterrupsPrevCycle = numInterrups;
            Serial1.print("Interrupt #");
            Serial1.print(numInterrups);
            Serial1.print(", calculated time: ");
            Serial1.print(numInterrups * TIMER_DURATION);
            Serial1.print(", real time: ");
            Serial1.print(millis());
            Serial1.println(" ms");
        }
    }
}


/* ------------------------------------------------------------*/
/* ----------------- HELPER FUNCTIONS -------------------------*/
/* ------------------------------------------------------------*/


void huntObject(){
    proximitySensors->read();
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
    if (Serial1.available() > 6) {
        int xDestination = 0;
        int yDestination = 0;

        bool leftFull = false;

        int stringIndex = 0;
        char inStringLeft[4];
        char inStringRight[4];

        for (int i = 0; i < 4; i++){
            inStringLeft[i] = '\0';
            inStringRight[i] = '\0';
        }

        int commandIndex = 0;
        char command[10];
        while(Serial1.available() > 0) {
            int inChar = Serial1.read();
            command[commandIndex] = (char)inChar;
            commandIndex += 1;
            if (!isDigit(inChar) && !leftFull) {
                leftFull = true;
                stringIndex = 0;
                continue;
            } else {
                // convert the incoming byte to a char and add it to the string:
                if (!leftFull)
                    inStringLeft[stringIndex] = (char)inChar;
                else
                    inStringRight[stringIndex] = (char)inChar;
                stringIndex += 1;
            }
        }


        if (command[0] == 'E' && command[1] == 'N' && command[2] == 'D') {
            terminate = true;
        }

        xDestination = atoi(inStringLeft);
        yDestination = atoi(inStringRight);
        Serial1.print("NEW DESTINATION IN QUEUE: (");
        Serial1.print(xDestination);
        Serial1.print(", ");
        Serial1.print(yDestination);
        Serial1.println(")");
        Serial1.flush();
        coordinateQueue->append(xDestination, yDestination);
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


/** set timer 4
 *
 * @param duration in ms
 */
void setTimer(int duration){
    uint8_t timer = 0;

    // select prescaler
    if (duration < 3){
        // prescaler 64 suffices
        timer = (duration * 125) -1 ;      // Collector runs on 1Mhz
        OCR4A = timer;

    } else {
        // prescaler 1024
        timer = (duration * 8) - 1;        // Collector runs on 1Mhz
        OCR4A = timer;
    }
    // ctc on OCR4A
    TCCR4B |= (1 << COM4A1 | (1 << PWM4A));

    // set ctc interrupt
    TIMSK4 |= (1 << OCIE4A);

    if (duration < 3){
        // prescaler 256 suffices
        TCCR4B &= ~(1 << CS43);
        TCCR4B |= (1 << CS40) | (1 << CS41) | (1 << CS42);
    } else {
        // prescaler 4096
        TCCR4B &= ~(1 << CS42);
        TCCR4B |= (1 << CS40) | (1 << CS41) | (1 << CS43);
    }

    // enable global interrupts
    sei();
}

ISR (TIMER4_COMPA_vect)
{
    numInterrups++;
}