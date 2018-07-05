#include <Arduino.h>
#include <Zumo32U4ProximitySensors.h>
#include "CollectorState.h"
#include "../utils/Coordinates.h"
#include "main.h"
#include "CollectorSPI.h"
#include <math.h>
#include "avr/io.h"
#include "avr/interrupt.h"
#include "../scout/main.h"
#include "CollectorMonitor.h"
#include "CollectorRF.h"
#include "CollectorSerial.h"

CollectorState *collectorState;
Zumo32U4ProximitySensors *proximitySensors;

boolean terminate = false;
int home[2] = {10, 20};      // home
int statusRF = 0;

void setup() {
    proximitySensors = new Zumo32U4ProximitySensors();
    generateBrightnessLevels();
    proximitySensors->initThreeSensors();

    /* initialization of Data structures */
    collectorState = new CollectorState();
    statusRF = 0;

    // initialize differential updateRoboterPositionAndAngles
    collectorState->setSpeeds(0, 0);
    collectorState->resetDifferentialDrive(0, 0, 0);
    collectorState->lastDiffDriveCall = millis();

    // initialize serial connection
    CollectorSerial::initCollectorSerial();

    /* Home is always our next destination */
    collectorState->nextDestinationX = home[0];
    collectorState->nextDestinationY = home[1];
    collectorState->nextDestinationCounter += 1;


#ifdef COLLECTOR_MONITOR
    CollectorMonitor::verifyState();
    /* TEST LOGGING SERIES */
    CollectorMonitor::logPingCollector();
    CollectorMonitor::logPingCollector();
    CollectorMonitor::logPingCollector();
    CollectorMonitor::logPingCollector();       // should give bad trace alarm.
    CollectorMonitor::logPongCollector();
    CollectorMonitor::logPingCollector();

    CollectorMonitor::logCheckProximity(true);
    CollectorMonitor::logAtHarvest(false);
    CollectorMonitor::logCheckProximity(false); // should give bad trace alarm.
    CollectorMonitor::logAtHarvest(false);
    CollectorMonitor::emptyBuffer();
    delay(600);
    CollectorMonitor::emptyBuffer();            // will come too late, bad trace
    CollectorMonitor::verifyState();
#endif

    CollectorSPI::SPIMasterInit();
    delay(50);
    CollectorSerial::serialWrite("--- SPI MASTER INITIALIZED ---\n", 0);
    CollectorRF::initializeRFModule();
    CollectorSerial::serialWrite("--- RF MODULE INITIALIZED ---\n", 0);
    delay(100);

}

void loop() {

#ifdef SCENARIO_HOMING
        homing();
#endif

    checkForNewRFMessage();


#ifdef SCENARIO_DEBUG_RF_REGISTER_CHECK
    delay(150);

    Serial1.println("REGISTER CHECk START:");
    CollectorSPI::debug_RFModule();
    Serial1.println("REGISTER CHECk END:");
    delay(1000);
#endif
}


/* ------------------------------------------------------------*/
/* ----------------- HELPER FUNCTIONS -------------------------*/
/* ------------------------------------------------------------*/


void receivePosUpdate(unsigned int angle, unsigned int x, unsigned int y){
    float currentAngle = (float)angle / 10000.0f;
    currentAngle += 0.5f * M_PI;
    int currentX = ARENA_SIZE_X - x / 10.0f;
    int currentY = y / 10.0f;

#ifdef DEBUG
    Serial1.print("Received POS update, Angle: ");
    Serial1.print(currentAngle);
    Serial1.print(" X: ");
    Serial1.print(currentX);
    Serial1.print(" Y: ");
    Serial1.println(currentY);
#endif

    collectorState->resetDifferentialDrive(currentX, currentY, currentAngle);

    collectorState->destinationReached = true;
};

void checkForNewRFMessage(){
    statusRF = CollectorRF::queryRFModule();
    char messageReceived = statusRF & (1 << 6);

    if (messageReceived){
        CollectorRF::processReceivedMessage(collectorState);
    }
}

void homing() {

    collectorState->navigate();

    delay(1);
    collectorState->updateRoboterPositionAndAngles();
}



void huntObject() {
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

    if (frontLeftSensorValue > PROXIMITY_THRESHOLD && frontRightSensorValue > PROXIMITY_THRESHOLD) {
        collectorState->setSpeeds(0.5 * collectorState->forwardSpeed, 0.5 * collectorState->forwardSpeed);
        return;
    }

    if (frontLeftSensorValue > PROXIMITY_THRESHOLD) {
        collectorState->setSpeeds(-0.7 * collectorState->turningSpeed, 0.7 * collectorState->turningSpeed);
        return;
    }

    if (frontRightSensorValue > PROXIMITY_THRESHOLD) {
        collectorState->setSpeeds(0.7 * collectorState->turningSpeed, -0.7 * collectorState->turningSpeed);
        return;
    }

    if (leftSensorValue > PROXIMITY_THRESHOLD && averageFrontSensorValue < leftSensorValue) {
        collectorState->setSpeeds(-0.7 * collectorState->turningSpeed, 0.7 * collectorState->turningSpeed);
        return;
    }

    if (rightSensorValue > PROXIMITY_THRESHOLD && averageFrontSensorValue < rightSensorValue) {
        collectorState->setSpeeds(0.7 * collectorState->turningSpeed, -0.7 * collectorState->turningSpeed);
        return;
    }
}


/**
 * Perform rotation defined by degree
 */
void performRotation(int degrees) {
    float startAngle = collectorState->currentAngle;
    float factor = 0.0f;
    if (degrees < 0){
        factor = ((float) (-1 * degrees)) / 360.0f;
    } else {
        factor = ((float) degrees) / 360.0f;
    }
    bool loopCondition = true;

    while (loopCondition) {
        if (degrees == 0)
            return;

        if (degrees > 0) {
            // turn right
            collectorState->setSpeeds(collectorState->turningSpeed, -collectorState->turningSpeed);
            collectorState->updateRoboterPositionAndAngles();
        } else {
            // turn left
            collectorState->setSpeeds(-collectorState->turningSpeed, collectorState->turningSpeed);
            collectorState->updateRoboterPositionAndAngles();
        }

        /* rotation completed condition */
        if (collectorState->currentAngle > startAngle + 2 * M_PI * factor ||
                collectorState->currentAngle < startAngle - 2 * M_PI * factor) {
            loopCondition = false;

#ifdef DEBUG
            //serial_send("One rotation performed!\n", 24);
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
        defaultBrightnessLevels[i] = static_cast<uint16_t>(magic * magic * 1 / 4.0f);
    }
    proximitySensors->setBrightnessLevels(defaultBrightnessLevels, numBrightnessLevels);
}