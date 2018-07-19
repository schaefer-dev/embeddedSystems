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
#include "Zumo32U4LineSensors.h"
#include "CollectorLineSensors.h"
#include "Zumo32U4Motors.h"
#include "CollectorLineSensors.h"
#include "platform_c.h"

const char messageInitSerial[]      = "-------------- Serial Initialized ---------------------\n";
const char messageInitSPI[]         = "------------- SPI Master Initialized ------------------\n";
const char messageInitProximity[]   = "-------------- Proximity Initialized ------------------\n";
const char messageInitLineSensors[] = "------------ Line Sensors Initialized -----------------\n";
const char messageInitRF[]          = "------------- RF Module  Initialized ------------------\n";


CollectorState *collectorState;
Zumo32U4ProximitySensors *proximitySensors;

int home[2] = {10, 20};      // home
int statusRF = 0;
bool terminate;

void setup() {
    // 1.1. Place the robot in the arena, 1 sec to do this
    //delay(1000);

    // 1.2 Start calibrating
    // initialize serial connection
    Serial1.begin(9600);
    Serial1.print(messageInitSerial);

    /* initialization of Data structures */
    collectorState = new CollectorState();
    statusRF = 0;

    CollectorLineSensors::init(collectorState);

    // initialize differential updateRoboterPositionAndAngles
    collectorState->setSpeeds(0, 0);
    collectorState->resetDifferentialDrive(0, 0, 0);
    collectorState->lastDiffDriveCall = millis();


#ifdef COLLECTOR_PROXIMITY_ENABLED
    proximitySensors = new Zumo32U4ProximitySensors();
    generateBrightnessLevels();
    proximitySensors->initThreeSensors();
    Serial1.print(messageInitProximity);
#endif


#ifdef COLLECTOR_LINE_SENSOR_READINGS
    // initialize line sensors
    collectorState->drivingDisabled = false;
    //uint8_t pins[] = { SENSOR_DOWN1, SENSOR_DOWN3}; // sensor 2 & 4 collision with proximity; 5 - timer4
    //lineSensors = Zumo32U4LineSensors(pins, 2);
    delay(10);
    collectorState->setSpeeds(0,0);
    collectorState->drivingDisabled = true;
    Serial1.print(messageInitLineSensors);
#endif


#ifdef COLLECTOR_SCENARIO_HOMING
    /* Home is always our next destination */
    collectorState->nextDestinationX = home[0];
    collectorState->nextDestinationY = home[1];
    collectorState->nextDestinationCounter += 1;
#endif


#ifdef COLLECTOR_COLLECTOR_MONITOR
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
    platform_init();
    //CollectorSPI::SPIMasterInit();
    delay(10);
    Serial1.print(messageInitSPI);
    ;
    CollectorRF::initializeRFModule();
    Serial1.print(messageInitRF);
    delay(10);


#ifdef COLLECTOR_GAME
    // time to cancel and restart the robot
    //delay(1000);

    // 2. Send a HELLO message to the referee on channel 111
    uint8_t payloadArray[2];
    payloadArray[0] = 0x42;
    payloadArray[1] = (uint8_t) (15);
    CollectorRF::sendMessageTo(CollectorRF::refereeAdress, payloadArray, 2);;
    Serial1.print("HELLO sent\n");
    collectorState->drivingDisabled = false;

    /*  Busy wait until config message received  */
    while (!collectorState->configurationReceived) {
        checkForNewRFMessage();
    }
    Serial1.print("CONFIG received\n");

    // wait until the light turns on
    delay(3100);

    int calibrationDuration = 2000;
    CollectorLineSensors::calibrate(collectorState, calibrationDuration);

    // wait until the light turns off
    Serial1.print("Waiting for GO\n");
    while(!collectorState->gameStarted) {
        checkForNewRFMessage();
    }

    /*
#ifdef COLLECTOR_LINE_SENSOR_READINGS
    CollectorLineSensors::driveOverLines(collectorState);
#endif
     */
#endif

    Serial1.print("GO!!!\n");
    collectorState->destinationReached = true;
    collectorState->drivingDisabled = false;

}

void loop() {
    /* ALWAYS check for new RF Message */

    checkForNewRFMessage();
    collectorState->navigate();

    /*if (terminate){
        Serial1.println(testInput);
        delay(2000);
        return;
    }

    Serial1.readBytes(testInput, 51);
    terminate = true;
     */

    /* if collector is not driving to a harvest destination,
     * check for proximity readings and try to hunt the enemy,
     * otherwise continue to go to random destinations */

    /*
    if (!collectorState->isHarvestDestination) {
        if (!huntObject()){
            collectorState->navigate();
        }
    } else {
        collectorState->navigate();
    } */

    //collectorState->navigate();

    delay(1);
    //collectorState->updateRoboterPositionAndAngles();


#ifdef COLLECTOR_SCENARIO_DEBUG_RF_REGISTER_CHECK
    delay(150);
#ifdef DEBUG
    Serial1.println("REGISTER CHECk START:");
    CollectorSPI::debug_RFModule();
    Serial1.println("REGISTER CHECk END:");
    delay(1000);
#endif
#endif
}


/* ------------------------------------------------------------*/
/* ----------------- HELPER FUNCTIONS -------------------------*/
/* ------------------------------------------------------------*/


void receivePosUpdate(unsigned int angle, unsigned int x, unsigned int y){
    float currentAngle = (float)angle / 10000.0f;
    currentAngle += 0.5f * M_PI;

    /* catch negative coordinates which are encoded to large ints */
    if (x > 2000)
        x = 0;
    if (y > 2000)
        y = 0;

    int currentX = ARENA_SIZE_X - x / 10.0f;
    int currentY = y / 10.0f;

#ifdef COLLECTOR_DEBUG
    Serial1.print("Received POS update, Angle: ");
    Serial1.print(currentAngle);
    Serial1.print(" X: ");
    Serial1.print(currentX);
    Serial1.print(" Y: ");
    Serial1.println(currentY);
    Serial1.flush();
#endif

    collectorState->resetDifferentialDrive(currentX, currentY, currentAngle);

    collectorState->destinationReached = true;
};


void checkForNewRFMessage(){
    statusRF = CollectorRF::queryRFModule();
    char messageReceived = statusRF & (1 << 6);

    if (messageReceived){
        /* clear status register, so we can respond asap again */
        CollectorRF::writeRegister(RF_REGISTER_STATUS, 64);
        CollectorRF::processReceivedMessage(collectorState);
    }
}

void homing() {

    collectorState->navigate();

    delay(1);
    collectorState->updateRoboterPositionAndAngles();
}


#ifdef COLLECTOR_HUNT_OBJECT
bool huntObject() {
    proximitySensors->read();
    uint8_t frontLeftSensorValue = proximitySensors->countsFrontWithLeftLeds();
    uint8_t frontRightSensorValue = proximitySensors->countsFrontWithRightLeds();
    uint8_t leftSensorValue = proximitySensors->countsLeftWithLeftLeds();
    uint8_t rightSensorValue = proximitySensors->countsRightWithRightLeds();

    float averageFrontSensorValue = (frontLeftSensorValue + frontRightSensorValue) / 2.0f;

#ifdef COLLECTOR_DEBUG
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
        return true;
    }

    if (frontLeftSensorValue > PROXIMITY_THRESHOLD) {
        collectorState->setSpeeds(-0.7 * collectorState->turningSpeed, 0.7 * collectorState->turningSpeed);
        return true;
    }

    if (frontRightSensorValue > PROXIMITY_THRESHOLD) {
        collectorState->setSpeeds(0.7 * collectorState->turningSpeed, -0.7 * collectorState->turningSpeed);
        return true;
    }

    if (leftSensorValue > PROXIMITY_THRESHOLD && averageFrontSensorValue < leftSensorValue) {
        collectorState->setSpeeds(-0.7 * collectorState->turningSpeed, 0.7 * collectorState->turningSpeed);
        return true;
    }

    if (rightSensorValue > PROXIMITY_THRESHOLD && averageFrontSensorValue < rightSensorValue) {
        collectorState->setSpeeds(0.7 * collectorState->turningSpeed, -0.7 * collectorState->turningSpeed);
        return true;
    }
}
#endif


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

#ifdef COLLECTOR_DEBUG
            //serial_send("One rotation performed!\n", 24);
#endif
            collectorState->setSpeeds(0, 0);
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


