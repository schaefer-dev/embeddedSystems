#include <Arduino.h>
#include <Zumo32U4ProximitySensors.h>
#include "CollectorState.h"
#include "../utils/Coordinates.h"
#include "main.h"
#include <math.h>
#include "avr/io.h"
#include "avr/interrupt.h"
#include "../scout/main.h"
#include "CollectorMonitor.h"
#include "Zumo32U4LineSensors.h"
#include "CollectorLineSensors.h"
#include "Zumo32U4Motors.h"
#include "CollectorLineSensors.h"
#include "platform.h"
#include "../common/rf.h"

uint8_t rf_my_address[5] = {0x98, 0x65, 0xFA, 0x29, 0xE6};
uint8_t rf_partner_address[5] = {0xE2, 0x91, 0xA8, 0x27, 0x85};
extern uint8_t rf_referee_address [5];
extern uint8_t rf_status;
extern uint8_t rf_intr;
static char serial_buffer [80];

const char messageInitAll[]      = "--------------------  Initialized ---------------------\n";
const char messageInitSerial[]      = "-------------- Serial Initialized ---------------------\n";
const char messageInitSPI[]         = "------------- SPI Master Initialized ------------------\n";
const char messageInitProximity[]   = "-------------- Proximity Initialized ------------------\n";
const char messageInitLineSensors[] = "------------ Line Sensors Initialized -----------------\n";
const char messageInitRF[]          = "------------- RF Module  Initialized ------------------\n";


CollectorState *collectorState;
Zumo32U4ProximitySensors *proximitySensors;

int home[2] = {10, 20};      // home
bool terminate;

void setup() {
    // 1.1. Place the robot in the arena, 1 sec to do this
    delay(1000);

    // 1.2 Start calibrating
    // initialize serial connection
    Serial1.begin(9600);
    platform_init();    // important to do this after Serial initialization
    _delay_ms(200);
    rf_setup(rf_my_address);
    _delay_ms(200);
    rf_start_listening();
    Serial1.print(messageInitAll);

    /* initialization of Data structures */
    collectorState = new CollectorState();

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

    terminate = false;


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


#ifdef COLLECTOR_GAME
    // time to cancel and restart the robot
    delay(1000);

    // 2. Send a HELLO message to the referee on channel 111
    uint8_t payloadArray[2];
    payloadArray[0] = 0x42;
    payloadArray[1] = (uint8_t) (15);
    rf_stop_listening();
    rf_sendto_blocking(rf_referee_address, payloadArray, 2);
    rf_start_listening();

    Serial1.print("HELLO sent\n");
    collectorState->drivingDisabled = false;

    /*  Busy wait until config message received  */
    uint8_t receiveBuffer[32];
    waitForMessage(0x43, receiveBuffer);

    Serial1.print("Message arrived\n");

    while (!collectorState->configurationReceived) {
        static uint8_t rf_buffer [32];

        if (rf_data_available()) {
            rf_standby();
            uint8_t len = rf_read_payload_dyn(rf_buffer);
            if ((len == 3) && (rf_buffer[0] == 0xFF)) {
                sprintf(serial_buffer, "Received ADC %d value for sensor: %d\n", rf_buffer[1], rf_buffer[2]);
                Serial1.print(serial_buffer);
            }
            rf_activate();

            if (rf_buffer[0] == 0x43 && len == 2){
                Serial1.print("unexpected message received!");
                uint8_t channel = rf_buffer[1];
                Serial1.print("New Channel: ");
                Serial1.println(channel);

                rf_write_register(RF_CH, channel);
                collectorState->configurationReceived = true;
                break;
            }
        }
        continue;
    }
    Serial1.print("CONFIG received\n");

    // wait until the light turns on
    delay(3100);

    int calibrationDuration = 2000;
    CollectorLineSensors::calibrate(collectorState, calibrationDuration);

    // wait until the light turns off
    Serial1.print("Waiting for GO\n");
    while(!collectorState->gameStarted) {
        static uint8_t rf_buffer [32];
        if (rf_data_available()) {
            rf_standby();
            uint8_t len = rf_read_payload_dyn(rf_buffer);
            rf_activate();
            if (len == 0)
                continue;
        } else {
            continue;
        }
        if (rf_buffer[0] != 0x44){
            Serial1.print("unexpected message received!");
        }

        collectorState->gameStarted = true;
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
    static uint8_t rf_buffer [32];

    if (rf_data_available()) {
        rf_standby();
        uint8_t len = rf_read_payload_dyn(rf_buffer);
        if ((len == 3) && (rf_buffer[0] == 0xFF)) {
            sprintf(serial_buffer, "Received ADC %d value for sensor: %d\n", rf_buffer[1], rf_buffer[2]);
            Serial1.print(serial_buffer);
        }
        rf_activate();
    }

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

void waitForMessage(uint8_t prefix, uint8_t* receiveBuffer){
    while (1){
        if(rf_data_available()){
            rf_read_payload_dyn(receiveBuffer);
            if (receiveBuffer[0] == prefix)
                return;
        }
    }
}

