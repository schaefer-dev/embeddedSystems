//
// Created by Daniel Schäfer on 06.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_SCOUT_MAIN_H
#define EMBEDDEDSYSTEMS18_SCOUT_MAIN_H

#include <stdlib.h>
#include "ScoutState.h"
#include "ScoutSPI.h"
#include "ScoutSerial.h"

/* debug define to enable serial output/inputs to save program memory */
#define ARENA_SIZE_X 140
#define ARENA_SIZE_Y 80

#define photophobicWaitThreshold 50
#define photophobicDanceThreshold 100
#define PHOTOPHOBIC_ROTATION 60
#define PHOTOSENSOR_MINUS_CALIBRATED 20

#define PHOTOSENSOR_THRESHOLD_NO_GAME 200


/* Scenarios enabling disabling */
// #define SCENARIO_RELAY
// #define SCENARIO_HOMING
// #define SCENARIO_PHOTOPHOBIC
// #define SCENARIO_DEBUG_SEND_MESSAGES_CONTINIOUS
// #define LINE_SENSOR_READINGS
// #define DEBUG_SERIAL_PORT_ECHO
// #define SCOUT_MONITOR

#define SCOUT_GAME

// #define DEBUG

#define checkPhotoSensorEveryXLoops 42
#define UDATE_POSITION_EVERY_X_LOOPS 10

#define MSToCheckPhotosensorsBeforeMaximumSent 1000


void initialize();
int main();
void performRotation(int degrees);
void debug_printPhotosensorReadings();
void photophobicScout();
void checkForNewRFMessage();
void homing();

void receivePosUpdate(unsigned int angle, unsigned int x, unsigned int y);

#endif //EMBEDDEDSYSTEMS18_SCOUT_MAIN_H

