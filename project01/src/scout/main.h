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
#define DEBUG
#define ARENA_SIZE_X 140

#define photophobicWaitThreshold 50
#define photophobicDanceThreshold 100
#define PHOTOPHOBIC_ROTATION 60

/* Scenarios enabling disabling */
// #define SCENARIO_RELAY
// #define SCENARIO_HOMING
// #define SCENARIO_PHOTOPHOBIC
// #define SCENARIO_DEBUG_SEND_MESSAGES_CONTINIOUS
#define LINE_SENSOR_READINGS
// #define DEBUG_SERIAL_PORT_ECHO

// #define SCOUT_MONITOR

// #define ROBOT_SIMULATOR


void initialize();
int main();
void performRotation(int degrees);
void debug_printPhotosensorReadings();
void performStraightDrive(int cmLength);
void photophobicScout();
void checkForNewRFMessage();
void debug_sendPingToCollector();
void homing();

void receivePosUpdate(unsigned int angle, unsigned int x, unsigned int y);

#endif //EMBEDDEDSYSTEMS18_SCOUT_MAIN_H

