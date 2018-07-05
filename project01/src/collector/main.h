//
// Created by Daniel Schäfer on 05.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_MAIN_H
#define EMBEDDEDSYSTEMS18_MAIN_H


/* debug define to enable serial output/inputs to save program memory */
#define DEBUG

#define ARENA_SIZE_X 140
#define PROXIMITY_THRESHOLD 7  // (10-7) * 5cm = 15cm

#define SERIAL_TIMEOUT_BLOCKING_READING 20000

#define DEBUG

/* Scenario enabling disabling */
// #define SCENARIO_RELAY
#define SCENARIO_HOMING
//#define SCENARIO_DEBUG_RF_REGISTER_CHECK


#define LINE_SENSOR_READINGS
// #define ROBOT_SIMULATOR

// #define COLLECTOR_MONITOR

void setup();
void loop();
void performRotation(int degrees);
void performStraightDrive(int cmLength);
void huntObject();
void generateBrightnessLevels();
void homing();
void checkForNewRFMessage();
bool detectLine();
void checkForLines();
void generateBrightnessLevels();


void receivePosUpdate(unsigned int angle, unsigned int x, unsigned int y);

#endif //EMBEDDEDSYSTEMS18_MAIN_H