//
// Created by Daniel Schäfer on 05.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_MAIN_H
#define EMBEDDEDSYSTEMS18_MAIN_H


#define ARENA_SIZE_X 140
#define ARENA_SIZE_Y 80
#define PROXIMITY_THRESHOLD 7  // (10-7) * 5cm = 15cm

#define SERIAL_TIMEOUT_BLOCKING_READING 20000

#define UDATE_POSITION_EVERY_X_LOOPS 10

//#define COLLECTOR_DEBUG

/* Scenario enabling disabling */
// #define COLLECTOR_SCENARIO_RELAY
// #define COLLECTOR_SCENARIO_HOMING
// #define COLLECTOR_SCENARIO_DEBUG_RF_REGISTER_CHECK
// #define COLLECTOR_PROXIMITY_ENABLED
// #define COLLECTOR_LINE_SENSOR_READINGS
// #define COLLECTOR_HUNT_OBJECT

// #define COLLECTOR_GAME

// #define COLLECTOR_MONITOR

void setup();
void loop();
void performRotation(int degrees);
bool huntObject();
void generateBrightnessLevels();
void homing();
void checkForNewRFMessage();
void generateBrightnessLevels();


void receivePosUpdate(unsigned int angle, unsigned int x, unsigned int y);



#endif //EMBEDDEDSYSTEMS18_MAIN_H
