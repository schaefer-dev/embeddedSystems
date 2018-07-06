#include "stdint.h"
#include "Arduino.h"
#include "CollectorLineSensors.h"
#include "Zumo32U4LineSensors.h"
#include "CollectorSerial.h"
#include "CollectorState.h"


Zumo32U4LineSensors CollectorLineSensors::lineSensors;
bool CollectorLineSensors::onLine;

void CollectorLineSensors::init(CollectorState *collectorState) {
    collectorState->drivingDisabled = false;
    lineSensors.initThreeSensors();

    calibrate(collectorState);
#ifdef COLLECTOR_DEBUG
    Serial1.print(messageInitLineSensors);
#endif
}

CollectorLineSensors::CollectorLineSensors() {
    uint8_t pins[] = {SENSOR_DOWN1, SENSOR_DOWN3, SENSOR_DOWN5}; // sensor 2 & 4 collision with proximity
    lineSensors = Zumo32U4LineSensors(pins, 3);
}

void CollectorLineSensors::calibrate(CollectorState *collectorState) {
    collectorState->drivingDisabled = false;

    for (int i = 0; i < 80; i++) {
        if (i < 40)
            collectorState->setSpeeds(80, 80);
        else
            collectorState->setSpeeds(-80, -80);

        lineSensors.calibrate();

        delay(20);
    }
    collectorState->setSpeeds(0, 0);
    collectorState->drivingDisabled = true;
#ifdef COLLECTOR_DEBUG
    Serial1.print(messageInitLineSensors);
#endif
}

bool CollectorLineSensors::detectLine() {
    uint16_t sensorReadings[3] = {0, 0, 0};
    bool lineDetected = false;

    lineSensors.readCalibrated(sensorReadings);

    delay(10);

    //Serial1.print("\nvals:\n");
    for (unsigned int sensorReading : sensorReadings) {
        //Serial1.print(sensorReading);
        //Serial1.print("\n");
    }

    if (sensorReadings[1] > 600) {
        if (!onLine) {
            lineDetected = true;
        }
        onLine = true;
    } else {
        onLine = false;
    }
    return lineDetected;
}


void CollectorLineSensors::driveOverLines(CollectorState *state) {
    /*int serialMessageLength = 0;
    char serialMessage[2];

    serialMessageLength = CollectorSerial::readMessageFromSerial(serialMessage);
    if (serialMessageLength < 1) {
        return;

    }*/

#ifdef COLLECTOR_DEBUG
    Serial1.write(serialMessage);
    Serial1.write("\n");
#endif

    int number = 6;
    /*
    int number = serialMessage[0] - 48;
    if (serialMessageLength > 1) {
        number *= 10;
        number += serialMessage[1] - 48;
    }*/

#ifdef COLLECTOR_DEBUG
    Serial1.write("drive lines: ");
    Serial1.write(number);
#endif
    state->drivingDisabled = false;
    while (number > 0) {
        state->setSpeeds(80, 80);
        if (detectLine()) {
            Serial1.write("Found a line\n");
            --number;
        }
    }
    state->setSpeeds(0, 0);
    state->drivingDisabled = true;
}