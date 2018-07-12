#include "stdint.h"
#include "Arduino.h"
#include "CollectorLineSensors.h"
#include "Zumo32U4LineSensors.h"
#include "CollectorState.h"


Zumo32U4LineSensors CollectorLineSensors::lineSensors;
bool CollectorLineSensors::onLine;
const uint8_t CollectorLineSensors::threshold;

void CollectorLineSensors::init(CollectorState *collectorState) {
    collectorState->drivingDisabled = false;
    lineSensors.initThreeSensors();

    calibrate(collectorState);
#ifdef COLLECTOR_DEBUG
    Serial1.print(messageInitLineSensors);
#endif
}

void CollectorLineSensors::calibrate(CollectorState *collectorState) {
    int calibrationSpeed = 50;
    collectorState->drivingDisabled = false;

    for (int i = 0; i < 80; i++) {
        if (i < 40) {
            collectorState->setSpeeds(calibrationSpeed, calibrationSpeed);
        }
        else {
            collectorState->setSpeeds(-calibrationSpeed, -calibrationSpeed);
        }
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

    Serial1.print(sensorReadings[1]);
    Serial1.print("\n");

    /*
    for (unsigned int sensorReading : sensorReadings) {
        Serial1.print(sensorReading);
        Serial1.print("\n");
    }*/

    if (sensorReadings[1] > threshold) {
        if (!onLine) {
            lineDetected = true;
        }
        onLine = true;
    } else {
        onLine = false;
    }
    return lineDetected;
}


void CollectorLineSensors::driveOverLines(CollectorState *collectorState) {
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

    int number = 0;
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
    collectorState->drivingDisabled = false;
    while (true) {
        detectLine();
        continue;
        collectorState->setSpeeds(collectorState->forwardSpeed, collectorState->forwardSpeed);
        if (detectLine()) {
            Serial1.write("Found a line\n");
            --number;
        }
    }
    collectorState->setSpeeds(0, 0);
    collectorState->drivingDisabled = true;
}
