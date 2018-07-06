#include "stdint.h"
#include "Arduino.h"
#include "CollectorLineSensors.h"
#include "Zumo32U4LineSensors.h"
#include "CollectorSerial.h"
#include "CollectorState.h"


Zumo32U4LineSensors lineSensors;

public:
CollectorLineSensors::CollectorLineSensors(CollectorState* collectorState){
    collectorState->drivingDisabled = false;
    uint8_t pins[] = { SENSOR_DOWN1, SENSOR_DOWN3, SENSOR_DOWN5}; // sensor 2 & 4 collision with proximity
    lineSensors = Zumo32U4LineSensors(pins, 3);
    for (int i = 0; i < 80; i++) {
        if(i < 20 || i >= 60)
            collectorState->setSpeeds(40,-40);
        else
            collectorState->setSpeeds(-40,40);

        lineSensors.calibrate(QTR_EMITTERS_ON_AND_OFF);

        delay(20);
    }
    collectorState->setSpeeds(0,0);
    collectorState->drivingDisabled = true;
#ifdef COLLECTOR_DEBUG
    Serial1.print(messageInitLineSensors);
#endif
}

CollectorLineSensors::CollectorLineSensors() {
    uint8_t pins[] = { SENSOR_DOWN1, SENSOR_DOWN3, SENSOR_DOWN5}; // sensor 2 & 4 collision with proximity
    lineSensors = Zumo32U4LineSensors(pins, 3);
}

static void calibrate(CollectorState* collectorState){
    collectorState->drivingDisabled = false;

    for (int i = 0; i < 80; i++) {
        if(i < 20 || i >= 60)
            collectorState->setSpeeds(40,-40);
        else
            collectorState->setSpeeds(-40,40);

        lineSensors.calibrate(QTR_EMITTERS_ON_AND_OFF);

        delay(20);
    }
    collectorState->setSpeeds(0,0);
    collectorState->drivingDisabled = true;
#ifdef COLLECTOR_DEBUG
    Serial1.print(messageInitLineSensors);
#endif
}

static bool detectLine() {
    unsigned int sensorReadings[3] = {0,0,0};
    bool lineDetected = false;

    lineSensors.readCalibrated(sensorReadings);

    delay(10);


    Serial1.print("\nvals:\n");
    for (unsigned int sensorReading : sensorReadings) {
        Serial1.print(sensorReading);
        Serial1.print("\n");
        if (sensorReading > 500){
            lineDetected = true;

        }
    }

    delay(500);
    return lineDetected;
}


static void checkForLines() {
    int serialMessageLength = 0;
    char serialMessage[2];

    serialMessageLength = CollectorSerial::readMessageFromSerial(serialMessage);
    if (serialMessageLength < 1) {
        return;

    }

#ifdef COLLECTOR_DEBUG
    Serial1.write(serialMessage);
    Serial1.write("\n");
#endif

    int number = serialMessage[0] - 48;
    if (serialMessageLength > 1) {
        number *= 10;
        number += serialMessage[1] - 48;
    }

#ifdef COLLECTOR_DEBUG
    Serial1.write("drive lines: ");
    Serial1.write(number);

    while(number > 0) {
        if (detectLine()) {
            Serial1.write("Found a line\n");
            --number;
            while (detectLine()) {
                delay(500);
                // wait until line is lost to count the next one
            }
            Serial1.write("Line lost\n");
        }
    }
#endif
}
