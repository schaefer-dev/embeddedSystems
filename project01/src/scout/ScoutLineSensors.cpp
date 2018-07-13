#include "ScoutLineSensors.h"
#include "Pololu3pi.h"
#include "ScoutSerial.h"
#include "ScoutState.h"
#include "main.h"

int driveLines = 0;

bool ScoutLineSensors::onLine;
const uint8_t ScoutLineSensors::threshold;

void ScoutLineSensors::init(){
    pololu_3pi_init(5000);     // recommended value between 2000 and 7500 depending on lighting condition
    delay(10);
}

void ScoutLineSensors::calibrate(ScoutState* state, int duration){
    int calibrationSpeed = state->forwardSpeed / 3;
    state->drivingDisabled = false;

    uint8_t calibrations = duration/20;

    for (int i = 0; i < calibrations; i++) {
        if (i < calibrations/4) {
            OrangutanMotors::setSpeeds(calibrationSpeed, calibrationSpeed);
        }
        else if (i < calibrations/2 && i >= calibrations/4) {
            OrangutanMotors::setSpeeds(calibrationSpeed/1.5, calibrationSpeed/1.5);
        }
        else if (i >= calibrations/2 && i <= calibrations/1.5) {
            OrangutanMotors::setSpeeds(-calibrationSpeed/1.5, -calibrationSpeed/1.5);
        }
        else {
            OrangutanMotors::setSpeeds(-calibrationSpeed, -calibrationSpeed);
        }
        calibrate_line_sensors(IR_EMITTERS_OFF);
        delay(20);
    }
    OrangutanMotors::setSpeeds(0, 0);
    state->drivingDisabled = true;
}

bool ScoutLineSensors::readNewLines() {
    if (driveLines == 0) {
        int serialMessageLength = 1;
        char serialMessage[50];
        char message = 0;
        message = ScoutSerial::readSingleCharFromSerial();

        //serialMessageLength = ScoutSerial::readMessageFromSerial(serialMessage);
        if (message == (char)0) {
            return false;
        }

        //ScoutSerial::serialWrite(message, serialMessageLength);
        //ScoutSerial::serialWrite("\n", 1);

        int number = message - 48;
        if (serialMessageLength > 1) {
            number *= 10;
            number += serialMessage[1] - 48;
        }

        ScoutSerial::serialWrite("drive lines: ", 13);
        ScoutSerial::serialWriteInt(number);
        driveLines = number;
        return true;
    }
    return false;
}

void ScoutLineSensors::checkForLines(ScoutState* state) {
    int number = 4;

#ifdef DEBUG
#endif
    state->drivingDisabled = false;
    while (number > 0) {
        state->setSpeeds(state->forwardSpeed/2, state->forwardSpeed/2);
        if (detectLine()) {
            ScoutSerial::serialWrite("Found line", 10);
            --number;
        }
    }
    state->setSpeeds(0, 0);
    state->drivingDisabled = true;
}

bool ScoutLineSensors::detectLine() {
    bool lineDetected = false;
    unsigned int sensorReadings[5] = {0, 0, 0, 0, 0};

    read_line(sensorReadings, IR_EMITTERS_OFF);
    delay(10);

#ifdef DEBUG
    ScoutSerial::serialWrite("vals:\n", 6);
    for (unsigned int sensorReading : sensorReadings) {
        ScoutSerial::serialWriteInt(sensorReading);
    }
#endif

    if (sensorReadings[2] > threshold) {
        if (!onLine) {
            lineDetected = true;
        }
        onLine = true;
    } else {
        onLine = false;
    }
    return lineDetected;
}