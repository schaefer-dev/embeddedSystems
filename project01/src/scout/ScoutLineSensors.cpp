#include "ScoutLineSensors.h"
#include "Pololu3pi.h"
#include "ScoutSerial.h"
#include "ScoutState.h"

int driveLines = 0;


ScoutLineSensors::ScoutLineSensors() {
    ;
}


void init(){
    pololu_3pi_init(5000);     // recommended value between 2000 and 7500 depending on lighting condition
    delay(10);

}

void calibrate(){
    for (int i = 0; i < 10; i++) {
        calibrate_line_sensors(IR_EMITTERS_OFF);
        //robot.calibrateLineSensors(IR_EMITTERS_ON);

        delay_ms(500);
    }
    OrangutanMotors::setSpeeds(0, 0);
}

bool readNewLines() {
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

void checkForLines() {
    ScoutLineSensors::detectLine();
    return;
    if (ScoutLineSensors::detectLine()) {
        ScoutSerial::serialWrite("Found a line\n", 13);
        --driveLines;
        while (ScoutLineSensors::detectLine()) {

            // wait until line is lost to count the next one
        }
        ScoutSerial::serialWrite("Line lost\n", 10);
    }
}

bool detectLine() {
    unsigned int sensorReadings[5] = {0, 0, 0, 0, 0};
    unsigned int sensorReadingsRaw[5] = {0, 0, 0, 0, 0};

    //unsigned int position = robot.readLine(sensorReadings, IR_EMITTERS_ON);
    unsigned int position = read_line(sensorReadings, IR_EMITTERS_OFF);
    delay(10);

    // robot.readLineSensors(sensorReadingsRaw, IR_EMITTERS_OFF);
    //read_line_sensors(sensorReadingsRaw, IR_EMITTERS_ON);

    ScoutSerial::serialWrite("\npos: ", 6);
    ScoutSerial::serialWriteInt(position);

    ScoutSerial::serialWrite("vals:\n", 6);
    for (unsigned int sensorReading : sensorReadings) {
        ScoutSerial::serialWriteInt(sensorReading);
    }

    /*ScoutSerial::serialWrite("raw:\n", 5);
    for (int i = 0; i < 5; ++i) {
        ScoutSerial::serialWriteInt(sensorReadingsRaw[i]);
    }*/
    delay(1000);
    return false;
}