//
// Created by Daniel Schäfer on 08.06.18.
//
#include "ScoutSerial.h"
#include <OrangutanSerial.h>
#include <stdio.h>
#include <stdlib.h>
#include <OrangutanTime.h>

ScoutSerial::ScoutSerial() {
    for (int i = 0; i < 99; i++){
        receiveBuffer[i] = 0;
    }
    OrangutanSerial::receive(receiveBuffer, 100);
    receiveIndex = 0;
}

void ScoutSerial::serialRead(char *buffer, unsigned char size) {

    OrangutanSerial::receive(buffer, size);
}

void ScoutSerial::serialWrite(char *buffer, unsigned char size) {

    OrangutanSerial::sendBlocking(buffer, size);
}

int* ScoutSerial::readCoordinates(){

    delay(10);
    int newReceiveIndex = OrangutanSerial::getReceivedBytes();
    if (receiveIndex == newReceiveIndex)
        return nullptr;

    int returnArray[2];

    char xString[4];
    char yString[4];

    for (int i = 0; i < 3; i++){
        xString[i] = receiveBuffer[receiveIndex + i];
    }

    for (int i = 0; i < 3; i++){
        yString[i] = receiveBuffer[receiveIndex + i + 3];
    }

    yString[3] = '\0';
    xString[3] = '\0';
    returnArray[0] = atoi(xString);
    returnArray[1] = atoi(yString);

    serialWrite("input x:", 8);
    serialWrite(xString, 3);
    serialWrite(" y:", 3);
    serialWrite(yString, 3);
    serialWrite(" received\n", 10);

    receiveIndex = newReceiveIndex;

    return returnArray;
}
