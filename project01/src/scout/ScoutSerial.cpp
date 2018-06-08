//
// Created by Daniel Schäfer on 08.06.18.
//
#include "ScoutSerial.h"
#include <OrangutanSerial.h>
#include <stdio.h>
#include <stdlib.h>

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

    OrangutanSerial::send(buffer, size);
}

int* ScoutSerial::readCoordinates(){

    int newReceiveIndex = OrangutanSerial::getReceivedBytes();
    if (receiveIndex == newReceiveIndex)
        return nullptr;

    int returnArray[2];

    char xString[4];
    char yString[4];

    int iterator = receiveIndex;
    while (receiveBuffer[iterator] != ','){
        xString[iterator] = receiveBuffer[iterator];
        iterator += 1;
    }

    iterator += 1;
    int i = 0;
    while (iterator < newReceiveIndex){
        yString[i] = receiveBuffer[iterator];
        iterator += 1;
        i += 1;
    }

    returnArray[0] = atoi(xString);
    returnArray[1] = atoi(yString);

    receiveIndex = newReceiveIndex;

    return returnArray;
}
