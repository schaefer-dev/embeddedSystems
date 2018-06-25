//
// Created by Daniel Schäfer on 08.06.18.
//
#include "ScoutSerial.h"
#include <OrangutanSerial.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <OrangutanTime.h>


unsigned char ScoutSerial::receiveIndex = 0;
char ScoutSerial::receiveBuffer[200];

// not used because static
ScoutSerial::ScoutSerial() {
}


void ScoutSerial::initScoutSerial() {
    for (int i = 0; i < 199; i++){
        receiveBuffer[i] = 0;
    }
    OrangutanSerial::receive(receiveBuffer, 200);
    ScoutSerial::receiveIndex = 0;
}


void ScoutSerial::serialRead(char *buffer, unsigned char size) {

    OrangutanSerial::receive(buffer, size);
}

void ScoutSerial::serialWrite(char *buffer, unsigned char size) {

    OrangutanSerial::sendBlocking(buffer, size);
}

void ScoutSerial::serialWriteInt(int input) {
    char toBePrinted[6];
    toBePrinted[5] = '\n';

    for (int i = 4; i >= 0; i--){
        int modValue = input % 10;

        toBePrinted[i] = (char)(modValue + 48);
        input = input / 10;
    }
    serialWrite(toBePrinted, 6);
}

void ScoutSerial::serialWrite8Bit(int input) {
    char toBePrinted[3];

    for (int i = 2; i >= 0; i--){
        int modValue = input % 10;

        toBePrinted[i] = (char)(modValue + 48);
        input = input / 10;
    }
    serialWrite(toBePrinted, 3);
}

void ScoutSerial::serialWrite8BitHex(int input) {
    char toBePrinted[2];

    for (int i = 1; i >= 0; i--){
        int modValue = input % 16;

        /* print hex chars if >= 10 */
        if (modValue < 10)
            toBePrinted[i] = (char)(modValue + 48);
        else
            toBePrinted[i] = (char)(modValue + 55);


        input = input / 16;
    }
    serialWrite(toBePrinted, 2);
}

void ScoutSerial::serialWrite8BitBinary(int input) {
    char toBePrinted[8];

    for (int i = 7; i >= 0; i--){
        int modValue = input % 2;

        if (modValue > 0)
            toBePrinted[i] = (char)(49);
        else
            toBePrinted[i] = (char)(48);


        input = input / 2;
    }
    serialWrite(toBePrinted, 8);
}

/* returns number of bytes read and fills into returnArray, writes at most 31 bytes */
unsigned int ScoutSerial::readMessageFromSerial(char *returnArray){
    delay(5);
    int newReceiveIndex = OrangutanSerial::getReceivedBytes();
    if (receiveIndex == newReceiveIndex)
        return 0;

    int iterator = receiveIndex;
    int byteCount = 0;

    while(true){
        returnArray[byteCount] = (char)receiveBuffer[iterator];
        if (iterator == newReceiveIndex | byteCount == 30)
            break;
        iterator += 1;
        byteCount += 1;
    }

    receiveIndex = newReceiveIndex;

    return (byteCount);
}

bool ScoutSerial::readCoordinates(int *returnArray){

    delay(50);
    int newReceiveIndex = OrangutanSerial::getReceivedBytes();
    if (receiveIndex == newReceiveIndex)
        return false;

    char xString[4];
    char yString[4];
    for (int i = 0; i < 4; i++){
        yString[i] = '\0';
        xString[i] = '\0';
    }

    int iterator = receiveIndex;

    int i = 0;
    while (receiveBuffer[iterator] >= 48 && receiveBuffer[iterator] <= 57 && i < 4){
        xString[i] = receiveBuffer[iterator];
        iterator += 1;
        i += 1;
    }

    /* skip non number character, like commata or space etc */
    iterator += 1;

    i = 0;
    while (receiveBuffer[iterator] >= 48 && receiveBuffer[iterator] <= 57 && iterator < newReceiveIndex && i < 4){
        yString[i] = receiveBuffer[iterator];
        iterator += 1;
        i += 1;
    }


    /* Debug stuff: */
    serialWrite("input x:", 8);
    serialWrite(xString, 3);
    serialWrite(" y:", 3);
    serialWrite(yString, 3);
    serialWrite(" received\n", 10);


    returnArray[0] = atoi(xString);
    returnArray[1] = atoi(yString);

    receiveIndex = newReceiveIndex;

    return true;
}
