//
// Created by Daniel Schäfer on 08.06.18.
//
#include "ScoutSerial.h"
#include <OrangutanSerial.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <OrangutanTime.h>
#include "main.h"


unsigned char ScoutSerial::receiveIndex = 0;
char ScoutSerial::receiveBuffer[SERIAL_BUFFER_SIZE];

// not used because static
ScoutSerial::ScoutSerial() {
}


void ScoutSerial::initScoutSerial() {
    for (int i = 0; i < 199; i++){
        receiveBuffer[i] = 0;
    }
    OrangutanSerial::receive(receiveBuffer, SERIAL_BUFFER_SIZE);
    ScoutSerial::receiveIndex = 0;
}


void ScoutSerial::serialRead(char *buffer, unsigned char size) {

    OrangutanSerial::receive(buffer, size);
}

void ScoutSerial::serialWrite(char *buffer, unsigned char size) {

    OrangutanSerial::sendBlocking(buffer, size);
}

void ScoutSerial::serialWriteInt(unsigned int input) {
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



#ifdef ROBOT_SIMULATOR
/* checks if the synchronizing prefix 0x7A (z) has been send over serial, if yes return true */
bool ScoutSerial::simulatorMessageIncoming(){
    delay(5);
    int newReceiveIndex = OrangutanSerial::getReceivedBytes();
    if (receiveIndex == newReceiveIndex)
        return false;

    int iterator = receiveIndex;

    if ('z' == (char)receiveBuffer[iterator]) {
        initScoutSerial();
        return true;
    }

    initScoutSerial();
    return false;
}
#endif



#ifdef ROBOT_SIMULATOR
/* reads exactly 32 bytes blocking into array */
void ScoutSerial::receiveSerialBlocking(char *returnArray){
    OrangutanSerial::receiveBlocking(returnArray, 32, TIMEOUT_BLOCKING_READING);
    return;
}
#endif




/* returns number of bytes read and fills into returnArray, writes at most 50 bytes */
unsigned int ScoutSerial::readMessageFromSerial(char *returnArray){
    delay(5);
    int newReceiveIndex = OrangutanSerial::getReceivedBytes();
    if (receiveIndex == newReceiveIndex)
        return 0;

    int iterator = receiveIndex;
    int byteCount = 0;

    while(true){
        returnArray[byteCount] = (char)receiveBuffer[iterator];
        if (iterator == newReceiveIndex | byteCount == 49)
            break;
        iterator += 1;
        byteCount += 1;
    }

    receiveIndex = newReceiveIndex;

    initScoutSerial();

    return (byteCount);
}