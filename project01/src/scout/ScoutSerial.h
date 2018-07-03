//
// Created by Daniel Schäfer on 08.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_SCOUTSERIAL_H
#define EMBEDDEDSYSTEMS18_SCOUTSERIAL_H


#include "../utils/Coordinates.h"

#define SERIAL_BUFFER_SIZE 50
#define TIMEOUT_BLOCKING_READING 10000

class ScoutSerial{

public:
    ScoutSerial();

    static void initScoutSerial();
    static void serialRead(char *buffer, unsigned char size);
    static void serialWrite(char *buffer, unsigned char size);

    static void serialWriteInt(unsigned int input);
    static void serialWrite8Bit(int input);
    static void serialWrite8BitBinary(int input);
    static void serialWrite8BitHex(int input);


    static unsigned int readMessageFromSerial(char *returnArray);
    static bool simulatorMessageIncoming();
    static void receiveSerialBlocking(char *returnArray);


    static char receiveBuffer[SERIAL_BUFFER_SIZE];
    static unsigned char receiveIndex;
};

#endif //EMBEDDEDSYSTEMS18_SCOUTSERIAL_H
