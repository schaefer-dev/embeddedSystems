//
// Created by Daniel Schäfer on 08.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_SCOUTSERIAL_H
#define EMBEDDEDSYSTEMS18_SCOUTSERIAL_H


#include "../utils/Coordinates.h"

class ScoutSerial{

public:
    ScoutSerial();

    static void initScoutSerial();
    static void serialRead(char *buffer, unsigned char size);
    static void serialWrite(char *buffer, unsigned char size);
    static bool readCoordinates(int *);

    static void serialWriteInt(int input);
    static void serialWrite8Bit(int input);
    static void serialWrite8BitBinary(int input);
    static void serialWrite8BitHex(int input);


    static unsigned int readMessageFromSerial(char *returnArray);

    static char receiveBuffer[100];
    static unsigned char receiveIndex;
};

#endif //EMBEDDEDSYSTEMS18_SCOUTSERIAL_H
