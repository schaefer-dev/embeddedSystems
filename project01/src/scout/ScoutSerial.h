//
// Created by Daniel Schäfer on 08.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_SCOUTSERIAL_H
#define EMBEDDEDSYSTEMS18_SCOUTSERIAL_H


#include "../collector/Coordinates.h"

class ScoutSerial{

public:
    ScoutSerial();

    static void initScoutSerial();
    static void serialRead(char *buffer, unsigned char size);
    static void serialWrite(char *buffer, unsigned char size);
    static bool readCoordinates(int *);

    static void serialWriteInt(int input);

    static char receiveBuffer[100];
    static unsigned char receiveIndex;
};

#endif //EMBEDDEDSYSTEMS18_SCOUTSERIAL_H
