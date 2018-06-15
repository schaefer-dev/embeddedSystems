//
// Created by Daniel Schäfer on 08.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_SCOUTSERIAL_H
#define EMBEDDEDSYSTEMS18_SCOUTSERIAL_H


#include "../collector/Coordinates.h"

class ScoutSerial{

public:
    ScoutSerial();
    void serialRead(char *buffer, unsigned char size);
    void serialWrite(char *buffer, unsigned char size);
    bool readCoordinates(int *);

    char receiveBuffer[100];
    unsigned char receiveIndex;
};

#endif //EMBEDDEDSYSTEMS18_SCOUTSERIAL_H
