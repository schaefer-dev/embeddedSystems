//
// Created by Daniel Schäfer on 08.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_SCOUTSERIAL_H
#define EMBEDDEDSYSTEMS18_SCOUTSERIAL_H


class ScoutSerial{

public:
    ScoutSerial();
    void serialRead(char *buffer, unsigned char size);
    void serialWrite(char *buffer, unsigned char size);
    int* readCoordinates();

    char receiveBuffer[100];
    unsigned char receiveIndex;
};

#endif //EMBEDDEDSYSTEMS18_SCOUTSERIAL_H
