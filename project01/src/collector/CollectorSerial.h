//
// Created by Daniel Schäfer on 05.07.18.
//

#ifndef EMBEDDEDSYSTEMS18_COLLECTORSERIAL_H
#define EMBEDDEDSYSTEMS18_COLLECTORSERIAL_H


class CollectorSerial {

public:

    static unsigned int readMessageFromSerial(char *returnArray);
    static bool simulatorMessageIncoming();
    static void receiveSerialBlocking(char *returnArray);

};


#endif //EMBEDDEDSYSTEMS18_COLLECTORSERIAL_H
