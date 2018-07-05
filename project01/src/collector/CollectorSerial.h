//
// Created by Daniel Schäfer on 05.07.18.
//

#ifndef EMBEDDEDSYSTEMS18_COLLECTORSERIAL_H
#define EMBEDDEDSYSTEMS18_COLLECTORSERIAL_H


#define TIMEOUT_BLOCKING_READING 10000

class CollectorSerial {

public:

    static void initCollectorSerial();
    static void serialRead(char *buffer, unsigned char size);
    static void serialWrite(char *buffer, unsigned char size);

    static void serialWriteInt(unsigned int input);
    static void serialWrite8Bit(int input);
    static void serialWrite8BitBinary(int input);
    static void serialWrite8BitHex(int input);


    static unsigned int readMessageFromSerial(char *returnArray);
    static bool simulatorMessageIncoming();
    static void receiveSerialBlocking(char *returnArray);

};


#endif //EMBEDDEDSYSTEMS18_COLLECTORSERIAL_H
