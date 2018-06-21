//
// Created by Daniel Schäfer on 20.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_COLLECTORSPI_H
#define EMBEDDEDSYSTEMS18_COLLECTORSPI_H

#include <stdint.h>

#define SLAVE_NONE 0
#define SLAVE_RF 1


class CollectorSPI {

private:
    static void waitNextSPIRisingEdge();
    static void waitNextSPIFallingEdge();
    static unsigned int readWriteSPI(unsigned int payload);


public:

    CollectorSPI();

    static void SPIMasterInit();

    static void slaveSelect(unsigned char slave);

    static void initializeRFModule();

    static void setTimer4Interrupt(uint16_t duration);

    static volatile unsigned char SPIClock;

    static bool runSPIClock;

    static int queryRFModule();
    static void debug_RFModule();

};


#endif //EMBEDDEDSYSTEMS18_COLLECTORSPI_H
