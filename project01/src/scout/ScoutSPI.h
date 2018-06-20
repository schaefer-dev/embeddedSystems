//
// Created by rafael on 06.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_SPIMASTER_H
#define EMBEDDEDSYSTEMS18_SPIMASTER_H

#include <stdint.h>

#define SLAVE_NONE 0
#define SLAVE_ADC 1
#define SLAVE_RF 2

class ScoutSPI
{

private:
    static void waitNextSPIRisingEdge();
    static void waitNextSPIFallingEdge();
    static void waitNextADCRisingEdge();
    static void waitNextADCFallingEdge();
    static unsigned int readWriteSPI(unsigned int payload);


public:

    ScoutSPI();

    static void SPIMasterInit();

    static void slaveSelect(unsigned char slave);

    static int readADC(char sensorAdress);
    static void initializeRFModule();

    static void setTimer1Interrupt(uint16_t factor);

    static void ADCConversionWait();

    static unsigned int interruptCounter;

    static volatile unsigned char SPIClock;
    static volatile unsigned char ADCClock;

    static bool runSPIClock;


    static int queryRFModule();
    static void debug_RFModule();

};


#endif
