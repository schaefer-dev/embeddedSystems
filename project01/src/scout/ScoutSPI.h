//
// Created by rafael on 06.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_SPIMASTER_H
#define EMBEDDEDSYSTEMS18_SPIMASTER_H

#include <stdint.h>

#define SLAVE_NONE 0
#define SLAVE_ADC 1
#define SLAVE_RF 2

#define SCK_VALUE (PORTB & (1<<PB4))

class ScoutSPI
{

private:
    static void waitNextRisingEdge();

    static void slaveSelect(unsigned char slave);

    static unsigned char transmitByte(unsigned char data);

    static unsigned char *transmitData(unsigned char *data, int size);


public:

    ScoutSPI();

    static void SPIMasterInit();

    static int readADC();

    static void setTimer1Interrupt(uint16_t factor);

    static void waitNextFallingEdge();

    static unsigned int interruptCounter;
};


#endif
