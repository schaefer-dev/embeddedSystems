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

class SPIMaster
{

private:
    void waitNextRisingEdge();

    void slaveSelect(unsigned char slave);

    unsigned char transmitByte(unsigned char data);

    unsigned char *transmitData(unsigned char *data, int size);


public:

    SPIMaster();

    void SPIMasterInit();

    int readADC();

    void setTimer1Interrupt(uint16_t factor);

};


#endif
