//
// Created by Daniel Schäfer on 20.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_COLLECTORSPI_H
#define EMBEDDEDSYSTEMS18_COLLECTORSPI_H

#include <stdint.h>

#define SLAVE_NONE 0
#define SLAVE_RF 1


/**
 * initialize Master node for SPI communication on Scout robot
 * PB0 - RF SCK
 * PD5 - MOSI
 * PD7 - MISO
 * PB3 - RF SS
 * PC7 - RF CE
 * PD2 - RF IRQ/DEV RX  depending on jumper position
 */


// define Pins
#define PIN_SS_RF_B PB3
#define PIN_MOSI_D  PD5
#define PIN_MISO_D  PD7
#define PIN_SPI_SCK_B PB0
#define PIN_RF_ENABLE_C PC7
#define PIN_RF_IRQ_D PD2


/* SPI_INTERRUPT_SPEED has to be 1 with the current setup */
#define SPI_INTERRUPT_SPEED 1

/* delay between each byte of communication with RF module in microseconds */
#define command_delay 50
#define delay_after_RF_select 1

//#define SPI_CLOCK_VALUE (PINB & (1 << PIN_SPI_SCK_B))


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

    static unsigned volatile char SPIClock;

    static bool volatile runSPIClock;

    static int queryRFModule();
    static void debug_RFModule();

    static unsigned int debug_interruptCounter;

};


#endif //EMBEDDEDSYSTEMS18_COLLECTORSPI_H
