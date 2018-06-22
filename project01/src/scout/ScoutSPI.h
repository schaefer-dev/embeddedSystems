//
// Created by rafael on 06.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_SPIMASTER_H
#define EMBEDDEDSYSTEMS18_SPIMASTER_H

#include <stdint.h>

/**
 * initialize Master node for SPI communication on Scout robot
 * PB0 -- MISO
 * PB1 -- ADC SCK
 * PB4 -- RF SCK / ADC I/O CLK
 * PB5 -- MOSI
 * PC5 -- ADC select
 * PD2 -- RF IRQ
 * PD4 -- RF select
 * PD7 -- RF enable
 */


// define Pins
#define PIN_SS_RF_D PD4
#define PIN_SS_ADC_C PC5
#define PIN_MOSI_B  PB5
#define PIN_MISO_B  PB0
#define PIN_SPI_SCK_B PB4
#define PIN_RF_ENABLE_D PD7
#define PIN_RF_IRQ_D PD2
#define PIN_ADC_SCK_B PB1


/* SPI_INTERRUPT_SPEED has to be 1 with the current setup */
#define SPI_INTERRUPT_SPEED 1

/* SPI_CLOCK_FACTOR defines every how many interrupts the SPI clock is inverted */
#define SPI_CLOCK_FACTOR 2

/* delay between each byte of communication with RF module in microseconds */
#define command_delay 50
#define delay_after_RF_select 1


// slave select ids
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

public:


    static unsigned int interruptCounter;
    static volatile unsigned char SPIClock;
    static volatile unsigned char ADCClock;
    static bool runSPIClock;



    ScoutSPI();

    static void SPIMasterInit();
    static void setTimer1Interrupt(uint16_t factor);

    static void slaveSelect(unsigned char slave);
    static int readADC(char sensorAdress);
    static void ADCConversionWait();
    static unsigned int readWriteSPI(unsigned int payload);

};


#endif
