//
// Created by rafael on 06.06.18.
// part rewrite from OrangutanSPIMaster.h
//

#include "SPIMaster.h"
#include "avr/io.h"
#include <OrangutanSerial.h>
#include <OrangutanTime.h>
#include "ScoutSerial.h"
#include "main.h"
#include "avr/interrupt.h"
#include "avr/io.h"
#include "math.h"


// Fix incorrect SPI register and bit names used by some
// versions of avr-libc
#if defined(SPCR0) && !defined(SPCR)
#define SPCR SPCR0
#define CPHA CPHA0
#define MSTR MSTR0
#define SPE  SPE0
#define SPSR SPSR0
#define SPIF SPIF0
#define SPDR SPDR0
#endif


// define Pins
#define PIN_SS_RF 4
#define PIN_SS_ADC 5
#define PIN_MOSI  5
#define PIN_MISO  0
#define PIN_SCK 4
#define PIN_RF_ENABLE 7

// By default, pick slowest possible speed.
#define SPI_DEFAULT_SPEED_DIVIDER SPI_SPEED_DIVIDER_128  // 20MHz / 128 = 156 kHz
#define SPI_DEFAULT_OPTIONS SPI_EDGE_LEADING


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


SPIMaster::SPIMaster() {
}

void SPIMaster::SPIMasterInit() {

    //Make sure slave select are output and pulled up
    if (!(DDRD & (1 << PIN_SS_RF)) && !(PORTD & (1 << PIN_SS_RF))) {
        PORTD |= 1 << PIN_SS_RF;

        // Delay a while to give the pull-up time
        delayMicroseconds(30);
    }

    if (!(DDRC & (1 << PIN_SS_ADC)) && !(PORTC & (1 << PIN_SS_ADC))) {
        PORTC |= 1 << PIN_SS_ADC;

        // Delay a while to give the pull-up time
        delayMicroseconds(30);
    }


    // Set MISO pin as input
    DDRB &= ~(1 << PIN_MISO);


    // Set MOSI and SCK as ouput
    DDRB |= (1 << PIN_MOSI) | (1 << PIN_SCK) | (1 << PB1);

    // Set Slave select as output
    DDRC |= (1 << PIN_SS_ADC);
    DDRD |= (1 << PIN_SS_RF);


    // drive PD4 high, PD7 low to deselect RF
    PORTD |= (1 << PIN_SS_RF);
    PORTD &= ~(1 << PIN_RF_ENABLE);


    // set timer for SCK frequency
    // TODO: this parameter might need optimization (maybe even change prescaler used in method)
    setTimer1Interrupt(30);


    delay(1);

}


/* returns whenever next rising edge occurs, used for synchronization */
void SPIMaster::waitNextRisingEdge() {

    volatile int counter;

    while (SCK_VALUE > 0) {
        counter++;
    }

    while (SCK_VALUE == 0) {
        counter++;
    }
}

/* returns whenever next falling edge occurs, used for synchronization */
void SPIMaster::waitNextFallingEdge() {

    volatile int counter;

    while (SCK_VALUE == 0) {
        counter++;
    }

    while (SCK_VALUE > 0) {
        counter++;
    }
}


/**
 *
 * IMPORTANT: Select slaves only when clock currently high, such that rising edge after slave select can be detected
 * with 100% probability.
 *
 * select or deselect current slave node on Scout
 * @param slave DESELECT (0), SELECT_ADC (1) or SELECT_RF (2)
 */
void SPIMaster::slaveSelect(unsigned char slave) {

    // check for selection
    if (slave < 1) {
        PORTD |= (1 << PIN_SS_RF);
        PORTC |= (1 << PIN_SS_ADC);
    }
    if (slave > 1) {
        // deselect ADC, select RF
        PORTC |= (1 << PIN_SS_ADC);
        // PORTD &= ~( 1 << PIN_SS_RF);
    } else {
        //deselect RF, select ADC
        PORTD |= (1 << PIN_SS_RF);
        PORTC &= ~(1 << PIN_SS_ADC);
    }
}

/** send one byte of data to slave node
 *
 * @param data byte to send
 * @return data sent by slave device
 */
unsigned char SPIMaster::transmitByte(unsigned char data) {
    // TODO rewrite method entirely

    // begin transmission
    SPDR = data;

    // wait while transmission is processing
    while (!(SPSR & (1 << SPIF))) {
        if (!(SPCR & (1 << MSTR))) {
            // The SPI module has left master mode, so return.
            // Otherwise, this will be an infinite loop.
            return 0;
        }
    }

    // return transmission from slave node
    return SPDR;
}


/** transmit data bytewise
 *
 * @param data
 * @return
 */
unsigned char *SPIMaster::transmitData(unsigned char *data, int size) {
    // TODO rewrite method entirely

    if (size <= 0) {
        int i = 0;
        for (i = 0; i < size; i++) {
            data[i] = 0;
        }
        return data;
    }

    int sizeM = size;
    char dataByte = 0;

    while (size > 0) {
        dataByte = data[sizeM - size];
        data[sizeM - size] = transmitByte(dataByte);

    }

    return data;
}


/* 
 * Timer1 is reset every duration, and prescaled to interrupt every duration
 * 
 * @param duration
 */
void SPIMaster::setTimer1Interrupt(uint16_t factor) {

    OCR1A = factor;

    // ctc on OCR1A (mode 4)
    TCCR1B |= (1 << WGM12);

    // set ctc interrupt
    TIMSK1 |= (1 << OCIE1A);

    // prescaler 1024
    TCCR1B |= (1 << CS12) | (1 << CS10);

    // enable global interrupts
    sei();
}


int SPIMaster::readADC() {
    int output = 0;
    int buf = 0;

    waitNextFallingEdge();

    // drive SS/CS low
    slaveSelect(SLAVE_ADC);

    // wait for 2 rising , 1 falling edge
    waitNextRisingEdge();

    waitNextRisingEdge();

    waitNextFallingEdge();

    // set byte to be sent over MOSI to 0
    PORTB &= ~(1 << PB5);

    // first bit 0 is being read
    waitNextRisingEdge();

    // second bit 0 is being read
    waitNextRisingEdge();

    // set byte to be sent over MOSI to 1
    PORTB |= (1 << PB5);

    // third bit 1 is being read
    waitNextRisingEdge();

    // set byte to be sent over MOSI to 0
    PORTB &= ~(1 << PB5);

    // fourth bit 0 is being read
    waitNextRisingEdge();

    /* at this point adc should have received adress 0010 */

    for (int k = 0; k < 4; k++)
        waitNextRisingEdge();

    waitNextFallingEdge();

    // drive SS/CS high
    slaveSelect(SLAVE_NONE);

    // wait for conversion (at least 36 cycles)
    for (int i = 0; i < 40; i++)
        waitNextRisingEdge();

    waitNextFallingEdge();

    // drive SS/CS low
    slaveSelect(SLAVE_ADC);

    // wait for 2 rising , 1 falling edge
    waitNextRisingEdge();

    waitNextRisingEdge();

    waitNextFallingEdge();

    // receive conversion data on MISO
    for (int j = 0; j < 8; j++){

        waitNextRisingEdge();

        buf = (PORTB & (1<<PB0));
        output += pow(2, 7-j) * buf;

    }

    waitNextFallingEdge();

    // drive SS/CS high
    slaveSelect(SLAVE_NONE);

    return output;
}



// ISR for timer1
ISR (TIMER1_COMPA_vect) {
    if (SCK_VALUE > 0) {
        PORTB &= (~(1 << PB4) & ~(1 << PB1));
    } else {
        PORTB |= (1 << PB4) | (1 << PB1);
    }

    /* Debug, check if MISO lane receives anything */
    if ((PORTB & (1<<PB0)) > 0){
        OrangutanSerial::sendBlocking("high!\n", 6);
    }



}