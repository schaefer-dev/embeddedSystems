//
// Created by rafael on 06.06.18.
//

#include "SPIMaster.h"
#include "avr/io.h"
#include "avr/interrupt.h"

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
void SPIMasterInit(unsigned char speed_divider, unsigned char options){

    // Set MISO pin as input
    DDRB &= ~( 1 << PIN_MISO );


    // Set MOSI and SCK as ouput
    DDRB |= ( 1 << PIN_MOSI );
    DDRB |= ( 1 << PIN_SCK );


    // drive PD4 high, PD7 low to deselect RF
    PORTD |= ( 1 << PIN_SS_RF);
    PORTD &= ~( 1 << PIN_RF_ENABLE);


    // Initiate SPI module
            SPCR = (1 << SPE) | (1 << MSTR) | (options & ~3) | (speed_divider & 3);
            SPSR = (speed_divider & 4) ? 1 : 0;

}


/**
 * select or deselect current slave node on Scout
 * @param slave DESELECT (0), SELECT_ADC (1) or SELECT_RF (2)
 */
void slaveSelect(unsigned char slave){

    // check for selection
    if (slave < 1){
        PORTD |= ( 1 << PIN_SS_RF);
        PORTC |= ( 1 << PIN_SS_ADC);
    }
    if (slave > 1){
        // deselect ADC, select RF
        PORTC |= ( 1 << PIN_SS_ADC);
        // PORTD &= ~( 1 << PIN_SS_RF);
    }
    else {
        //deselect RF, select ADC
        PORTD |= ( 1 << PIN_SS_RF);
        PORTC &= ~( 1 << PIN_SS_ADC);
    }

}

/** send one byte of data to slave node
 *
 * @param data byte to send
 * @return data sent by slave device
 */
unsigned char transmitByte(unsigned char data){

    // enable SPI in default mode
    if ( !(SPCR & (1<<SPE)) )
    {
        SPIMasterInit(SPI_DEFAULT_SPEED_DIVIDER, SPI_DEFAULT_OPTIONS);
    }

    // if the SPI module is not in master mode
    if (!(SPCR&(1<<MSTR)))
    {
        SPCR |= 1<<MSTR;
    }

    // begin transmission
    SPDR = data;

    // wait while transmission is processing
    while(!(SPSR & (1<<SPIF)))
    {
        if (!(SPCR & (1<<MSTR)))
        {
            // The SPI module has left master mode, so return.
            // Otherwise, this will be an infinite loop.
            return 0;
        }
    }

    // return transmission from slave node
    return SPDR;
}


/** set timer1 to interrupt after duration in ms
 *
 * @param duration
 */
void setTimer(int duration){
    OCR1A = 0x3D08;

    TCCR1B |= (1 << WGM12);
    // Mode 4, CTC on OCR1A

    TIMSK1 |= (1 << OCIE1A);
    //Set interrupt on compare match

    TCCR1B |= (1 << CS12) | (1 << CS10);
    // set prescalar to 1024 and start the timer


    sei();
    // enable interrupts
}