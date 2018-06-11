//
// Created by rafael on 06.06.18.
// part rewrite from OrangutanSPIMaster.h
//

#include "SPIMaster.h"
#include "avr/io.h"
#include "avr/interrupt.h"
#include <OrangutanSerial.h>
#include <OrangutanTime.h>
#include "ScoutSerial.h"


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

void SPIMaster::SPIMasterInit(unsigned char speed_divider, unsigned char options){

    //Make sure slave select are output and pulled up
    if ( !(DDRD & (1<<PIN_SS_RF)) && !(PORTD & (1<<PIN_SS_RF)) )
    {
        PORTD |= 1<<PIN_SS_RF;

        // Delay a while to give the pull-up time
        delayMicroseconds(30);
    }

    if ( !(DDRC & (1<<PIN_SS_ADC)) && !(PORTC & (1<<PIN_SS_ADC)) )
    {
        PORTD |= 1<<PIN_SS_ADC;

        // Delay a while to give the pull-up time
        delayMicroseconds(30);
    }



    // Set MISO pin as input
    DDRB &= ~( 1 << PIN_MISO );


    // Set MOSI and SCK as ouput
    DDRB |= ( 1 << PIN_MOSI ) | ( 1 << PIN_SCK );

    // Set Slave select as output
    DDRC |= ( 1 << PIN_SS_ADC);
    DDRD |= ( 1 << PIN_SS_RF);


    // drive PD4 high, PD7 low to deselect RF
    PORTD |= ( 1 << PIN_SS_RF);
    PORTD &= ~( 1 << PIN_RF_ENABLE);


    // Initiate SPI module
    SPCR = (1 << SPE) | (1 << MSTR) | (options & ~3) | (speed_divider & 3);
    SPSR = (speed_divider & 4) ? 1 : 0;

    delay(1);

}


/**
 * select or deselect current slave node on Scout
 * @param slave DESELECT (0), SELECT_ADC (1) or SELECT_RF (2)
 */
void SPIMaster::slaveSelect(unsigned char slave){

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
    delayMicroseconds(30);

}

/** send one byte of data to slave node
 *
 * @param data byte to send
 * @return data sent by slave device
 */
unsigned char SPIMaster::transmitByte(unsigned char data){

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


/** transmit data bytewise
 *
 * @param data
 * @return
 */
unsigned char* SPIMaster::transmitData(unsigned char* data, int size){

    if (size <= 0){
        int i=0;
        for (i = 0; i < size; i++){
            data[i] = 0;
        }
        return data;
    }

    int sizeM = size;
    char dataByte = 0;

    while (size > 0){
        dataByte = data[sizeM-size];
        data[sizeM-size] = transmitByte(dataByte);

    }

    return data;
}


/** set timer1 to interrupt after duration in ms
 *
 * @param duration
 */
void SPIMaster::setTimer(int duration){

    uint16_t timer = 0;

    // select prescaler
    if (duration < 32){
        // prescaler 8 suffices
        timer = (duration * 2000) -1 ;
        OCR1A = timer;

    } else {
        // prescaler 1024
        timer = (duration * 20) - 1;
        OCR1A = timer;
    }
    // ctc on OCR1A (mode 4)
    TCCR1B |= (1 << WGM12);

    // set ctc interrupt
    TIMSK1 |= (1 << OCIE1A);

    if (duration < 32){
        // prescaler 8 suffices
        TCCR1B |= (1 << CS11);
    } else {
        // prescaler 1024
        TCCR1B |= (1 << CS12) | (1 << CS10);
    }

    // enable global interrupts
    sei();

}

int SPIMaster::readADC() {
/*
    // select ADC
    slaveSelect(SELECT_ADC);

    // read dummy byte
    unsigned char address = 0x00f0;
    unsigned char data[16];
    data[0] = address;
    transmitData(data, 16);
    delay(1);

    // read ADC
    unsigned char reply[16];
    reply = transmitData(data, 16);*/
    return 0;
}



// ISR for timer1
ISR (TIMER1_COMPA_vect)
{
    serial_send_blocking("!!\n", 3);
}