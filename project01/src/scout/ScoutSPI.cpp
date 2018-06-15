//
// Created by rafael on 06.06.18.
// part rewrite from OrangutanSPIMaster.h
//

#include "ScoutSPI.h"
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
#define PIN_SS_RF 4
#define PIN_SS_ADC 5
#define PIN_MOSI  PB5
#define PIN_MISO  PB0
#define PIN_SPI_SCK PB4
#define PIN_RF_ENABLE 7
#define PIN_ADC_SCK PB1


/* SPI_INTERRUPT_SPEED has to be 1 with the current setup */
#define SPI_INTERRUPT_SPEED 1

/* SPI_CLOCK_FACTOR defines every how many interrupts the SPI clock is inverted
 * (ADC system clock is inverted every interrupt) */
#define SPI_CLOCK_FACTOR 2

unsigned int ScoutSPI::interruptCounter = 0;


// not used because static
ScoutSPI::ScoutSPI() {
}

void ScoutSPI::SPIMasterInit() {

    ScoutSPI::interruptCounter = 0;

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

    // Set MOSI and SCK for SPI as ouput, additionally also the SystemClock of the ADC (PB1)
    DDRB |= (1 << PIN_MOSI) | (1 << PIN_SPI_SCK) | (1 << PIN_ADC_SCK);


    // Set Slave select as output
    DDRC |= (1 << PIN_SS_ADC);
    DDRD |= (1 << PIN_SS_RF);


    // drive PD4 high, PD7 low to deselect RF
    PORTD |= (1 << PIN_SS_RF);
    PORTD &= ~(1 << PIN_RF_ENABLE);



    setTimer1Interrupt(SPI_INTERRUPT_SPEED);

    delay(1);
}


/* returns whenever next rising edge occurs, used for synchronization */
void ScoutSPI::waitNextRisingEdge() {

    volatile int counter;

    while (SCK_VALUE > 0) {
        counter++;
    }

    while (SCK_VALUE == 0) {
        counter++;
    }
}

/* returns whenever next falling edge occurs, used for synchronization */
void ScoutSPI::waitNextFallingEdge() {

    volatile int counter;

    while ( SCK_VALUE == 0) {
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
void ScoutSPI::slaveSelect(unsigned char slave) {

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
unsigned char ScoutSPI::transmitByte(unsigned char data) {
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
unsigned char *ScoutSPI::transmitData(unsigned char *data, int size) {
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
 * Timer1 is reset every duration, and prescaled to interrupt every duration.
 * 
 * @param duration
 */
void ScoutSPI::setTimer1Interrupt(uint16_t factor) {

    OCR1A = factor;

    /* simple interrupt without PWM */

    // ctc on OCR1A (mode 4)
    TCCR1B |= (1 << WGM12);

    // set ctc interrupt
    TIMSK1 |= (1 << OCIE1A);

    // prescaler 1024
    TCCR1B |= (1 << CS12) | (1 << CS10);



    /* PWM TRY (not helping)

    // ctc on OCR1A (mode 4)
    TCCR1B |= (1 << WGM13);

    // PWM on pin PB1
    TCCR1A |= (1 << COM1A1) | (1 << COM1A0);

    // set ctc interrupt
    TIMSK1 |= (1 << OCIE1A);

    // prescaler 1024
    TCCR1B |= (1 << CS12) | (1 << CS10);
     */

    // enable global interrupts
    sei();
}


int ScoutSPI::readADC() {
    int output = 0;
    int buf = 0;

    waitNextFallingEdge();

    // drive SS/CS low
    slaveSelect(SLAVE_ADC);
    ScoutSerial::serialWrite("S\n", 2);

    // wait for 2 rising , 1 falling edge
    waitNextRisingEdge();
    //waitNextRisingEdge();
    waitNextFallingEdge();

    // set byte to be sent over MOSI to 1
    PORTB |= (1 << PB5);

    // first bit 1 is being read
    waitNextRisingEdge();

    // set byte to be sent over MOSI to 0
    PORTB &= ~(1 << PB5);

    // second bit 0 is being read
    waitNextRisingEdge();

    // set byte to be sent over MOSI to 1
    PORTB |= (1 << PB5);

    // third bit 1 is being read
    waitNextRisingEdge();

    // fourth bit 1 is being read
    waitNextRisingEdge();

    // set byte to be sent over MOSI to 0
    PORTB &= ~(1 << PB5);


    /* at this point adc should have received adress 1011 */

    for (int k = 0; k < 4; k++)
        waitNextRisingEdge();

    waitNextFallingEdge();

    // drive SS/CS high
    slaveSelect(SLAVE_NONE);
    ScoutSerial::serialWrite("D\n", 2);

    // wait for conversion (at least 36 cycles of ADC_SystemClock)
    for (int i = 0; i < 20; i++)
        waitNextRisingEdge();

    waitNextFallingEdge();

    // drive SS/CS low
    slaveSelect(SLAVE_ADC);
    ScoutSerial::serialWrite("S\n", 2);

    // wait for 2 rising , 1 falling edge
    waitNextRisingEdge();
    //waitNextRisingEdge();
    waitNextFallingEdge();

    // receive conversion data on MISO
    for (int j = 0; j < 8; j++){

        waitNextFallingEdge();

        buf = (PORTB & (1 << PIN_MISO));
        output += pow(2, 7-j) * buf;

    }

    waitNextFallingEdge();

    // drive SS/CS high
    slaveSelect(SLAVE_NONE);
    ScoutSerial::serialWrite("D\n", 2);

    return output;
}



// ISR for timer1
ISR (TIMER1_COMPA_vect) {

    ScoutSPI::interruptCounter = (ScoutSPI::interruptCounter + 1) % SPI_CLOCK_FACTOR;


    /* switch SPI sck only every ADC_SCK_SPEED_FACTOR times this interrupt is triggered */
    if (ScoutSPI::interruptCounter == 0) {
        if (SCK_VALUE > 0) {
            //PORTB &= (~(1 << PIN_SPI_SCK) & ~(1 << PIN_ADC_SCK));
            PORTB &= (~(1 << PIN_SPI_SCK));
            //ScoutSerial::serialWrite("L",1);
        } else {
            //PORTB |= (1 << PIN_SPI_SCK) | (1 << PIN_ADC_SCK);
            PORTB |= (1 << PIN_SPI_SCK);
            //ScoutSerial::serialWrite("H",1);
        }


        /* Debug, check if MOSI line currently sends anything */
        if ((PORTB & (1<<PIN_MOSI)) > 0){
            //ScoutSerial::serialWrite("1\n", 2);
        } else {
            //ScoutSerial::serialWrite("0\n", 2);
        }
    }


    /* switch adc system clock every time the interrupt is triggered */
    if ((PORTB & (1 << PIN_ADC_SCK)) > 0){
        PORTB &= (~(1 << PIN_ADC_SCK));
    } else {
        PORTB |= (1 << PIN_ADC_SCK);
    }


    /* Debug, check if MISO lane receives anything */
    if ((PORTB & (1<< PIN_MISO)) > 0){
        ScoutSerial::serialWrite("!!!\n", 4);
    }

}