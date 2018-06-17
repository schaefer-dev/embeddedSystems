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
#define PIN_SS_RF_D PD4
#define PIN_SS_ADC_C PC5
#define PIN_MOSI_B  PB5
#define PIN_MISO_B  PB0
#define PIN_SPI_SCK_B PB4
#define PIN_RF_ENABLE_D PD7
#define PIN_ADC_SCK_B PB1


/* SPI_INTERRUPT_SPEED has to be 1 with the current setup */
#define SPI_INTERRUPT_SPEED 1

/* SPI_CLOCK_FACTOR defines every how many interrupts the SPI clock is inverted
 * (ADC system clock is inverted every interrupt) */
#define SPI_CLOCK_FACTOR 2

unsigned int ScoutSPI::interruptCounter = 0;
unsigned volatile char ScoutSPI::SPIClock = 0;
unsigned volatile char ScoutSPI::ADCClock = 0;


// not used because static
ScoutSPI::ScoutSPI() {
}

void ScoutSPI::SPIMasterInit() {

    ScoutSPI::interruptCounter = 0;

    //Make sure slave select are output and pulled up
    if (!(DDRD & (1 << PIN_SS_RF_D)) && !(PORTD & (1 << PIN_SS_RF_D))) {
        PORTD |= 1 << PIN_SS_RF_D;

        // Delay a while to give the pull-up time
        delayMicroseconds(30);
    }

    if (!(DDRC & (1 << PIN_SS_ADC_C)) && !(PORTC & (1 << PIN_SS_ADC_C))) {
        PORTC |= 1 << PIN_SS_ADC_C;

        // Delay a while to give the pull-up time
        delayMicroseconds(30);
    }


    // Set MISO pin as input
    DDRB &= ~(1 << PIN_MISO_B);

    // Set MOSI and SCK for SPI as ouput, additionally also the SystemClock of the ADC (PB1)
    DDRB |= (1 << PIN_MOSI_B) | (1 << PIN_SPI_SCK_B) | (1 << PIN_ADC_SCK_B);


    /* drive SPICLock and ADCClock low initially */
    PORTB &= (~(1 << PIN_SPI_SCK_B) & ~(1 << PIN_ADC_SCK_B));
    ADCClock = 0;
    SPIClock = 0;



    // Set Slave select as output
    DDRC |= (1 << PIN_SS_ADC_C);
    DDRD |= (1 << PIN_SS_RF_D);


    // drive all SlaveSelect lanes high, PD7 low to deselect RF
    PORTD |= (1 << PIN_SS_RF_D);
    PORTC |= (1 << PIN_SS_ADC_C);
    PORTD &= ~(1 << PIN_RF_ENABLE_D);



    setTimer1Interrupt(SPI_INTERRUPT_SPEED);

    delay(1);
}


/* returns whenever next rising edge occurs, used for synchronization */
void ScoutSPI::waitNextSPIRisingEdge() {

    volatile int counter;

    while (SPIClock > 0) {
        counter++;
    }

    while (SPIClock == 0) {
        counter++;
    }
}

/* returns whenever next falling edge occurs, used for synchronization */
void ScoutSPI::waitNextSPIFallingEdge() {

    volatile int counter;

    while ( SPIClock == 0) {
        counter++;
    }

    while (SPIClock > 0) {
        counter++;
    }
}

/* returns whenever next rising edge occurs, used for synchronization */
void ScoutSPI::waitNextADCRisingEdge() {

    volatile int counter;

    while (ADCClock > 0) {
        counter++;
    }

    while (ADCClock == 0) {
        counter++;
    }
}

/* returns whenever next falling edge occurs, used for synchronization */
void ScoutSPI::waitNextADCFallingEdge() {

    volatile int counter;

    while ( ADCClock == 0) {
        counter++;
    }

    while (ADCClock > 0) {
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
        PORTD |= (1 << PIN_SS_RF_D);
        PORTC |= (1 << PIN_SS_ADC_C);
    }
    if (slave > 1) {
        // deselect ADC, select RF
        PORTC |= (1 << PIN_SS_ADC_C);
        // PORTD &= ~( 1 << PIN_SS_RF_D);
    } else {
        //deselect RF, select ADC
        PORTD |= (1 << PIN_SS_RF_D);
        PORTC &= ~(1 << PIN_SS_ADC_C);
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


    // prescaler 256
    //TCCR1B |= (1 << CS12);



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


int ScoutSPI::readADC(char sensorAdress) {
    int output = 0;
    int buf = 0;

    /* catch illegal sensorAdresses */
    if (sensorAdress > 11){
        ScoutSerial::serialWrite("W: sensorAdress > 11\n", 21);
        return 0;
    }

    /* already start sending the first bit of adress */
    char outputBit = sensorAdress / 8;
    if (outputBit > 0)
        PORTB |= (1 << PIN_MOSI_B);
    else
        PORTB &= ~(1 << PIN_MISO_B);


    waitNextSPIFallingEdge();

    // drive SS/CS low
    slaveSelect(SLAVE_ADC);
    ScoutSerial::serialWrite("S\n", 2);

    // wait for 2 rising , 1 falling edge of ADC clock
    waitNextADCRisingEdge();
    waitNextADCRisingEdge();
    waitNextADCFallingEdge();

    // wait such that first bit is being read by ADC
    waitNextSPIRisingEdge();

    /* read value on rising edge of SPI */
    if( (PORTB & (1 << PIN_MISO_B)) > 0)
        output += 128;

    /* on falling edge put next bit on MOSI */
    waitNextSPIFallingEdge();
    outputBit = (sensorAdress % 8) / 4;
    if (outputBit > 0)
        PORTB |= (1 << PIN_MOSI_B);
    else
        PORTB &= ~(1 << PIN_MISO_B);

    // second bit is being read by ADC
    waitNextSPIRisingEdge();

    /* read value on rising edge of SPI */
    if( (PORTB & (1 << PIN_MISO_B)) > 0)
        output += 64;

    /* on falling edge put next bit on MOSI */
    waitNextSPIFallingEdge();
    outputBit = (sensorAdress % 4) / 2;
    if (outputBit > 0)
        PORTB |= (1 << PIN_MOSI_B);
    else
        PORTB &= ~(1 << PIN_MISO_B);

    // third bit is being read by ADC
    waitNextSPIRisingEdge();

    /* read value on rising edge of SPI */
    if( (PORTB & (1 << PIN_MISO_B)) > 0)
        output += 32;

    /* on falling edge put next bit on MOSI */
    waitNextSPIFallingEdge();
    outputBit = (sensorAdress % 2) / 1;
    if (outputBit > 0)
        PORTB |= (1 << PIN_MOSI_B);
    else
        PORTB &= ~(1 << PIN_MISO_B);

    // fourth bit is being read by ADC
    waitNextSPIRisingEdge();

    /* read value on rising edge of SPI */
    if( (PORTB & (1 << PIN_MISO_B)) > 0)
        output += 16;

    // set byte to be sent over MOSI to 0 after adress sending has been completed
    PORTB &= ~(1 << PB5);


    waitNextSPIRisingEdge();

    /* read value on rising edge of SPI */
    if( (PORTB & (1 << PIN_MISO_B)) > 0)
        output += 8;

    waitNextSPIRisingEdge();

    /* read value on rising edge of SPI */
    if( (PORTB & (1 << PIN_MISO_B)) > 0)
        output += 4;

    waitNextSPIRisingEdge();

    /* read value on rising edge of SPI */
    if( (PORTB & (1 << PIN_MISO_B)) > 0)
        output += 2;

    waitNextSPIRisingEdge();

    /* read value on rising edge of SPI */
    if( (PORTB & (1 << PIN_MISO_B)) > 0)
        output += 1;

    waitNextSPIFallingEdge();

    // drive SS/CS high
    slaveSelect(SLAVE_NONE);
    ScoutSerial::serialWrite("D\n", 2);

    // wait for conversion (at least 36 cycles of ADC_SystemClock)
    for (int i = 0; i < 40; i++)
        waitNextADCRisingEdge();

    return output;

    /* no longer necessary as it can be called consecetive times now to output all sensors after each other
    waitNextSPIFallingEdge();

    // drive SS/CS low
    slaveSelect(SLAVE_ADC);
    ScoutSerial::serialWrite("S\n", 2);

    // wait for 2 rising , 1 falling edge of ADC clock
    waitNextADCRisingEdge();
    waitNextADCRisingEdge();
    waitNextADCFallingEdge();

    // receive conversion data on MISO
    for (int j = 0; j < 8; j++){

        waitNextSPIRisingEdge();

        buf = (PORTB & (1 << PIN_MISO_B));
        output += pow(2, 7-j) * buf;

    }

    waitNextSPIFallingEdge();

    // drive SS/CS high
    slaveSelect(SLAVE_NONE);
    ScoutSerial::serialWrite("D\n", 2);

    return output;
     */
}



// ISR for timer1
ISR (TIMER1_COMPA_vect) {

    ScoutSPI::interruptCounter = (ScoutSPI::interruptCounter + 1) % SPI_CLOCK_FACTOR;


    /* switch SPI sck only every ADC_SCK_SPEED_FACTOR times this interrupt is triggered */
    if (ScoutSPI::interruptCounter == 0) {
        if (ScoutSPI::SPIClock > 0) {
            //PORTB &= (~(1 << PIN_SPI_SCK_B) & ~(1 << PIN_ADC_SCK_B));
            PORTB &= (~(1 << PIN_SPI_SCK_B));
            //ScoutSerial::serialWrite("L",1);
            ScoutSPI::SPIClock = 0;
        } else {
            //PORTB |= (1 << PIN_SPI_SCK_B) | (1 << PIN_ADC_SCK_B);
            PORTB |= (1 << PIN_SPI_SCK_B);
            //ScoutSerial::serialWrite("H",1);
            ScoutSPI::SPIClock = 1;
        }


        /* Debug, check if MOSI line currently sends anything
        if ((PORTB & (1<<PIN_MOSI_B)) > 0){
            ScoutSerial::serialWrite("1\n", 2);
        } else {
            ScoutSerial::serialWrite("0\n", 2);
        }*/
    }


    /* switch adc system clock every time the interrupt is triggered */
    if (ScoutSPI::ADCClock > 0){
        PORTB &= (~(1 << PIN_ADC_SCK_B));
        ScoutSPI::ADCClock = 0;
    } else {
        PORTB |= (1 << PIN_ADC_SCK_B);
        ScoutSPI::ADCClock = 1;
    }


    /* Debug, check if MISO lane receives anything */
    if ((PORTB & (1<< PIN_MISO_B)) > 0){
        ScoutSerial::serialWrite("!!!\n", 4);
    }

}