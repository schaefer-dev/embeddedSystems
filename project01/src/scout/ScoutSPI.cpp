//
// Created by rafael on 06.06.18.
// part rewrite from OrangutanSPIMaster.h
//

#include "ScoutSPI.h"
#include "avr/io.h"
#include <OrangutanTime.h>
#include "ScoutSerial.h"
#include "main.h"
#include "avr/interrupt.h"
#include "avr/io.h"
#include "math.h"


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

/* SPI_CLOCK_FACTOR defines every how many interrupts the SPI clock is inverted */
#define SPI_CLOCK_FACTOR 2

unsigned int ScoutSPI::interruptCounter = 0;
unsigned volatile char ScoutSPI::SPIClock = 0;
unsigned volatile char ScoutSPI::ADCClock = 0;

bool ScoutSPI::ADCConvertingState = false;
unsigned int ScoutSPI::ADCCommunicationState = 0;


// not used because static
ScoutSPI::ScoutSPI() {
}

void ScoutSPI::SPIMasterInit() {

    ScoutSPI::interruptCounter = 0;

    // Set MISO pin as input
    DDRB &= ~(1 << PIN_MISO_B);
    delayMicroseconds(30);

    // Set MOSI and SCK for SPI as ouput, additionally also the SystemClock of the ADC (PB1)
    DDRB |= (1 << PIN_MOSI_B) | (1 << PIN_SPI_SCK_B) | (1 << PIN_ADC_SCK_B);
    delayMicroseconds(30);


    /* drive SPICLock and ADCClock low initially */
    PORTB &= (~(1 << PIN_SPI_SCK_B) & ~(1 << PIN_ADC_SCK_B));
    ADCClock = 0;
    SPIClock = 0;
    delayMicroseconds(30);



    // Set Slave select as output
    DDRC |= (1 << PIN_SS_ADC_C);
    delayMicroseconds(30);
    DDRD |= (1 << PIN_SS_RF_D);
    delayMicroseconds(30);


    // drive all SlaveSelect lanes high, PD7 low to deselect RF
    PORTD |= (1 << PIN_SS_RF_D);
    delayMicroseconds(30);
    PORTC |= (1 << PIN_SS_ADC_C);
    delayMicroseconds(30);
    PORTD &= ~(1 << PIN_RF_ENABLE_D);
    delayMicroseconds(30);


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

    if (slave < 1) {
        /* deselect everyone */
        PORTD |= (1 << PIN_SS_RF_D);
        delayMicroseconds(30);
        PORTC |= (1 << PIN_SS_ADC_C);
        delayMicroseconds(30);
    }
    if (slave > 1) {
        /* select RF module (TODO: disabled currently) */
        PORTC |= (1 << PIN_SS_ADC_C);
        delayMicroseconds(30);
        // PORTD &= ~( 1 << PIN_SS_RF_D);
    } else {
        /* select ADC */
        PORTD |= (1 << PIN_SS_RF_D);
        delayMicroseconds(30);
        PORTC &= ~(1 << PIN_SS_ADC_C);
        delayMicroseconds(30);
    }
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

void ScoutSPI::readADCInInterrupt(char sensorAdress){
    ScoutSPI::ADCCommunicationState = 1;
}


int ScoutSPI::readADC(char sensorAdress) {
    int output = 0;

    /* catch illegal sensorAdresses */
    if (sensorAdress > 11){
        ScoutSerial::serialWrite("W: sensorAdress > 11\n", 21);
        return 0;
    }

    /* already start sending the first bit of adress */
    char outputBit = sensorAdress / 8;
    if (outputBit > 0) {
        PORTB |= (1 << PIN_MOSI_B);
        delayMicroseconds(30);
    } else {
        PORTB &= ~(1 << PIN_MOSI_B);
        delayMicroseconds(30);
    }

    waitNextSPIFallingEdge();

    // drive SS/CS low
    slaveSelect(SLAVE_ADC);


    // wait for 2 rising , 1 falling edge of ADC clock
    waitNextADCRisingEdge();
    waitNextADCRisingEdge();
    waitNextADCFallingEdge();
    waitNextADCRisingEdge();
    waitNextADCRisingEdge();

    // wait such that first bit is being read by ADC
    waitNextSPIRisingEdge();

    /* read value on rising edge of SPI */
    if( (PINB & (1 << PIN_MISO_B)) > 0)
        output += 128;

    /* on falling edge put next bit on MOSI */
    waitNextSPIFallingEdge();
    outputBit = (sensorAdress % 8) / 4;
    if (outputBit > 0) {
        PORTB |= (1 << PIN_MOSI_B);
        delayMicroseconds(30);
    } else {
        PORTB &= ~(1 << PIN_MOSI_B);
        delayMicroseconds(30);
    }

    // second bit is being read by ADC
    waitNextSPIRisingEdge();

    /* read value on rising edge of SPI */
    if( (PINB & (1 << PIN_MISO_B)) > 0)
        output += 64;

    /* on falling edge put next bit on MOSI */
    waitNextSPIFallingEdge();
    outputBit = (sensorAdress % 4) / 2;
    if (outputBit > 0) {
        PORTB |= (1 << PIN_MOSI_B);
        delayMicroseconds(30);
    } else {
        PORTB &= ~(1 << PIN_MOSI_B);
        delayMicroseconds(30);
    }

    // third bit is being read by ADC
    waitNextSPIRisingEdge();

    /* read value on rising edge of SPI */
    if( (PINB & (1 << PIN_MISO_B)) > 0)
        output += 32;

    /* on falling edge put next bit on MOSI */
    waitNextSPIFallingEdge();
    outputBit = (sensorAdress % 2) / 1;
    if (outputBit > 0) {
        PORTB |= (1 << PIN_MOSI_B);
        delayMicroseconds(30);
    } else {
        PORTB &= ~(1 << PIN_MOSI_B);
        delayMicroseconds(30);
    }

    // fourth bit is being read by ADC
    waitNextSPIRisingEdge();

    /* read value on rising edge of SPI */
    if( (PINB & (1 << PIN_MISO_B)) > 0)
        output += 16;

    // set byte to be sent over MOSI to 0 after adress sending has been completed
    PORTB &= ~(1 << PB5);
    delayMicroseconds(30);


    waitNextSPIRisingEdge();

    /* read value on rising edge of SPI */
    if( (PINB & (1 << PIN_MISO_B)) > 0)
        output += 8;

    waitNextSPIRisingEdge();

    /* read value on rising edge of SPI */
    if( (PINB & (1 << PIN_MISO_B)) > 0)
        output += 4;

    waitNextSPIRisingEdge();

    /* read value on rising edge of SPI */
    if( (PINB & (1 << PIN_MISO_B)) > 0)
        output += 2;

    waitNextSPIRisingEdge();

    /* read value on rising edge of SPI */
    if( (PINB & (1 << PIN_MISO_B)) > 0)
        output += 1;

    waitNextSPIFallingEdge();

    // drive SS/CS high
    slaveSelect(SLAVE_NONE);

    // wait for conversion (at least 36 cycles of ADC_SystemClock)
    for (int i = 0; i < 40; i++)
        waitNextADCRisingEdge();

    return output;
}

// ISR for timer1
ISR (TIMER1_COMPA_vect) {

    ScoutSPI::interruptCounter = (ScoutSPI::interruptCounter + 1) % SPI_CLOCK_FACTOR;


    /* switch SPI sck only every ADC_SCK_SPEED_FACTOR times this interrupt is triggered */
    if (ScoutSPI::interruptCounter == 0 && !ScoutSPI::ADCConvertingState) {
        if (ScoutSPI::SPIClock > 0) {
            //PORTB &= (~(1 << PIN_SPI_SCK_B) & ~(1 << PIN_ADC_SCK_B));
            PORTB &= (~(1 << PIN_SPI_SCK_B));
            //ScoutSerial::serialWrite("L",1);
            delayMicroseconds(30);
            ScoutSPI::SPIClock = 0;
        } else {
            //PORTB |= (1 << PIN_SPI_SCK_B) | (1 << PIN_ADC_SCK_B);
            PORTB |= (1 << PIN_SPI_SCK_B);
            //ScoutSerial::serialWrite("H",1);
            delayMicroseconds(30);
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
        delayMicroseconds(30);
        ScoutSPI::ADCClock = 0;
    } else {
        PORTB |= (1 << PIN_ADC_SCK_B);
        delayMicroseconds(30);
        ScoutSPI::ADCClock = 1;

    }


    /* Debug, check if MISO lane receives anything */
    if ((PINB & (1<< PIN_MISO_B)) > 0){
        //ScoutSerial::serialWrite("!!!\n", 4);
    }
    if ((PORTB & (1<< PIN_MISO_B)) > 0){
        ScoutSerial::serialWrite("!!!\n", 4);
    }

    switch(ScoutSPI::ADCCommunicationState){
        case 0:
            return;
        case 1:
            /* select ADC if SPI Clock */
            if (ScoutSPI::SPIClock == 0){
                ScoutSPI::slaveSelect(SLAVE_ADC);
                ScoutSPI::ADCCommunicationState += 1;

                // write first adress bit B3
                PORTB &= ~(1 << PIN_MOSI_B);
            }
            break;
        case 2:
            if (ScoutSPI::ADCClock > 0){
                ScoutSPI::ADCCommunicationState += 1;
            }
            break;
        case 3:
            if (ScoutSPI::ADCClock > 0){
                ScoutSPI::ADCCommunicationState += 1;
            }
            break;
        case 4:
            if (ScoutSPI::ADCClock > 0){
                ScoutSPI::ADCCommunicationState = 20;
            }
            break;

        case 20:
            /* SPI high step 1, ADC reads first adress bit */
            if (ScoutSPI::SPIClock > 0){
                ScoutSPI::ADCCommunicationState += 1;
            }
            break;
        case 21:
            /* SPI low step 1, write next adress bit */
            if (ScoutSPI::SPIClock == 0){
                // write second adress bit B2
                PORTB &= ~(1 << PIN_MOSI_B);
                ScoutSPI::ADCCommunicationState += 1;
            }
            break;
        case 22:
            /* SPI high step 2, ADC reads second adress bit */
            if (ScoutSPI::SPIClock > 0){
                ScoutSPI::ADCCommunicationState += 1;
            }
            break;
        case 23:
            /* SPI low step 2, write next adress bit */
            if (ScoutSPI::SPIClock == 0){
                // write third adress bit B1
                PORTB &= ~(1 << PIN_MOSI_B);
                ScoutSPI::ADCCommunicationState += 1;
            }
            break;
        case 24:
            /* SPI high step 3, ADC reads third adress bit */
            if (ScoutSPI::SPIClock > 0){
                ScoutSPI::ADCCommunicationState += 1;
            }
            break;
        case 25:
            /* SPI low step 3, write fourth adress bit */
            if (ScoutSPI::SPIClock == 0){
                // write fourth adress bit 01
                PORTB &= ~(1 << PIN_MOSI_B);
                ScoutSPI::ADCCommunicationState += 1;
            }
            break;
        case 26:
            /* SPI high step 4, ADC reads fourth adress bit */
            if (ScoutSPI::SPIClock > 0){
                ScoutSPI::ADCCommunicationState += 1;
            }
            break;
        case 27:
            /* wait for high step 5 */
            if (ScoutSPI::SPIClock > 0){
                ScoutSPI::ADCCommunicationState += 1;
            }
            break;
        case 28:
            /* wait for high step 6 */
            if (ScoutSPI::SPIClock > 0){
                ScoutSPI::ADCCommunicationState += 1;
            }
            break;
        case 29:
            /* wait for high step 7 */
            if (ScoutSPI::SPIClock > 0){
                ScoutSPI::ADCCommunicationState += 1;
            }
            break;
        case 30:
            /* wait for high step 8 */
            if (ScoutSPI::SPIClock > 0){
                ScoutSPI::ADCCommunicationState += 1;
            }
            break;
        case 31:
            /* deselect when SPI low */
            if (ScoutSPI::SPIClock == 0){
                ScoutSPI::slaveSelect(SLAVE_NONE);
                ScoutSPI::ADCCommunicationState = 0;
            }
            break;

        default:
            return;
    }
}