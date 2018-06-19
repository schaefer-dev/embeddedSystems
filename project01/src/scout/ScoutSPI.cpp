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
#include "Utility.h"


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

bool ScoutSPI::runSPIClock;


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
    ScoutSPI::runSPIClock = false;
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

    while (SPIClock == 0) {
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

    while (ADCClock == 0) {
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
        PORTD &= ~(1 << PIN_SS_RF_D);
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


    // prescaler 256, seems to be too fast
    // TCCR1B |= (1 << CS12);


    // enable global interrupts
    sei();
}


void ScoutSPI::initializeRFModule(){

    /* drive RF module enable pin */
    PORTD |= (1 << PIN_RF_ENABLE_D);
    delayMicroseconds(30);

    int output = 1;

}


/* transmits 1 byte and reads 1 byte over SPI (most significant to least significant )*/
int ScoutSPI::readWriteSPI(int payload){
    int output = 0;


    for (int i = 8; i > 0; i --) {
        int modValue = Utility::int_pow(2, i);
        int divValue = Utility::int_pow(2, (i - 1));

        /* wait until SPI clock falls (unless first iteration,
         * because SPIClock not yet enabled) */
        if (i < 8){
            waitNextSPIFallingEdge();
        }

        int payloadBit = (payload % modValue) / divValue;
        if (payloadBit > 0) {
            PORTB |= (1 << PIN_MOSI_B);
            if (i <= 4){
                ScoutSerial::serialWrite("ERROR at ",9);
                ScoutSerial::serialWriteInt(payload);
                ScoutSerial::serialWrite("mod  ",4);
                ScoutSerial::serialWriteInt(modValue);
                ScoutSerial::serialWrite("div  ",4);
                ScoutSerial::serialWriteInt(divValue);
            }
        } else {
            PORTB &= ~(1 << PIN_MOSI_B);
        }

        if (i == 8) {
            runSPIClock = true;
        }

        /* wait until SPI clock high */
        waitNextSPIRisingEdge();

        /* read value on rising edge of SPI */
        if ((PINB & (1 << PIN_MISO_B)) > 0) {
            output += divValue;
        }
    }

    /* wait until SPI clock drops low */
    waitNextSPIFallingEdge();

    runSPIClock = false;

    /* TODO assert that SPIClock true here */

    return output;
}

void ScoutSPI::ADCConversionWait(){
    // wait for conversion (at least 36 cycles of ADC_SystemClock)
    for (int i = 0; i < 40; i++)
        waitNextADCRisingEdge();
    return;
}

int ScoutSPI::readADC(char sensorAdress) {
    int output = 0;

    /* scale adress to 8 byte payload */
    int payload = sensorAdress * 16;

    /* catch illegal sensorAdresses */
    if (sensorAdress > 11){
        ScoutSerial::serialWrite("W: sensorAdress > 11\n", 21);
        return 0;
    }

    // drive SS/CS low
    slaveSelect(SLAVE_ADC);


    // wait for 2 rising , 1 falling edge of ADC clock
    waitNextADCRisingEdge();
    waitNextADCRisingEdge();
    waitNextADCRisingEdge();
    waitNextADCRisingEdge();
    waitNextADCFallingEdge();


    // TODO: fix readWriteSPI, to mimic the behaviour of the function below
    output = readWriteSPI(payload);
    slaveSelect(SLAVE_NONE);
    return output;
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
            delayMicroseconds(30);
            ScoutSPI::SPIClock = 0;
        } else {
            /* only set SPI clock to 1 if allowed to run */
            if (ScoutSPI::runSPIClock) {
                //PORTB |= (1 << PIN_SPI_SCK_B) | (1 << PIN_ADC_SCK_B);
                PORTB |= (1 << PIN_SPI_SCK_B);
                //ScoutSerial::serialWrite("H",1);
                delayMicroseconds(30);
                ScoutSPI::SPIClock = 1;
            }
        }

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
}