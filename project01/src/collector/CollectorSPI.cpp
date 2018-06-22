#include "CollectorSPI.h"
#include "avr/io.h"
#include <Arduino.h>
#include "main.h"
#include "avr/interrupt.h"
#include "avr/io.h"
#include "../utils/Utility.h"


/* breaks if Clocks not volatile, because ISR has to be forced to write to disk such
 * wait conditions are getting notified on change of clocks */
unsigned volatile char CollectorSPI::SPIClock = 0;

bool volatile CollectorSPI::runSPIClock;


// not used because static
CollectorSPI::CollectorSPI() {
}

void CollectorSPI::SPIMasterInit() {

    // Set MISO pin as input
    DDRD &= ~(1 << PIN_MISO_D);
    delayMicroseconds(30);


    // Set MOSI as output
    DDRD |= (1 << PIN_MOSI_D);

    // Set SCK for SPI as ouput
    DDRB |= (1 << PIN_SPI_SCK_B);
    delayMicroseconds(30);
    SPIClock = 0;


    /* drive SPICLock and ADCClock low initially */
    PORTB &= ~(1 << PIN_SPI_SCK_B);
    CollectorSPI::runSPIClock = false;
    delayMicroseconds(30);


    // Set Slave select as output
    DDRB |= (1 << PIN_SS_RF_B);
    delayMicroseconds(30);


    // drive all SlaveSelect lanes high
    PORTB |= (1 << PIN_SS_RF_B);
    delayMicroseconds(30);

    CollectorSPI::setTimer4Interrupt(SPI_INTERRUPT_SPEED);

    delay(1);
}


/* returns whenever next rising edge occurs, used for synchronization */
void CollectorSPI::waitNextSPIRisingEdge() {

    while (SPIClock > 0) {
        asm("");
    }

    while (SPIClock == 0) {
        asm("");
    }
}

/* returns whenever next falling edge occurs, used for synchronization */
void CollectorSPI::waitNextSPIFallingEdge() {

    while (SPIClock == 0) {
        asm("");
    }

    while (SPIClock > 0) {
        asm("");
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
void CollectorSPI::slaveSelect(unsigned char slave) {

    if (slave < 1) {
        /* deselect everyone */
        PORTB |= (1 << PIN_SS_RF_B);
        delayMicroseconds(30);
    }
    if (slave == 1) {
        /* select RF module */
        PORTB &= ~(1 << PIN_SS_RF_B);
        delayMicroseconds(30);
    }
}

void CollectorSPI::writeRegister(uint8_t reg, uint8_t setting){

    slaveSelect(SLAVE_RF);

    readWriteSPI(RF_COMMAND_W_REGISTER | (RF_COMMAND_REGISTER_MASK & reg)); // write command for register
    delayMicroseconds(command_delay);
    readWriteSPI(setting);

    slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}


/* TODO: change to take unsigned char argument instead and return unsigned char (or maybe uint8) */
/* transmits 1 byte and reads 1 byte over SPI (most significant to least significant )*/
unsigned int CollectorSPI::readWriteSPI(unsigned int payload) {
    unsigned int output = 0;


    for (int i = 8; i > 0; i--) {
        int modValue = Utility::int_pow(2, i);
        int divValue = Utility::int_pow(2, (i - 1));

        /* wait until SPI clock falls (unless first iteration,
         * because SPIClock not yet enabled) */
        if (i < 8) {
            waitNextSPIFallingEdge();
        }

        int payloadBit = (payload % modValue) / divValue;
        if (payloadBit > 0) {
            PORTD |= (1 << PIN_MOSI_D);
        } else {
            PORTD &= ~(1 << PIN_MOSI_D);
        }

        if (i == 8) {
            runSPIClock = true;
        }

        /* wait until SPI clock high */
        while (SPIClock == 0){
            asm("");
        }

        /* read value on rising edge of SPI */
        if ((PIND & (1 << PIN_MISO_D)) > 0) {
            output += divValue;
        }
    }

    /* wait until SPI clock drops low */
    waitNextSPIFallingEdge();

    runSPIClock = false;

    return output;
}

/** set timer 4
 *
 * @param duration in ms
 */
void CollectorSPI::setTimer4Interrupt(uint16_t duration){

    // use this if duration 1 should correspond to be 1ms
    // uint8_t timer = (duration * 8) - 1;        // Collector runs on 1Mhz

    OCR4A = 127;

    // ctc on OCR4A
    TCCR4A |= (1 << COM4A1);

    // set ctc interrupt
    TIMSK4 |= (1 << OCIE4A);

    // prescaler 1024
    //TCCR4B |= (1 << CS40) | (1 << CS41) | (1 << CS43);

    // prescaler 128
    // TCCR4B |= (1 << CS43);

    // prescaler 64
    // TCCR4B |= (1 << CS42) | (1 << CS41) | (1 << CS40);

    // prescaler 32
    // TCCR4B |= (1 << CS42) | (1 << CS41);

    // prescaler 8
    TCCR4B |= (1 << CS42);

    // enable global interrupts
    sei();
}

// ISR for timer4
ISR (TIMER4_COMPA_vect) {
    //CollectorSPI::debug_interruptCounter += 1;
    //Serial1.println(CollectorSPI::debug_interruptCounter);
    /* switch SPI sck every time this interrupt is triggered */
    if (CollectorSPI::SPIClock > 0) {
        PORTB &= (~(1 << PIN_SPI_SCK_B));
        delayMicroseconds(30);
        CollectorSPI::SPIClock = 0;
    } else {
        /* only set SPI clock to 1 if allowed to run */
        if (CollectorSPI::runSPIClock) {
            PORTB |= (1 << PIN_SPI_SCK_B);
            delayMicroseconds(30);
            CollectorSPI::SPIClock = 1;
        }
    }


    /* DEBUG: Alert if RF module receives something */
    if ((PIND & (1 << PIN_RF_IRQ_D) == 0)){
        Serial1.println("P");
    }
}