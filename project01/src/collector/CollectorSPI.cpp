#include "CollectorSPI.h"
#include "avr/io.h"
#include <Arduino.h>
#include "main.h"
#include "avr/interrupt.h"
#include "avr/io.h"
#include "../scout/Utility.h"


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

/* SPI_CLOCK_FACTOR defines every how many interrupts the SPI clock is inverted */
#define SPI_CLOCK_FACTOR 2

/* delay between each byte of communication with RF module in microseconds */
#define command_delay 50
#define delay_after_RF_select 1

unsigned int CollectorSPI::interruptCounter = 0;

/* breaks if Clocks not volatile, because ISR has to be forced to write to disk such
 * wait conditions are getting notified on change of clocks */
volatile unsigned char CollectorSPI::SPIClock = 0;

bool CollectorSPI::runSPIClock;


// not used because static
CollectorSPI::CollectorSPI() {
}

void CollectorSPI::SPIMasterInit() {

    CollectorSPI::interruptCounter = 0;

    // Set MISO pin as input
    DDRD &= ~(1 << PIN_MISO_D);
    delayMicroseconds(30);


    // Set MOSI as output
    DDRD |= (1 << PIN_MOSI_D);

    // Set SCK for SPI as ouput
    DDRB |= (1 << PIN_SPI_SCK_B);
    delayMicroseconds(30);


    /* drive SPICLock and ADCClock low initially */
    PORTB &= ~(1 << PIN_SPI_SCK_B);
    SPIClock = 0;
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


void CollectorSPI::initializeRFModule() {

    /* for every register in RF module:
     * select RF as slave
     * command address write register XX
     * at least one clock cycle delay
     * payload for register setup
     * deselect slave
     * short delay
     * */

    /* TODO: some of this is default set already, so can be optimized */

    slaveSelect(SLAVE_RF);
    readWriteSPI(34); // write command register 02
    delayMicroseconds(command_delay);
    readWriteSPI(63); // enable all data pipes
    slaveSelect(SLAVE_NONE);

    delay(delay_after_RF_select);

    slaveSelect(SLAVE_RF);
    readWriteSPI(36); // write command register 04
    delayMicroseconds(command_delay);
    readWriteSPI(138); // enable retries, up to 10 w delay 2ms
    slaveSelect(SLAVE_NONE);

    delay(delay_after_RF_select);

    slaveSelect(SLAVE_RF);
    readWriteSPI(37); // write command register 05
    delayMicroseconds(command_delay);
    readWriteSPI(111); // set channel to 111
    slaveSelect(SLAVE_NONE);

    delay(delay_after_RF_select);

    slaveSelect(SLAVE_RF);
    readWriteSPI(38); // write command register 06
    delayMicroseconds(command_delay);
    readWriteSPI(6); // data rate 1 mbps, max power
    slaveSelect(SLAVE_NONE);

    delay(delay_after_RF_select);


    /* write command register 11 to 16 to maximum RX payload (32 Bytes) */
    for (int i = 0; i < 6; i ++){
        slaveSelect(SLAVE_RF);
        readWriteSPI(49 + i);
        delayMicroseconds(command_delay);
        readWriteSPI(32);
        slaveSelect(SLAVE_NONE);

        delay(delay_after_RF_select);
    }

    slaveSelect(SLAVE_RF);
    readWriteSPI(60); // write command register 1D
    delayMicroseconds(command_delay);
    readWriteSPI(7); // enable dyn payload w dynamic ack
    slaveSelect(SLAVE_NONE);

    delay(delay_after_RF_select);

    slaveSelect(SLAVE_RF);
    readWriteSPI(35); // write command register 03
    delayMicroseconds(command_delay);
    readWriteSPI(3); // adresses defined to hold 5 bytes
    slaveSelect(SLAVE_NONE);

    delay(delay_after_RF_select);

    for (int i = 0; i < 2; i ++) {

        slaveSelect(SLAVE_RF);
        readWriteSPI(42 + i); // write roboter receive adress in register 0A
        delayMicroseconds(command_delay);
        /* write  receive adress 15: 0xe629fa6598 LSByte to MSByte */
        readWriteSPI(152); // write 98
        delayMicroseconds(command_delay);
        readWriteSPI(101); // write 65
        delayMicroseconds(command_delay);
        readWriteSPI(250); // write fa
        delayMicroseconds(command_delay);
        readWriteSPI(41); // write 29
        delayMicroseconds(command_delay);
        readWriteSPI(230); // write e6
        slaveSelect(SLAVE_NONE);

        delay(delay_after_RF_select);
    }


    /* DEBUG CODE BEGIN */

    /* write LSB receive adress in register 0C to 0F */
    for (int i = 0; i < 4; i ++){
        slaveSelect(SLAVE_RF);
        readWriteSPI(44 + i); // write roboter receive adress LSB in register 0C
        delayMicroseconds(command_delay);
        readWriteSPI(152); // write 98
        slaveSelect(SLAVE_NONE);
        delay(delay_after_RF_select);
    }

    /* DEBUG CODE END */

    slaveSelect(SLAVE_RF);
    readWriteSPI(32); // write command register
    delayMicroseconds(command_delay);
    readWriteSPI(15); // enable crc in 16 bit, pwr up and set to RX mode
    slaveSelect(SLAVE_NONE);

    /* drive RF module enable pin, apparently this should happen
     * at the very end of configuration, but not 100% sure */
    PORTC |= (1 << PIN_RF_ENABLE_C);
    delayMicroseconds(30);

}

void CollectorSPI::debug_RFModule(){
    int output = 0;
    for (int i = 0; i < 29; i++){
        slaveSelect(SLAVE_RF);
        readWriteSPI(i); // query reading of register i
        delayMicroseconds(command_delay);
        output = readWriteSPI(255); // write nop and read answer
        slaveSelect(SLAVE_NONE);

        delay(delay_after_RF_select);
        Serial1.print("Register: (");
        Serial1.print(output);
        Serial1.println(")");
        delay(20);

    }
}


int CollectorSPI::queryRFModule(){
    slaveSelect(SLAVE_RF);
    unsigned int payload = 255;
    unsigned int statusRF = readWriteSPI(payload);
    slaveSelect(SLAVE_NONE);

    Serial1.print("RF Status Register: (");
    Serial1.print(statusRF);
    Serial1.println(")");
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
    SPIClock = 0;

    return output;
}

/** set timer 4
 *
 * @param duration in ms
 */
void CollectorSPI::setTimer4Interrupt(uint16_t duration){

    // use this if duration 1 should correspond to be 1ms
    // uint8_t timer = (duration * 8) - 1;        // Collector runs on 1Mhz

    OCR4A = duration;

    // ctc on OCR4A
    TCCR4B |= (1 << COM4A1) | (1 << PWM4A);

    // set ctc interrupt
    TIMSK4 |= (1 << OCIE4A);

    // prescaler 1024
    TCCR4B &= ~(1 << CS42);
    //TCCR4B |= (1 << CS40) | (1 << CS41) | (1 << CS43);

    // prescaler 128
    TCCR4B |= (1 << CS43);

    // enable global interrupts
    sei();
}

// ISR for timer4
ISR (TIMER4_COMPA_vect) {

    CollectorSPI::interruptCounter = (CollectorSPI::interruptCounter + 1) % SPI_CLOCK_FACTOR;


    /* switch SPI sck only every ADC_SCK_SPEED_FACTOR times this interrupt is triggered */
    if (CollectorSPI::interruptCounter == 0) {
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

    }

    /* DEBUG: Alert if RF module receives something */
    if ((PIND & (1 << PIN_RF_IRQ_D) == 0)){
        Serial1.println("P");
    }
}