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
#define PIN_RF_IRQ_D PD2
#define PIN_ADC_SCK_B PB1


/* SPI_INTERRUPT_SPEED has to be 1 with the current setup */
#define SPI_INTERRUPT_SPEED 1

/* SPI_CLOCK_FACTOR defines every how many interrupts the SPI clock is inverted */
#define SPI_CLOCK_FACTOR 2

/* delay between each byte of communication with RF module in microseconds */
#define command_delay 50
#define delay_after_RF_select 1

unsigned int ScoutSPI::interruptCounter = 0;

/* breaks if Clocks not volatile, because ISR has to be forced to write to disk such
 * wait conditions are getting notified on change of clocks */
volatile unsigned char ScoutSPI::SPIClock = 0;
volatile unsigned char ScoutSPI::ADCClock = 0;

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

    while (SPIClock > 0) {
        asm("");
    }

    while (SPIClock == 0) {
        asm("");
    }
}

/* returns whenever next falling edge occurs, used for synchronization */
void ScoutSPI::waitNextSPIFallingEdge() {

    while (SPIClock == 0) {
        asm("");
    }

    while (SPIClock > 0) {
        asm("");
    }
}

/* returns whenever next rising edge occurs, used for synchronization */
void ScoutSPI::waitNextADCRisingEdge() {

    while (ADCClock > 0) {
        asm("");
    }

    while (ADCClock == 0) {
        asm("");
    }
}

/* returns whenever next falling edge occurs, used for synchronization */
void ScoutSPI::waitNextADCFallingEdge() {

    while (ADCClock == 0) {
        asm("");
    }

    while (ADCClock > 0) {
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


void ScoutSPI::initializeRFModule() {

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

    slaveSelect(SLAVE_RF);
    readWriteSPI(42); // write roboter receive adress in register 0A
    delayMicroseconds(command_delay);
    /* write  receive adress 14: 0x8527a891e2 LSByte to MSByte */
    readWriteSPI(226); // write e2
    delayMicroseconds(command_delay);
    readWriteSPI(145); // write 91
    delayMicroseconds(command_delay);
    readWriteSPI(168); // write a8
    delayMicroseconds(command_delay);
    readWriteSPI(39); // write 27
    delayMicroseconds(command_delay);
    readWriteSPI(133); // write 85
    slaveSelect(SLAVE_NONE);

    delay(delay_after_RF_select);


    /* DEBUG CODE BEGIN */

    slaveSelect(SLAVE_RF);
    readWriteSPI(43); // write roboter receive adress in register 0B
    delayMicroseconds(command_delay);
    /* write  receive adress 14: 0x8527a891e2 LSByte to MSByte */
    readWriteSPI(226); // write e2
    delayMicroseconds(command_delay);
    readWriteSPI(145); // write 91
    delayMicroseconds(command_delay);
    readWriteSPI(168); // write a8
    delayMicroseconds(command_delay);
    readWriteSPI(39); // write 27
    delayMicroseconds(command_delay);
    readWriteSPI(133); // write 85
    slaveSelect(SLAVE_NONE);

    delay(delay_after_RF_select);

    /* write LSB receive adress in register 0C to 0F */
    for (int i = 0; i < 4; i ++){
        slaveSelect(SLAVE_RF);
        readWriteSPI(44 + i); // write roboter receive adress LSB in register 0C
        delayMicroseconds(command_delay);
        readWriteSPI(226); // write e2
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
    PORTD |= (1 << PIN_RF_ENABLE_D);
    delayMicroseconds(30);

}

void ScoutSPI::debug_RFModule(){
    int output = 0;
    for (int i = 0; i < 29; i++){
        slaveSelect(SLAVE_RF);
        readWriteSPI(i); // query reading of register i
        delayMicroseconds(command_delay);
        output = readWriteSPI(255); // write nop and read answer
        slaveSelect(SLAVE_NONE);

        delay(delay_after_RF_select);
        ScoutSerial::serialWrite("Register: (", 11);
        ScoutSerial::serialWrite8Bit(output);
        ScoutSerial::serialWrite(")\n", 2);
        delay(20);

    }
}



int ScoutSPI::queryRFModule(){
    slaveSelect(SLAVE_RF);
    unsigned int payload = 255;
    unsigned int statusRF = readWriteSPI(payload);
    slaveSelect(SLAVE_NONE);
    
    ScoutSerial::serialWrite("RF Status Register: (", 21);
    ScoutSerial::serialWrite8Bit(statusRF);
    ScoutSerial::serialWrite(")\n", 2);
}


/* TODO: change to take unsigned char argument instead and return unsigned char (or maybe uint8) */
/* transmits 1 byte and reads 1 byte over SPI (most significant to least significant )*/
unsigned int ScoutSPI::readWriteSPI(unsigned int payload) {
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
            PORTB |= (1 << PIN_MOSI_B);
        } else {
            PORTB &= ~(1 << PIN_MOSI_B);
        }

        if (i == 8) {
            runSPIClock = true;
        }

        /* wait until SPI clock high */
        while (SPIClock == 0){
            asm("");
        }

        /* read value on rising edge of SPI */
        if ((PINB & (1 << PIN_MISO_B)) > 0) {
            output += divValue;
        }
    }

    /* wait until SPI clock drops low */
    waitNextSPIFallingEdge();

    runSPIClock = false;
    SPIClock = 0;

    return output;
}

void ScoutSPI::ADCConversionWait() {
    // wait for conversion (at least 36 cycles of ADC_SystemClock)
    for (int i = 0; i < 40; i++)
        waitNextADCRisingEdge();
    return;
}

int ScoutSPI::readADC(char sensorAdress) {
    int output = 0;

    /* scale adress to 8 byte payload */
    unsigned int payload = sensorAdress * 16;

    /* catch illegal sensorAdresses */
    if (sensorAdress > 11) {
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

    /* DEBUG: Alert if RF module receives something */
    if ((PIND & (1 << PIN_RF_IRQ_D) == 0)){
        ScoutSerial::serialWrite("P\n", 2);
    }

    /* switch adc system clock every time the interrupt is triggered */
    if (ScoutSPI::ADCClock > 0) {
        PORTB &= (~(1 << PIN_ADC_SCK_B));
        delayMicroseconds(30);
        ScoutSPI::ADCClock = 0;
    } else {
        PORTB |= (1 << PIN_ADC_SCK_B);
        delayMicroseconds(30);
        ScoutSPI::ADCClock = 1;

    }
}