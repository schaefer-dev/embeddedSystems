//
// Created by rafael on 06.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_SPIMASTER_H
#define EMBEDDEDSYSTEMS18_SPIMASTER_H

#include <stdint.h>

// RF module command bytes
#define R_REGISTER		0x00
#define W_REGISTER		0x20
#define REGISTER_MASK	0x1F
#define R_RX_PL_WID     0x60
#define R_RX_PAYLOAD	0x61
#define W_TX_PAYLOAD	0xA0
#define FLUSH_TX		0xE1
#define FLUSH_RX		0xE2
#define REUSE_TX_PL		0xE3
#define NOP				0xFF


/* RF Register adresses */
#define RF_STATUS       0x07
#define TX_REGISTER     0x10
#define RF_CONFIG       0x00

// slave select ids
#define SLAVE_NONE 0
#define SLAVE_ADC 1
#define SLAVE_RF 2

class ScoutSPI
{

private:
    static void waitNextSPIRisingEdge();
    static void waitNextSPIFallingEdge();
    static void waitNextADCRisingEdge();
    static void waitNextADCFallingEdge();
    static unsigned int readWriteSPI(unsigned int payload);
    static void readAdressRegister(uint8_t reg, int* outputArray);



public:

    ScoutSPI();

    static void SPIMasterInit();

    static void slaveSelect(unsigned char slave);

    static int readADC(char sensorAdress);
    static void initializeRFModule();

    static void setTimer1Interrupt(uint16_t factor);

    static void ADCConversionWait();

    static unsigned int interruptCounter;

    static volatile unsigned char SPIClock;
    static volatile unsigned char ADCClock;

    static bool runSPIClock;


    static int queryRFModule();
    static void debug_RFModule();
    static void writeRegister(uint8_t reg, uint8_t setting);
    static int readRegister(uint8_t reg);

    static void fillCommandWithNops(int *commandArray, int arrayLength);
    static void getCommandAnswer(int *answerArray, int byteCount, int8_t command);
    static void sendCommandWithPayload(int *commandArray, int byteCount);

    static void sendPongToReferee(uint16_t nonce);
    static void write5ByteAdress(int reg, int* bytes);

};


#endif
