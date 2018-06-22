//
// Created by Daniel Schäfer on 22.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_COLLECTORRF_H
#define EMBEDDEDSYSTEMS18_COLLECTORRF_H

#include <stdint.h>


// RF module command bytes
#define RF_COMMAND_R_REGISTER		0x00
#define RF_COMMAND_W_REGISTER		0x20
#define RF_COMMAND_R_RX_PL_WID      0x60
#define RF_COMMAND_R_RX_PAYLOAD	    0x61
#define RF_COMMAND_W_TX_PAYLOAD	    0xA0
#define RF_COMMAND_FLUSH_TX		    0xE1
#define RF_COMMAND_FLUSH_RX		    0xE2
#define RF_COMMAND_REUSE_TX_PL		0xE3
#define RF_COMMAND_NOP				0xFF


/* RF Register adresses */
#define RF_REGISTER_STATUS          0x07
#define RF_REGISTER_TX_REG          0x10
#define RF_REGISTER_RX_ADDR_P0      0x0A
#define RF_REGISTER_CONFIG          0x00
#define RF_MASK_REGISTER	        0x1F


class CollectorRF {

public:

    static void initializeRFModule();

    static int queryRFModule();
    static void debug_RFModule();
    static void writeRegister(uint8_t reg, uint8_t setting);
    static int readRegister(uint8_t reg);

    static void getCommandAnswer(int *answerArray, int byteCount, int8_t command);
    static void sendCommandWithPayload(uint8_t *commandArray, int byteCount);
    static void write5ByteAdress(int reg, int* bytes);

    static void readAdressRegister(uint8_t reg, int* outputArray);

    static void processReceivedMessage();

    static void flushRXTX();

    static void sendMessageTo(int* receiverAdress, int* payloadArray, int payloadArrayLength);

    static int refereeAdress[5];
    static int scoutAdress[5];
    static int collectorAdress[5];

};


#endif //EMBEDDEDSYSTEMS18_COLLECTORRF_H
