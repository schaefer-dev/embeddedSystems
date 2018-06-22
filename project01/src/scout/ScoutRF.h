//
// Created by Daniel Schäfer on 22.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_SCOUTRF_H
#define EMBEDDEDSYSTEMS18_SCOUTRF_H

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


class ScoutRF {

public:

    static void initializeRFModule();

    static int queryRFModule();
    static void debug_RFModule();
    static void writeRegister(uint8_t reg, uint8_t setting);
    static int readRegister(uint8_t reg);

    static void getCommandAnswer(int *answerArray, int byteCount, int8_t command);
    static void sendCommandWithPayload(int *commandArray, int byteCount);

    static void sendPongToReferee(uint16_t nonce);
    static void write5ByteAdress(int reg, int* bytes);

    static void readAdressRegister(uint8_t reg, int* outputArray);

};


#endif //EMBEDDEDSYSTEMS18_SCOUTRF_H
