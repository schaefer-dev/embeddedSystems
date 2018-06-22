//
// Created by Daniel Schäfer on 22.06.18.
//

#include "CollectorRF.h"
#include "CollectorSPI.h"
#include <Arduino.h>
#include "main.h"


int CollectorRF::refereeAdress[5];
int CollectorRF::scoutAdress[5];
int CollectorRF::collectorAdress[5];


void CollectorRF::initializeRFModule() {

    /* for every register in RF module:
     * select RF as slave
     * command address write register XX
     * at least one clock cycle delay
     * payload for register setup
     * deselect slave
     * short delay
     * */


    /* create arrays to store the adress values of referee,
     * collector and scout (already inverted) */
    refereeAdress[4] = 0x00e1;
    refereeAdress[3] = 0x00f0;
    refereeAdress[2] = 0x00f0;
    refereeAdress[1] = 0x00f0;
    refereeAdress[0] = 0x00f0;
    scoutAdress[4] = 0x00e2;
    scoutAdress[3] = 0x0091;
    scoutAdress[2] = 0x00a8;
    scoutAdress[1] = 0x0027;
    scoutAdress[0] = 0x0085;
    collectorAdress[4] = 0x0098;
    collectorAdress[3] = 0x0065;
    collectorAdress[2] = 0x00fa;
    collectorAdress[1] = 0x0029;
    collectorAdress[0] = 0x00e6;

    /* TODO: some of this is default set already, so can be optimized */

    // write command register 02: enable all data pipes
    writeRegister(0x00000002,  63);

    // write command register 04: enable 10 retries w delay 2ms
    writeRegister(0x00000004, 138);

    // write register 05: set channel to 111
    writeRegister(0x00000005, 111);

    // write register 06: data rate 1 mbps, max power
    writeRegister(0x00000006,   6);



    /* write command register 11 to 16 to maximum RX payload (32 Bytes) */
    for (int i = 0x00000011; i < 0x00000017; i ++){
        writeRegister( i,  32);
    }

    // write register 1D: enable dyn payload, dyn ack
    writeRegister(0x0000001C,   63);

    // write register 1D: enable dyn payload, dyn ack
    writeRegister(0x0000001D,   7);


    write5ByteAdress(RF_REGISTER_RX_ADDR_P0, collectorAdress);


    // write register 00: enable crc 16 bit, pwr up, rx mode
    writeRegister(0x00000000, 15);

    /* drive RF module enable pin, apparently this should happen
     * at the very end of configuration, but not 100% sure */
    PORTC |= (1 << PIN_RF_ENABLE_C);
    delayMicroseconds(30);

}

void CollectorRF::debug_RFModule(){
    int output = 0;
    for (int i = 0; i < 30; i++){
        output = readRegister(i);

        Serial1.print("Register ");
        Serial1.print(i);
        Serial1.print(" = (");
        Serial1.print(output);
        Serial1.print(")");

        delay(20);

    }

    int adressArray[5];
    readAdressRegister(0x000A, adressArray);
    Serial1.print("ADDR Register: 0A (");
    for (int i=0; i < 5; i++) {
        Serial1.print(adressArray[i]);
    }
    Serial1.println(")");

    readAdressRegister(0x000B, adressArray);
    Serial1.print("ADDR Register: 0B (");
    for (int i=0; i < 5; i++) {
        Serial1.print(adressArray[i]);
    }
    Serial1.println(")");

}


int CollectorRF::queryRFModule(){
    CollectorSPI::slaveSelect(SLAVE_RF);
    unsigned int payload = 255;
    unsigned int statusRF = CollectorSPI::readWriteSPI(payload);
    CollectorSPI::slaveSelect(SLAVE_NONE);

    //Serial1.print("RF Status Register: (");
    //Serial1.print(statusRF);
    //Serial1.println(")");

    return statusRF;
}




void CollectorRF::processReceivedMessage() {
    /* case for Message arrived */
    int answerArray[1];
    getCommandAnswer(answerArray, 1, RF_COMMAND_R_RX_PL_WID);

    /* if no next payload there */
    if (answerArray[0] == 0) {
        return;
    }

    /* read message from pipe */
    int payloadArray[answerArray[0]];
    getCommandAnswer(payloadArray, answerArray[0], RF_COMMAND_R_RX_PAYLOAD);

    Serial1.print("Message: ");

    for (int i = 0; i < answerArray[0]; i++){
        Serial1.print(payloadArray[i]);
    }
    Serial1.print("\n");

    /* clear status register */
    writeRegister(RF_REGISTER_STATUS, 64);

    switch(payloadArray[0]){
        case 0x50: {
                /* PING case -> simply respond with PONG which contains nonce+1 */
                uint16_t incNonce = 0;
                incNonce = payloadArray[1] * 256 + payloadArray[2] + 1;
                payloadArray[0] = 0x51;
                payloadArray[1] = (uint8_t) (incNonce / 256);
                payloadArray[2] = (uint8_t) (incNonce % 256);
                sendMessageTo(refereeAdress, payloadArray, 3);
            }
            break;
        case 0x60:
            /* POS update case */
            receivePosUpdate(payloadArray[1] * 256 + payloadArray[2],
                             payloadArray[3] * 256 + payloadArray[4],
                             payloadArray[5] * 256 + payloadArray[6]);
            break;
        case 0x70:
            /* MESSAGE case */
            break;
        case 0x80:
            /* case for RELAY, scount sends and collector echos sends message back to scout */
            payloadArray[0] = 0x81;
            sendMessageTo(scoutAdress, payloadArray, answerArray[0]);
            break;

        default:
            Serial1.println("Illegal Message Identifer");
    }
}

void CollectorRF::sendMessageTo(int* receiverAdress, int* payloadArray, int payloadArrayLength){
    /* Write Referee adress to TX Register */
    write5ByteAdress(RF_REGISTER_TX_REG, receiverAdress);
    /* Write Referee adress to TX Register */
    write5ByteAdress(RF_REGISTER_RX_ADDR_P0, receiverAdress);

    /* switch to TX mode */
    writeRegister(RF_REGISTER_CONFIG, 14);

    uint8_t commandArray[payloadArrayLength + 1];
    commandArray[0] = RF_COMMAND_W_TX_PAYLOAD;
    for (int i = 1; i < payloadArrayLength + 1; i++){
        commandArray[i] = payloadArray[i-1];
    }

    sendCommandWithPayload(commandArray, payloadArrayLength + 1);

    int status = 0;
    long timeout = millis();

    delay(10);
    flushRXTX();
    while (true){
        status = queryRFModule();

        /* either message sent or max retries reached case */
        if (((1 << 4) & status) > 0) {
#ifdef DEBUG
            Serial1.println("PONG sending maxRetries");
#endif
            break;
        }


        if (((1 << 5) & status) > 0) {
#ifdef DEBUG
            Serial1.println("PONG sent succesfully");
#endif
            break;
        }

        /* in case something goes wrong cancel after 1s */
        if (millis() - timeout > 2000) {
#ifdef DEBUG
            Serial1.println("Manual sending timeout");
#endif
            flushRXTX();
            break;
        }
    }

    /* clear received status */
    writeRegister(RF_REGISTER_STATUS, (1 << 4)|(1 << 5) );

    /* Write Scout adress in RX Register Pipe 0 */
    write5ByteAdress(RF_REGISTER_RX_ADDR_P0, collectorAdress);

    /* switch back to RX mode */
    writeRegister(RF_REGISTER_CONFIG, 15);
}


void CollectorRF::flushRXTX() {

    /* flush TX and RX */
    uint8_t payload[1] = {RF_COMMAND_FLUSH_RX};
    sendCommandWithPayload(payload, 1);
    payload[0] = RF_COMMAND_FLUSH_TX;
    sendCommandWithPayload(payload, 1);

}


void CollectorRF::sendCommandWithPayload(uint8_t *commandArray, int byteCount){

    CollectorSPI::slaveSelect(SLAVE_RF);

    delayMicroseconds(command_delay);
    for (int i = 0; i < byteCount; i++) {
        CollectorSPI::readWriteSPI(commandArray[i]);
        delayMicroseconds(command_delay);
    }
    CollectorSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}



void CollectorRF::getCommandAnswer(int *answerArray, int byteCount, int8_t command){

    CollectorSPI::slaveSelect(SLAVE_RF);

    CollectorSPI::readWriteSPI(command); // write command for register
    delayMicroseconds(command_delay);
    for (int i = 0; i < byteCount; i++) {
        answerArray[i] = CollectorSPI::readWriteSPI(RF_COMMAND_NOP);
        delayMicroseconds(command_delay);
    }
    CollectorSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}

void CollectorRF::writeRegister(uint8_t reg, uint8_t setting){

    CollectorSPI::slaveSelect(SLAVE_RF);

    CollectorSPI::readWriteSPI(RF_COMMAND_W_REGISTER | (RF_MASK_REGISTER & reg)); // write command for register
    delayMicroseconds(command_delay);
    CollectorSPI::readWriteSPI(setting);

    CollectorSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}


/* Write bytes to adress !!! THIS FUNCTION TAKES CARE OF INVERTING !!! */
void CollectorRF::write5ByteAdress(int reg, int* bytes){

    CollectorSPI::slaveSelect(SLAVE_RF);

    CollectorSPI::readWriteSPI(RF_COMMAND_W_REGISTER | (RF_MASK_REGISTER & reg)); // write command for register
    delayMicroseconds(command_delay);
    for (int i = 4; i >= 0; i--){
        CollectorSPI::readWriteSPI(bytes[i]);
        delayMicroseconds(command_delay);
    }

    CollectorSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}


int CollectorRF::readRegister(uint8_t reg){

    CollectorSPI::slaveSelect(SLAVE_RF);

    CollectorSPI::readWriteSPI(RF_COMMAND_R_REGISTER | (RF_MASK_REGISTER & reg)); // write command for register
    delayMicroseconds(command_delay);
    int output = CollectorSPI::readWriteSPI(RF_COMMAND_NOP);

    CollectorSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);

    return output;
}

void CollectorRF::readAdressRegister(uint8_t reg, int* outputArray){
    CollectorSPI::slaveSelect(SLAVE_RF);

    CollectorSPI::readWriteSPI(RF_COMMAND_R_REGISTER | (RF_MASK_REGISTER & reg)); // write command for register
    delayMicroseconds(command_delay);
    for (int i = 0; i < 5; i++) {
        outputArray[i] = CollectorSPI::readWriteSPI(RF_COMMAND_NOP);
        delayMicroseconds(command_delay);
    }

    CollectorSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}