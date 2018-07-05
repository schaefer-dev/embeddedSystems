//
// Created by Daniel Schäfer on 22.06.18.
//

#include "CollectorRF.h"
#include "CollectorSPI.h"
#include <Arduino.h>
#include "main.h"
#include "CollectorMonitor.h"
#include "CollectorSerial.h"


uint8_t CollectorRF::refereeAdress[5];
uint8_t CollectorRF::scoutAdress[5];
uint8_t CollectorRF::collectorAdress[5];


void CollectorRF::initializeRFModule() {

    /* for every register in RF module:
     * select RF as slave
     * command address write register XX
     * at least one clock cycle delay
     * payload for register setup
     * deselect slave
     * short delay
     * */

#ifndef ROBOT_SIMULATOR
    /* create arrays to store the adress values of referee,
     * collector and scout (already inverted) */
    refereeAdress[4] = 0xe1;
    refereeAdress[3] = 0xf0;
    refereeAdress[2] = 0xf0;
    refereeAdress[1] = 0xf0;
    refereeAdress[0] = 0xf0;
    scoutAdress[4] = 0xe2;
    scoutAdress[3] = 0x91;
    scoutAdress[2] = 0xa8;
    scoutAdress[1] = 0x27;
    scoutAdress[0] = 0x85;
    collectorAdress[4] = 0x98;
    collectorAdress[3] = 0x65;
    collectorAdress[2] = 0xfa;
    collectorAdress[1] = 0x29;
    collectorAdress[0] = 0xe6;

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

#endif

}

#ifndef ROBOT_SIMULATOR
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

    uint8_t adressArray[5];
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
#endif


int CollectorRF::queryRFModule(){
#ifndef ROBOT_SIMULATOR
    CollectorSPI::slaveSelect(SLAVE_RF);
    unsigned int payload = 255;
    unsigned int statusRF = CollectorSPI::readWriteSPI(payload);
    CollectorSPI::slaveSelect(SLAVE_NONE);

    //Serial1.print("RF Status Register: (");
    //Serial1.print(statusRF);
    //Serial1.println(")");

    return statusRF;
#endif

#ifdef ROBOT_SIMULATOR
    if (CollectorSerial::simulatorMessageIncoming()) {
        Serial1.println("Message arrived");
        Serial1.flush();
        return (1 << 6);
    } else {
        return 0;
    }
#endif
}




void CollectorRF::processReceivedMessage(CollectorState *collectorState) {
#ifndef ROBOT_SIMULATOR
    /* case for Message arrived */
    uint8_t answerArray[1];
    getCommandAnswer(answerArray, 1, RF_COMMAND_R_RX_PL_WID);

    /* if no next payload there */
    if (answerArray[0] == 0) {
        return;
    }

    /* read message from pipe */
    uint8_t payloadArray[answerArray[0]];
    getCommandAnswer(payloadArray, answerArray[0], RF_COMMAND_R_RX_PAYLOAD);

    Serial1.print("Message: ");

    for (int i = 0; i < answerArray[0]; i++){
        Serial1.print(payloadArray[i]);
    }
    Serial1.print("\n");

    /* clear status register */
    writeRegister(RF_REGISTER_STATUS, 64);

#endif

#ifdef ROBOT_SIMULATOR
    int answerArray[1];
    char serialInputArray[32];

    /* length of message here */
    answerArray[0] = 32;
    CollectorSerial::receiveSerialBlocking(serialInputArray);
    uint8_t payloadArray[32];

    for (int i = 0; i < 32; i++){
        payloadArray[i] = serialInputArray[i];
    }

#endif

    switch(payloadArray[0]){
        case 0x50: {
                /* PING case -> simply respond with PONG which contains nonce+1 */
#ifdef COLLECTOR_MONITOR
                CollectorMonitor::logPingCollector();
#endif
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
            collectorState->drivingDisabled = false;
#ifdef ROBOT_SIMULATOR
            Serial1.println("Position update received!");
#endif
            receivePosUpdate(payloadArray[1] * 256 + payloadArray[2],
                             payloadArray[3] * 256 + payloadArray[4],
                             payloadArray[5] * 256 + payloadArray[6]);
            break;
        case 0x61:
#ifdef ROBOT_SIMULATOR
            Serial1.println("OOB Message received!");
            Serial1.flush();
#endif
            /* Out of Bounds Message */
            collectorState->drivingDisabled = false;
            collectorState->outOfBoundsMessage();
            receivePosUpdate(payloadArray[1] * 256 + payloadArray[2],
                             payloadArray[3] * 256 + payloadArray[4],
                             payloadArray[5] * 256 + payloadArray[6]);
            break;
        case 0x70:
            /* MESSAGE case */
            break;
        case 0x80:
            Serial1.println("Message from scout arrived, echo performing ...");
            /* case for RELAY, scount sends and collector echos sends message back to scout */
            payloadArray[0] = 0x81;
            // Give scout time to switch back into Listening Mode
            delay(100);
            sendMessageTo(scoutAdress, payloadArray, answerArray[0]);
            break;

        default:
            Serial1.println("Illegal Message Identifer");
    }
}

void CollectorRF::sendMessageTo(uint8_t* receiverAdress, uint8_t * payloadArray, int payloadArrayLength){
#ifndef ROBOT_SIMULATOR
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
#endif

#ifdef COLLECTOR_MONITOR
    if (payloadArray[0] == 0x51) {
        /* the message was the pong response, log it */
        CollectorMonitor::logPongCollector();
    }
#endif

#ifdef ROBOT_SIMULATOR
    char outputArray[32];
    for (int i = 0; i < 32; i++){
        outputArray[i] = payloadArray[i];
    }
    if (payloadArrayLength < 32) {
        for (int i = payloadArrayLength; i < 31; i++){
            outputArray[i] = '_';
        }
        outputArray[31] = '\0';
    }

    Serial1.print(outputArray);

    Serial1.print("\n");


    Serial1.print("Sending Message to ");
    if (receiverAdress == refereeAdress)
        Serial1.print("referee ");
    if (receiverAdress == collectorAdress)
        Serial1.print("collector ");
    if (receiverAdress == scoutAdress)
        Serial1.print("scout ");

    Serial1.print("with content: ");
    Serial1.println(outputArray);
    Serial1.flush();

#endif


#ifndef ROBOT_SIMULATOR
    /* clear received status */
    writeRegister(RF_REGISTER_STATUS, (1 << 4)|(1 << 5) );

    /* Write Scout adress in RX Register Pipe 0 */
    write5ByteAdress(RF_REGISTER_RX_ADDR_P0, collectorAdress);

    /* switch back to RX mode */
    writeRegister(RF_REGISTER_CONFIG, 15);
#endif
}


#ifndef ROBOT_SIMULATOR
void CollectorRF::flushRXTX() {

    /* flush TX and RX */
    uint8_t payload[1] = {RF_COMMAND_FLUSH_RX};
    sendCommandWithPayload(payload, 1);
    payload[0] = RF_COMMAND_FLUSH_TX;
    sendCommandWithPayload(payload, 1);
}
#endif


#ifndef ROBOT_SIMULATOR
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
#endif


#ifndef ROBOT_SIMULATOR
void CollectorRF::getCommandAnswer(uint8_t *answerArray, int byteCount, int8_t command){

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
#endif

#ifndef ROBOT_SIMULATOR
void CollectorRF::writeRegister(uint8_t reg, uint8_t setting){

    CollectorSPI::slaveSelect(SLAVE_RF);

    CollectorSPI::readWriteSPI(RF_COMMAND_W_REGISTER | (RF_MASK_REGISTER & reg)); // write command for register
    delayMicroseconds(command_delay);
    CollectorSPI::readWriteSPI(setting);

    CollectorSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}
#endif


#ifndef ROBOT_SIMULATOR
/* Write bytes to adress !!! THIS FUNCTION TAKES CARE OF INVERTING !!! */
void CollectorRF::write5ByteAdress(int reg, uint8_t* bytes){

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
#endif


#ifndef ROBOT_SIMULATOR
int CollectorRF::readRegister(uint8_t reg){

    CollectorSPI::slaveSelect(SLAVE_RF);

    CollectorSPI::readWriteSPI(RF_COMMAND_R_REGISTER | (RF_MASK_REGISTER & reg)); // write command for register
    delayMicroseconds(command_delay);
    int output = CollectorSPI::readWriteSPI(RF_COMMAND_NOP);

    CollectorSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);

    return output;
}
#endif

#ifndef ROBOT_SIMULATOR
void CollectorRF::readAdressRegister(uint8_t reg, uint8_t* outputArray){
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
#endif