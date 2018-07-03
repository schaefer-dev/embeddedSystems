//
// Created by Daniel Schäfer on 22.06.18.
//

#include "ScoutRF.h"
#include "ScoutSPI.h"
#include "ScoutSerial.h"
#include <OrangutanTime.h>
#include "avr/io.h"
#include "main.h"
#include "ScoutMonitor.h"


/* If ROBOT_SIMULATOR is set instead of communicating over the Module with SPI
 * We always wait for a message with content "0x01" and after that we wait for
 * 32 bytes sent over serial port in a blocking manner.
 *
 * It is important to wait blocking to avoid that the message is split.
 */


char ScoutRF::refereeAdress[5];
char ScoutRF::scoutAdress[5];
char ScoutRF::collectorAdress[5];

void ScoutRF::initializeRFModule() {

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


    write5ByteAdress(RF_REGISTER_RX_ADDR_P0, scoutAdress);


    // write register 00: enable crc 16 bit, pwr up, rx mode
    writeRegister(0x00000000, 15);

    flushRXTX();

    /* drive RF module enable pin, apparently this should happen
     * at the very end of configuration, but not 100% sure */
    PORTD |= (1 << PIN_RF_ENABLE_D);
    delayMicroseconds(30);
#endif

}


#ifndef ROBOT_SIMULATOR
void ScoutRF::flushRXTX() {
    /* flush TX and RX */
    uint8_t payload[1] = {RF_COMMAND_FLUSH_RX};
    sendCommandWithPayload(payload, 1);
    payload[0] = RF_COMMAND_FLUSH_TX;
    sendCommandWithPayload(payload, 1);
}
#endif


#ifndef ROBOT_SIMULATOR
void ScoutRF::debug_RFModule(){
    int output = 0;
    for (int i = 0; i < 30; i++){
        output = readRegister(i);

        ScoutSerial::serialWrite("Register ", 9);
        ScoutSerial::serialWrite8BitHex(i);
        ScoutSerial::serialWrite(" = (", 4);
        ScoutSerial::serialWrite8BitBinary(output);
        ScoutSerial::serialWrite(")\n", 2);
        delay(20);

    }

    char adressArray[5];
    readAdressRegister(0x0A, adressArray);
    ScoutSerial::serialWrite("ADDR Register: 0A (", 19);
    for (int i=0; i < 5; i++) {
        ScoutSerial::serialWrite8BitHex(adressArray[i]);
    }
    ScoutSerial::serialWrite(")\n", 2);

    readAdressRegister(0x0B, adressArray);
    ScoutSerial::serialWrite("ADDR Register: 0B (", 19);
    for (int i=0; i < 5; i++) {
        ScoutSerial::serialWrite8BitHex(adressArray[i]);
    }
    ScoutSerial::serialWrite(")\n", 2);
}
#endif


int ScoutRF::queryRFModule(){
#ifndef ROBOT_SIMULATOR
    ScoutSPI::slaveSelect(SLAVE_RF);
    unsigned int payload = 255;
    unsigned int statusRF = ScoutSPI::readWriteSPI(payload);
    ScoutSPI::slaveSelect(SLAVE_NONE);

    //ScoutSerial::serialWrite("RF Status Register: (", 21);
    //ScoutSerial::serialWrite8Bit(statusRF);
    //ScoutSerial::serialWrite(")\n", 2);

    return statusRF;
#endif

#ifdef ROBOT_SIMULATOR
    if (ScoutSerial::simulatorMessageIncoming()) {
        ScoutSerial::serialWrite("Message arrived\n", 16);
        return (1 << 6);
    } else {
        return 0;
    }
#endif
}

void ScoutRF::processReceivedMessage(ScoutState *scoutState) {
#ifndef ROBOT_SIMULATOR
    /* case for Message arrived */
    char answerArray[1];
    ScoutRF::getCommandAnswer(answerArray, 1, RF_COMMAND_R_RX_PL_WID);

    /* if no next payload there */
    if (answerArray[0] == 0) {
        return;
    }

    /* read message from pipe */
    char payloadArray[answerArray[0]];
    ScoutRF::getCommandAnswer(payloadArray, answerArray[0], RF_COMMAND_R_RX_PAYLOAD);

    ScoutSerial::serialWrite("Message: ", 9);

    for (int i = 0; i < answerArray[0]; i++){
        ScoutSerial::serialWrite8BitHex(payloadArray[i]);
    }
    ScoutSerial::serialWrite(" (only first 3 bytes displayed)\n", 32);

    /* clear status register */
    ScoutRF::writeRegister(RF_REGISTER_STATUS, 64);

#endif

#ifdef ROBOT_SIMULATOR
    int answerArray[1];
    char serialInputArray[32];

    /* length of message here */
    answerArray[0] = 32;
    ScoutSerial::receiveSerialBlocking(serialInputArray);
    char payloadArray[32];

    for (int i = 0; i < 32; i++){
        payloadArray[i] = serialInputArray[i];
    }

#endif

    switch(payloadArray[0]){
        case 0x50: {
            /* PING case -> simply respond with PONG which contains nonce+1 */
#ifdef SCOUT_MONITOR
            ScoutMonitor::logPingScout();
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
#ifdef ROBOT_SIMULATOR
            ScoutSerial::serialWrite("Position update received!\n",26);
#endif
            receivePosUpdate(payloadArray[1] * 256 + payloadArray[2],
                             payloadArray[3] * 256 + payloadArray[4],
                             payloadArray[5] * 256 + payloadArray[6]);
            break;
        case 0x61:
#ifdef ROBOT_SIMULATOR
            ScoutSerial::serialWrite("OOB Message received!\n",22);
#endif
            /* Out of Bounds Message */
            scoutState->outOfBoundsMessage();
            receivePosUpdate(payloadArray[1] * 256 + payloadArray[2],
                             payloadArray[3] * 256 + payloadArray[4],
                             payloadArray[5] * 256 + payloadArray[6]);
            break;
        case 0x70:
            /* MESSAGE case */
            break;
        case 0x81:
            /* case for RELAY, scount sends and collector echos message with prefix 81 */
            /* Scout has to print messages here to serial */

            for (int i = 0; i < answerArray[0]; i++){
                char buffer[1] = {payloadArray[i]};
                ScoutSerial::serialWrite(buffer, 1);
            }
            ScoutSerial::serialWrite("\n",1);
            break;
        default:
            ScoutSerial::serialWrite("Illegal Message Identifer\n",26);
    }
}




void ScoutRF::sendMessageTo(char* receiverAdress, char* payloadArray, int payloadArrayLength){
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
            ScoutSerial::serialWrite("Message sending maxRetries\n", 27);
#endif
            break;
        }


        if (((1 << 5) & status) > 0) {
#ifdef DEBUG
            ScoutSerial::serialWrite("Message sent succesfully\n", 25);
#endif
            break;
        }

        /* in case something goes wrong cancel after 1s */
        if (millis() - timeout > 2000) {
#ifdef DEBUG
            ScoutSerial::serialWrite("Manual sending timeout\n", 23);
#endif
            flushRXTX();
            break;
        }
    }
#endif

#ifdef SCOUT_MONITOR
    if (payloadArray[0] == 0x51) {
        /* the message was the pong response, log it */
        ScoutMonitor::logPongScout();
    }
#endif


#ifdef ROBOT_SIMULATOR
    ScoutSerial::serialWrite("Sending Message to ", 19);
    if (receiverAdress == refereeAdress)
        ScoutSerial::serialWrite("referee ", 8);
    if (receiverAdress == collectorAdress)
        ScoutSerial::serialWrite("collector ", 10);
    if (receiverAdress == scoutAdress)
        ScoutSerial::serialWrite("scout ", 10);

    ScoutSerial::serialWrite("with content: ", 14);
    char outputArray[32];
    for (int i = 0; i < 32; i++){
        outputArray[i] = payloadArray[i];
    }

    ScoutSerial::serialWrite(outputArray, payloadArrayLength);
    ScoutSerial::serialWrite("\n", 1);
#endif



#ifndef ROBOT_SIMULATOR

    /* clear received status */
    writeRegister(RF_REGISTER_STATUS, (1 << 4)|(1 << 5) );

    /* Write Scout adress in RX Register Pipe 0 */
    write5ByteAdress(RF_REGISTER_RX_ADDR_P0, scoutAdress);

    /* switch back to RX mode */
    writeRegister(RF_REGISTER_CONFIG, 15);
#endif
}


#ifndef ROBOT_SIMULATOR
void ScoutRF::sendCommandWithPayload(uint8_t *commandArray, int byteCount){

    ScoutSPI::slaveSelect(SLAVE_RF);

    delayMicroseconds(command_delay);
    for (int i = 0; i < byteCount; i++) {
        ScoutSPI::readWriteSPI(commandArray[i]);
        delayMicroseconds(command_delay);
    }
    ScoutSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}
#endif


#ifndef ROBOT_SIMULATOR
void ScoutRF::getCommandAnswer(char *answerArray, int byteCount, int8_t command){

    ScoutSPI::slaveSelect(SLAVE_RF);

    ScoutSPI::readWriteSPI(command); // write command for register
    delayMicroseconds(command_delay);
    for (int i = 0; i < byteCount; i++) {
        answerArray[i] = ScoutSPI::readWriteSPI(RF_COMMAND_NOP);
        delayMicroseconds(command_delay);
    }
    ScoutSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}
#endif


#ifndef ROBOT_SIMULATOR
void ScoutRF::writeRegister(uint8_t reg, uint8_t setting){

    ScoutSPI::slaveSelect(SLAVE_RF);

    ScoutSPI::readWriteSPI(RF_COMMAND_W_REGISTER | (RF_MASK_REGISTER & reg)); // write command for register
    delayMicroseconds(command_delay);
    ScoutSPI::readWriteSPI(setting);

    ScoutSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}
#endif


#ifndef ROBOT_SIMULATOR
/* Write bytes to adress !!! THIS FUNCTION TAKES CARE OF INVERTING !!! */
void ScoutRF::write5ByteAdress(int reg, char* bytes){

    ScoutSPI::slaveSelect(SLAVE_RF);

    ScoutSPI::readWriteSPI(RF_COMMAND_W_REGISTER | (RF_MASK_REGISTER & reg)); // write command for register
    delayMicroseconds(command_delay);
    for (int i = 4; i >= 0; i--){
        ScoutSPI::readWriteSPI(bytes[i]);
        delayMicroseconds(command_delay);
    }

    ScoutSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}
#endif


#ifndef ROBOT_SIMULATOR
int ScoutRF::readRegister(uint8_t reg){

    ScoutSPI::slaveSelect(SLAVE_RF);

    ScoutSPI::readWriteSPI(RF_COMMAND_R_REGISTER | (RF_MASK_REGISTER & reg)); // write command for register
    delayMicroseconds(command_delay);
    int output = ScoutSPI::readWriteSPI(RF_COMMAND_NOP);

    ScoutSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);

    return output;
}
#endif


#ifndef ROBOT_SIMULATOR
void ScoutRF::readAdressRegister(uint8_t reg, char* outputArray){
    ScoutSPI::slaveSelect(SLAVE_RF);

    ScoutSPI::readWriteSPI(RF_COMMAND_R_REGISTER | (RF_MASK_REGISTER & reg)); // write command for register
    delayMicroseconds(command_delay);
    for (int i = 0; i < 5; i++) {
        outputArray[i] = ScoutSPI::readWriteSPI(RF_COMMAND_NOP);
        delayMicroseconds(command_delay);
    }

    ScoutSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}
#endif