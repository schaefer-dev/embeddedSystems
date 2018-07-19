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
#include "spi_s.h"
#include "platform_s.h"


/* If ROBOT_SIMULATOR is set instead of communicating over the Module with SPI
 * We always wait for a message with content "0x01" and after that we wait for
 * 32 bytes sent over serial port in a blocking manner.
 *
 * It is important to wait blocking to avoid that the message is split.
 */


uint8_t ScoutRF::refereeAdress[5];
uint8_t ScoutRF::scoutAdress[5];
uint8_t ScoutRF::collectorAdress[5];
uint8_t ScoutRF::teamChannel;

void ScoutRF::initializeRFModule() {

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
    writeRegister(0x00000002, 63);

    // write command register 04: enable 10 retries w delay 2ms
    writeRegister(0x00000004, 138);

    // write register 05: set channel to 111
    writeRegister(0x00000005, 111);

    // write register 06: data rate 1 mbps, max power
    writeRegister(0x00000006, 6);



    /* write command register 11 to 16 to maximum RX payload (32 Bytes) */
    for (int i = 0x00000011; i < 0x00000017; i++) {
        writeRegister(i, 32);
    }

    // write register 1D: enable dyn payload, dyn ack
    writeRegister(0x0000001C, 63);

    // write register 1D: enable dyn payload, dyn ack
    writeRegister(0x0000001D, 7);


    write5ByteAdress(RF_REGISTER_RX_ADDR_P0, scoutAdress);


    // write register 00: enable crc 16 bit, pwr up, rx mode
    writeRegister(0x00000000, 15);

    flushRXTX();

    /* drive RF module enable pin, apparently this should happen
     * at the very end of configuration, but not 100% sure */
    PORTD |= (1 << PIN_RF_ENABLE_D);
    delayMicroseconds(30);
}


void ScoutRF::flushRXTX() {
    /* flush TX and RX */
    uint8_t payload[1] = {RF_COMMAND_FLUSH_RX};
    sendCommandWithPayload(payload, 1);
    payload[0] = RF_COMMAND_FLUSH_TX;
    sendCommandWithPayload(payload, 1);
}


void ScoutRF::debug_RFModule() {
    int output = 0;
    for (int i = 0; i < 30; i++) {
        output = readRegister(i);

        ScoutSerial::serialWrite("Register ", 9);
        ScoutSerial::serialWrite8BitHex(i);
        ScoutSerial::serialWrite(" = (", 4);
        ScoutSerial::serialWrite8BitBinary(output);
        ScoutSerial::serialWrite(")\n", 2);
        delay(20);

    }

    uint8_t adressArray[5];
    readAdressRegister(0x0A, adressArray);
    ScoutSerial::serialWrite("ADDR Register: 0A (", 19);
    for (int i = 0; i < 5; i++) {
        ScoutSerial::serialWrite8BitHex(adressArray[i]);
    }
    ScoutSerial::serialWrite(")\n", 2);

    readAdressRegister(0x0B, adressArray);
    ScoutSerial::serialWrite("ADDR Register: 0B (", 19);
    for (int i = 0; i < 5; i++) {
        ScoutSerial::serialWrite8BitHex(adressArray[i]);
    }
    ScoutSerial::serialWrite(")\n", 2);
}


int ScoutRF::queryRFModule() {
    SELECT_RF();
    //unsigned int payload = 255;
    uint8_t buffer[1] = {255};

    //unsigned int statusRF = ScoutSPI::readWriteSPI(payload);
    spi_transfer( buffer, 1 );
    //ScoutSerial::serialWrite("RF Status Register: (", 21);
    //ScoutSerial::serialWrite8Bit(statusRF);
    //ScoutSerial::serialWrite(")\n", 2);
    UNSELECT_RF();

    return buffer[0];
}

void ScoutRF::processReceivedMessage(ScoutState *scoutState) {
    /* case for Message arrived */
    uint8_t answerArray[1];
    ScoutRF::getCommandAnswer(answerArray, 1, RF_COMMAND_R_RX_PL_WID);

    /* if no next payload there */
    if (answerArray[0] == 0) {
        return;
    }

    /* read message from pipe */
    uint8_t payloadArray[answerArray[0]];
    ScoutRF::getCommandAnswer(payloadArray, answerArray[0], RF_COMMAND_R_RX_PAYLOAD);

    ScoutSerial::serialWrite("Message: ", 9);

    for (int i = 0; i < answerArray[0]; i++) {
        ScoutSerial::serialWrite8BitHex(payloadArray[i]);
    }
    ScoutSerial::serialWrite(" (only first 3 bytes displayed)\n", 32);


    switch (payloadArray[0]) {
        case 0x01:
            /* Config case -> simply set Team channel according to received message */
            // TODO
            break;

        case 0x02:
            /* Collision case -> force robot to drive backwards (with low speed) for 500ms*/
            scoutState->unhandledCollisionFlag = true;
            ScoutSerial::serialWrite("Collision received!\n", 20);
            receivePosUpdate(payloadArray[1] * 256 + payloadArray[2],
                             payloadArray[3] * 256 + payloadArray[4],
                             payloadArray[5] * 256 + payloadArray[6]);
            break;

        case 0x30:
            /* received Position of Teammate */
            scoutState->collectorAngle = (float) (payloadArray[1] * 256 + payloadArray[2]);
            scoutState->collectorX = (float) (payloadArray[3] * 256 + payloadArray[4]);
            scoutState->collectorX = (float) (payloadArray[5] * 256 + payloadArray[6]);

            break;

        case 0x42:
            // Hello
            ScoutSerial::serialWrite("Ref sent HELLO\n", 15);
            break;

        case 0x43: {
            // Config
            uint8_t channel = payloadArray[1];

            ScoutSerial::serialWrite("New channel: ", 13);
            ScoutSerial::serialWriteInt(channel);

            // switch to new comminucation channel
            writeRegister(0x00000005, channel);
            flushRXTX();
            scoutState->configurationReceived = true;
        }
            break;

        case 0x44:
            // Go
            scoutState->gameStarted = true;
            break;

        case 0x45:
            // End
            //TODO
            break;

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
            scoutState->drivingDisabled = false;
            ScoutSerial::serialWrite("Position update received!\n", 26);
            receivePosUpdate(payloadArray[1] * 256 + payloadArray[2],
                             payloadArray[3] * 256 + payloadArray[4],
                             payloadArray[5] * 256 + payloadArray[6]);
            break;

        case 0x61:
            ScoutSerial::serialWrite("OOB POS update received!\n", 26);
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

            ScoutSerial::serialWrite("Message: \n",11);
            for (int i = 0; i < answerArray[0]; i++) {
                char buffer[1] = {payloadArray[i]};
                ScoutSerial::serialWrite(buffer, 1);
            }
            ScoutSerial::serialWrite("\n", 1);
            break;

        case 0x66: {
            /* case for monitor status request */
            char status[9];
#ifdef SCOUT_MONITOR
            ScoutMonitor::getStatus(status);
#endif
            char command[1] = {0x67};
            ScoutSerial::serialWrite(command, 1);
            ScoutSerial::serialWrite(status, 9);
            break;
        }

        default:
            ScoutSerial::serialWrite("Illegal Message Identifer\n", 26);
    }
}


void ScoutRF::sendMessageTo(uint8_t *receiverAdress, uint8_t *payloadArray, int payloadArrayLength) {

    /* Write Referee adress to TX Register */
    write5ByteAdress(RF_REGISTER_TX_REG, receiverAdress);
    /* Write Referee adress to TX Register */
    write5ByteAdress(RF_REGISTER_RX_ADDR_P0, receiverAdress);

    /* switch to TX mode */
    writeRegister(RF_REGISTER_CONFIG, 14);

    uint8_t commandArray[payloadArrayLength + 1];
    commandArray[0] = RF_COMMAND_W_TX_PAYLOAD;
    for (int i = 1; i < payloadArrayLength + 1; i++) {
        commandArray[i] = payloadArray[i - 1];
    }

    sendCommandWithPayload(commandArray, payloadArrayLength + 1);
    int status = 0;
    long timeout = millis();

    //delay(10); d

    while (true) {
        delay(10);
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
        if (millis() - timeout > SCOUT_MESSAGE_SEND_TIMEOUT_MS) {
#ifdef DEBUG
            ScoutSerial::serialWrite("Manual sending timeout\n", 23);
#endif
            flushRXTX();
            break;
        }
    }


#ifdef SCOUT_MONITOR
    if (payloadArray[0] == 0x51) {
        /* the message was the pong response, log it */
        ScoutMonitor::logPongScout();
    }
#endif

    /* clear received status */
    writeRegister(RF_REGISTER_STATUS, (1 << 4) | (1 << 5));

    /* Write Scout adress in RX Register Pipe 0 */
    write5ByteAdress(RF_REGISTER_RX_ADDR_P0, scoutAdress);

    /* switch back to RX mode */
    writeRegister(RF_REGISTER_CONFIG, 15);
}


void ScoutRF::sendCommandWithPayload(uint8_t *commandArray, int byteCount) {

    SELECT_RF();

    spi_transfer(commandArray, byteCount);
    /*
    for (int i = 0; i < byteCount; i++) {
        ScoutSPI::readWriteSPI(commandArray[i]);
        delayMicroseconds(command_delay);
    } */

    UNSELECT_RF();
}


void ScoutRF::getCommandAnswer(uint8_t *answerArray, int byteCount, int8_t command) {

    SELECT_RF();
    uint8_t commandArray[1] = {command};
    spi_transfer(commandArray, 1);

    //ScoutSPI::readWriteSPI(command); // write command for register
    delayMicroseconds(command_delay);
    spi_transfer(answerArray, byteCount);

    /*
    for (int i = 0; i < byteCount; i++) {
        answerArray[i] = ScoutSPI::readWriteSPI(RF_COMMAND_NOP);
        delayMicroseconds(command_delay);
    }*/
    UNSELECT_RF();
    delay(delay_after_RF_select);
}


void ScoutRF::writeRegister(uint8_t reg, uint8_t setting) {

    SELECT_RF();

    uint8_t registerArray[1] = { RF_COMMAND_W_REGISTER | (RF_MASK_REGISTER & reg) };
    spi_transfer(registerArray, 1);
    //ScoutSPI::readWriteSPI(RF_COMMAND_W_REGISTER | (RF_MASK_REGISTER & reg)); // write command for register
    delayMicroseconds(command_delay);

    uint8_t settingArray[1] = { setting };
    spi_transfer(settingArray, 1);
    //ScoutSPI::readWriteSPI(setting);

    UNSELECT_RF();
    delay(delay_after_RF_select);
}


/* Write bytes to adress !!! THIS FUNCTION TAKES CARE OF INVERTING !!! */
void ScoutRF::write5ByteAdress(int reg, uint8_t *bytes) {

    uint8_t byteArray[5];
    int j = 0;
    for (int i = 4; i >= 0; i--) {
        byteArray[j] = bytes[i];
        j++;
        /*
        ScoutSPI::readWriteSPI(bytes[i]);
        delayMicroseconds(command_delay);
         */
    }

    SELECT_RF();

    uint8_t registerArray[1] = { RF_COMMAND_W_REGISTER | (RF_MASK_REGISTER & reg) };
    spi_transfer(registerArray, 1);
    // ScoutSPI::readWriteSPI(RF_COMMAND_W_REGISTER | (RF_MASK_REGISTER & reg)); // write command for register
    delayMicroseconds(command_delay);
    
    UNSELECT_RF();
    delay(delay_after_RF_select);
}


int ScoutRF::readRegister(uint8_t reg) {

    SELECT_RF();

    uint8_t registerArray[1] = { RF_COMMAND_R_REGISTER | (RF_MASK_REGISTER & reg) };
    spi_transfer(registerArray, 1);
    // ScoutSPI::readWriteSPI(RF_COMMAND_R_REGISTER | (RF_MASK_REGISTER & reg)); // write command for register
    delayMicroseconds(command_delay);

    uint8_t outputArray[1];
    spi_transfer(outputArray, 1);
    // int output = ScoutSPI::readWriteSPI(RF_COMMAND_NOP);

    UNSELECT_RF();
    delay(delay_after_RF_select);

    return outputArray[0];
}


void ScoutRF::readAdressRegister(uint8_t reg, uint8_t *outputArray) {
    SELECT_RF();

    uint8_t registerArray[1] = { RF_COMMAND_R_REGISTER | (RF_MASK_REGISTER & reg) };
    spi_transfer(registerArray, 1);
    // ScoutSPI::readWriteSPI(RF_COMMAND_R_REGISTER | (RF_MASK_REGISTER & reg)); // write command for register
    delayMicroseconds(command_delay);

    /*
    for (int i = 0; i < 5; i++) {
        outputArray[i] = ScoutSPI::readWriteSPI(RF_COMMAND_NOP);
        delayMicroseconds(command_delay);
    }*/
    spi_transfer(outputArray, 5);

    UNSELECT_RF();
    delay(delay_after_RF_select);
}

void ScoutRF::setTeamChannel(uint8_t teamChannel) {
    /* drive RF module enable pin, might not be necessary */
    PORTD &= ~(1 << PIN_RF_ENABLE_D);
    delayMicroseconds(30);

    // write register 05: set channel to 111
    writeRegister(0x00000005, teamChannel);

    /* drive RF module enable pin, might not be necessary */
    PORTD |= (1 << PIN_RF_ENABLE_D);
    delayMicroseconds(30);

}
