//
// Created by Daniel Schäfer on 22.06.18.
//

#include "CollectorRF.h"
#include "CollectorSPI.h"
#include <Arduino.h>
#include "main.h"
#include "CollectorMonitor.h"
#include "spi_c.h"
#include "platform_c.h"
#include <Zumo32U4Motors.h>


uint8_t CollectorRF::refereeAdress[5];
uint8_t CollectorRF::scoutAdress[5];
uint8_t CollectorRF::collectorAdress[5];
int CollectorRF::channelChanges;


void CollectorRF::initializeRFModule() {

    /* for every register in RF module:
     * select RF as slave
     * command address write register XX
     * at least one clock cycle delay
     * payload for register setup
     * deselect slave
     * short delay
     * */

    channelChanges = 0;

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
    collectorAdress[0] = 0xe2;

    /* TODO: some of this is default set already, so can be optimized */

    // write command register 02: enable all data pipes
    writeRegister(0x00000002, 63);

    // write command register 04: enable 10 retries w delay 2ms
    writeRegister(0x00000004, 138);

    // set channel to 111
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


    write5ByteAdress(RF_REGISTER_RX_ADDR_P0, collectorAdress);


    // write register 00: enable crc 16 bit, pwr up, rx mode
    writeRegister(0x00000000, 15);

    /* drive RF module enable pin, apparently this should happen
     * at the very end of configuration, but not 100% sure */
    PORTC |= (1 << PIN_RF_ENABLE_C);
    delayMicroseconds(30);

}

#ifdef COLLECTOR_DEBUG

void CollectorRF::debug_RFModule() {
    int output = 0;
    for (int i = 0; i < 30; i++) {
        output = readRegister(i);

        Serial1.print("Register ");
        Serial1.print(i);
        Serial1.print(" = (");
        Serial1.print(output);
        Serial1.println(")");

        delay(20);

    }

    uint8_t adressArray[5];
    readAdressRegister(0x0A, adressArray);
    Serial1.print("ADDR Register: 0A (");
    for (int i = 0; i < 5; i++) {
        Serial1.print(adressArray[i]);
    }
    Serial1.println(")");

    readAdressRegister(0x10, adressArray);
    Serial1.print("ADDR Register: 10 (");
    for (int i = 0; i < 5; i++) {
        Serial1.print(adressArray[i]);
    }
    Serial1.println(")");

}

#endif


int CollectorRF::queryRFModule() {
    SELECT_RF();
    //unsigned int payload = 255;
    uint8_t buffer[1] = {25};
    // unsigned int statusRF = CollectorSPI::readWriteSPI(payload);
    spi_transfer( buffer, 1 );
    UNSELECT_RF();

    //Serial1.print("RF Status Register: (");
    //Serial1.print(statusRF);
    //Serial1.println(")");

    return buffer[0];
}


void CollectorRF::processReceivedMessage(CollectorState *collectorState) {
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

#ifdef COLLECTOR_DEBUG
    Serial1.print("Message: ");

    for (int i = 0; i < answerArray[0]; i++) {
        Serial1.print(payloadArray[i]);
    }
    Serial1.print("\n");
#endif
    

    switch (payloadArray[0]) {

        case 0x01:
            /* Config case -> simply set Team channel according to received message */
            // TODO
            break;

        case 0x30: {
            /* Scout position update */
            float angle = payloadArray[1] * 256 + payloadArray[2];
            float posX = payloadArray[3] * 256 + payloadArray[4];
            float posY = payloadArray[5] * 256 + payloadArray[6];

            collectorState->scoutPositionMessage(angle, posX, posY);
        }
            break;

        case 0x42:
            // Hello
            Serial1.print("Ref sent HELLO\n");
            break;

        case 0x43: {
            // Config
            uint8_t channel = payloadArray[1];

            Serial1.println("New channel");
            Serial1.println(channel);

            // switch to new comminucation channel
            writeRegister(0x00000005, channel);
            flushRXTX();
            collectorState->configurationReceived = true;
        }
            break;

        case 0x44:
            // Go
            collectorState->gameStarted = true;
            break;

        case 0x45:
            /* END */
        {
            uint8_t outcome = payloadArray[1];
            switch (outcome) {
                case 0: // Loss
                    Serial1.print("We lost :(");
                    break;
                case 1: // Win
                    Serial1.print("We won :)");
                    collectorState->danceBlocking();
                    break;
                case 2: // Tie
                    Serial1.print("We tied :/");
                    break;
                case 3: // Disqualified
                    Serial1.print("We were disqualified :°(");
                    break;
                default:
                    break;
            }
        }
            break;

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
            receivePosUpdate(payloadArray[1] * 256 + payloadArray[2],
                             payloadArray[3] * 256 + payloadArray[4],
                             payloadArray[5] * 256 + payloadArray[6]);
            break;

        case 0x61:
            /* Out of Bounds Message */
            collectorState->drivingDisabled = false;
            collectorState->outOfBoundsMessage();
            receivePosUpdate(payloadArray[1] * 256 + payloadArray[2],
                             payloadArray[3] * 256 + payloadArray[4],
                             payloadArray[5] * 256 + payloadArray[6]);
            collectorState->destinationReached = false;
            collectorState->destinationX = ARENA_SIZE_X / 2;
            collectorState->destinationY = ARENA_SIZE_Y / 2;
            break;
        case 0x62:
            /* Collision case -> force robot to drive backwards (with low speed) for 500ms*/
            collectorState->unhandledCollisionFlag = true;
            receivePosUpdate(payloadArray[1] * 256 + payloadArray[2],
                             payloadArray[3] * 256 + payloadArray[4],
                             payloadArray[5] * 256 + payloadArray[6]);
            break;

        case 0x66: {
            /* case for monitor status request */
            char status[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
#ifdef COLLECTOR_MONITOR
            CollectorMonitor::getStatus(status);
#endif
            char command[1] = {0x67};
            Serial1.print(command);
            Serial1.print(status);
            Serial1.print(0x00);
            Serial1.flush();
            break;
        }

        case 0x69: {
            // Harvest Position
            uint8_t value = payloadArray[2];

            int posX = (payloadArray[3] * 256 + payloadArray[4]) / 10;
            int posY = (payloadArray[5] * 256 + payloadArray[6]) / 10;


            Serial1.print("Val: ");
            Serial1.println(value);
            Serial1.print("X: ");
            Serial1.println(posX);
            Serial1.print("Y: ");
            Serial1.println(posY);

            collectorState->harvestPositionMessage(value, posX, posY);
        }
            break;

        case 0x70:
            /* MESSAGE case */
            break;

        case 0x80:
#ifdef COLLECTOR_DEBUG
            Serial1.println("Message from scout arrived, echo performing ...");
#endif
            /* case for RELAY, scount sends and collector echos sends message back to scout */
            payloadArray[0] = 0x81;
            // Give scout time to switch back into Listening Mode
            delay(100);
            sendMessageTo(scoutAdress, payloadArray, answerArray[0]);
            break;

        default: {
            Serial1.println("Illegal Message Identifer");
        }
    }
}

void CollectorRF::sendMessageTo(uint8_t *receiverAdress, uint8_t *payloadArray, int payloadArrayLength) {
    /* Write Referee adress to TX Register */
    write5ByteAdress(RF_REGISTER_TX_REG, receiverAdress);
    /* Write Referee adress to TX Register */
    write5ByteAdress(RF_REGISTER_RX_ADDR_P0, receiverAdress);
#ifdef COLLECTOR_DEBUG
    Serial1.print("DEBUG: adresses written");
    CollectorRF::debug_RFModule();
#endif
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

    // delay(10);
    // flushRXTX();
    while (true) {
        status = queryRFModule();

        /* either message sent or max retries reached case */
        if (((1 << 4) & status) > 0) {
#ifdef COLLECTOR_DEBUG
            Serial1.println("Message sending maxRetries");
#endif
            break;
        }


        if (((1 << 5) & status) > 0) {
#ifdef COLLECTOR_DEBUG
            Serial1.println("Message sent succesfully");
#endif
            break;
        }

        /* in case something goes wrong cancel after timeout received */
        if (millis() - timeout > COLLECTOR_RF_MESSAGE_SEND_TIMEOUT_MS) {
#ifdef COLLECTOR_DEBUG
            Serial1.println("Manual sending timeout");
#endif
            flushRXTX();
            break;
        }
    }

#ifdef COLLECTOR_MONITOR
    if (payloadArray[0] == 0x51) {
        /* the message was the pong response, log it */
        CollectorMonitor::logPongCollector();
    }
#endif

    /* clear received status */
    writeRegister(RF_REGISTER_STATUS, (1 << 4) | (1 << 5));

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


void CollectorRF::sendCommandWithPayload(uint8_t *commandArray, int byteCount) {

    SELECT_RF();

    delayMicroseconds(command_delay);

    spi_transfer(commandArray, byteCount);
    /*
    for (int i = 0; i < byteCount; i++) {
        CollectorSPI::readWriteSPI(commandArray[i]);
        delayMicroseconds(command_delay);
    }*/
    UNSELECT_RF();
    delay(delay_after_RF_select);
}


void CollectorRF::getCommandAnswer(uint8_t *answerArray, int byteCount, int8_t command) {

    SELECT_RF();

    uint8_t commandArray[1] = {command};
    spi_transfer(commandArray, 1);
    //CollectorSPI::readWriteSPI(command); // write command for register
    delayMicroseconds(command_delay);

    spi_transfer(answerArray, byteCount);
    /*
    for (int i = 0; i < byteCount; i++) {
        answerArray[i] = CollectorSPI::readWriteSPI(RF_COMMAND_NOP);
        delayMicroseconds(command_delay);
    } */

    UNSELECT_RF();
    delay(delay_after_RF_select);
}


void CollectorRF::writeRegister(uint8_t reg, uint8_t setting) {

    if (reg == 0x00000005){
        channelChanges += 1;

        if (channelChanges > 2){
            Serial1.println("TEAM CHANNEL ONLY ALLOWED TO BE SET AT MOST TWICE!!!");
            return;
        }
    }

    SELECT_RF();

    uint8_t registerArray[1] = { RF_COMMAND_W_REGISTER | (RF_MASK_REGISTER & reg) };
    spi_transfer(registerArray, 1);
    //CollectorSPI::readWriteSPI(RF_COMMAND_W_REGISTER | (RF_MASK_REGISTER & reg)); // write command for register
    delayMicroseconds(command_delay);

    uint8_t settingArray[1] = { setting };
    spi_transfer(settingArray, 1);
    //CollectorSPI::readWriteSPI(setting);

    UNSELECT_RF();
    delay(delay_after_RF_select);
}


/* Write bytes to adress !!! THIS FUNCTION TAKES CARE OF INVERTING !!! */
void CollectorRF::write5ByteAdress(int reg, uint8_t *bytes) {

    uint8_t byteArray[5];
    int j = 0;
    for (int i = 4; i >= 0; i--) {
        byteArray[j] = bytes[i];
        j++;
        /*
        CollectorSPI::readWriteSPI(bytes[i]);
        delayMicroseconds(command_delay);
         */
    }


    SELECT_RF();

    uint8_t registerArray[1] = { RF_COMMAND_W_REGISTER | (RF_MASK_REGISTER & reg) };
    spi_transfer(registerArray, 1);
    // CollectorSPI::readWriteSPI(RF_COMMAND_W_REGISTER | (RF_MASK_REGISTER & reg)); // write command for register
    delayMicroseconds(command_delay);

    spi_transfer(byteArray, 5);

    UNSELECT_RF();
    delay(delay_after_RF_select);
}


int CollectorRF::readRegister(uint8_t reg) {

    SELECT_RF();

    uint8_t registerArray[1] = { RF_COMMAND_R_REGISTER | (RF_MASK_REGISTER & reg) };
    spi_transfer(registerArray, 1);
    // CollectorSPI::readWriteSPI(RF_COMMAND_R_REGISTER | (RF_MASK_REGISTER & reg)); // write command for register
    delayMicroseconds(command_delay);
    uint8_t outputArray[1];
    spi_transfer(outputArray, 1);
    // int output = CollectorSPI::readWriteSPI(RF_COMMAND_NOP);

    UNSELECT_RF();
    delay(delay_after_RF_select);

    return outputArray[0];
}

void CollectorRF::readAdressRegister(uint8_t reg, uint8_t *outputArray) {
    SELECT_RF();

    uint8_t registerArray[1] = { RF_COMMAND_R_REGISTER | (RF_MASK_REGISTER & reg) };
    spi_transfer(registerArray, 1);
    // CollectorSPI::readWriteSPI(RF_COMMAND_R_REGISTER | (RF_MASK_REGISTER & reg)); // write command for register
    delayMicroseconds(command_delay);

    /*
    for (int i = 0; i < 5; i++) {
        outputArray[i] = CollectorSPI::readWriteSPI(RF_COMMAND_NOP);
        delayMicroseconds(command_delay);
    }*/
    spi_transfer(outputArray, 5);

    UNSELECT_RF();
    delay(delay_after_RF_select);
}
