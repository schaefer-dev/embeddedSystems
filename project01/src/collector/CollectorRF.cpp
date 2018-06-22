//
// Created by Daniel Schäfer on 22.06.18.
//

#include "CollectorRF.h"
#include "CollectorSPI.h"
#include <Arduino.h>



void CollectorRF::initializeRFModule() {

    /* for every register in RF module:
     * select RF as slave
     * command address write register XX
     * at least one clock cycle delay
     * payload for register setup
     * deselect slave
     * short delay
     * */

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


    /* Recent changes: adress only written in PIPE 0 */
    for (int i = 0; i < 1; i ++) {

        CollectorSPI::slaveSelect(SLAVE_RF);
        CollectorSPI::readWriteSPI(42 + i); // write roboter receive adress in register 0A
        delayMicroseconds(command_delay);
        /* write  receive adress 14: 0x8527a891e2 LSByte to MSByte */
        CollectorSPI::readWriteSPI(226); // write e2
        delayMicroseconds(command_delay);
        CollectorSPI::readWriteSPI(145); // write 91
        delayMicroseconds(command_delay);
        CollectorSPI::readWriteSPI(168); // write a8
        delayMicroseconds(command_delay);
        CollectorSPI::readWriteSPI(39); // write 27
        delayMicroseconds(command_delay);
        CollectorSPI::readWriteSPI(133); // write 85
        CollectorSPI::slaveSelect(SLAVE_NONE);

        delay(delay_after_RF_select);
    }


    /* Recent changes: adress only written in PIPE 0, so not necessary here anymore */
    /* write LSB receive adress in register 0C to 0F
    for (int i = 0; i < 4; i ++){
        slaveSelect(SLAVE_RF);
        readWriteSPI(44 + i); // write roboter receive adress LSB in register 0C
        delayMicroseconds(command_delay);
        readWriteSPI(226); // write e2
        slaveSelect(SLAVE_NONE);
        delay(delay_after_RF_select);
    } */


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

    Serial1.print("RF Status Register: (");
    Serial1.print(statusRF);
    Serial1.println(")");

    return statusRF;
}




void CollectorRF::sendPongToReferee(uint16_t nonce){
    int refereeAdress[5];
    refereeAdress[4] = 0xe1;
    refereeAdress[3] = 0xf0;
    refereeAdress[2] = 0xf0;
    refereeAdress[1] = 0xf0;
    refereeAdress[0] = 0xf0;

    /* Write Referee adress to TX Register */
    write5ByteAdress(RF_REGISTER_TX_REG, refereeAdress);

    /* switch to TX mode */
    writeRegister(RF_REGISTER_CONFIG, 14);

    int commandArray[4] = {RF_COMMAND_W_TX_PAYLOAD, ((nonce % 256) + 1), (nonce/256), 0x51};
    sendCommandWithPayload(commandArray, 4);

    Serial1.print("PONG with nonce (");
    Serial1.print((nonce/256));
    Serial1.print((nonce%256) + 1);
    Serial1.println(") should send now");

}


void CollectorRF::sendCommandWithPayload(int *commandArray, int byteCount){

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

    CollectorSPI::readWriteSPI(RF_COMMAND_W_REGISTER | (RF_COMMAND_REGISTER_MASK & reg)); // write command for register
    delayMicroseconds(command_delay);
    CollectorSPI::readWriteSPI(setting);

    CollectorSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}


/* Write bytes to adress !!! THIS FUNCTION TAKES CARE OF INVERTING !!! */
void CollectorRF::write5ByteAdress(int reg, int* bytes){

    CollectorSPI::slaveSelect(SLAVE_RF);

    CollectorSPI::readWriteSPI(RF_COMMAND_W_REGISTER | (RF_COMMAND_REGISTER_MASK & reg)); // write command for register
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

    CollectorSPI::readWriteSPI(RF_COMMAND_R_REGISTER | (RF_COMMAND_REGISTER_MASK & reg)); // write command for register
    delayMicroseconds(command_delay);
    int output = CollectorSPI::readWriteSPI(RF_COMMAND_NOP);

    CollectorSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);

    return output;
}

void CollectorRF::readAdressRegister(uint8_t reg, int* outputArray){
    CollectorSPI::slaveSelect(SLAVE_RF);

    CollectorSPI::readWriteSPI(RF_COMMAND_R_REGISTER | (RF_COMMAND_REGISTER_MASK & reg)); // write command for register
    delayMicroseconds(command_delay);
    for (int i = 0; i < 5; i++) {
        outputArray[i] = CollectorSPI::readWriteSPI(RF_COMMAND_NOP);
        delayMicroseconds(command_delay);
    }

    CollectorSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}