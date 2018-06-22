//
// Created by Daniel Schäfer on 22.06.18.
//

#include "ScoutRF.h"
#include "ScoutSPI.h"
#include "ScoutSerial.h"
#include <OrangutanTime.h>
#include "avr/io.h"


void ScoutRF::initializeRFModule() {

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

        ScoutSPI::slaveSelect(SLAVE_RF);
        ScoutSPI::readWriteSPI(42 + i); // write roboter receive adress in register 0A
        delayMicroseconds(command_delay);
        /* write  receive adress 14: 0x8527a891e2 LSByte to MSByte */
        ScoutSPI::readWriteSPI(226); // write e2
        delayMicroseconds(command_delay);
        ScoutSPI::readWriteSPI(145); // write 91
        delayMicroseconds(command_delay);
        ScoutSPI::readWriteSPI(168); // write a8
        delayMicroseconds(command_delay);
        ScoutSPI::readWriteSPI(39); // write 27
        delayMicroseconds(command_delay);
        ScoutSPI::readWriteSPI(133); // write 85
        ScoutSPI::slaveSelect(SLAVE_NONE);

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
    PORTD |= (1 << PIN_RF_ENABLE_D);
    delayMicroseconds(30);

}

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

    int adressArray[5];
    readAdressRegister(0x000A, adressArray);
    ScoutSerial::serialWrite("ADDR Register: 0A (", 19);
    for (int i=0; i < 5; i++) {
        ScoutSerial::serialWrite8BitHex(adressArray[i]);
    }
    ScoutSerial::serialWrite(")\n", 2);

    readAdressRegister(0x000B, adressArray);
    ScoutSerial::serialWrite("ADDR Register: 0B (", 19);
    for (int i=0; i < 5; i++) {
        ScoutSerial::serialWrite8BitHex(adressArray[i]);
    }
    ScoutSerial::serialWrite(")\n", 2);

}


int ScoutRF::queryRFModule(){
    ScoutSPI::slaveSelect(SLAVE_RF);
    unsigned int payload = 255;
    unsigned int statusRF = ScoutSPI::readWriteSPI(payload);
    ScoutSPI::slaveSelect(SLAVE_NONE);

    ScoutSerial::serialWrite("RF Status Register: (", 21);
    ScoutSerial::serialWrite8Bit(statusRF);
    ScoutSerial::serialWrite(")\n", 2);

    return statusRF;
}




void ScoutRF::sendPongToReferee(uint16_t nonce){
    int refereeAdress[5];
    refereeAdress[4] = 0xe1;
    refereeAdress[3] = 0xf0;
    refereeAdress[2] = 0xf0;
    refereeAdress[1] = 0xf0;
    refereeAdress[0] = 0xf0;

    /* Write Referee adress to TX Register */
    write5ByteAdress(TX_REGISTER, refereeAdress);

    /* switch to TX mode */
    writeRegister(RF_CONFIG, 14);

    int commandArray[4] = {W_TX_PAYLOAD, ((nonce % 256) + 1), (nonce/256), 0x51};
    sendCommandWithPayload(commandArray, 4);

    ScoutSerial::serialWrite("PONG with nonce (", 17);
    ScoutSerial::serialWrite8BitHex((nonce/256));
    ScoutSerial::serialWrite8BitHex((nonce%256) + 1);

    ScoutSerial::serialWrite(") should send now\n", 18);
}


void ScoutRF::sendCommandWithPayload(int *commandArray, int byteCount){

    ScoutSPI::slaveSelect(SLAVE_RF);

    delayMicroseconds(command_delay);
    for (int i = 0; i < byteCount; i++) {
        ScoutSPI::readWriteSPI(commandArray[i]);
        delayMicroseconds(command_delay);
    }
    ScoutSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}



void ScoutRF::getCommandAnswer(int *answerArray, int byteCount, int8_t command){

    ScoutSPI::slaveSelect(SLAVE_RF);

    ScoutSPI::readWriteSPI(command); // write command for register
    delayMicroseconds(command_delay);
    for (int i = 0; i < byteCount; i++) {
        answerArray[i] = ScoutSPI::readWriteSPI(NOP);
        delayMicroseconds(command_delay);
    }
    ScoutSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}

void ScoutRF::writeRegister(uint8_t reg, uint8_t setting){

    ScoutSPI::slaveSelect(SLAVE_RF);

    ScoutSPI::readWriteSPI(W_REGISTER | (REGISTER_MASK & reg)); // write command for register
    delayMicroseconds(command_delay);
    ScoutSPI::readWriteSPI(setting);

    ScoutSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}


/* Write bytes to adress !!! THIS FUNCTION TAKES CARE OF INVERTING !!! */
void ScoutRF::write5ByteAdress(int reg, int* bytes){

    ScoutSPI::slaveSelect(SLAVE_RF);

    ScoutSPI::readWriteSPI(W_REGISTER | (REGISTER_MASK & reg)); // write command for register
    delayMicroseconds(command_delay);
    for (int i = 4; i >= 0; i--){
        ScoutSPI::readWriteSPI(bytes[i]);
        delayMicroseconds(command_delay);
    }

    ScoutSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}


int ScoutRF::readRegister(uint8_t reg){

    ScoutSPI::slaveSelect(SLAVE_RF);

    ScoutSPI::readWriteSPI(R_REGISTER | (REGISTER_MASK & reg)); // write command for register
    delayMicroseconds(command_delay);
    int output = ScoutSPI::readWriteSPI(NOP);

    ScoutSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);

    return output;
}

void ScoutRF::readAdressRegister(uint8_t reg, int* outputArray){
    ScoutSPI::slaveSelect(SLAVE_RF);

    ScoutSPI::readWriteSPI(R_REGISTER | (REGISTER_MASK & reg)); // write command for register
    delayMicroseconds(command_delay);
    for (int i = 0; i < 5; i++) {
        outputArray[i] = ScoutSPI::readWriteSPI(NOP);
        delayMicroseconds(command_delay);
    }

    ScoutSPI::slaveSelect(SLAVE_NONE);
    delay(delay_after_RF_select);
}