#include <string.h>
#include <util/delay.h>
#include "spi.h"
#include "rf.h"
#include "platform.h"

volatile uint8_t rf_intr = 0;
uint8_t rf_status;
static uint8_t rf_config;
uint8_t rf_referee_address [5] = {0xE1, 0xF0, 0xF0, 0xF0, 0xF0};

static void rf_transfer(uint8_t *buffer, uint8_t length) {
    SPI_CLEAR_CLOCK();
    SELECT_RF();
    spi_transfer(buffer, length);
    UNSELECT_RF();
    rf_status = *buffer;
}

void rf_write_register_var (uint8_t reg, const uint8_t *data, uint8_t length) {
    uint8_t buffer[6]; // longest register 5 bytes
    buffer[0] = (W_REGISTER | (REGISTER_MASK & reg));
    ASSERT(length <= 5, "Illegal register length!");
    memcpy((void*)(buffer + 1), data, length);
    rf_transfer(buffer, length + 1);
}

void rf_read_register_var (uint8_t reg, uint8_t *dest, uint8_t length) {
    uint8_t buffer[6];
    buffer[0] = (R_REGISTER | (REGISTER_MASK & reg));
    ASSERT(length <= 5, "Illegal register length!");
    rf_transfer(buffer, length + 1);
    memcpy(dest, (const void*)(buffer + 1), length);
}

void rf_send_command (uint8_t command) {
    uint8_t buffer = command;
    rf_transfer(&buffer, 1);
}

void rf_write_payload (const uint8_t *src, uint8_t length, bool ack) {
     uint8_t buffer[33];
    if (ack)
        buffer[0] = W_TX_PAYLOAD;
    else
        buffer[0] = W_TX_PAYLOAD_NO_ACK;
    memcpy((void*)(buffer + 1), src, length);
    rf_transfer(buffer, length + 1);
}

uint8_t rf_read_payload_dyn (uint8_t *dest) {
    uint8_t buffer[33];
    buffer[0] = R_RX_PL_WID;
    rf_transfer(buffer, 2);
    uint8_t length = buffer[1];
    if (0 < length && length < 33) {
        buffer[0] = R_RX_PAYLOAD;
        rf_transfer(buffer, length + 1);
        memcpy(dest, (const void*)(buffer + 1), length);
    }
    return length;
}

void rf_activate() {
    ENABLE_RF();
}

void rf_standby () {
    DISABLE_RF();
}


void rf_write_register(uint8_t reg, uint8_t value) {
    uint8_t buffer[2];
    buffer[0] = (W_REGISTER | (REGISTER_MASK & reg));
    buffer[1] = value;
    rf_transfer(buffer, 2);
}

uint8_t rf_read_register(uint8_t reg) {
    uint8_t buffer[2];
    buffer[0] = (R_REGISTER | (REGISTER_MASK & reg));
    rf_transfer(buffer, 2);
    return buffer[1];
}

void rf_send (const uint8_t *src, uint8_t length) {
    rf_write_payload(src, length, true);
    rf_activate();
    _delay_us(RF_CE_PULSE_TIME);
    rf_standby();
}

bool rf_send_blocking (const uint8_t *src, uint8_t length) {
    rf_send(src, length);
    _delay_us(2*130 + 350 + 80); // 130us PLL lock, roughly 350 us for 1Mbit/s and 32 bytes payload, roughly 80 us for ACK without payload
    for(uint8_t i = 0; i < RF_RETRIES + 1; i++) {
        _delay_us(RF_RETRY_TIME * 250 + 250 + 130 + 350);
        if(!rf_intr)
            continue;
        rf_intr = 0;
        rf_send_command(RF24_NOP);
        if (rf_status & (1 << MAX_RT)) {
            rf_send_command(FLUSH_TX);
            rf_write_register(NRF_STATUS, (1 << MAX_RT)); // clear interrupt
            return false;
        }
        if (rf_status & (1 << TX_DS)) {
            rf_write_register(NRF_STATUS, (1 << TX_DS)); // clear interrupt
            return true;
        }
    }
    Serial1.println("Something went wrong in RF module send!");
    //ASSERT(false, "RF module behaved unexpectedly on send!");
}

void rf_sendto (uint8_t* address, const uint8_t *src, uint8_t length) {
    rf_write_register_var(RX_ADDR_P0, address, 5);
    rf_write_register_var(TX_ADDR, address, 5);
    rf_send(src, length);
}

bool rf_sendto_blocking (uint8_t* address, const uint8_t *src, uint8_t length) {
    rf_write_register_var(RX_ADDR_P0, address, 5);
    rf_write_register_var(TX_ADDR, address, 5);
    return rf_send_blocking(src, length);
}

bool rf_data_available() {
    static bool sure_emtpy = false;
    if (sure_emtpy && !rf_intr)
        return false;

    uint8_t fifo_status = rf_read_register(FIFO_STATUS);
    if (fifo_status & (1 << RX_EMPTY)) {
        sure_emtpy = true;
        return false;
    } else {
        rf_intr = 0;
        rf_write_register(NRF_STATUS, (1 << RX_DR)); // clear possible interrupt
        sure_emtpy = false;
        return true;
    }
}


void rf_setup(uint8_t* address) {
    rf_write_register(FEATURE, (1 << EN_DPL) | (1 << EN_ACK_PAY) | (1 << EN_DYN_ACK));
    rf_config = (1 << EN_CRC) | (1 << CRCO) | (0 << PRIM_RX) | (1 << PWR_UP);
    rf_write_register(RF_SETUP, (0 << RF_DR_HIGH) | (0 << RF_DR_LOW) | (0 << RF_PWR_HIGH) | (1 << RF_PWR_LOW));
    rf_write_register(RF_CH, 111);
    rf_write_register(SETUP_RETR, (RF_RETRY_TIME << 4) | (RF_RETRIES));
    rf_send_command(FLUSH_RX);
    rf_send_command(FLUSH_TX);

    rf_write_register_var(RX_ADDR_P0, rf_referee_address, 5);
    rf_write_register_var(TX_ADDR, rf_referee_address, 5);
    rf_write_register_var(RX_ADDR_P1, address, 5);
    rf_write_register(EN_RXADDR, (1 << ERX_P0) | (1 << ERX_P1));
    rf_write_register(DYNPD, (1 << DPL_P0) | (1 << DPL_P1));

    rf_write_register(NRF_CONFIG, rf_config);
}

void rf_stop_listening() {
    rf_standby();
    rf_write_register(NRF_CONFIG, rf_config & ~(1 << PRIM_RX));
    rf_write_register(EN_RXADDR, (1 << ERX_P0) | (1 << ERX_P1));
}

void rf_start_listening() {
    rf_write_register(EN_RXADDR, (0 << ERX_P0) | (1 << ERX_P1));
    rf_write_register(NRF_CONFIG, rf_config | (1 << PRIM_RX));
    rf_activate();
}