#ifndef EMBEDDEDSYSTEMS18_RF_H
#define EMBEDDEDSYSTEMS18_RF_H

#include <stdint.h>
#include "nRF24L01.h"

#define RF_CE_PULSE_TIME 15 // in us
#define RF_RETRIES 5
#define RF_RETRY_TIME 3 // multiplier


/* core functions */

/* sends a commend without any data to the rf module */
void rf_send_command (uint8_t command);

/* writes variable size data into a register */
void rf_write_register_var (uint8_t reg, const uint8_t *data, uint8_t length);

/* reads variable size data from a register */
void rf_read_register_var (uint8_t reg, uint8_t *dest, uint8_t length);

/* writes variable size payload
 * ack determines whether W_TX_PAYLOAD or W_TX_PAYLAOD_NO_ACK is used */
void rf_write_payload (const uint8_t *src, uint8_t length, bool ack);

/* first reads the length of the available payload and then reads the payload itself
 * returns the payload length  (may be zero if nothing received) */
uint8_t rf_read_payload_dyn (uint8_t *dest);

/* enables the high-power modes of the module by setting CE high */
void rf_activate();

/* brings the module back to low-power modes by setting CE low */
void rf_standby ();


/* convenience functions */

/* writes a value to a register */
void rf_write_register(uint8_t reg, uint8_t value);

/* reads a value from a register */
uint8_t rf_read_register(uint8_t reg);

/* initiates sending of data (with ack). Does not wait for completion */
void rf_send (const uint8_t *src, uint8_t length);

/* sends data (with ack) and waits for completion of the process
 * returns true on success, false on failure (max retries) */
bool rf_send_blocking (const uint8_t *src, uint8_t length);

/* initiates sending of data (with ack) the specified address. Does not wait for completion */
void rf_sendto (uint8_t* address, const uint8_t *src, uint8_t length);

/* sends data (with ack) to the specified address and waits for completion of the process
 * returns true on success, false on failure (max retries) */
bool rf_sendto_blocking (uint8_t* address, const uint8_t *src, uint8_t length);

/* returns true if there is data to read */
bool rf_data_available();


/* ES funcions */

/* sets up the module with ES parameters and its unique address (matching the aruco) */
void rf_setup(uint8_t* address);

/* sets the module to standby and prepares it for transmitting */
void rf_stop_listening();

/* prepares the module for receiving and activates it */
void rf_start_listening();


#endif //EMBEDDEDSYSTEMS18_RF_H
