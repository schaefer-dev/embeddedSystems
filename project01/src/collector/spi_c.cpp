#include <util/delay.h>

#include "platform_c.h"
#include "spi_c.h"

void spi_transfer(uint8_t *buffer, uint8_t length) {
    SPI_CLEAR_CLOCK();
    for (int byte = 0; byte < length; byte++) {
        for (int bit = 7; bit >= 0; bit--) {
            uint8_t value = (uint8_t) ((buffer[byte] >> bit) & 1);
            if (value) {
                SPI_SET_MOSI();
            } else {
                SPI_CLEAR_MOSI();
            }
            _delay_us(SPI_DELAY);
            SPI_SET_CLOCK();
            if (SPI_READ_MISO()) {
                buffer[byte] |= (1 << bit);
            } else {
                buffer[byte] &= ~(1 << bit);
            }
            _delay_us(SPI_DELAY);
            SPI_CLEAR_CLOCK();
        }
    }
}


