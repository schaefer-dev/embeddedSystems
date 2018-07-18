#ifndef EMBEDDED_SYSTEMS18_SPI_H
#define EMBEDDED_SYSTEMS18_SPI_H

#include <stdint.h>

#define SPI_DELAY 1  // 1 us

/* sends the buffer to a SPI device while simultaneously reading into the buffer */
void spi_transfer(uint8_t *buffer, uint8_t length);

#endif
