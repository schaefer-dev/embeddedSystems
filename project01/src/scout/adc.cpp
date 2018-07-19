#include <time.h>
#include "adc.h"
#include "spi_s.h"
#include "platform_s.h"

uint8_t adc_communicate(uint8_t sensor) {
    SPI_CLEAR_CLOCK();
    SELECT_ADC();
    delay_us(ADC_CS_DELAY);
    uint8_t buffer = sensor << 4;
    spi_transfer(&buffer, 1);
    UNSELECT_ADC();
    return  buffer;
}

uint8_t adc_get_sensor(uint8_t sensor) {
    adc_communicate(sensor);
    delay_us(ADC_COMPUTE_DELAY);
    return adc_communicate(0);
}