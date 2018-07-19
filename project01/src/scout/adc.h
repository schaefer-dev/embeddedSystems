#ifndef EMBEDDEDSYSTEMS18_ADC_H
#define EMBEDDEDSYSTEMS18_ADC_H

#include <stdint.h>
#include "platform_s.h"

#define ADC_CS_DELAY 3 // us
#define ADC_COMPUTE_DELAY 19 // us

/* core functions */

/* sends the requested sensor index to the ADC while reading the data from the last computation */
uint8_t adc_communicate(uint8_t sensor);


/* convenience functions */

/* reads the value for the specified sensor. waits until the ADC has finished the computation */
uint8_t adc_get_sensor(uint8_t sensor);

#endif //EMBEDDEDSYSTEMS18_ADC_H
