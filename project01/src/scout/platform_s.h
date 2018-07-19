#ifndef EMBEDDED_SYSTEMS18_SCOUT_H
#define EMBEDDED_SYSTEMS18_SCOUT_H

#include <avr/io.h>
#include <serial.h>
#include <stdint.h>

#define SPI_SET_CLOCK() PORTB |= (1 << PB4)
#define SPI_CLEAR_CLOCK() PORTB &= ~(1 << PB4)

#define ADC_SET_CLOCK() PORTB |= (1 << PB1)
#define ADC_CLEAR_CLOCK() PORTB &= ~(1 << PB1)

#define SPI_SET_MOSI() PORTB |= 1 << PB5
#define SPI_CLEAR_MOSI() PORTB &= ~(1 << PB5)

#define SPI_READ_MISO() ((PINB >> PB0) & 1)

#define SELECT_RF() PORTD &= ~(1 << PD4)
#define UNSELECT_RF() PORTD |= 1 << PD4

#define SELECT_ADC() PORTC &= ~(1 << PC5)
#define UNSELECT_ADC() PORTC |= 1 << PC5

#define ENABLE_RF() PORTD |= 1 << PD7
#define DISABLE_RF() PORTD &= ~(1 << PD7)

#define ASSERT(cond, string) assert(cond, string, sizeof(string))


void platform_init();

void assert(bool b, const char *s, uint8_t len);

#endif
