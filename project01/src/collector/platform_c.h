#ifndef EMBEDDED_SYSTEMS18_COLLECTOR_H
#define EMBEDDED_SYSTEMS18_COLLECTOR_H

#include <avr/io.h>
#include <HardwareSerial.h>
#include <stdint.h>
#include <Arduino.h>

#define SPI_SET_CLOCK() PORTB |= (1 << PB0)
#define SPI_CLEAR_CLOCK() PORTB &= ~(1 << PB0)

#define SPI_SET_MOSI() PORTD |= 1 << PD5
#define SPI_CLEAR_MOSI() PORTD &= ~(1 << PD5)

#define SPI_READ_MISO() ((PIND >> PD7) & 1)

#define SELECT_RF() PORTB &= ~(1 << PB3); delayMicroseconds(30)
#define UNSELECT_RF() PORTB |= 1 << PB3; delayMicroseconds(30)

#define ENABLE_RF() PORTC |= 1 << PC7
#define DISABLE_RF() PORTC &= ~(1 << PC7)

#define ASSERT(cond, string) assert(cond, string)

#define COLLECTOR_RF_IRQ_JUMPERED


void platform_init();

void assert(bool b, const char *s);


#endif
