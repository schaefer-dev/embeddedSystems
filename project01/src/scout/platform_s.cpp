/**
 * Scout Hardware
 * ================================
 * Manages communication with the extension board, i.e., the TLC541I ADC
 * and the RF24 radio module. Components are connected as follows:
 *
 * - PB4: common clock (RF: SCK, ADC: I/O clock)
 * - PB5: common MOSI (RF: MOSI, ADC: address input)
 * - PB0: common MISO (RF: MISO, ADC: data out)
 * - PD4: RF chip select
 * - PD7: RF chip enable
 * - PD2: RF interrupt
 * - PB1: ADC system clock
 * - PC5: ADC chip select
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#include "platform_s.h"

extern volatile uint8_t rf_intr;

void platform_init() {
    /* configure output pins */
    DDRB |= (1 << PB1) | (1 << PB5) | (1 << PB4);
    DDRD |= (1 << PD4) | (1 << PD7);
    DDRC |= (1 << PC5);

    /* configure input pins */
    DDRB &= ~(1 << PB0);
    DDRD &= ~(1 << PD2);

    /* deselect both chips */
    UNSELECT_ADC();
    UNSELECT_RF();

    /* disable RF */
    DISABLE_RF();

    /* configure timer 1 (4MHz clock on PB1)*/
    TCCR1A = (1 << COM1A0);
    TCCR1B = (1 << WGM12) | (1 << CS10);
    TCCR1C = 0;
    TCNT1 = 0;
    OCR1A = 4;
    OCR1B = 0;

    /* enable pull-up resistor on PD2 (RF IRQ) */
    PORTD |= (1 << PD2);

    /* trigger interrupt 0 (RF IRQ) on falling edge */
    EICRA |= (1 << ISC01);
    EICRA &= ~(1 << ISC00);

    /* enable INT0 */
    EIMSK |= (1 << INT0);
}

ISR (INT0_vect) {
    rf_intr = 1;
}

void assert(bool b, const char *s, uint8_t len) {
#ifndef NO_ASSERT
    if(!b) {
        serial_send_blocking((char*)s, len);
        abort();
        }
    else {}
#else
    0;
#endif
}