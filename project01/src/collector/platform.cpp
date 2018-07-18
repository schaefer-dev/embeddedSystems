/**
 * Zumo Hardware
 * ===============================
 * Manages communication with the extension board, i.e., the RF24 radio
 * module. Components are connected as follows:
 *
 * - PC7: RF chip enable
 * - PB3: RF chip select
 * - PB0: RF SCK
 * - PD5: RF MOSI
 * - PD7: RF MISO
 * - PD2: RF IRQ or UART RX (jumpered)
 */

#include <avr/interrupt.h>
#include "../common/platform.h"


#define COLLECTOR_RF_IRQ_JUMPERED

extern volatile uint8_t rf_intr;

void platform_init() {
    /* configure output pins */
    DDRB |= (1 << PB3) | (1 << PB0);
    DDRD |= (1 << PD5);
    DDRC |= (1 << PC7);

    /* configure input pins */
    DDRD &= ~((1 << PD2) | (1 << PD7));

    /* unselect and disable RF */
    UNSELECT_RF();
    DISABLE_RF();

#ifdef COLLECTOR_RF_IRQ_JUMPERED
    /* disable hardware serial RX */
    UCSR1B &= ~(1 << RXEN1);

    /* enable pull-up resistor on PD2 (RF IRQ) */
    PORTD |= (1 << PD2);

    /* trigger interrupt 2 (RF IRQ) on falling edge */
    EICRA |= (1 << ISC21);
    EICRA &= ~(1 << ISC20);

    /* enable INT2 */
    EIMSK |= (1 << INT2);
#endif
}

#ifdef COLLECTOR_RF_IRQ_JUMPERED
ISR (INT2_vect) {
    rf_intr = 1;
}
#endif

void assert(bool b, const char *s) {
#ifndef NO_ASSERT
    if(!b) {
        Serial1.print(s);
        abort();
    }
    else {}
#else
    0;
#endif
}