 #include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>

#include "bit_manipulation.h"

void digitalWrite(uint8_t pin, volatile uint8_t *port, uint8_t val)
{

        if (val == 0) {
                *port &= ~(1<<pin);
        } else {
                *port |= (1<<pin);
        };
}

uint8_t digitalRead(uint8_t pin, volatile uint8_t *port)
{

        if (bit_is_clear(*port, pin)) {
                return 0;
        } else {
                return 1;
        }
}

