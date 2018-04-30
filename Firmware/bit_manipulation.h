
#ifndef BIT_MANIPULATION_H
#define BIT_MANIPULATION_H


#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

void digitalWrite(uint8_t pin, volatile uint8_t *port, uint8_t val);

uint8_t digitalRead(uint8_t pin, uint8_t port);

#endif

