#ifndef SanyoCCB_h
#define SanyoCCB_h

#include <inttypes.h>

#define _CCB_SEND    0
#define _CCB_RECEIVE 1

#define CE_PORT PORTC
#define CE_DDR  DDRC
#define CE_PIN  PINC
#define CE      0

#define DI_PORT PORTC
#define DI_DDR  DDRC
#define DI_PIN  PINC
#define DI      1

#define CL_PORT PORTC
#define CL_DDR  DDRC
#define CL_PIN  PINC
#define CL      2

#define DO_PORT PORTC
#define DO_DDR  DDRC
#define DO_PIN  PINC
#define DO      3


void LC72131_init();
void writeByte(uint8_t data);
uint8_t readByte();
void ccb(uint8_t address, uint8_t *data, int8_t dataLength, uint8_t mode);
uint8_t diPinState();
void LC72131_write(uint8_t address, uint8_t *data, int8_t dataLength);
void LC72131_read(uint8_t address, uint8_t *data, int8_t dataLength);

#endif

