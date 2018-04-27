#include <avr/io.h>
#include <inttypes.h>
#include <util/delay.h>

#include "SanyoCCB.h"
#include "bit_manipulation.h"

// Base delay (us).  Also used to time the CL (clock) line.
// 100us should be enough even for slow CCB devices.
#define CCB_DELAY 100


/******************************************\
 *                 init()                 *
 *  Set pin functions and initial states  *
\******************************************/
void LC72131_init() {

    CE_DDR |=  (1<<CE);   // CE pin as Output.

    DI_DDR &=  ~(1<<DI);  // DI pin as Input.
    DI_PORT |= (1<<DI);   // DI pin pull-up resistor enabled.

    CL_DDR |=  (1<<CL);   // CL pin as Output.

    DO_DDR |=  (1<<DO);   // DO pin as Output.

	DO_PORT &= ~(1<<DO);  // DO pin low.
	CL_PORT &= ~(1<<CL);  // Clock pin low.

	// Toggling CE one to "flush" the bus.
	CE_PORT |= (1<<CE);
	_delay_us(CCB_DELAY);
	CE_PORT &= ~(1<<CE);
	_delay_us(CCB_DELAY);
}


/************************************\
 *           writeByte()            *
 *  Send a single byte via CCB bus  *
\************************************/
void writeByte(uint8_t data) {
	// Send one byte out bia CCB bus (LSB first)
	for(int8_t i = 0; i <= 7; i++) {
		digitalWrite(DO, &DO_PORT, bitRead(data, i));
		digitalWrite(CL, &CL_PORT, 1); _delay_us(CCB_DELAY);
		digitalWrite(CL, &CL_PORT, 0); _delay_us(CCB_DELAY);
	}
}


/***************************************\
 *             readByte()              *
 *  Receive a single byte via CCB bus  *
\***************************************/
uint8_t readByte() {
	uint8_t data = 0;
	// Receive one byte from the CCB bus (MSB first)
	for(int8_t i = 7; i >= 0; i--) {
		digitalWrite(CL, &CL_PORT, 1); _delay_us(CCB_DELAY);
		bitWrite(data, i, digitalRead(DI, &DI_PIN));
		digitalWrite(CL, &CL_PORT, 0); _delay_us(CCB_DELAY);
	}
	return data;
}


/*****************************************\
 *                 ccb()                 *
 *  The universal send/receive function  *
\*****************************************/
void ccb(uint8_t address, uint8_t *data, int8_t dataLength, uint8_t mode) {
	int8_t i; // i may reach negative values in the counters
	          // dataLength is typed "int8_t" for compatibility with this counter

	// Send the address, with the nibbles swapped (required by the CCB protocol to support 4-bit addresses)
	writeByte((address >> 4) | (address << 4));

	// Enter the data transfer mode
	digitalWrite(CL, &CL_PORT, 0);
	digitalWrite(CE, &CL_PORT, 1);
	_delay_us(CCB_DELAY);

	switch(mode) {
		case _CCB_SEND:
		// Send data
		// Note: as CCB devices usually reads registers data from MSB to LSB, the buffer is read from left to right
		for(i = dataLength - 1; i >= 0; i--)
				writeByte(data[i]);
		digitalWrite(DO, &DO_PORT, 0);
		break;

	case _CCB_RECEIVE:
		// Receive data
		for(i = 0; i < dataLength; i++)
			data[i] = readByte();
		break;
	}

	digitalWrite(CE, &CE_PORT, 0);
	_delay_us(CCB_DELAY);
}


/*********************************************************\
 *                      diPinState()                     *
 *  Return the state of the DI pin                       *
 * Some CCB devices uses the DO pin for other functions  *
 * when the data bus is idle.  This method makes reading *
 * it easier                                             *
\*********************************************************/
uint8_t diPinState() {
	return digitalRead(DI, &DI_PIN);
}


/********************************************************\
 *                     write()                          *
 *  Send dataLength (up to 127) bytes via CCB bus       *
 * Note: the contents of the input buffer is send       *
 * backwards (from the rightmost to the leftmost byte), *
 * so the order of the data bytes must be the opposite  *
 * as the one shown on the device's datasheets          *
\********************************************************/
void LC72131_write(uint8_t address, uint8_t *data, int8_t dataLength) {
	ccb(address, data, dataLength, _CCB_SEND);
}


/******************************************************\
 *                      read()                        *
 *  receive dataLength (up to 127) bytes via CCB bus  *
\******************************************************/
void LC72131_read(uint8_t address, uint8_t *data, int8_t dataLength) {
	ccb(address, data, dataLength, _CCB_RECEIVE);
}
