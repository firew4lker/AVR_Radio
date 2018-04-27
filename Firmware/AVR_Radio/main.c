#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>

#include "hd44780.h"
#include "SanyoCCB.h"
#include "bit_manipulation.h"


// LC72131 IN1, byte 0. Page 9 of the Datasheet.
#define IN1_SNS    0
#define IN1_DVS    1
#define IN1_CTE    2
#define IN1_XS     3
#define IN1_R0     4
#define IN1_R1     5
#define IN1_R2     6
#define IN1_R3     7

// LC72131 IN2, byte 0. Page 9 of the Datasheet.
#define IN2_GT0    0
#define IN2_GT1    1
#define IN2_TBC    2
#define IN2_DLC    3
#define IN2_IFS    4
#define IN2_TEST0  5
#define IN2_TEST1  6
#define IN2_TEST2  7

// LC72131 IN2, byte 1. Page 9 of the Datasheet.
#define IN2_DNC    0
#define IN2_DOC0   1
#define IN2_DOC1   2
#define IN2_DOC2   3
#define IN2_UL0    4
#define IN2_UL1    5
#define IN2_DZ0    6
#define IN2_DZ1    7

// LC72131 IN2, byte 2. Page 9 of the Datasheet.
#define IN2_IOC1   0
#define IN2_IOC2   1
#define IN2_IO1    2
#define IN2_IO2    3
#define IN2_BO1    4
#define IN2_BO2    5
#define IN2_BO3    6
#define IN2_BO4    7

// LC72131 DO, byte 0. Page 12 of the Datasheet.
#define DO_UL      4
#define DO_IN1     6
#define DO_IN2     7

// Definitions for PLL control.
#define PLL_MONO    1
#define PLL_STEREO  2
#define PLL_MUTE    3
#define PLL_UNMUTE  4
#define PLL_BAND_FM 5
#define PLL_BAND_AM 6

uint8_t pll_in1[3];  // IN1 consist of 3 bytes in total. Page 9 of the Datasheet.
uint8_t pll_in2[3];  // IN2 consist of 3 bytes in total. Page 9 of the Datasheet.

// Initial frequencies for the PLL.
uint16_t FMFrequency = 978;   // MHz * 10
uint16_t AMFrequency = 53;    // KHZ / 10

uint8_t band = PLL_BAND_FM;
uint8_t tuned = 0;

void PLL_Init(void);
void PLL_SetMode(uint8_t);
uint8_t PLL_Tune(uint16_t);

int main(void){

    lcd_init();             // LCD initialization.
    lcd_clrscr();           // Clear the LCD.
    lcd_home();             // Set the cursor at home position.

    lcd_puts("This is a test!");

    _delay_ms(2000);

    lcd_clrscr();

    LC72131_init();
    PLL_SetMode(PLL_BAND_FM);
    tuned = PLL_Tune(FMFrequency);

    _delay_ms(5000);

     PLL_SetMode(PLL_MUTE);

     _delay_ms(5000);

     PLL_SetMode(PLL_UNMUTE);

    while(1){

    };

    return 0;
}


/************************************************\
 *                PLL_Init()                    *
 * Initialize the PLL settings vectors with     *
 * parameters common to booth AM and FM modes   *
\************************************************/
void PLL_Init() {

    pll_in1[0]=0;   // Zeroing byte 0 of pll_in1.
    pll_in1[1]=0;   // Zeroing byte 1 of pll_in1.
    pll_in1[2]=0;   // Zeroing byte 2 of pll_in1.

    pll_in2[0]=0;   // Zeroing byte 0 of pll_in2.
    pll_in2[1]=0;   // Zeroing byte 1 of pll_in2.
    pll_in2[2]=0;   // Zeroing byte 2 of pll_in2.

    bitSet(pll_in2[0], IN2_IFS);   // IF counter in normal mode. Page 12 and 15 of the Datasheet.
    bitSet(pll_in2[1], IN2_UL0);   // Phase error detection width = 0us. UL1=0, UL0=1. Page 11 of the Datasheet.
    bitSet(pll_in2[2], IN2_BO2);   // Mute off / normal tuner mode
}

void PLL_SetMode(uint8_t mode) {

  switch(mode) {
    case PLL_STEREO:
      bitClear(pll_in2[2], IN2_BO3);
      break;

    case PLL_MONO:
      bitSet(pll_in2[2], IN2_BO3);
      break;

    case PLL_MUTE:
      bitClear(pll_in2[2], IN2_BO2);
      break;

    case PLL_UNMUTE:
      bitSet(pll_in2[2], IN2_BO2);
      break;

    case PLL_BAND_FM:
      band = PLL_BAND_FM;
      bitWrite(pll_in1[0], IN1_R0,  1); // Reference frequency = 50kHz
      bitWrite(pll_in1[0], IN1_R3,  0); //
      bitWrite(pll_in1[0], IN1_XS,  1); // The PLL uses a 7.2 MHz Crystal. Page 10 of the Datasheet.
      bitWrite(pll_in1[0], IN1_DVS, 1); // Programmable Divider divisor = 2
      bitWrite(pll_in2[0], IN2_GT0, 0); // IF counter mesurement period = 32ms
      bitWrite(pll_in2[0], IN2_GT1, 1); //
      bitWrite(pll_in2[1], IN2_DZ0, 1); // Dead zone = DZB
      bitWrite(pll_in2[1], IN2_DZ1, 0); //
      bitWrite(pll_in2[2], IN2_BO1, 0); // FM mode
      break;

    case PLL_BAND_AM:
      band = PLL_BAND_AM;
      bitWrite(pll_in1[0], IN1_R0,  0); // Reference frequency = 10kHz
      bitWrite(pll_in1[0], IN1_R3,  1); //
      bitWrite(pll_in1[0], IN1_DVS, 0); // Programmable Divider divisor = 1
      bitWrite(pll_in2[0], IN2_GT0, 1); // IF counter mesurement period = 8ms
      bitWrite(pll_in2[0], IN2_GT1, 0); //
      bitWrite(pll_in2[1], IN2_DZ0, 0); // Dead zone = DZC
      bitWrite(pll_in2[1], IN2_DZ1, 1); //
      bitWrite(pll_in2[2], IN2_BO1, 1); // AM mode
      break;
  }
  LC72131_write(0x82, pll_in1, 3);
  LC72131_write(0x92, pll_in2, 3);
}

/************************************************************\
 *                        PLL_Tune()                        *
 * Set the tuner frequency and return 1 if it is tuned      *
 * or 0 otherwise.                                          *
 *                                                          *
 * The frequency divisors was chosen in a way the frequency *
 * representation can be directly sent to the PLL and is    *
 * easy to represent:                                       *
 * - FM mode (divisor = 100): frequency (MHz) * 10          *
 * - AM mode (divisor = 10):  frequency (kHZ) / 10          *
\************************************************************/
uint8_t PLL_Tune(uint16_t frequency) {

    uint16_t fpd = 0;
    uint8_t i = 0;
    uint8_t r[3];

    switch(band) {
        case PLL_BAND_FM:
        // FM: fpd = (frequency + FI) / (50 * 2)
        fpd = (frequency + 107);
        break;

        case PLL_BAND_AM:
        // AM: fpd = ((frequency + FI) / 10) << 4
        fpd = (frequency + 45) << 4;
        break;

        default: return 1;
    }

    PLL_SetMode(PLL_MUTE);   // YST93x only injects FI signal into the PLL when in MUTE mode

    // Reset the IF counter and program the Frequency Programmable Divider (fpd)
    bitClear(pll_in1[0], IN1_CTE);
    pll_in1[1] = (uint8_t) (fpd >> 8);
    pll_in1[2] =  (uint8_t) (fpd & 0x00ff);
    LC72131_write(0x82, pll_in1, 3);

    // Start the IF counter
    bitSet(pll_in1[0], IN1_CTE);
    LC72131_write(0x82, pll_in1, 3);

    // Wait for PLL to be locked (DO_UL == 1)
    while(i < 50) {
        _delay_ms(10);
        LC72131_read(0xa2, r, 3);  // Discard the 1st result: it is latched from the last count (as said on the datasheet)
        LC72131_read(0xa2, r, 3);  // The 20 rightmost bits from r[0..2] are the IF counter result
        i = (bitRead(r[0], DO_UL)) ? 100 : i + 1;
    };

    PLL_SetMode(PLL_UNMUTE);   // Mute off / normal tuner mode

    return 0;
}
