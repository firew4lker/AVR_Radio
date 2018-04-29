/************************************************\
 *            A simple AVR Radio                *
 *    The code is based to the code written     *
 *      by Rodolfo Broco Manin (RodLophus)      *
 *    https://github.com/RodLophus/SanyoCCB     *
\************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>

#include "hd44780.h"
#include "LC72131.h"
#include "bit_manipulation.h"

// LC72131 addresses. Page 13 of the Datasheet.
#define LC72131_ADDR_IN1	0x82
#define LC72131_ADDR_IN2	0x92
#define LC72131_ADDR_OUT	0xA2

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
//#define PLL_BAND_AM 6

#define TUNED      PC4   // Pin 14 of the PLL. When pulled LOW, PLL is locked.
#define TUNED_PORT PORTC
#define TUNED_DDR  DDRC
#define TUNED_PIN  PINC

#define STEREO      PC5   // Pin 15 of the PLL. When pulled LOW, PLL is stereo.
#define STEREO_PORT PORTC
#define STEREO_DDR  DDRC
#define STEREO_PIN  PINC

#define UPSW       PB0      // Switch for increasing the frequency.
#define UPSW_PORT  PORTB
#define UPSW_DDR   DDRB
#define UPSW_PIN   PINB

#define DOWNSW       PB1    // Switch for decreasing the frequency.
#define DOWNSW_PORT  PORTB
#define DOWNSW_DDR   DDRB
#define DOWNSW_PIN   PINB

#define TUNESW       PB2    // Switch for FM/AM mode.
#define TUNESW_PORT  PORTB
#define TUNESW_DDR   DDRB
#define TUNESW_PIN   PINB

#define NONE    0   // Values used to the switch detection press function.
#define UP      1
#define DOWN    2
#define TNMODE  3

#define STSYMBOL 0 // Custom lcd symbol for stereo mode.
#define TNSYMBOL 1 // Custom lcd symbol for tuned mode.

/*

PLL pin    Direction       Function
BO0        PLL -> Tuner    Not used
BO1        PLL -> Tuner    Mute / IF output (0 = Mute / IF counter mode.
BO2        PLL -> Tuner    Band selector (0 = FM; 1 = AM).
BO3        PLL -> Tuner    Not used.
BO4        PLL -> Tuner    Audio mode (0 = Stereo; 1 = Mono).
IO0        Tuner -> PLL    Not used (pulled high.  Reads "1").
IO1        Tuner -> PLL    Not used (pulled high.  Reads "1").

  FM
Antenna +-----------------------------+
    +---|                          1  |- CE
    +---|                          2  |- DI
        |        UNKNOWN PLL       3  |- CL
        |       WITH LC721131      4  |- DO
        |        AND LA1838        5  |- GND
        |                          6  |- FM IF OUT
        |                          7  |- VDD +5 v
        |                          8  |- VCC +12 v
        |                          9  |- FM DET OUT
        |                         10  |- Out-L
        |                         11  |- GND
        |                         12  |- Out-R
  AM    |                         13  |- MUTE
Antenna |                         14  |- TUNED
    +---|                         15  |- STEREO
    |   |                             |
    |   |                             |
    +---|                             |
        +-----------------------------+

PLL to ATmega328p connections:

ATmega328p              PLL
PCO (CE)----------------CE  (1)
PC1 (DI)----------------DO  (4)
PC2 (CL)----------------CL  (3)
PC3 (DO)----------------D1  (2)

PC4 (TUNED)-------------TUNED  (14)
PC5 (STEREO)------------STEREO (15)
*/

uint8_t pll_in1[3];  // IN1 consist of 3 bytes in total. Page 9 of the Datasheet.
uint8_t pll_in2[3];  // IN2 consist of 3 bytes in total. Page 9 of the Datasheet.

// Initial frequencies for the PLL.
volatile uint16_t FMFrequency = 978;   // MHz / 10
//volatile uint16_t AMFrequency = 73;    // KHz * 10

uint8_t band = PLL_BAND_FM;
uint8_t tuned = 0;

volatile uint16_t tick=0;  // Time keeping variable in milliseconds.
volatile uint16_t now=0;   // Time reference variable.

void PLL_Init(void);          // Function to initialize the PLL.
void PLL_SetMode(uint8_t);    // Function to control the mode of the PLL.
uint8_t PLL_Tune(uint16_t);   // Function to tune in the frequency of the PLL.
void millis_init(void);       // Function to initialize the timer/counter0 running.
void lcdupdate(void);         // Function to update the lcd content.
uint8_t readsw (void);        // Function for polling the control switches.
void customchar(void);        // Function for crating some custom lcd characters.

void utofix(uint16_t, char *);    // Function to convert an unsigned int to printable number with decimal point.
                                  // E.g. 1009 gets converted to the char array "100.9" (5 bytes).

int main(void){

    uint8_t toggletn=(1^0);
    uint8_t tunemode=0;

    lcd_init();                   // LCD initialization.
    lcd_clrscr();                 // Clear the LCD.
    lcd_home();                   // Set the cursor at home position.
    customchar();                 // Create custom characters for STEREO and TUNED symbols.

    TUNED_DDR &=  ~(1<<TUNED);    // TUNED pin as Input.
    TUNED_PORT |= (1<<TUNED);     // TUNED pin pull-up resistor enabled.

    STEREO_DDR &=  ~(1<<STEREO);  // STEREO pin as Input.
    STEREO_PORT |= (1<<STEREO);   // STEREO pin pull-up resistor enabled.

    UPSW_DDR &=  ~(1<<UPSW);      // UP switch pin as Input.
    UPSW_PORT |= (1<<UPSW);       // UP switch pin pull-up resistor enabled.

    DOWNSW_DDR &=  ~(1<<DOWNSW);  // UP switch pin as Input.
    DOWNSW_PORT |= (1<<DOWNSW);   // UP switch pin pull-up resistor enabled.

    TUNESW_DDR &=  ~(1<<TUNESW);  // UP switch pin as Input.
    TUNESW_PORT |= (1<<TUNESW);   // UP switch pin pull-up resistor enabled.


    millis_init();                // Starting the time keeping function.

    sei();                        // Enabling global Interrupts.

    lcd_puts_P("-- AVR RADIO --");
    _delay_ms(2000);

    lcd_clrscr();

    LC72131_init();
    PLL_SetMode(PLL_BAND_FM);
    tuned = PLL_Tune(FMFrequency);

    while(1){

        if (readsw()==UP && tunemode==1){
            tuned=0;
            while(tuned==0){
                FMFrequency++;
                if (FMFrequency >= 1080) FMFrequency=875;
                tuned = PLL_Tune(FMFrequency);
                lcdupdate();
            };
        } else if (readsw()==UP && tunemode==0){
            FMFrequency++;
            if (FMFrequency >= 1080) FMFrequency=875;
            tuned = PLL_Tune(FMFrequency);
        };

        if (readsw()==DOWN && tunemode==1){
            tuned=0;
            while(tuned==0){
                FMFrequency--;
                if (FMFrequency <= 875) FMFrequency=1080;
                tuned = PLL_Tune(FMFrequency);
                lcdupdate();
            };
        } else if (readsw()==DOWN && tunemode==0){
            FMFrequency--;
            if (FMFrequency <= 875) FMFrequency=1080;
            tuned = PLL_Tune(FMFrequency);
        };

        if (readsw()==TNMODE){
            tunemode ^= toggletn;
        };

        if (tunemode==0){
            lcd_gotoxy(0,1);
            lcd_puts_P("MANUAL SCAN");
            _delay_ms(200);
        }else{
            lcd_gotoxy(0,1);
            lcd_puts_P("AUTO SCAN  ");
            _delay_ms(200);
        };

        lcdupdate();

        _delay_ms(1);

    };

    return 0;
}

uint8_t readsw (){

    if (digitalRead(UPSW,&UPSW_PIN)==0){
        _delay_ms(10);  // Some debounce time.
        if (digitalRead(UPSW,&UPSW_PIN)==0) {
            return UP;
        };
    };

    if (digitalRead(DOWNSW,&DOWNSW_PIN)==0){
        _delay_ms(10);  // Some debounce time.
        if (digitalRead(DOWNSW,&DOWNSW_PIN)==0) {
            return DOWN;
        };
    };

    if (digitalRead(TUNESW,&TUNESW_PIN)==0){
        _delay_ms(10);  // Some debounce time.
        if (digitalRead(TUNESW,&TUNESW_PIN)==0) {
            return TNMODE;
        };
    };

    return NONE;

}

void lcdupdate() {

    char s[10];                   // Temporary char array for printing on the LCD.

    lcd_home();

//    if (band==PLL_BAND_FM){
        utofix(FMFrequency,s);
        lcd_puts_P("FM ");
        lcd_puts(s);
        lcd_puts_P(" MHz ");
//    } else {
//        utofix(AMFrequency,s);
//        lcd_puts_P("AM ");
//        lcd_puts(s);
//        lcd_puts_P(" KHz ");
//    };

    if (digitalRead(STEREO,&STEREO_PIN)==0){
        lcd_gotoxy(13,0);
        lcd_putc('[');
        lcd_putc(STSYMBOL);
        lcd_putc(']');
    } else {
        lcd_gotoxy(13,0);
        lcd_puts("[ ]");
    };

    if (tuned==1){
        lcd_gotoxy(13,1);
        lcd_putc('[');
        lcd_putc(TNSYMBOL);
        lcd_putc(']');

    } else {
        lcd_gotoxy(13,1);
        lcd_puts("[ ]");
    };
}

/************************************************\
 *                PLL_Init()                    *
 *        Initialize the PLL settings.          *
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
    bitSet(pll_in2[2], IN2_BO1);   // Mute off / normal tuner mode
}

void PLL_SetMode(uint8_t mode) {

    switch(mode) {

        case PLL_STEREO:
            bitClear(pll_in2[2], IN2_BO4);
            break;

        case PLL_MONO:
            bitSet(pll_in2[2], IN2_BO4);
            break;

        case PLL_MUTE:
            bitClear(pll_in2[2], IN2_BO1);
            break;

        case PLL_UNMUTE:
            bitSet(pll_in2[2], IN2_BO1);
            break;

        case PLL_BAND_FM:
            band = PLL_BAND_FM;
            bitWrite(pll_in1[0], IN1_R0,  1); // Reference frequency = 50kHz. R3=0, R2=0, R2=0, R1=1.
            bitWrite(pll_in1[0], IN1_R1,  0);
            bitWrite(pll_in1[0], IN1_R2,  0);
            bitWrite(pll_in1[0], IN1_R3,  0);

            bitWrite(pll_in1[0], IN1_XS,  1); // The PLL uses a 7.2 MHz Crystal. Page 10 of the Datasheet.
            bitWrite(pll_in1[0], IN1_DVS, 1); // Programmable Divider divisor = 2
            bitWrite(pll_in2[0], IN2_GT0, 0); // IF counter measurement period = 32ms
            bitWrite(pll_in2[0], IN2_GT1, 1); //
            bitWrite(pll_in2[1], IN2_DZ0, 1); // Dead zone (DZB).
            bitWrite(pll_in2[1], IN2_DZ1, 0); //
            bitWrite(pll_in2[2], IN2_BO2, 1); // FM mode
            break;

/*        case PLL_BAND_AM:
            band = PLL_BAND_AM;
            bitWrite(pll_in1[0], IN1_R0,  0); // Reference frequency = 10kHz. R3=1, R2=0, R2=0, R1=0.
            bitWrite(pll_in1[0], IN1_R1,  0);
            bitWrite(pll_in1[0], IN1_R2,  0);
            bitWrite(pll_in1[0], IN1_R3,  1);

            bitWrite(pll_in1[0], IN1_XS,  1); // The PLL uses a 7.2 MHz Crystal. Page 10 of the Datasheet.
            bitWrite(pll_in1[0], IN1_DVS, 0); // Programmable Divider divisor = 1
            bitWrite(pll_in2[0], IN2_GT0, 1); // IF counter measurement period = 8ms
            bitWrite(pll_in2[0], IN2_GT1, 0); //
            bitWrite(pll_in2[1], IN2_DZ0, 0); // Dead zone (DZC).
            bitWrite(pll_in2[1], IN2_DZ1, 1); //
            bitWrite(pll_in2[2], IN2_BO2, 0); // AM mode
            break;
*/
        }

    LC72131_write(LC72131_ADDR_IN1, pll_in1, 3);
    LC72131_write(LC72131_ADDR_IN2, pll_in2, 3);

}

/************************************************************\
 *                        PLL_Tune()                        *
 * Set the tuner frequency and return 1 if it is tuned      *
 * or 0 otherwise.                                          *
 *                                                          *
 * The frequency divisors was chosen in a way the frequency *
 * representation can be directly sent to the PLL and is    *
 * easy to represent:                                       *
 *                                                          *
 *            FPD=(FM_FREQ+IF)/REF_FREQ/PRESCALER           *
 *                                                          *
 *           E.g. for an FM frequency of 90 MHz             *
 *           FPD=(90000000+10700000/50000/2=1007            *
 *                                                          *
 *               This is equal to 900+107=1007              *
 *   So, we can load the frequency+IF directly to the FPD   *
 *                                                          *
 *                                                          *
\************************************************************/
uint8_t PLL_Tune(uint16_t frequency) {

    uint16_t fpd = 0;      // Frequency Programmable Divider (FPD).

    uint8_t tuned=0;

    switch(band) {
        case PLL_BAND_FM:
        fpd = (frequency + 107);
        break;

/*        case PLL_BAND_AM:
        // AM: fpd = ((frequency + FI) / 10) << 4
        fpd = (frequency + 45) << 4;
        break;
*/
        default: return 1;
    }

    pll_in1[1] = (uint8_t) (fpd >> 8);          // Loading the HIGH byte to the Programmable Divider.
    pll_in1[2] =  (uint8_t) (fpd & 0x00ff);     // Loading the LOW byte to the Programmable Divider.

    LC72131_write(LC72131_ADDR_IN1, pll_in1, 3);

    _delay_ms(20); //Some delay to give time to the PLL tune output to set.

    if (digitalRead(TUNED,&TUNED_PIN)==0){
        tuned=1;
    };

    return tuned;
}

void millis_init(void){

    TIMSK0 |= (1 << TOIE0);           // Enable overflow Interrupt υπερχείλισης for Timer/Counter0.
    TCNT0 = 6;                        // Preload Timer with the calculated value for 1 msec.
    TCCR0B |= (1<<CS01) | (1<<CS00);  // Start Timer/Counter0 with Prescaler 64.
}

// Interrupt every 1 msec. More than enough to read the encoders.
ISR(TIMER0_OVF_vect){
    tick++;
    TCNT0 += 6;  // Preload Timer with the calculated value for 1 msec.
}

void utofix(uint16_t x, char *s){

    uint16_t temp;
    char str[10];

    temp = x/10;
    ultoa(temp,str,10);
    strcpy(s,str);
    strcat(s,".");
    temp = (x%10);
    ultoa(temp,str,10);
    strcat(s,str);

}

void customchar(){

    lcd_command(_BV(LCD_CGRAM)+STSYMBOL*8);     // ______
    lcd_putc(0b01110);                          //| ### |
    lcd_putc(0b01001);                          //| #  #|
    lcd_putc(0b01111);                          //| ####|
    lcd_putc(0b01001);                          //| #  #|
    lcd_putc(0b11001);                          //|##  #|
    lcd_putc(0b11011);                          //|## ##|
    lcd_putc(0b00011);                          //|   ##|
    lcd_putc(0b00000);                          //|_____|
    lcd_goto(0);

    lcd_command(_BV(LCD_CGRAM)+TNSYMBOL*8);     // ______
    lcd_putc(0b10001);                          //|#   #|
    lcd_putc(0b10101);                          //|# # #|
    lcd_putc(0b10101);                          //|# # #|
    lcd_putc(0b01110);                          //| ### |
    lcd_putc(0b00100);                          //|  #  |
    lcd_putc(0b00100);                          //|  #  |
    lcd_putc(0b00100);                          //|  #  |
    lcd_putc(0b00000);                          //|_____|
    lcd_goto(0);
}
