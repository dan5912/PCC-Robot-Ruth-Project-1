/*
 * File:   main.c
 * Author: daniel
 *
 * Created on April 21, 2016, 1:16 PM
 */
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)


#include <xc.h>

#define DOWN                0
#define UP                  1

#define SWITCH              PORTAbits.RA2

#define LED_RIGHT           1
#define LED_LEFT            0
#define _XTAL_FREQ 500000

    /* -------------------LATC-----------------
     * Bit#:  -7---6---5---4---3---2---1---0---
     * LED:   ---------------|DS4|DS3|DS2|DS1|-
     *-----------------------------------------
     */

unsigned char _direction;                       //a global variable
void main(void) {
                                                //general init
    OSCCON = 0b00111000;                        //500KHz clock speed
    TRISC = 0;                                  //all LED pins are outputs
    LATC = 0;                                   //init LEDs in OFF state

    LATCbits.LATC3 = 1;                         //DS4 is lit
    _direction = LED_RIGHT;                     //start with LEDs rotating from right to left

                                                //setup switch (SW1)
    TRISAbits.TRISA2 = 1;                       //switch as input
    ANSELAbits.ANSA2 = 0;                       //digital switch

                                                //by using the internal resistors, you can save cost by eleminating an external pull-up/down resistor
#ifdef PULL_UPS
    WPUA2 = 1;                                  //enable the weak pull-up for the switch
    nWPUEN = 0;                                 //enable the global weak pull-up bit
#endif
                                                //setup TIMER0 as the delay
                                                //1:256 prescaler for a delay of: (insruction-cycle * 256-counts)*prescaler = ((8uS * 256)*256) =~ 524mS
    OPTION_REG = 0b00000111;                    //setup TIMER0
    INTCONbits.TMR0IE = 1;                      //enable the TMR0 rollover interrupt

                                                //setup interrupt on change for the switch
    INTCONbits.IOCIE = 1;                       //enable interrupt on change global
    IOCANbits.IOCAN2 = 1;                       //when SW1 is pressed, enter the ISR
    GIE = 1;                                    //enable global interupts


    while (1) {
        continue;                               //can spend rest of time doing something critical here
    }
}

void interrupt ISR(void) {

    if (IOCAF) {                                //SW1 was just pressed
        IOCAF = 0;                              //must clear the flag in software
        __delay_ms(5);                          //debounce by waiting and seeing if still held down
        if (SWITCH == DOWN) {
            _direction ^= 1;                    //change directions
        }
    }

    if (INTCONbits.T0IF) {
        INTCONbits.T0IF = 0;

        if (_direction == LED_RIGHT) {
            LATC >> = 1;                        //rotate right
            if (STATUSbits.C == 1)              //when the last LED is lit, restart the pattern
                LATCbits.LATC3 = 1;
        } else{
            LATC << = 1;                        //rotate left
            if (LATCbits.LATC4 == 1)            //when the last LED is lit, restart the pattern
                LATCbits.LATC0 = 1;

        }
    }


}