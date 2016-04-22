/*******************************************************************************
 * File:   LCDmain.c
 * Author: Steven Lowery
 * Purpose: This code operates a 2x16 LCD screen made with a Hitachi HD44780 LCD controller
 * Microcontroller: PIC18F46K22 40-Pin DIP
 * NOTE: Microcontroller assumes a 500kHz clock speed by default. If xc8 compiler is used
 *			and 500kHz clock speed is assumed, the __delay_ms() macro can be used.
 *			Otherwise, the __delay_ms() macro must be replaced - it is not crucial to the LCD code.
 * How to use:  Write a string to line one with the LCDwriteLineOne() function
 *              Write a string to line two with the LCDwriteLineTwo() function
 *              Write a string to the LCD screen (both lines) with the LCDputs() function
 *              - (This functions exactly as the stdio.h puts() function)
 *              Clear the LCD screen with the LCDclear() function
 *              Write an individual character to the LCD with LCDwrite() function -
 *              - cursor position automatically increments after writing one character
 *              - after writing the 16th character, use LCDgotoLineTwo() function
 *
 * For programming assistance see datasheet for Hitachi HD44780U (LCD-II)
 *
 * Created on March 31, 2015, 8:08 PM
 ******************************************************************************/


#include <xc.h>                         //PIC hardware mapping
#include <stdio.h>
#include <stdlib.h>

#define _XTAL_FREQ 2000000               //Used by the compiler for the delay_ms(x) macro

/******************************************************************************/
//Pin connections for LCD screen (using PIC18F46K22 40-pin DIP)
/*******************************************************************************
 * LCD Register Select (RS) pin:    RE0
 * LCD Read/Write (R/W) pin:        RE1
 * LCD Enable (E) Pin:              RE2
 * LCD data (D0-D7) pins:           Register A (RA0-RA7)
 * LCD Power:                       +5V
 * LCD GND:                         0V
 * LCD VL pin (contrast):           10k potentiometer to +5V & GND or 0V for high contrast
 * LCD backlight pins (A & K):      +5V & GND
*******************************************************************************/


/******************************************************************************/
//Function Prototypes for LCD control
/******************************************************************************/
void LCDinit(void);                 //Initializes the LCD
void LCDcmd(char);                  //Writes a command byte to the LCD
void LCDcheckBF(void);              //Checks the busy flag of the LCD
void LCDwrite(char);                //Writes one char to the LCD
void LCDclear(void);                //Clears the LCD screen
void LCDgotoLineOne(void);          //Points the LCD cursor to the beginning of line one
void LCDgotoLineTwo(void);          //Points the LCD cursor to the beginning of line two
void LCDputs(char[]);               //Puts a string to the LCD screen (maximum 32 characters)
void LCDwriteLineOne(char[]);       //Puts a string to Line one only (maximum 16 characters)
void LCDwriteLineTwo(char[]);       //Puts a string to Line two only (maximum 16 characters)
/******************************************************************************/


void LCDinit(void)
{
    //Initialize Pins for LCD screen
    TRISA = 0;                  //Make Register A all outputs for LCD data
    TRISEbits.RE0 = 0;          //Make RE0 an output for LCD RS pin
    TRISEbits.RE1 = 0;          //Make RE1 an output for LCD R/W pin
    TRISEbits.RE2 = 0;          //Make RE2 an output for LCD E pin

    //Initialize LCD; Busy Flag cannot be checked yet!
    LATEbits.LATE0 = 0;         //Turn RS pin low for LCD initialization
    LATEbits.LATE1 = 0;         //Turn R/W pin low for LCD initialization
    LATEbits.LATE2 = 1;         //Turn E pin high to wait for first instruction

    __delay_ms(50);
    LCDcmd(0b00110000);         //Function Set for 8-bit interface
    __delay_ms(60);
    LCDcmd(0b00110000);         //Function Set for 8-bit interface
    __delay_ms(60);
    LCDcmd(0b00110000);         //Function Set for 8-bit interface
    __delay_ms(60);
    LCDcmd(0b00111000);         //8-bits, 2 lines, 5x8 font
    __delay_ms(60);
    LCDcmd(0b00001000);         //Display Off (for initialization)
    __delay_ms(60);
    LCDcmd(0b00000001);         //Display Clear
    __delay_ms(60);
    LCDcmd(0b00000110);         //Entry Set Mode: increment AC, no shift
    __delay_ms(60);             //Busy flag can now be checked

    //Now turn on display, cursor, and blink
    LCDcmd(0b00001111);         //Display ON, Cursor ON, Blink ON
    LCDcheckBF();               //Check busy flag before doing anything else
}

void LCDcmd(char command)
{
    LATA = command;             //Data is written to LCD pins
    LATEbits.LATE2 = 0;         //Turn E pin low to activate read/write
    LATEbits.LATE2 = 1;         //Turn E pin high to wait for next instruction
}

void LCDcheckBF(void)
{
    TRISAbits.TRISA7 = 1;       //Make RA7 an input to check flag
    LATEbits.LATE0 = 0;         //Turn RS pin low for instruction register
    LATEbits.LATE1 = 1;         //Turn R/W pin high for read
    LATEbits.LATE2 = 0;
    LATEbits.LATE2 = 1;         //quickly toggle enable bit to check busy flag

    while(PORTAbits.RA7!=0)
    {
        LATEbits.LATE2 = 0;
        LATEbits.LATE2 = 1;     //quickly toggle enable bit to check busy flag
    }
    LATEbits.LATE2 = 1;         //Make sure Enable is HIGH before exiting function
    TRISAbits.TRISA7 = 0;       //Make RA7 an output after checking busy flag
}

void LCDwrite(char input)
{
    LATEbits.LATE0 = 1;         //Turn RS pin HIGH for Data Register
    LATEbits.LATE1 = 0;         //Turn R/W pin low for write
    LCDcmd(input);                  //Write character to LCD
    LCDcheckBF();               //Check the busy flag before doing anything else
}

void LCDclear(void)
{
    LATEbits.LATE0 = 0;         //Turn RS pin low for Instruction Register
    LATEbits.LATE1 = 0;         //Turn R/W pin low for write
    LCDcmd(0b00000001);         //Clears the LCD display
    LCDcheckBF();               //Check the busy flag before doing anything else
}

void LCDgotoLineOne(void)
{
    LATEbits.LATE0 = 0;         //Turn RS pin low for Instruction Register
    LATEbits.LATE1 = 0;         //Turn R/W pin low for write
    LCDcmd(0b10000000);         //Sets DDRAM address to 00h (begin line 1)
    LCDcheckBF();               //Check the busy flag before doing anything else
}

void LCDgotoLineTwo(void)
{
    LATEbits.LATE0 = 0;         //Turn RS pin low for Instruction Register
    LATEbits.LATE1 = 0;         //Turn R/W pin low for write
    LCDcmd(0b11000000);         //Sets DDRAM address to 40h (begin line 2)
    LCDcheckBF();               //Check the busy flag before doing anything else
}

void LCDputs(char string[])
{
    unsigned char i = 0;        //create a counter variable

    LCDclear();                 //Clear the LCD before writing

    while(string[i])            //This will continue until the /0 (end of string) character is reached
    {
        LCDwrite(string[i]);    //Write the string one element at a time
        i++;                    //Move to the next element
        if(i==16)
            LCDgotoLineTwo();   //Go to line two after element 16
                                //otherwise the string will be written off screen
    }
}


void LCDwriteLineOne(char string[])
{
    unsigned char i = 0;        //create a counter variable

    LCDgotoLineOne();

     while(string[i])           //This will continue until the /0 (end of string) character is reached
    {
        LCDwrite(string[i]);    //Write string one element at a time
        i++;                    //Move to the next element
    }
}

void LCDwriteLineTwo(char string[])
{
    unsigned char i = 0;        //create a counter variable

    LCDgotoLineTwo();           //Begin on Line two

     while(string[i])           //This will continue until the /0 (end of string) character is reached
    {
        LCDwrite(string[i]);    //Write string one element at a time
        i++;                    //Move to the next element
    }
}
