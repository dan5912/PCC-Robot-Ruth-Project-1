/*******************************************
 
 * Author: Daniel Hill
 * Copyright: Free to use and distribute in any way
 *              shape or form -- if modified, 
 *                  remove this notice
 
 ********************************************/


#ifndef __LCD__LIB__H

    #define __LCD__LIB__H



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
#endif