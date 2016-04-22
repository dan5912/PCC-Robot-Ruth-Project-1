/*
 * File:   left_comparator.c
 * Author: kyle
 *
 * Created on April 20, 2016, 9:04 PM
 */


#include <xc.h>

void main(void) 
{
     
    
    /*    CM2CON1      */
    CM2CON1bits.C1RSEL = 1;     // FVR BUF1 routed to C1VREF input
    CM2CON1bits.C1HYS  = 0;     // Comparator C1 hysteresis disabled
    CM2CON1bits.C1SYNC = 0;     // C1 output is asynchronous
    
    /*    CM1CON0      */  
    CM1CON0bits.C1OUT  = 0;     // CxVIN+ > CxVIN
    CM1CON0bits.C1OE   = 0;     // C1OUT is internal only
    CM1CON0bits.C1POL  = 0;     // C1OUT logic is not inverted
    CM1CON0bits.C1SP   = 1;     // operates in Normal-Power, Higher Speed mode
    CM1CON0bits.C1R    = 1;     // C1VIN+ connects to C1VREF output  
    CM1CON0bits.C1CH   = 0b11;  // pin of C1 connects to C1VIN- (aka RB1)  
    CM1CON0bits.C1ON   = 1;     // Comparator C1 is enabled    
    
    return;
}
