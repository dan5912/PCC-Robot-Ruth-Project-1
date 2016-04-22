/*
 * File:   Motor.c
 * Author: ShawnBeals
 *
 * Created on April 20, 2016, 8:58 PM
 */


#include <xc.h>

void main(void) 
{
    VREFCON0bits.FVREN=1;   //Fixed Voltage Reference (FVR) is enabled
    VREFCON0bits.FVRST=0;   //FVR Ready flag bit set to "not ready"
    VREFCON0bits.FVRS=0b01;   //This sets the FVR to 1.024Volts
    return;
}
