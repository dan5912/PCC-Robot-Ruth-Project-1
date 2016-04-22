#include<xc.h>
void main()
{
    /*This is the second comparator bit config*/
    CM2CON0bits.C2ON = 1;/*C2 is enabled*/
    CM2CON0bits.C2OUT = 0;/*When C2Vin+ > C2Vin-*/
    CM2CON0bits.C2OE = 0;/*The output is internal only*/
    CM2CON0bits.C2POL = 1;/*logic is inverted*/
    CM2CON0bits.C2SP = 1;/*determines speed high*/
    CM2CON0bits.C2R = 1;/*C2Vin+ is connected to Vref */
    CM2CON0bits.C2CH = 0b00000010;/*selecting the pin C12IN2-*/
    
    CM2CON1bits.C2RSEL = 1;/*FVR BUF1 routed to C2Vref*/
    CM2CON1bits.C2HYS  = 0;/*hysteresis disabled*/
    CM2CON1bits.C2SYNC = 0;/* C2 is asynchronous*/
    return 0;
}

