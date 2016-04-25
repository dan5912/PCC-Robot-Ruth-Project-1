/*
 * File:   main.c
 * Author: daniel
 *
 * Created on April 20, 2016, 8:55 PM
 */


// <editor-fold defaultstate="collapsed" desc="Config Bits">
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1H
#pragma config FOSC = INTIO67      // Oscillator Selection bits (HS oscillator (medium power 4-16 MHz))
#pragma config PLLCFG = OFF     // 4X PLL Enable (Oscillator used directly)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock is always enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = ON      // Power-up Timer Enable bit (Power up timer enabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (Watch dog timer is always disabled. SWDTEN has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTB3  // CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<5:0> pins are configured as digital I/O on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is multiplexed with RB5)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC output and ready status are not delayed by the oscillator stable status)
#pragma config T3CMX = PORTB5   // Timer3 Clock input mux bit (T3CKI is on RB5)
#pragma config P2BMX = PORTD2   // ECCP2 B output mux bit (P2B is on RD2)
#pragma config MCLRE = EXTMCLR  // MCLR Pin Enable bit (MCLR pin enabled, RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled if MCLRE is also 1)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)
// </editor-fold>

#include <xc.h>
#include "printfLib.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
// <editor-fold defaultstate="collapsed" desc="Defines">


#define CIRCLE_FACTOR 1.1f
#define HYSTERESIS 1

#define BUTTON PORTBbits.RB0    // B0 is pin interrupt INT0

#define LEFT_WHEEL_NEUTRAL 745
#define RIGHT_WHEEL_NEUTRAL 725

#define RIGHT_SPEED_COUNT_RATIO 5.4
#define LEFT_SPEED_COUNT_RATIO  5.4

#define TOP_SPEED_FACTOR 2

#define RADIUS 55                       //radius in millimeters
#define HOLES 32                        //number of holes in each wheel

#define PWM1 LATDbits.LATD0
#define PWM2 LATDbits.LATD1 

#define CONTROL_MS 100
#define VELOCITY_CALC_MS 500
#define LCD_UPDATE_MS 1000

#define SPEED_GAIN 1
#define COMPENSATION_GAIN sqrt(rightWheelCount - leftWheelCount) + 5
#define DOWN_COMPENSATION_GAIN sqrt(leftWheelCount - rightWheelCount)

#define CIRCLE_COMPENSATION_GAIN sqrt(rightWheelCount - leftWheelCount)
#define CIRCLE_DOWN_COMPENSATION_GAIN sqrt(leftWheelCount - rightWheelCount)

#define CIRCLE_LEFT_GAIN 3
#define CIRCLE_RIGHT_GAIN 3

#define _XTAL_FREQ 8000000
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Global Declarations">


unsigned long elapsedMillis = 0;        // Total milliseconds elapsed since power on - overflows after 47 days
unsigned long leftWheelCount = 0;       // Left wheel encoder count
unsigned long rightWheelCount = 0;      // Right wheel encoder count
int rightWheelMeasuredSpeed = 0;        // Right wheel measured speed in mm/s
int leftWheelMeasuredSpeed = 0;         // Right wheel measured speed in mm/s
unsigned int leftWheelCommandedPW = LEFT_WHEEL_NEUTRAL;
unsigned int rightWheelCommandedPW = RIGHT_WHEEL_NEUTRAL;

void wheelVelocity(char wheel, int speed, int speedCompensation);
void limitWheelSpeeds(int* leftWheelMeasuredSpeed, int* rightWheelMeasuredSpeed, int* speedCompensation);
void configureComparators();
void configureTimers();
void excerciseControl();

int speedCompensation = 0;


unsigned char event = 0;                // Event number for switch case

// </editor-fold>

void main(void) {
    
    OSCCONbits.IRCF = 0b110; // 8MHz clock
    
    
    configureComparators();
    configureTimers();
    
    TRISBbits.TRISB4 = 1;   // B4 (Button) is input
    ANSELBbits.ANSB4 = 0;   // Digital input
    
    
    
    unsigned long controlLastMillis = 0;
    unsigned long lcdLastMillis = 0;
    unsigned long speedLastMillis = 0;
    int leftCountTracker = 0;               //keeps track of past counts for instantaneous speed calculation
    int rightCountTracker = 0;              //keeps track of past counts for instantaneous speed calculation
    
    
    LCDinit();                      //Setup LCD
    printf("Event: %3d", event);
    

    
    while(1)
    {
       // <editor-fold defaultstate="collapsed" desc="main loop (timed functions)">
        
 
        // LCD Update
        if(lcdLastMillis + 250 < elapsedMillis)
        {
            LCDclear();
            printf("Speed: %d", rightWheelMeasuredSpeed);
            lcdLastMillis = elapsedMillis;
            LCDgotoLineTwo();
            printf("LW: %d ",  leftWheelCount);
            printf(" RW: %d", rightWheelCount);
        }
        

        // Drive system controller
        if(controlLastMillis + CONTROL_MS < elapsedMillis) // Trigger every (CONTROL_FREQUENCY)ms
        {
            controlLastMillis = elapsedMillis;
            excerciseControl();
        }
        
        // Speed calculator
        if(speedLastMillis + VELOCITY_CALC_MS < elapsedMillis)
        {
            // <editor-fold defaultstate="collapsed" desc="Calculate Velocity">   

            //5.3996 is the millimeters per count for the wheel
            //tracks the instantaneous right wheel speed
            rightWheelMeasuredSpeed = (int) ((rightWheelCount - rightCountTracker) * (RIGHT_SPEED_COUNT_RATIO * VELOCITY_CALC_MS ) / (elapsedMillis - speedLastMillis)); 
            //tracks the instantaneous left wheel speed
            leftWheelMeasuredSpeed = (int) ((leftWheelCount - leftCountTracker) * (LEFT_SPEED_COUNT_RATIO * VELOCITY_CALC_MS) / (elapsedMillis - speedLastMillis));   
            //sets the speedElapsedMillis the same as the current time tick
            speedLastMillis = elapsedMillis;
            //sets the right count tracker the same as right wheel count
            rightCountTracker = rightWheelCount;
            //sets the left count tracker the same as left wheel count
            leftCountTracker = leftWheelCount;
            // </editor-fold>
        }
        
        // </editor-fold>
    }

    return;
}

void limitWheelSpeeds(int* leftWheelSpeed, int* rightWheelSpeed, int* speedCompensation)
{
    // Limit wheel speeds to +- 60 for right wheel and +- 90 for left wheel
    // This allows left wheel to always be able to go faster or slower than the right 
    // wheel for closed loop control
    if (*leftWheelSpeed >= 60)       
    {
        *leftWheelSpeed = 60;
    }
    if (*leftWheelSpeed <= -60)
    {
        *leftWheelSpeed = -60;
    }
    
    if (*rightWheelSpeed >= 60)
    {
        *rightWheelSpeed = 60;
    }
    if (*rightWheelSpeed <= -60)
    {
        *rightWheelSpeed = -60;
    }
    if(*speedCompensation > 30)
    {
        *speedCompensation = 30;
    }
    if(*speedCompensation < -30)
    {
        *speedCompensation = -30;
    }
}

void wheelVelocity(char wheel, int speed, int speedCompensation)
{
    if(wheel == 'r')
    {
        // if speed is 100, pw will be 1000 (1.75ms - forward) , if speed is -100, pw is 625 (1.25ms - reverse)
        // if speed is 0, pw will be 750 1.5ms (neutral)
        rightWheelCommandedPW = (int) (RIGHT_WHEEL_NEUTRAL + speed * TOP_SPEED_FACTOR);
    }
    
    if(wheel == 'l')
    {
        // if speed is 100, pw will be 875 (1.75ms - forward) , if speed is -100, pw is 625 (1.25ms - reverse)
        // if speed is 0, pw will be 750 1.5ms (neutral)
        leftWheelCommandedPW = (int) (LEFT_WHEEL_NEUTRAL - (speedCompensation + speed * TOP_SPEED_FACTOR));  
    }
}


void interrupt ISR()
{
    // <editor-fold defaultstate="collapsed" desc="Compare Interrupts">
    if (CCP2IE && CCP2IF)
    {
        // Read the LATCH (not the PORT)
        if(PWM2 == 1)    // If last state was high, turn off pin and set compare to interrupt after completing the ~16ms cycle (first PW2 = ~16ms - PW1) 
        {
            PWM2 = 0;
            CCPR2 = 8000 - CCPR2;
        }
        else
        {
            PWM2 = 1;
            CCPR2 = leftWheelCommandedPW;
        }
        TMR5 = 0;
        CCP2IF = 0;
        return;
    }
    
    
    
    if (CCP3IE && CCP3IF)
    {
        // Read the LATCH (not the PORT)
        if(PWM1 == 1)       // If last state was high, turn off pin and set compare to interrupt after completing the ~16ms cycle (first PW2 = ~16ms - PW1)
        {
            PWM1 = 0;
            CCPR3 = 8000 - CCPR3;
        }
        else
        {
            PWM1 = 1;
            CCPR3 = rightWheelCommandedPW;  // Set the high PW to commanded PW
        }
        CCP3IF = 0;
        TMR1 = 0;
        
        return;

        
    }
    // </editor-fold>
    
    // <editor-fold defaultstate="collapsed" desc="Comparator Interrupts">
        
    if(C1IF && C1IE)                //check to see if the first interrupt flag was set
    {
        leftWheelCount++;   //if the comparator 1 interrupt flag is set, left wheel count is increased by 1
        char dummy = CM1CON0;       // Has to be read to clear latch
        C1IF = 0;           //reset the interrupt flag
    }
    if(C2IF && C2IE)                //check to see if the second interrupt flag was set
    {
        rightWheelCount++;          //if the comparator 2 interrupt flag is set, right wheel count is increased by 1
        
        char dummy = CM2CON0;       // Has to be read to clear latch
        C2IF = 0;                   //reset the interrupt flag
    }
    
    // </editor-fold>
    
    // <editor-fold defaultstate="collapsed" desc="Timer 2 (ms timer) Interrupt">
    if (TMR2IE && TMR2IF)
    {
        elapsedMillis++;                // Increment millis counter
        PIR1bits.TMR2IF = 0;            // Clear interrupt flag
        return;
    }
    // </editor-fold>
}


void excerciseControl()
{
    static int leftWheelCommandedSpeed = 35;        // Commanded Speed for left wheel
    static int rightWheelCommandedSpeed = 25;       // Commanded Speed for right wheel
    static unsigned long startCircleMillis = 0;
    
    switch(event){
                case 0:                         // Drive straight forward          
                    // <editor-fold defaultstate="collapsed" desc="drive forward">
                 
                   
                    if(rightWheelMeasuredSpeed < 50)     // if speed < 50 mm/s increase speed of both wheels
                    {
                        rightWheelCommandedSpeed += SPEED_GAIN;
                        leftWheelCommandedSpeed += SPEED_GAIN;
                        wheelVelocity('r', rightWheelCommandedSpeed, speedCompensation);
                        wheelVelocity('l', leftWheelCommandedSpeed, speedCompensation);
                    }
                    if(rightWheelMeasuredSpeed > 120)     // if speed > 120 mm/s decrease speed of both wheels
                    {
                        rightWheelCommandedSpeed -= SPEED_GAIN;
                        leftWheelCommandedSpeed -= SPEED_GAIN;
                        wheelVelocity('r', --rightWheelCommandedSpeed, speedCompensation);
                        wheelVelocity('l', --leftWheelCommandedSpeed, speedCompensation);
                    }
                    if(leftWheelCount  > rightWheelCount + HYSTERESIS)    // Adjust left wheel speed to match right
                                                                // with hysteresis
                    {
                        speedCompensation -= DOWN_COMPENSATION_GAIN;
                        wheelVelocity('l', leftWheelCommandedSpeed, speedCompensation);
                    }
                    if(rightWheelCount  > (leftWheelCount + HYSTERESIS) )    // Adjust left wheel speed to match right
                                                                // with hysteresis
                    {
                        speedCompensation += COMPENSATION_GAIN;
                        wheelVelocity('l', leftWheelCommandedSpeed, speedCompensation);     
                    }
                    
                    if(rightWheelCount >= 113) // Stop after x counts and wait for button press to start next event
                    {   
                        if(leftWheelCount >= 113)
                        {
                            rightWheelCount = 0;
                            leftWheelCount = 0;
                            leftWheelCommandedSpeed = 0;
                            rightWheelCommandedSpeed = 0;
                            event += 1;
                            speedCompensation = 0;
                            wheelVelocity('r', rightWheelCommandedSpeed, speedCompensation);
                            wheelVelocity('l', leftWheelCommandedSpeed, speedCompensation);
                            for(int i = 0; i<20 ; i++)
                            {
                                __delay_ms(50);
                            }
                            rightWheelCommandedSpeed = 25;
                            leftWheelCommandedSpeed = -30;
                        }
                        else
                        {
                            rightWheelCommandedSpeed = 0;
                            leftWheelCommandedSpeed = 30;
                            speedCompensation = 0;
                            wheelVelocity('r', rightWheelCommandedSpeed, speedCompensation);
                            wheelVelocity('r', rightWheelCommandedSpeed, speedCompensation);
                        }
                        
                        
                    }
                    
                    limitWheelSpeeds(&leftWheelCommandedSpeed, &rightWheelCommandedSpeed, &speedCompensation);
                    break;
                    // </editor-fold>
                    
                case 1:                         // turn stationary circle 90 degrees
                    // <editor-fold defaultstate="collapsed" desc="stationary circle">
            

                    if(rightWheelMeasuredSpeed < 40)     // if speed < 40 mm/s increase speed of both wheels (opposite directions)
                    {
                        wheelVelocity('r', ++rightWheelCommandedSpeed, speedCompensation);
                        wheelVelocity('l', --leftWheelCommandedSpeed, speedCompensation);
                    }
                    if(rightWheelMeasuredSpeed > 80)     // if speed > 80 mm/s decrease speed of both wheels
                    {
                        wheelVelocity('r', --rightWheelCommandedSpeed, speedCompensation);
                        wheelVelocity('l', ++leftWheelCommandedSpeed, speedCompensation);
                    }
                    if(leftWheelCount > (rightWheelCount + HYSTERESIS))    // Adjust left wheel speed to match right
                                                                // with hysteresis
                    {
                        speedCompensation -= (DOWN_COMPENSATION_GAIN);
                        wheelVelocity('l', leftWheelCommandedSpeed, speedCompensation);
                    }
                    if(rightWheelCount > leftWheelCount + HYSTERESIS)    // Adjust left wheel speed to match right
                                                                // with hysteresis
                    {
                        speedCompensation += COMPENSATION_GAIN;
                        wheelVelocity('l', leftWheelCommandedSpeed,speedCompensation);     
                    }
                    
                    if(rightWheelCount >= 22)  // Stop after x counts and wait for button press to start next event
                    {
                        if(leftWheelCount >= 22)
                        {
                            rightWheelCount = 0;
                            leftWheelCount = 0;
                            leftWheelCommandedSpeed = 0;
                            rightWheelCommandedSpeed = 0;
                            speedCompensation = 0;
                            wheelVelocity('r', rightWheelCommandedSpeed, speedCompensation);
                            wheelVelocity('l', leftWheelCommandedSpeed, speedCompensation);
                            leftWheelCommandedSpeed = 35;
                            rightWheelCommandedSpeed = 25;
                            event++;
                            for(int i = 0; i<20 ; i++)
                            {
                                __delay_ms(50);
                            }
                            startCircleMillis = elapsedMillis;
                        }
                        else
                        {
                            rightWheelCommandedSpeed = 0;
                            leftWheelCommandedSpeed = -30;
                            speedCompensation = 0;
                            wheelVelocity('r', rightWheelCommandedSpeed, speedCompensation);
                            wheelVelocity('r', rightWheelCommandedSpeed, speedCompensation); 
                        }
                        
                    }
                    limitWheelSpeeds(&leftWheelCommandedSpeed, &rightWheelCommandedSpeed, &speedCompensation);
                    break;
                    // </editor-fold>
                    
                case 2:                         // Drive in a circle
                    // <editor-fold defaultstate="collapsed" desc="driving in circles">
                    
                    if(rightWheelCount * 133 < (elapsedMillis -  startCircleMillis))
                    {
                        rightWheelCommandedSpeed += CIRCLE_RIGHT_GAIN;
                        wheelVelocity('r', rightWheelCommandedSpeed, 0);
                    }
                    if(leftWheelCount * 123 < (elapsedMillis -  startCircleMillis))
                    {
                        leftWheelCommandedSpeed += CIRCLE_LEFT_GAIN;
                        wheelVelocity('l', rightWheelCommandedSpeed, 0);
                    }
                    if(rightWheelCount * 133 > (elapsedMillis -  startCircleMillis))
                    {
                        rightWheelCommandedSpeed -= CIRCLE_RIGHT_GAIN;
                        wheelVelocity('r', rightWheelCommandedSpeed, speedCompensation);
                    }
                    if(leftWheelCount * 123 > (elapsedMillis -  startCircleMillis))
                    {
                        leftWheelCommandedSpeed -= CIRCLE_LEFT_GAIN;
                        wheelVelocity('l', rightWheelCommandedSpeed, 0);
                    }
                    
                    
                    /*if(rightWheelMeasuredSpeed < 20)     // if speed < 20 mm/s increase speed of both wheels
                    {
                        wheelVelocity('r', ++rightWheelCommandedSpeed, speedCompensation);
                        wheelVelocity('l', ++leftWheelCommandedSpeed, speedCompensation);
                    }
                    if(round(CIRCLE_FACTOR * (leftWheelCount)) > (rightWheelCount + HYSTERESIS) )    // Adjust left wheel speed to match right
                                                                                                    // with hysteresis
                    {
                        speedCompensation -= CIRCLE_DOWN_COMPENSATION_GAIN;
                        wheelVelocity('l', leftWheelCommandedSpeed, speedCompensation);
                    }
                    if(rightWheelCount > round(CIRCLE_FACTOR * (leftWheelCount + HYSTERESIS)))    // Adjust left wheel speed to match right
                                                                                                // with hysteresis
                    {
                        speedCompensation += CIRCLE_COMPENSATION_GAIN;
                        wheelVelocity('l', leftWheelCommandedSpeed, speedCompensation);     
                    }*/
                    
                    
                    if(rightWheelCount >= 225)  // Stop after x counts and wait for button press to start next event
                    {
                        rightWheelCount = 0;
                        leftWheelCount = 0;
                        leftWheelCommandedSpeed = 0;
                        rightWheelCommandedSpeed = 0;
                        wheelVelocity('r', rightWheelCommandedSpeed, speedCompensation);
                        wheelVelocity('l', leftWheelCommandedSpeed, speedCompensation);
                        event++;
                    }
                    limitWheelSpeeds(&leftWheelCommandedSpeed, &rightWheelCommandedSpeed, &speedCompensation);
                    break;
                    // </editor-fold>
                    
            }
}

void configureTimers()
{
    di();

    T1CON = 0;
    T1CONbits.TMR1CS = 0b00;  // Timer 1 clock is instruction clock (Fosc/4)
    T1CONbits.T1CKPS = 0b10;
    T5CON = 0;
    T5CONbits.TMR5CS = 0b00;  // Timer 5 clock is instruction clock (Fosc/4)
    T5CONbits.T5CKPS = 0b10;



    
    CCP3CON = 0;
    CCP2CON = 0;
    CCP3CONbits.CCP3M = 0b1010;     // Triggers interrupt on compare match
    CCP2CONbits.CCP2M = 0b1010;     // Triggers interrupt on compare match
    
    


    
 
    CCPTMRS0bits.C3TSEL = 0;        // Timer 1 is clock source for compare module 3
    CCPTMRS0bits.C2TSEL = 0b10;        // Timer 5 is clock source for compare module 2
    
    CCPR3 = 675;
    CCPR2 = 675;
    
    CCP3IE = 1;                     // Compare Match Interrupt Enable (3)
    CCP2IE = 1;                     // Compare Match Interrupt Enable (2)
    
    // Timer 2 config for 1ms
    PIE1bits.TMR2IE = 1;
    T2CON = 0;
    T2CONbits.T2CKPS = 0b10;    // 16x Prescaler
    T2CONbits.TMR2ON = 1;
    PR2 = 125;
    
    
    // Button Interrupt code
    //INT0IE = 1;
    
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    
    TMR1 = 0;
    TMR5 = 0;
    TMR1ON = 1;
    TMR5ON = 1;
    
    ei();
    
    TRISDbits.TRISD0 = 0;   // C0 (PWM1) is output
    TRISDbits.TRISD1 = 0;   // C1 (PWM2) is output
    ANSELD = 0;
    
}

void configureComparators()
{
    // Configure Internal Vref for use by comparators (Shawn)
    
    VREFCON0bits.FVREN=1;           //Fixed Voltage Reference (FVR) is enabled
    VREFCON0bits.FVRST=0;           //FVR Ready flag bit set to "not ready"
    VREFCON0bits.FVRS=0b01;         //This sets the FVR to 1.024Volts
    
    TRISBbits.TRISB1 = 1;
    ANSELBbits.ANSB1 = 1;
    
    
    // Comparator 2 (James)
    CM2CON0bits.C2ON = 1;           /*C2 is enabled*/
    CM2CON0bits.C2OUT = 0;          /*When C2Vin+ > C2Vin-*/
    CM2CON0bits.C2OE = 0;           /*The output is internal only*/
    CM2CON0bits.C2POL = 1;          /*logic is inverted*/
    CM2CON0bits.C2SP = 1;           /*determines speed high*/
    CM2CON0bits.C2R = 1;            /*C2Vin+ is connected to Vref */
    CM2CON0bits.C2CH = 0b11;        /*selecting the pin C12IN3-*/
    
    CM2CON1bits.C2RSEL = 1;         /*FVR BUF1 routed to C2Vref*/
    CM2CON1bits.C2HYS  = 0;         /*hysteresis enabled*/
    CM2CON1bits.C2SYNC = 0;         /* C2 is asynchronous*/
    
    
    

    
    TRISBbits.TRISB3 = 1;
    ANSELBbits.ANSB3 = 1;
    
    // Comparator 1 (Kyle)
    CM1CON0bits.C1ON = 1;           /*C2 is enabled*/
    CM1CON0bits.C1OUT = 0;          /*When C2Vin+ > C2Vin-*/
    CM1CON0bits.C1OE = 0;           /*The output is internal only*/
    CM1CON0bits.C1POL = 1;          /*logic is inverted*/
    CM1CON0bits.C1SP = 1;           /*determines speed high*/
    CM1CON0bits.C1R = 1;            /*C2Vin+ is connected to Vref */
    CM1CON0bits.C1CH = 0b10;        /*selecting the pin C12IN3-*/
    
    CM2CON1bits.C1RSEL = 1;         /*FVR BUF1 routed to C2Vref*/
    CM2CON1bits.C1HYS  = 0;         /*hysteresis enabled*/
    CM2CON1bits.C1SYNC = 0;         /* C2 is asynchronous*/
    
    
    PIR2bits.C1IF = 0;
    PIR2bits.C2IF = 0;
    PIE2bits.C2IE = 1;
    PIE2bits.C1IE = 1;
  
    
}