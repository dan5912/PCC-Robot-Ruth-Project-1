/*
 * File:   main.c
 * Author: daniel
 *
 * Created on April 20, 2016, 8:55 PM
 */


#include <xc.h>
#define CIRCLE_FACTOR 1.1f
#define BUTTON PORTCbits.RC3


unsigned long elapsedMillis = 0;        // Total milliseconds elapsed since power on - overflows after 47 days
unsigned long leftWheelCount = 0;       // Left wheel encoder count
unsigned long rightWheelCount = 0;      // Right wheel encoder count
char rightWheelMeasuredSpeed = 0;       // Right wheel measured speed in mm/s

void wheelVelocity(char wheel, char speed, char speedCompensation);
void limitWheelSpeeds(char* leftWheelSpeed, char* rightWheelSpeed);


void main(void) {
    
    di();
    
    //T1CONbits.TMR1CS = 0b00; // Timer 1 clock is instruction clock (Fosc/4)
    T1CONbits.TMR1CS = 0b01;  // Timer 1 clock is system clock (Fosc)
    TMR1IE = 1;
    T1CONbits.RD16 = 1;        // 16-bit read mode
    PEIE = 1;
    CCP1CONbits.CCP1M = 0b1001;    // Compare mode clear CCP1 on compare match and triggers interrupt
    CCPR1H = 2;                    // Set Compare match 1 to 750 (1.5ms)
    CCPR1L = 238;
    
    T3CONbits.TMR3CS = 0b01;  // Timer 3 clock is system clock (Fosc)
    TMR3IE = 1;
    T3CONbits.RD163 = 1;        // 16-bit read mode
    PEIE = 1;
    CCP3CONbits.CCP3M = 0b1001;    // Compare mode clear CCP1 on compare match and triggers interrupt
    CCPR3H = 2;                    // Set Compare match 3 to 750 (1.5ms)
    CCPR3L = 238;
    
       // Turn timers on
    T3CONbits.TMR3ON = 1;
    
    
    ei();
    
    
    
    unsigned long pwmLastMillis = 0;
    unsigned long controlLastMillis = 0;
    unsigned long lcdLastMillis = 0;
    
    unsigned char event = 0;
    
    char speedCompensation = 0;
    char rightWheelSpeed = 0;               // Right wheel commanded speed
    char leftWheelSpeed = 0;                // Left wheel commanded speed
    
    while(1)
    {
        
        // LCD Update
        if(lcdLastMillis + 500 < elapsedMillis)
        {
            lcdLastMillis = elapsedMillis;
        }
        
        // PWM Update
        if(pwmLastMillis + 20 < elapsedMillis)  // Trigger every 20ms (50 Hz)
        {
            pwmLastMillis = elapsedMillis;
            
            TMR1H = 0x00;               // Reset timer 1 to 0
            TMR1L = 0x00;
            CCP1CONbits.CCP1M = 0b1001; // Re-enable compare mode
            
            TMR3H = 0x00;               // Since timer is turned on 1 instruction cycle after TMR1
            TMR3L = 0x04;               // TMR3 will be behind ~ 4 counts (@Fosc)
            CCP3CONbits.CCP3M = 0b1001; // Re-enable compare mode
            
            TMR1ON = 1;                 // Turn the timers back on
            TMR3ON = 1;      
        }
        // Drive system controller
        if(controlLastMillis + 200 < elapsedMillis) // Trigger every 50ms
        {
            controlLastMillis = 0;
            switch(event){
                case 0:                         // Drive straight forward
                    if(rightWheelSpeed < 50)     // if speed < 50 mm/s increase speed of both wheels
                    {
                        wheelVelocity('r', ++rightWheelSpeed);
                        wheelVelocity('l', ++leftWheelSpeed);
                    }
                    if(rightWheelSpeed > 55)     // if speed > 55 mm/s decrease speed of both wheels
                    {
                        wheelVelocity('r', --rightWheelSpeed);
                        wheelVelocity('l', --leftWheelSpeed);
                    }
                    if(leftWheelCount > rightWheelCount + 1)    // Adjust left wheel speed to match right
                                                                // with hysteresis
                    {
                        speedCompensation--;
                        wheelVelocity('l', leftWheelSpeed);
                    }
                    if(rightWheelCount > leftWheelCount + 1)    // Adjust left wheel speed to match right
                                                                // with hysteresis
                    {
                        speedCompensation++;
                        wheelVelocity('l', leftWheelSpeed);     
                    }
                    if(rightWheelCount >= 300) // Stop after x counts and wait for button press to start next event
                    {
                        rightWheelCount = 0;
                        leftWheelCount = 0;
                        wheelVelocity('r', 0);
                        wheelVelocity('l', 0);
                        while(BUTTON == 1)
                        {
                            continue;
                        }
                        event++;
                    }
                    limitWheelSpeeds(&leftWheelSpeed, &rightWheelSpeed);
                    break;
                    
                case 1:                         // turn stationary circle 90 degrees
                    
                    if(rightWheelSpeed < 10)     // if speed < 10 mm/s increase speed of both wheels (opposite directions)
                    {
                        wheelVelocity('r', ++rightWheelSpeed);
                        wheelVelocity('l', --leftWheelSpeed);
                    }
                    if(rightWheelSpeed > 12)     // if speed > 12 mm/s decrease speed of both wheels
                    {
                        wheelVelocity('r', --rightWheelSpeed);
                        wheelVelocity('l', ++leftWheelSpeed);
                    }
                    if(leftWheelCount > (rightWheelCount + 1))    // Adjust left wheel speed to match right
                                                                // with hysteresis
                    {
                        speedCompensation++;
                        wheelVelocity('l', leftWheelSpeed);
                    }
                    if(rightWheelCount > leftWheelCount + 1)    // Adjust left wheel speed to match right
                                                                // with hysteresis
                    {
                        speedCompensation--;
                        wheelVelocity('l', leftWheelSpeed);     
                    }
                    
                    if(rightWheelCount >= 25)  // Stop after x counts and wait for button press to start next event
                    {
                        rightWheelCount = 0;
                        leftWheelCount = 0;
                        wheelVelocity('r', 0);
                        wheelVelocity('l', 0);
                        while(BUTTON == 1)
                        {
                            continue;
                        }
                        event++;
                    }
                    limitWheelSpeeds(&leftWheelSpeed, &rightWheelSpeed);
                    break;
                    
                case 2:                         // Drive in a circle
                    if(rightWheelSpeed < 30)     // if speed < 30 mm/s increase speed of both wheels
                    {
                        wheelVelocity('r', ++rightWheelSpeed);
                        wheelVelocity('l', ++leftWheelSpeed);
                    }
                    if((leftWheelCount * CIRCLE_FACTOR) > (rightWheelCount + 1) )    // Adjust left wheel speed to match right
                                                                // with hysteresis
                    {
                        speedCompensation--;
                        wheelVelocity('l', leftWheelSpeed);
                    }
                    if(rightWheelCount > ((leftWheelCount + 1)*CIRCLE_FACTOR))    // Adjust left wheel speed to match right
                                                                // with hysteresis
                    {
                        speedCompensation++;
                        wheelVelocity('l', leftWheelSpeed);     
                    }
                    if(rightWheelCount >= 500)  // Stop after x counts and wait for button press to start next event
                    {
                        rightWheelCount = 0;
                        leftWheelCount = 0;
                        wheelVelocity('r', 0);
                        wheelVelocity('l', 0);
                        while(BUTTON == 1)
                        {
                            continue;
                        }
                        event++;
                    }
                    limitWheelSpeeds(&leftWheelSpeed, &rightWheelSpeed);
                    break;
            }
            
        }
    }
    
    
    
    return;
}

void limitWheelSpeeds(char* leftWheelSpeed, char* rightWheelSpeed)
{
    // Limit wheel speeds to +- 80 for right wheel and +- 100 for left wheel
    // This allows left wheel to always be able to go faster or slower than the right 
    // wheel for closed loop control
    if (leftWheelSpeed >= 100)       
    {
        leftWheelSpeed = 100;
    }
    if (leftWheelSpeed <= -100)
    {
        leftWheelSpeed = -100;
    }
    
    if (rightWheelSpeed >= 80)
    {
        rightWheelSpeed = 80;
    }
    if (rightWheelSpeed <= -80)
    {
        rightWheelSpeed = -80;
    }
}

void wheelVelocity(char wheel, char speed, char speedCompensation)
{
    if(wheel == 'r')
    {
        // if speed is 100, temp will be 1000 (2ms - forward) , if speed is -100, temp is 500 (1ms - reverse)
        // if speed is 0, speed will be 750 1.5ms (neutral)
        int temp = 750 + speed * 2.5;  
        CCPR1H = (temp >> 8) & 0xFF;                    
        CCPR1L = temp & 0xFF;
    }
    
    if(wheel == 'l')
    {
        // if speed is 100, temp will be 1000 (2ms - forward) , if speed is -100, temp is 500 (1ms - reverse)
        // if speed is 0, speed will be 750 1.5ms (neutral)
        int temp = 750 + speedCompensation + speed * 2.5;  
        CCPR3H = (temp >> 8) & 0xFF;                    
        CCPR3L = temp & 0xFF;
    }
}


void interrupt ISR()
{
    if (CCP1IE && CCP1IF)
    {
        CCP1CONbits.CCP1M = 0b0000;     // Reset and turn off the Compare module and wait for software re-enable after 20ms are up
        CCP1IF = 0;                     // Clear interrupt flag
        TMR1ON = 0;                     // Turn timer off
    }
    
    if (CCP3IE && CCP3IF)
    {
        CCP3CONbits.CCP3M = 0b0000;     // Reset and turn off the Compare module and wait for software re-enable after 20ms are up
        CCP3IF = 0;                     // Clear interrupt flag
        TMR3ON = 0;                     // Turn the timer off
    }
    
    if (TMR2IE && TMR2IF)
    {
        elapsedMillis++;                // Increment millis counter
        TMR2IF = 0;                     // Clear interrupt flag
    }
}
