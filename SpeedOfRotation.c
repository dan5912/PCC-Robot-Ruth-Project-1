/*
 * File:   SpeedOfRotation.c
 * Author: 123709
 *
 * Created on April 21, 2016, 3:29 PM
 */


#include <xc.h>
#define RADIUS 55                       //radius in millimeters
#define HOLES 32                        //number of holes in each wheel
int rightWheelSpeed= 0;                 //global variable to track right wheel speed
int leftWheelSpeed= 0;                  //global variable to track left wheel speed
unsigned long leftWheelCount = 0;       //global variable to track hole/space count of left wheel
unsigned long rightWheelCount = 0;      //global variable to track hole/space count of right wheel
unsigned long elapsedMillis = 0;        //global variable to track time ticks in milliseconds
unsigned long speedElapsedMillis = 0;   //global variable to track instantaneous elapsed time

void main(void) 
{
    while (1)
    {
        //5.3996 is the millimeters per count for the wheel
        //tracks the instantaneous right wheel speed
        rightWheelSpeed = rightWheelCount * 5.3996 / (elapsedMillis - speedElapsedMillis); 
        //tracks the instantaneous left wheel speed
        leftWheelSpeed = leftWheelCount * 5.3996 / (elapsedMillis - speedElapsedMillis);   
        //sets the speedElapsedMillis the same as the current time tick
        speedElapsedMillis = elapsedMillis;
    }
    return;
}

void interrupt ISR()
{
    if(C1IF)                //check to see if the first interrupt flag was set
    {
        leftWheelCount++;   //if the first interrupt flag is set, left wheel count is increased by 1
        C1IF = 0;           //reset the first interrupt flag
    }
    if(C2IF)                //check to see if the second interrupt flag was set
    {
        rightWheelCount++;  //if the second interrupt flag is set, right wheel count is increased by 1
        C2IF = 0;           //reset the second interrupt flag
    }
}
