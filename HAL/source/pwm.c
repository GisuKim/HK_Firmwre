/*
 * pwm.c
 *
 *  Created on: 2023. 1. 17.
 *      Author: kskwi
 */

#include "HAL/include/pwm.h"


void Init_PWM_GPIO(void)
{
    // Configure GPIO
    //TimerB PWM GPIO

    P3DIR |= BIT4 | BIT5 | BIT6 | BIT7;                   // P3.4, P3.5 and P3.6 output
    P3SEL0 |= BIT4 | BIT5 | BIT6 | BIT7;                  // P3.4, P3.5 and P3.6 options select
    P3SEL1 &= ~(BIT4 | BIT5 | BIT6 | BIT7);

    P1DIR |= BIT4;                                  // P1.4 Channel 5
    P1SEL0 |= BIT4;                                 // P1.4 options select
    P1SEL1 &= ~(BIT4);

}
