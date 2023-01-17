/*
 * i2c.c
 *
 *  Created on: 2023. 1. 17.
 *      Author: kskwi
 */

#include "HAL/include/i2c.h"

void i2cInit(void){
    P7SEL1 &= ~BIT1;    // SCL
    P7SEL0 |= BIT1;

    P7SEL1 &= ~BIT0;    // SDA
    P7SEL0 |= BIT0;

    UCB2CTLW0 |= UCSWRST;   // Enters reset state, USCI stops operation

    //UCB2TBCNT = UCTBCNT3;   // Expecting to receive 3 bytes of data
    UCB2CTLW1 |= UCASTP_2;  // Sends stop bit when UCTBCNT is reached

    UCB2CTLW0 |= UCMST      // Master Mode
              |  UCMODE_3   // I2C Mode
              |  UCSSEL_3;  // Sets SMCLK as source
    UCB2BRW    = 0x0028;    // SMCLK/10

    UCB2CTLW0 &= ~UCSWRST;  // Exits reset mode, USCI enters operation
    UCB2IE    |= UCTXIE0    // Data received interrupt enable
              |  UCRXIE0    // Data ready to transmit interrupt enable
              |  UCNACKIE;  // NACK interrupt enable
}
