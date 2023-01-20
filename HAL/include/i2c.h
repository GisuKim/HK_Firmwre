/*
 * i2c.h
 *
 *  Created on: 2023. 1. 17.
 *      Author: kskwi
 */

#ifndef HAL_INCLUDE_I2C_H_
#define HAL_INCLUDE_I2C_H_

#include "msp430.h"
#include "stdint.h"

int TB_Sensor_Flag;

#define TBP_ADDR 0x3A

int     RX_Byte_Ctr,        // Coutner to make sure all of the information is received
        TX_Byte_Ctr,        // Counter to make sure all of the information is sent
        i;                  // Integer used for counting sent bytes
char    i2cData[3],          // Creates an array to store data
        //sending[1],         // Creates an array to store the sent data
        *pointer,           // Creates a pointer to access the array
        *txPtr;

int16_t _rawObject;
int16_t result_object;
//int16_t _rawSensor;
uint8_t BUF[5];
int16_t *dest_call;
int PEC_ture;


void i2cInit(void);
void getData(void);
#endif /* HAL_INCLUDE_I2C_H_ */
