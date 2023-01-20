/*
 * TBP-H08_Themo.h
 *
 *  Created on: 2023. 1. 20.
 *      Author: kskwi
 */

#ifndef DRIVER_INCLUDE_TBP_H08_THEMO_H_
#define DRIVER_INCLUDE_TBP_H08_THEMO_H_

#include "HAL/include/i2c.h"

extern const unsigned char crc8_table[256];

float CalcTemp(int rawTemp);
uint8_t GetObject(void);
uint8_t CalPEC(uint8_t *crc, uint8_t nBytes);


#endif /* DRIVER_INCLUDE_TBP_H08_THEMO_H_ */
