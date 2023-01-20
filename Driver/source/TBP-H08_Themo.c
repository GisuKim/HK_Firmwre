/*
 * TBP-H08_Themo.c
 *
 *  Created on: 2023. 1. 20.
 *      Author: kskwi
 */


#include "Driver/include/TBP-H08_Themo.h"

float CalcTemp(int rawTemp)
{
  float retTemp;
  retTemp = ((float)rawTemp)*0.02;
  retTemp -= 273.15;
  return retTemp;
}

uint8_t GetObject(void)                   //i2c ¿Âµµ
{
  int16_t rawObj = 0;
  if(PEC_ture == 1)
  {
    if (rawObj & 0x8000)
    {
      return 0;
    }
    result_object = _rawObject;
    return 1;
  }
}

uint8_t CalPEC(uint8_t *crc, uint8_t nBytes)
{
  uint8_t data, count;
  uint16_t remainder = 0;

  for(count=0; count<nBytes; ++count)
  {
     data = *(crc++) ^ remainder;
     remainder = crc8_table[data] ^ (remainder >> 8);
  }
  return (uint8_t)remainder;
}
