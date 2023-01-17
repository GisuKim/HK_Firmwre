/*
 * LNJT103F_NTC.c
 *
 *  Created on: 2023. 1. 16.
 *      Author: kskwi
 */

unsigned int temp_adc_table[] = { 74,94,119,149,187,232,286,351,427,515,616,730,857,997,
                                         1148,1309,1476,1649,1823,1996,2166,2331,2488,2636,2774,
                                         2903,3021,3129,3226};


unsigned char temp_Calculator(unsigned int adc_val)
{
    char i, temp, offset;

    for(i=0;i<29;i++)
    {
        if(adc_val < temp_adc_table[i])
        {
            break;
        }
    }

    if(adc_val == temp_adc_table[i-1])
    {
        offset = 5;
    }
    else
    {
        offset = ((adc_val - temp_adc_table[i-1]) * 5 / (temp_adc_table[i] - temp_adc_table[i-1]));
    }

    temp = (i*5) + offset;

    return temp;

}
