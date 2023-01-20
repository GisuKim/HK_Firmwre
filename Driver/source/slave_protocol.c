/*
 * slave_protocol.c
 *
 *  Created on: 2023. 1. 20.
 *      Author: kskwi
 */

#include "Driver/include/slave_protocol.h"

void ConnectMsgSendToSlave(void)
{
    fput_data2(0xfd);
    fput_data2(0xB0);

    fput_data2(0x00);
    fput_data2(0x00);

    fput_data2(0x00);
    fput_data2(0x00);

    fput_data2(0x00);
    fput_data2(0x00);

    fput_data2(0x00);
    fput_data2(0x00);

    fput_data2(0x00);
    fput_data2(0x00);

    fput_data2(0x00);

    fput_data2(0xa5);
}

void ConnectAckMsgSendToSlave(void)
{
    fput_data2(0xfd);   //0 STX
    fput_data2(0xb1);   //1 MODE

    fput_data2(0x00);   //2
    fput_data2(0x00);   //3

    fput_data2(0x00);   //4
    fput_data2(0x00);   //5

    fput_data2(0x00);   //6
    fput_data2(0x00);   //7

    fput_data2(0x00);   //8
    fput_data2(0x00);   //9

    fput_data2(0x00);   //10
    fput_data2(0x00);   //11

    fput_data2(0x00);   //12

    fput_data2(0xa5);   //13 ETX

}

void sendCommandToSlave(Data data)
{
    fput_data2(0xfd);   //0 STX
    fput_data2(0xb3);   //1 MODE

    fput_data2(data.data[0]);   //2
    fput_data2(data.data[1]);   //3

    fput_data2(data.data[2]);   //4
    fput_data2(data.data[3]);   //5

    fput_data2(data.data[4]);   //6
    fput_data2(data.data[5]);   //7

    fput_data2(data.data[6]);   //8
    fput_data2(data.data[7]);   //9

    fput_data2(data.data[8]);   //10
    fput_data2(data.data[9]);   //11

    fput_data2(data.data[10]);   //12

    fput_data2(0xa5);   //13 ETX

}



void MessageTxMaster(void)
{
    // Verify Request connect (DP 1.2)
    if(DP_SendMessageFlag && (DP_reQuestFlag || rxReFlag))
    {
        //UCA1IE &= ~UCRXIE;
        //UCA3IE &= ~UCRXIE;
        rxReFlag = 0;
        fput_data2(0xfd);   //0 STX
        fput_data2(0xa0);   //1 MODE ACK

        fput_data2(0x00);   //2
        fput_data2(0x00);   //3

        fput_data2(0x00);   //4
        fput_data2(0x00);   //5

        fput_data2(0x00);   //6
        fput_data2(0x00);   //7

        fput_data2(0x00);   //8
        fput_data2(0x00);   //9

        fput_data2(0x00);   //10
        fput_data2(0x00);   //11

        fput_data2(0x00);   //12
        fput_data2(0xa5);   //13 ETX

        DP_SendMessageFlag = 0;

        //UCA3IE |= UCRXIE;
        //UCA1IE |= UCRXIE;                  // prevent to occur overrun (rx)
    }

    // Send message about current Temperature and target temperature (DP 2.1 / 3.2)
    if(DP_SendMessageFlag == 1 && DP_NORMAL_FLAG == 1)
    {

        fput_data2(0xfd);                    //0 STX
        fput_data2(0xa1);                    //1 MODE

        fput_data2(TempVal[0]);              //2
        fput_data2(TargetTemp[0]);           //3

        fput_data2(TempVal[1]);              //4
        fput_data2(TargetTemp[1]);           //5

        fput_data2(TempVal[2]);              //6
        fput_data2(TargetTemp[2]);           //7

        fput_data2(TempVal[3]);              //8
        fput_data2(TargetTemp[3]);           //9

        fput_data2(TempVal[4]);              //10
        fput_data2(TargetTemp[4]);           //11

        fput_data2(SaveMasterOnOffState);    //12
        fput_data2( 0xa5 );                  //13 ETX

        DP_SendMessageFlag = 0;

            //UCA3IE |= UCRXIE;
            //UCA1IE |= UCRXIE;                  // prevent to occur overrun (rx)
    }
}


void MessageTxSlave(void)
{
    // Verify Request connect (DP 1.2)
    if(DP_SendMessageFlag && (DP_reQuestFlag || rxReFlag))
    {
        //UCA1IE &= ~UCRXIE;
        //UCA3IE &= ~UCRXIE;
        rxReFlag = 0;
        fput_data2(0xfb);   //0 STX
        fput_data2(0xa0);   //1 MODE ACK

        fput_data2(0x00);   //2
        fput_data2(0x00);   //3

        fput_data2(0x00);   //4
        fput_data2(0x00);   //5

        fput_data2(0x00);   //6
        fput_data2(0x00);   //7

        fput_data2(0x00);   //8
        fput_data2(0x00);   //9

        fput_data2(0x00);   //10
        fput_data2(0x00);   //11

        fput_data2(0x00);   //12
        fput_data2(0xa5);   //13 ETX

        DP_SendMessageFlag = 0;

        //UCA3IE |= UCRXIE;
        //UCA1IE |= UCRXIE;                  // prevent to occur overrun (rx)
    }

    // Send message about current Temperature and target temperature (DP 2.1 / 3.2)
    if(DP_SendMessageFlag == 1 && DP_NORMAL_FLAG == 1)
    {

        if( ((tempIntVal_ch1 & (0xFF << 8)) >> 8)  < 0x03 && ((tempIntVal_ch2 & (0xFF << 8)) >> 8)  < 0x03
                && ((tempIntVal_ch3 & (0xFF << 8)) >> 8)  < 0x03 && ((tempIntVal_ch4 & (0xFF << 8)) >> 8)  < 0x03
                && ((targetIntVal_ch1 & (0xFF << 8)) >> 8) < 0x03 && ((targetIntVal_ch2 & (0xFF << 8)) >> 8) < 0x03
                && ((targetIntVal_ch3 & (0xFF << 8)) >> 8) < 0x03 && ((targetIntVal_ch4 & (0xFF << 8)) >> 8) < 0x03
        )

        //if(tempIntVal_ch1 < 0x2BC && tempIntVal_ch1 < 0x)
        {
            //UCA1IE &= ~UCRXIE;                  // prevent to occur overrun (rx)
            //UCA3IE &= ~UCRXIE;

            fput_data2(0xfb);

            fput_data2(0xA0);

            fput_data2( (tempIntVal_ch1 & (0xFF << 8)) >> 8 );
            fput_data2( (tempIntVal_ch1 & (0xFF << 0)) >> 0 );

            fput_data2( (targetIntVal_ch1 & (0xFF << 8)) >> 8 );
            fput_data2( (targetIntVal_ch1 & (0xFF << 0)) >> 0 );

            fput_data2( (tempIntVal_ch2 & (0xFF << 8)) >> 8 );
            fput_data2( (tempIntVal_ch2 & (0xFF << 0)) >> 0 );

            fput_data2( (targetIntVal_ch2 & (0xFF << 8)) >> 8 );
            fput_data2( (targetIntVal_ch2 & (0xFF << 0)) >> 0 );
            //fput_data2( (targetIntVal_ch2 & (0xFF << 8)) >> 8 );
            //fput_data2( (targetIntVal_ch2 & (0xFF << 0)) >> 0 );

            fput_data2( (tempIntVal_ch3 & (0xFF << 8)) >> 8 );
            fput_data2( (tempIntVal_ch3 & (0xFF << 0)) >> 0 );

            fput_data2( (targetIntVal_ch3 & (0xFF << 8)) >> 8 );
            fput_data2( (targetIntVal_ch3 & (0xFF << 0)) >> 0 );
            //fput_data2( (targetIntVal_ch3 & (0xFF << 8)) >> 8 );
            //fput_data2( (targetIntVal_ch3 & (0xFF << 0)) >> 0 );

            fput_data2( (tempIntVal_ch4 & (0xFF << 8)) >> 8 );
            fput_data2( (tempIntVal_ch4 & (0xFF << 0)) >> 0 );

            fput_data2( (targetIntVal_ch4 & (0xFF << 8)) >> 8 );
            fput_data2( (targetIntVal_ch4 & (0xFF << 0)) >> 0 );
            //fput_data2( (targetIntVal_ch4 & (0xFF << 8)) >> 8 );
            //fput_data2( (targetIntVal_ch4 & (0xFF << 0)) >> 0 );

            //fput_data2( (tempIntVal_i2c & (0xFF << 8)) >> 8 );
            //fput_data2( (tempIntVal_i2c & (0xFF << 0)) >> 0 );

            fput_data2( SaveMasterOnOffState );

            fput_data2( 0xa5 );

            DP_SendMessageFlag = 0;

            //UCA3IE |= UCRXIE;
            //UCA1IE |= UCRXIE;                  // prevent to occur overrun (rx)
        }
    }
}
