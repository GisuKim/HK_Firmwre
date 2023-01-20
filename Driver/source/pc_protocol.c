/*
 * pc_protocol.c
 *
 *  Created on: 2023. 1. 17.
 *      Author: kskwi
 */

#include "Driver/include/pc_protocol.h"


void MessageTx(void)
{
    // Verify Request connect (PC 1.2)
    if(PC_SendMessageFlag && PC_reQuestFlag)
    {
        //UCA1IE &= ~UCRXIE;
        //UCA2IE &= ~UCRXIE;

        fput_data(0xfe);

        fput_data(0xB0);

        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);

        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);

        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);

        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);

        fput_data(0xa5);

        PC_SendMessageFlag = 0;

        //UCA2IE |= UCRXIE;
        //UCA1IE |= UCRXIE;                  // prevent to occur overrun (rx)
    }

    // Send message about current Temperature and target temperature (PC 1.4 / 2.1 / 3.1)
    if(PC_SendMessageFlag == 1 && AT_NORMAL_FLAG == 1)
    {


        //UCA1IE &= ~UCRXIE;
        //UCA3IE &= ~UCRXIE;

        fput_data(0xfe);            //0

        fput_data(0x90);            //1

        fput_data(TempVal[0]);      //2
        fput_data(TargetTemp[0]);   //3

        fput_data(TempVal[1]);      //4
        fput_data(TargetTemp[1]);   //5

        fput_data(TempVal[2]);      //6
        fput_data(TargetTemp[2]);   //7

        fput_data(TempVal[3]);      //8
        fput_data(TargetTemp[3]);   //9

        fput_data(TempVal[4]);      //10
        fput_data(TargetTemp[4]);   //11

        fput_data(TempVal[5]);      //12
        fput_data(TargetTemp[5]);   //13

        fput_data(TempVal[6]);      //14
        fput_data(TargetTemp[6]);   //15

        fput_data(TempVal[7]);      //16
        fput_data(TargetTemp[7]);   //17

        fput_data(TempVal[8]);      //18
        fput_data(TargetTemp[8]);   //19

        fput_data(TempVal[9]);      //20
        fput_data(TargetTemp[9]);   //21

        fput_data( SaveMasterOnOffState );  //22
        fput_data( SaveSlaveOnOffState );   //23

        fput_data( 0xa5 );                  //24

        PC_SendMessageFlag = 0;

        //UCA3IE |= UCRXIE;
        //UCA1IE |= UCRXIE;                  // prevent to occur overrun (rx)

    }

    // Send message about current Temperature in Tuning Start (4.2)
    if(PC_SendMessageFlag && (AT_START_FLAG))
    {
        if( ((AT_currentTemp & (0xFF << 8)) >> 8)  < 0x03 )
        {
            //UCA1IE &= ~UCRXIE;
            //UCA3IE &= ~UCRXIE;                  // prevent to occur overrun (rx)

            fput_data(0xfe);                                            // 1

            fput_data(0x92);                                            // 2

            fput_data( (AT_currentTemp & (0xFF << 8)) >> 8 );            // 3
            fput_data( (AT_currentTemp & (0xFF << 0)) >> 0 );            // 4

            //fput_data( (AT_curretTemp & (0xFF << 8)) >> 8 );
            //fput_data( (AT_curretTemp & (0xFF << 0)) >> 0 );

            fput_data(0x00);                                            // 5
            fput_data(0x00);                                            // 6
            fput_data(0x00);                                            // 7
            fput_data(0x00);                                            // 8
            fput_data(0x00);                                            // 9
            fput_data(0x00);                                            // 10

            fput_data(0x00);                                            // 11
            fput_data(0x00);                                            // 12
            fput_data(0x00);                                            // 13
            fput_data(0x00);                                            // 14
            fput_data(0x00);                                            // 15

            fput_data(0x00);                                            // 16
            fput_data(0x00);                                            // 17
            fput_data(0x00);                                            // 18
            fput_data(0x00);                                            // 19
            fput_data(0x00);                                            // 20

            //fput_data2( SaveOnOffState );

            fput_data(Selected_Channel);                                  // 21

            fput_data( 0xa5 );                                          // 22

            PC_SendMessageFlag = 0;

            //UCA3IE |= UCRXIE;
            //UCA1IE |= UCRXIE;                  // prevent to occur overrun (rx)
        }
    }

    // Send message about Tuning Stop (4.5)
    if(PC_SendMessageFlag && AT_STOP_FLAG)
    {
        //UCA1IE &= ~UCRXIE;
        //UCA3IE &= ~UCRXIE;                  // prevent to occur overrun (rx)

        fput_data(0xfe);                                            // 1

        fput_data(0x91);                                            // 2

        fput_data(0x00);                                            // 3
        fput_data(0x00);                                            // 4

        //fput_data( (AT_curretTemp & (0xFF << 8)) >> 8 );
        //fput_data( (AT_curretTemp & (0xFF << 0)) >> 0 );

        fput_data(0x00);                                            // 5
        fput_data(0x00);                                            // 6
        fput_data(0x00);                                            // 7
        fput_data(0x00);                                            // 8
        fput_data(0x00);                                            // 9
        fput_data(0x00);                                            // 10

        fput_data(0x00);                                            // 11
        fput_data(0x00);                                            // 12
        fput_data(0x00);                                            // 13
        fput_data(0x00);                                            // 14
        fput_data(0x00);                                            // 15

        fput_data(0x00);                                            // 16
        fput_data(0x00);                                            // 17
        fput_data(0x00);                                            // 18
        fput_data(0x00);                                            // 19
        fput_data(0x00);                                            // 20

        //fput_data2( SaveOnOffState );

        fput_data(Selected_Channel);                                  // 21

        fput_data( 0xa5 );                                          // 22

        PC_SendMessageFlag = 0;

        //UCA3IE |= UCRXIE;
        //UCA1IE |= UCRXIE;                  // prevent to occur overrun (rx)
    }
}


void Tx_PID_Tuning(unsigned char commandTx, unsigned char SelectedChannel)
{
    unsigned char tIndex = 0;
    if(commandTx == 1)
    {
        // save to return one cycle

        fput_data(0xfe);

        fput_data(0x90);

        fput_data( P_CastingBuffer[SelectedChannel][3] );     // P-Gain
        fput_data( P_CastingBuffer[SelectedChannel][2] );     // P-Gain
        fput_data( P_CastingBuffer[SelectedChannel][1] );     // P-Gain
        fput_data( P_CastingBuffer[SelectedChannel][0] );     // P-Gain

        fput_data( I_CastingBuffer[SelectedChannel][3] );     // I-Gain
        fput_data( I_CastingBuffer[SelectedChannel][2] );     // I-Gain
        fput_data( I_CastingBuffer[SelectedChannel][1] );     // I-Gain
        fput_data( I_CastingBuffer[SelectedChannel][0] );     // I-Gain

        fput_data( D_CastingBuffer[SelectedChannel][3] );     // D-Gain
        fput_data( D_CastingBuffer[SelectedChannel][2] );     // D-Gain
        fput_data( D_CastingBuffer[SelectedChannel][1] );     // D-Gain
        fput_data( D_CastingBuffer[SelectedChannel][0] );     // D-Gain

        fput_data(SelectedChannel);     // Channel      0~3 (4-channels)

        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);

        fput_data(0x07);        // block command

        fput_data(0xa5);
    }
    if(commandTx == 2)
    {
        // save to return four cycle (all channels)
        for(tIndex = 0; tIndex < 4; tIndex++)
        {
            fput_data(0xfe);

            fput_data(0x90);

            fput_data( P_CastingBuffer[tIndex][3] );     // P-Gain
            fput_data( P_CastingBuffer[tIndex][2] );     // P-Gain
            fput_data( P_CastingBuffer[tIndex][1] );     // P-Gain
            fput_data( P_CastingBuffer[tIndex][0] );     // P-Gain

            fput_data( I_CastingBuffer[tIndex][3] );     // I-Gain
            fput_data( I_CastingBuffer[tIndex][2] );     // I-Gain
            fput_data( I_CastingBuffer[tIndex][1] );     // I-Gain
            fput_data( I_CastingBuffer[tIndex][0] );     // I-Gain

            fput_data( D_CastingBuffer[tIndex][3] );     // D-Gain
            fput_data( D_CastingBuffer[tIndex][2] );     // D-Gain
            fput_data( D_CastingBuffer[tIndex][1] );     // D-Gain
            fput_data( D_CastingBuffer[tIndex][0] );     // D-Gain

            fput_data(tIndex);     // Channel      0~3 (4-channels)

            fput_data(0x00);
            fput_data(0x00);
            fput_data(0x00);
            fput_data(0x00);
            fput_data(0x00);

            fput_data(0x07);        // block command

            fput_data(0xa5);
        }
    }
    if(commandTx == 3)
    {
        // save to return one cycle about SelectedChannel

        fput_data(0xfe);

        fput_data(0x90);

        fput_data( P_CastingBuffer[SelectedChannel][3] );     // P-Gain
        fput_data( P_CastingBuffer[SelectedChannel][2] );     // P-Gain
        fput_data( P_CastingBuffer[SelectedChannel][1] );     // P-Gain
        fput_data( P_CastingBuffer[SelectedChannel][0] );     // P-Gain

        fput_data( I_CastingBuffer[SelectedChannel][3] );     // I-Gain
        fput_data( I_CastingBuffer[SelectedChannel][2] );     // I-Gain
        fput_data( I_CastingBuffer[SelectedChannel][1] );     // I-Gain
        fput_data( I_CastingBuffer[SelectedChannel][0] );     // I-Gain

        fput_data( D_CastingBuffer[SelectedChannel][3] );     // D-Gain
        fput_data( D_CastingBuffer[SelectedChannel][2] );     // D-Gain
        fput_data( D_CastingBuffer[SelectedChannel][1] );     // D-Gain
        fput_data( D_CastingBuffer[SelectedChannel][0] );     // D-Gain

        fput_data(SelectedChannel);     // Channel      0~3 (4-channels)

        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);

        fput_data(0x07);        // block command

        fput_data(0xa5);
    }

}
