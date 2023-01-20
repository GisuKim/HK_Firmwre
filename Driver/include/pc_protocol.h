/*
 * pc_protocol.h
 *
 *  Created on: 2023. 1. 17.
 *      Author: kskwi
 */

#ifndef DRIVER_INCLUDE_PC_PROTOCOL_H_
#define DRIVER_INCLUDE_PC_PROTOCOL_H_

#include "HAL/include/uart.h"

extern unsigned char PC_SendMessageFlag;
extern unsigned char PC_reQuestFlag;
extern unsigned char AT_NORMAL_FLAG;
extern unsigned char AT_STOP_FLAG;
extern unsigned char AT_START_FLAG;
extern unsigned char SaveMasterOnOffState;
extern unsigned char SaveSlaveOnOffState;
extern unsigned short int AT_currentTemp;
extern unsigned char Selected_Channel;

extern signed char TempVal[10];
extern char TargetTemp[10];


extern unsigned char P_CastingBuffer[4][4];
extern unsigned char I_CastingBuffer[4][4];
extern unsigned char D_CastingBuffer[4][4];

extern unsigned char pid_CastingBuffer[4][4];


void MessageTx(void);
void Tx_PID_Tuning(unsigned char commandTx, unsigned char SelectedChannel);

#endif /* DRIVER_INCLUDE_PC_PROTOCOL_H_ */
