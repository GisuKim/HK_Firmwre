/*
 * slave_protocol.h
 *
 *  Created on: 2023. 1. 20.
 *      Author: kskwi
 */

#ifndef DRIVER_INCLUDE_SLAVE_PROTOCOL_H_
#define DRIVER_INCLUDE_SLAVE_PROTOCOL_H_

#include "HAL/include/uart.h"

extern unsigned char DP_SendMessageFlag;
extern unsigned char DP_reQuestFlag;
extern unsigned char rxReFlag;
extern unsigned char DP_NORMAL_FLAG;

extern unsigned char SaveMasterOnOffState;

extern unsigned short int tempIntVal_ch1;
extern unsigned short int targetIntVal_ch1;

extern unsigned short int tempIntVal_ch2;
extern unsigned short int targetIntVal_ch2;

extern unsigned short int tempIntVal_ch3;
extern unsigned short int targetIntVal_ch3;

extern unsigned short int tempIntVal_ch4;
extern unsigned short int targetIntVal_ch4;

extern signed char TempVal[10];
extern char TargetTemp[10];
void ConnectMsgSendToSlave(void);
void ConnectAckMsgSendToSlave(void);
void MessageTxMaster(void);
void MessageTxSlave(void);
void sendCommandToSlave(Data data);
#endif /* DRIVER_INCLUDE_SLAVE_PROTOCOL_H_ */
