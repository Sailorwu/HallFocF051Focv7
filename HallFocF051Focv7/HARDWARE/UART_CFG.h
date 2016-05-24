#ifndef _UART_CFG_H_
#define _UART_CFG_H_


#include "system_define.h"


void UART_Iinitialization(void);
void Send_data(void);
void UART_FromPc(void);
void SendDataTo(void);

extern uint8_t	RxBuffer[20],TxBuffer[20];
extern uint8_t TxDat_len,RxDat_len,UART_Dleay_temp;
extern uint8_t TXFinishFlag,RXFinishFlag;


#endif




