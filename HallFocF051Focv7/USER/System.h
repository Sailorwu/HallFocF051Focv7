#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include "system_define.h"
 
#define PWM_FREQ  	PWMFREQUENCY 
#define PWM_PERIOD 	((SystemCoreClock / (2*PWM_FREQ) ) - 1)
#define DEADTIME		DDEADTIME
#define T2MSTEMP		(PWM_FREQ/500)
#define T100MSTEMP 	(PWM_FREQ/10) 
#define AMPLITUDE_MIN 5000
#define VF_CURVE		80
#define CURRENTMAX	3500 		//4096 4.4A
#define SHORTI_TIME	10			//1S

/*
#define IDC_Channl			1		//?
#define IA_Channl				2
#define IB_Channl				3
#define IC_Channl				4		//?
#define V_BEMF_U				5	
#define V_BEMF_V				6
#define V_BEMF_W				7
#define SP_Channl				8
#define VDC_Channl			0
*/
#define IA_Channl				0
#define IB_Channl				1
#define SP_Channl				2
#define VDC_Channl			3
 
#define LED1_ON()     GPIO_SetBits(GPIOC,GPIO_Pin_14)
#define LED1_OFF()    GPIO_ResetBits(GPIOC,GPIO_Pin_14)
#define LED1Toggle()	GPIOC->ODR^=GPIO_Pin_14

#define LED2_ON()     GPIO_SetBits(GPIOC,GPIO_Pin_15)
#define LED2_OFF()   	GPIO_ResetBits(GPIOC,GPIO_Pin_15)
#define LED2Toggle()	GPIOC->ODR^=GPIO_Pin_15

#define REALY_ON()		GPIO_SetBits(GPIOC,GPIO_Pin_15)
#define REALY_OFF()		GPIO_ResetBits(GPIOC,GPIO_Pin_15)

#define HALL_DATA() 	((GPIO_ReadInputData(GPIOC)>>6)&0x07)


void System_Ctrl(void);
void OverCurrent(void);
void Open_PWM(void);
void Close_PWM(void);
#endif
