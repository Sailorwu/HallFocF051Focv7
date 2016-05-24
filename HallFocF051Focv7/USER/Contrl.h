#ifndef _CONTRL_H_
#define _CONTRL_H_

#include "system_define.h"
//#define OPEN_I

#define INIT 		0
#define START 	1
#define RUN 		2
#define BREAK 	3
#define STOP 		4
#define WAIT    5
#define DOWN		6

#define SPVMIN	100
#define SPVDTA	10
#define SPVMAX	600

#define ANGLE_60D	10922
#define A30D		(0xFFFF/12)

#define VQ_MIN	1000
#define VQ_MAX	32700
#define PLLMAX_ERROR	16384
#define POWER_MAX	8000


typedef struct LPF_32Parameters
{
	int32_t acc;
	int32_t acc_pre;
}LPF_32PARAMETERS;

typedef struct LPF_16Parameters
{
	int16_t acc;
	int16_t acc_pre;
}LPF_16PARAMETERS;

extern uint16_t SpeedData,SpeedCnt,SetSpeed;
extern uint8_t Hall_Temp,Hall_TempOld,SpeedIO,SpeedIOOld;
extern const uint16_t Hall_Angle[8];
extern const uint8_t HallData[8];
extern const uint8_t HallTemp[8];
extern int16_t DCbus,VQ_SetValue;
extern uint8_t Motor_State,FOC_Flag,ShortI_Flag,ShortI_Counter;
extern int16_t hPhaseAOffset,hPhaseBOffset;
extern int16_t SPOffset,VoltageOffset;
extern int16_t OMEGA_Old,Error_OMEGA;
extern uint8_t FOC_Flag;
extern uint16_t SB_FilterHe,SB_Temp;
extern LPF_16PARAMETERS SB_Filter_t,DC_Filter_tMotorSpeed_t;
extern uint8_t T2ms_Flag,T100ms_Flag;
extern uint16_t T2ms_Temp,T100ms_Temp;

extern uint8_t TRun_Flag,TCnt_Num;
extern uint16_t THallAngle,TDelta_angle,TPWM_Cnt[],TPWM_Num[];
extern uint16_t PLL_Angle;
extern int16_t ErrorAngle,HallError;

void HallPLLCtrl(void);
void HardwareInit(void);
void GPIO_Iinitialization(void);
void DAC_Iinitialization(void);
void Motor_Init(void);
void Motor_Start(void);
void Motor_Stop(void);
void Motor_Run(void);
void Main_Loop(void);
void Motor_Break(void);
void ADCTemp_Init(uint16_t* ADCnum);
int32_t SP_32LPF(LPF_32PARAMETERS * p_LPF_input, int32_t x_in,uint8_t Z);
int16_t SP_16LPF(LPF_16PARAMETERS * p_LPF_input, int16_t x_in,uint8_t Z);
void SpeedCalculate(void);
#endif

