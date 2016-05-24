#include "Contrl.h"

uint16_t SpeedData,SpeedCnt,SetSpeed;
int16_t DCbus,VQ_SetValue;
uint8_t Motor_State,FOC_Flag,ShortI_Flag=0,ShortI_Counter;
int16_t hPhaseAOffset,hPhaseBOffset;	
int16_t SPOffset,VoltageOffset;
uint16_t SB_FilterHe,SB_Temp;
uint8_t T2ms_Flag,T100ms_Flag;
uint16_t T2ms_Temp,T100ms_Temp;
uint8_t Hall_Temp,Hall_TempOld;
const uint16_t Hall_Angle[8]={0,9*A30D,A30D,11*A30D,5*A30D,7*A30D,3*A30D,0};		//
const uint8_t HallTemp[8]={1,3,6,2,5,1,4,1};
const uint8_t HallData[8];

LPF_16PARAMETERS SB_Filter_t,DC_Filter_t,MotorSpeed_t;

uint16_t THallAngle,TDelta_angle,TPWM_Cnt[8],TPWM_Num[8];
uint16_t PLL_Angle;
uint8_t TRun_Flag,TCnt_Num;
int16_t AngleError;
	
void Main_Loop(void)
{
	static uint8_t SpeedLoopcnt;
	static uint32_t LsData1,LsData2;
	if(T2ms_Flag)
	{		
		T2ms_Flag=0;
		ReadSignedADC0( &ReadADCParm );

		SB_Temp=SB_FilterHe;					//12bit
		
		if(SB_Temp>200) 		//
		{
			if(Motor_State!=RUN) 
			{
				Motor_Start();
			}	
		}	
		else if(SB_Temp>190) ;
		else 
		{
			if(Motor_State!=WAIT) Motor_Stop();
		}
		
		if(++SpeedLoopcnt>1)		
		{
			SetSpeed=SB_Temp+200;    //100;//
			if(SetSpeed>NOMINALSPEEDINRPM) SetSpeed=NOMINALSPEEDINRPM;
			
			SpeedLoopcnt=0;
			LsData1=60*PWMFREQUENCY;
			LsData2=SpeedCnt*POLEPAIRS;
			SpeedData=LsData1/LsData2;
#ifdef OPEN_I			
			LsData1=(Curr_q_d.qI_Component1*VQ_Reference);
			LsData1/=32768;	
			if(LsData1>POWER_MAX) VQ_Reference-=100;	
			else 
			{
				VQ_Reference=PID_Regulator(SetSpeed,SpeedData,&SPEED_PID_t);			
			}
#else			
			IQ_Reference=PID_Regulator(SetSpeed,SpeedData,&SPEED_PID_t);
#endif
		}
	}
	
	if(T100ms_Flag)
	{
		//LED2Toggle();
		T100ms_Flag=0;	
	}
	
}

void HallPLLCtrl(void)
{
	static uint8_t ii,jj;
	int32_t LSdata1,LSdata2;
	LED2Toggle();
	Hall_Temp=HALL_DATA();
	//if(Hall_TempOld!=Hall_Temp)
	if(Hall_Temp==HallTemp[Hall_TempOld])
	{
		Hall_TempOld=Hall_Temp;
				
		TPWM_Num[Hall_TempOld]=TPWM_Cnt[Hall_TempOld];
		TPWM_Cnt[Hall_TempOld]=0;
//Speed		
 		LSdata1=0;
 		for(jj=0;jj<6;jj++) LSdata1+=TPWM_Num[jj+1];
		SpeedCnt=LSdata1/6;
//Angle		
		TDelta_angle=65536/TPWM_Num[Hall_TempOld];
		
		if(TCnt_Num>100) 															//100次霍尔换向允许切换到360度校准一次模式
		{
			if(SpeedData>600) TRun_Flag=1;							//600RPM次霍尔换向切换到360度校准一次模式
			else TRun_Flag=0;
		}
		else TCnt_Num++;
		
		if(TRun_Flag) 		
		{
			Delta_angle=TDelta_angle;  
			AngleError=SVM_Angle-Hall_Angle[Hall_TempOld];
			//AngleError=_Q15abs(AngleError);
			
			if(Hall_TempOld==2) 
			{
				SVM_Angle=Hall_Angle[Hall_TempOld];
			} 
			LED1_ON();			
		}
		else 
		{
			if(PWM_Cnt) Delta_angle=ANGLE_60D/PWM_Cnt;
			else Delta_angle=1;
			
			if(TCnt_Num<10) 														//方波启动
			{
				SVM_Angle=Hall_Angle[Hall_TempOld]+ANGLE_60D/2;
				Delta_angle=0;
			}
			else																				//10个霍尔变化之后切入FOC运行
			{
				SVM_Angle=Hall_Angle[Hall_TempOld];
			}
			LED1_OFF();			
		}	
		
		PWM_Cnt=0;
		Limit_angle=0;
	}
	else if(Hall_TempOld==HallTemp[Hall_Temp])
	{
		Hall_TempOld=Hall_Temp;
		TCnt_Num=0;
		Delta_angle=0;
		SVM_Angle=Hall_Angle[Hall_TempOld]+ANGLE_60D/2;
		PWM_Cnt=0xFFFF;
	}
	else{}
}

void HardwareInit(void)
{
	GPIO_Iinitialization();
	TMER_Iinitialization();
	UART_Iinitialization();
	ADC_Iinitialization();
	Motor_Init();
	DAC_Iinitialization();
}

void Motor_Init(void)
{
	Motor_State=INIT;
	SB_Filter_t.acc=0;
	//*************PID INIT*************
	ID_PID_t.PI_Kp=ID_KP;
	ID_PID_t.PI_Ki=ID_KI;
	ID_PID_t.PI_Kd=ID_KD;
	ID_PID_t.PI_Kc=ID_KC;
	ID_PID_t.PI_Outmax=ID_MAX;
	ID_PID_t.PI_Outmin=-10000;					
	
	IQ_PID_t.PI_Kp=IQ_KP;
	IQ_PID_t.PI_Ki=IQ_KI;
	IQ_PID_t.PI_Kd=IQ_KD;
	IQ_PID_t.PI_Kc=IQ_KC;
	IQ_PID_t.PI_Outmax=IQ_MAX;
	IQ_PID_t.PI_Outmin=200;
	
	SPEED_PID_t.PI_Kp=SP_KP;
	SPEED_PID_t.PI_Ki=SP_KI;
	SPEED_PID_t.PI_Kd=SP_KD;
	SPEED_PID_t.PI_Kc=SP_KC;
	SPEED_PID_t.PI_Outmax=SP_MAX;
	SPEED_PID_t.PI_Outmin=SP_MIN;
	ID_PID_t.PI_Sum=0;
	IQ_PID_t.PI_Sum=0;
}

void Motor_Start(void)
{
	uint8_t ii;
	Motor_Stop();
	Hall_Temp=HALL_DATA();
	Hall_TempOld=Hall_Temp;
	SVM_Angle=Hall_Angle[Hall_TempOld]+ANGLE_60D/2;
	THallAngle=SVM_Angle;
	PWM_Cnt=1000;
	
	TRun_Flag=0;
	TCnt_Num=0;
	for(ii=0;ii<6;ii++)  
	{
		TPWM_Cnt[ii+1]=0xFFFF;
		TPWM_Num[ii+1]=0xFFFF;
	}
	
	Open_PWM();
	Motor_State=RUN;
	ShortI_Flag=0;
	FOC_Flag=1;

	SpeedCnt=5000;
	SpeedData=0;
	ID_Reference=0;
	VQ_Reference=1000;
	IQ_Reference=1000;
	ID_PID_t.PI_Sum=0;
	IQ_PID_t.PI_Sum=0;
	SPEED_PID_t.PI_Sum=32768*2000;
}

void Motor_Stop(void)
{
	Close_PWM();
	Motor_State=WAIT;
	FOC_Flag=0;
}
