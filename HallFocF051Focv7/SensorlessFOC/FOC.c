#include "FOC.h"

int16_t IQ_Reference,ID_Reference,VQ_Reference;
uint16_t SVM_Angle,Delta_angle,Limit_angle;
uint16_t QEI_angle,PWM_Cnt;
uint16_t SVM_Angle,PWM_Period;
int16_t swSpeedErr,swSpeedErr1,swSpeedErr2;

static Trig_Components Vector_Components;
Curr_Components Curr_a_b;              /*Stator currents Ia,Ib*/ 
Curr_Components Curr_alfa_beta;        /*Ialpha & Ibeta, Clarke's  
                                            transformations of Ia & Ib */
Curr_Components Curr_q_d;              /*Iq & Id, Parke's transformations of 
                                            Ialpha & Ibeta, */
Volt_Components Volt_a_b;              /*Stator voltages Va, Vb*/ 
Volt_Components Volt_q_d;              /*Vq & Vd, voltages on a reference
                                            frame synchronous with the rotor flux*/
Volt_Components Volt_alfa_beta;        /*Valpha & Vbeta, RevPark transformations
                                             of Vq & Vd*/
PID_Struct_t ID_PID_t,IQ_PID_t,SPEED_PID_t;

tReadADCParm ReadADCParm;	
tMeasCurrParm MeasCurrParm;

uint16_t ADCFilter1[5],ADCFilter2[5];
uint16_t ADCFilterHe1,ADCFilterHe2;
uint8_t ADCCnt;

const uint16_t circle_limit_table[71]=     
{
32613,32310,32016,31872,31589,31314,31046,30784,30529,30404,
30158,29919,29684,29456,29343,29122,28906,28695,28488,28285,
28186,27990,27798,27610,27425,27245,27155,26980,26808,26639,
26473,26392,26230,26072,25917,25764,25614,25540,25394,25250,
25109,24970,24901,24766,24633,24501,24372,24245,24182,24058,
23936,23816,23697,23580,23522,23408,23295,23184,23075,23021,
22913,22808,22703,22600,22499,22449,22349,22251,22154,22059,
21964
};

const int16_t hSin_Cos_hTimePhAble[256] = SIN_COS_TABLE;

//---------------------FOC-------------------------------//
void InitMeasCompCurr( short Offset_a, short Offset_b )
{
	MeasCurrParm.Offseta=Offset_a;
	MeasCurrParm.Offsetb=Offset_b;
}

void ReadSignedADC0( tReadADCParm* pParm )
{
	SB_FilterHe=SP_16LPF(&SB_Filter_t,ADC_Tab[SP_Channl],8);
	pParm->qADValue=SB_FilterHe<<3;
	
	DCbus=ADC_Tab[VDC_Channl]<<3;
}

void SVPWM_2ShuntGetPhaseCurrentValues(Curr_Components* Local_Stator_Currents)
{
	Local_Stator_Currents->qI_Component1=2*(MeasCurrParm.Offseta-(int16_t)ADC_Tab[IA_Channl]*8);			//*8 MARK
	Local_Stator_Currents->qI_Component2=2*(MeasCurrParm.Offsetb-(int16_t)ADC_Tab[IB_Channl]*8);
}

void Clarke(Curr_Components* SVM_Clarke_Out,Curr_Components* SVM_Clarke_In)
{
  int32_t divSQRT3_tmp;

  // qIalpha = qIas
  SVM_Clarke_Out->qI_Component1= SVM_Clarke_In->qI_Component1;
  //qIbeta = (2*qIbs+qIas)/sqrt(3)
  divSQRT3_tmp = SVM_Clarke_In->qI_Component1+ SVM_Clarke_In->qI_Component2+ SVM_Clarke_In->qI_Component2;
	divSQRT3_tmp*=divSQRT_3;
	
  divSQRT3_tmp /=32768;
  SVM_Clarke_Out->qI_Component2=(int16_t)divSQRT3_tmp; 
}

void Park(Curr_Components* SVM_Park_Out,Curr_Components* SVM_Park_In,int16_t Theta)
{
  int32_t qIdq_tmp_1, qIdq_tmp_2;      
  int16_t qIdq_1, qIdq_2; 
	
	Trig_Functions(&Vector_Components,Theta);
  
  //No overflow guaranteed
  qIdq_tmp_1 = SVM_Park_In->qI_Component2 * Vector_Components.hCos;  	
  qIdq_tmp_1 /= 32768;
  
  //No overflow guaranteed
  qIdq_tmp_2 = SVM_Park_In->qI_Component1 *Vector_Components.hSin;
  qIdq_tmp_2 /= 32768;
 
  qIdq_1 = ((int16_t)(qIdq_tmp_1));
  qIdq_2 = ((int16_t)(qIdq_tmp_2));

  //Iq component in Q1.15 Format 
  SVM_Park_Out->qI_Component1 = ((qIdq_1)-(qIdq_2));	
  
  //No overflow guaranteed
  qIdq_tmp_1 = SVM_Park_In->qI_Component2 * Vector_Components.hSin;
  qIdq_tmp_1 /= 32768;
  
  //No overflow guaranteed
  qIdq_tmp_2 = SVM_Park_In->qI_Component1 * Vector_Components.hCos;
  qIdq_tmp_2 /= 32768;
  
  qIdq_1 = ((int16_t)(qIdq_tmp_1));
  qIdq_2 = ((int16_t)(qIdq_tmp_2));				

   //Id component in Q1.15 Format   
  SVM_Park_Out->qI_Component2 = ((qIdq_1)+(qIdq_2));  
}

int16_t PID_Regulator(int16_t hReference, int16_t hPresentFeedback, PID_Struct_t* PID_Struct)
{
	int32_t PI_Err,PI_Exc,PI_U,PI_Out;
	PI_Err  = hReference - hPresentFeedback;
	PI_U= PID_Struct->PI_Sum + PID_Struct->PI_Kp * PI_Err + PID_Struct->PI_Kd*(PI_Err-PID_Struct->OldError);
	PI_U/=32768;
	PID_Struct->OldError=PI_Err;
	
	if( PI_U > PID_Struct->PI_Outmax ) PI_Out = PID_Struct->PI_Outmax;
	else if( PI_U < PID_Struct->PI_Outmin ) PI_Out = PID_Struct->PI_Outmin;					
	else PI_Out = PI_U;
	
	PI_Exc = PI_U - PI_Out;
	PID_Struct->PI_Sum = PID_Struct->PI_Sum + PID_Struct->PI_Ki * PI_Err - PID_Struct->PI_Kc * PI_Exc;
	return((int16_t)(PI_Out));
} 

void RevPark_Circle_Limitation(Volt_Components* Limit_Input)
{
	int32_t temp;
	       
	temp = Limit_Input->qV_Component1 * Limit_Input->qV_Component1 
	       + Limit_Input->qV_Component2 * Limit_Input->qV_Component2;  // min value 0, max value 2*32767*32767
	        
	if ( temp > (int32_t)(( MAX_MODULE * MAX_MODULE) ) ) // (Vd^2+Vq^2) > MAX_MODULE^2 ?
	{
		uint16_t index;
		        
		temp /= (int32_t)(512*32768);  // min value START_INDEX, max value 127
		temp -= START_INDEX ;   // min value 0, max value 127 - START_INDEX
		index = circle_limit_table[(uint8_t)temp];
		        
		temp = (int16_t)Limit_Input->qV_Component1 * (uint16_t)(index); 
		Limit_Input->qV_Component1 = (int16_t)(temp/32768);  
		        
		temp = (int16_t)Limit_Input->qV_Component2 * (uint16_t)(index); 
		Limit_Input->qV_Component2 = (int16_t)(temp/32768);  
	}
}

void Rev_Park(Volt_Components* SVM_RevPark_Out,Volt_Components* SVM_RevPark_Input)
{ 	
  int32_t qV_tmp1,qV_tmp2;
  int16_t qV_1,qV_2;
   
  //No overflow guaranteed V¦Â
  qV_tmp1 = SVM_RevPark_Input->qV_Component1 * Vector_Components.hCos;
  qV_tmp1 /= 32768;
  
  qV_tmp2 = SVM_RevPark_Input->qV_Component2 * Vector_Components.hSin;
  qV_tmp2 /= 32768;
		
  qV_1 = (int16_t)(qV_tmp1);		
  qV_2 = (int16_t)(qV_tmp2);			

  SVM_RevPark_Out->qV_Component2 = ((qV_1)+(qV_2));
 
  //Va
  qV_tmp1 = SVM_RevPark_Input->qV_Component1 * Vector_Components.hSin;
  qV_tmp1 /= 32768;
  
  qV_tmp2 = SVM_RevPark_Input->qV_Component2 * Vector_Components.hCos;
  qV_tmp2 /= 32768;

  qV_1 = (int16_t)(qV_tmp1);				
  qV_2 = (int16_t)(qV_tmp2);
   				
  SVM_RevPark_Out->qV_Component1 = (-qV_1)+(qV_2);
}

void SVPWM_2ShuntCalcDutyCycles (Volt_Components* SVM_Volt_Input)
{
   int32_t wX, wY, wZ, wUAlpha, wUBeta,T1W,T2W;
   uint16_t  hTimePhA=0, hTimePhB=0, hTimePhC=0;
   
//wUAlpha=Va*SQRT_3*4*PWM_PERIOD     //Q13 8192
   wUAlpha = SVM_Volt_Input->qV_Component1 * SQRT_3;
	 wUAlpha/=2048;
	 wUAlpha*=PWM_PERIOD;
	
   wUBeta = (SVM_Volt_Input->qV_Component2 * T);

   wX = wUBeta;
   wY = (-wUBeta + wUAlpha)/2;
   wZ = (-wUBeta - wUAlpha)/2;
   
  // Sector calculation from wX, wY, wZ 
  if( wX >= 0 )
  {       
    // (xx1)
    if( wY >= 0 )
		{
			// (x11)
			// Must be Sector 3 since Sector 7 not allowed
			// Sector 3: (0,1,1)  0-60 degrees
			T2W = wY;
			T1W = wX;
				
			T1W=T1W/4;
			T2W=T2W/4;	
			hTimePhC=(T/8)-((T1W+T2W)/65536);
			hTimePhB=hTimePhC+(T1W/32768);
			hTimePhA=hTimePhB+(T2W/32768);
				
			TIM1->CCR1 = hTimePhA;
			TIM1->CCR2 = hTimePhB;
			TIM1->CCR3 = hTimePhC;
		}
		else
    {            
			// (x01)
			if( wZ >= 0 )
			{
				// Sector 5: (1,0,1)  120-180 degrees
				T2W = wX;
				T1W = wZ;
					
				T1W=T1W/4;
				T2W=T2W/4;	
				hTimePhC=(T/8)-((T1W+T2W)/65536);
				hTimePhB=hTimePhC+(T1W/32768);
				hTimePhA=hTimePhB+(T2W/32768);
					
				TIM1->CCR1 = hTimePhC;
				TIM1->CCR2 = hTimePhA;
				TIM1->CCR3 = hTimePhB;
			}
			else
      {
				// Sector 1: (0,0,1)  60-120 degrees
				T2W = -wY;
				T1W = -wZ;
					
				T1W=T1W/4;
				T2W=T2W/4;	
				hTimePhC=(T/8)-((T1W+T2W)/65536);
				hTimePhB=hTimePhC+(T1W/32768);
				hTimePhA=hTimePhB+(T2W/32768);
					
				TIM1->CCR1 = hTimePhB;
				TIM1->CCR2 = hTimePhA;
				TIM1->CCR3 = hTimePhC;
      }
    }
  }
  else
  {
    // (xx0)
    if( wY >= 0 )
    {
			// (x10)
			if( wZ >= 0 )
			{
				// Sector 6: (1,1,0)  240-300 degrees
				T2W = wZ;
				T1W = wY;
					
				T1W=T1W/4;
				T2W=T2W/4;	
				hTimePhC=(T/8)-((T1W+T2W)/65536);
				hTimePhB=hTimePhC+(T1W/32768);
				hTimePhA=hTimePhB+(T2W/32768);
					
				TIM1->CCR1 = hTimePhB;
				TIM1->CCR2 = hTimePhC;
				TIM1->CCR3 = hTimePhA;							
			}
			else
      {
				// Sector 2: (0,1,0)  300-0 degrees
				T2W = -wZ;
				T1W = -wX;

				T1W=T1W/4;
				T2W=T2W/4;	
				hTimePhC=(T/8)-((T1W+T2W)/65536);
				hTimePhB=hTimePhC+(T1W/32768);
				hTimePhA=hTimePhB+(T2W/32768);							

				TIM1->CCR1 = hTimePhA;
				TIM1->CCR2 = hTimePhC;
				TIM1->CCR3 = hTimePhB;							
      }
    }
		else
		{            
			// (x00)
			// Must be Sector 4 since Sector 0 not allowed
			// Sector 4: (1,0,0)  180-240 degrees
			T2W = -wX;
			T1W = -wY;

			T1W=T1W/4;
			T2W=T2W/4;	
			hTimePhC=(T/8)-((T1W+T2W)/65536);
			hTimePhB=hTimePhC+(T1W/32768);
			hTimePhA=hTimePhB+(T2W/32768);				

			TIM1->CCR1 = hTimePhC;
			TIM1->CCR2 = hTimePhB;
			TIM1->CCR3 = hTimePhA;					
		}
  }
}


void Trig_Functions(Trig_Components* Angle_Out,int16_t hAngle)
{
  uint16_t hindex;

  /* 10 bit index computation  */  
  hindex = (uint16_t)(hAngle + 32768);  
	hindex /= 64; 
  
  switch (hindex & SIN_MASK) 
  {
  case U0_90:
    Angle_Out->hSin = hSin_Cos_hTimePhAble[(uint8_t)(hindex)];
    Angle_Out->hCos = hSin_Cos_hTimePhAble[(uint8_t)(0xFF-(uint8_t)(hindex))];
    break;
  
  case U90_180:  
     Angle_Out->hSin = hSin_Cos_hTimePhAble[(uint8_t)(0xFF-(uint8_t)(hindex))];
     Angle_Out->hCos = -hSin_Cos_hTimePhAble[(uint8_t)(hindex)];
    break;
  
  case U180_270:
     Angle_Out->hSin = -hSin_Cos_hTimePhAble[(uint8_t)(hindex)];
     Angle_Out->hCos = -hSin_Cos_hTimePhAble[(uint8_t)(0xFF-(uint8_t)(hindex))];
    break;
  
  case U270_360:
     Angle_Out->hSin =  -hSin_Cos_hTimePhAble[(uint8_t)(0xFF-(uint8_t)(hindex))];
     Angle_Out->hCos =  hSin_Cos_hTimePhAble[(uint8_t)(hindex)]; 
    break;
  default:
    break;
  }
}
