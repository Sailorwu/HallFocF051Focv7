

/* Includes ------------------------------------------------------------------*/
#include "system_define.h"
/** @addtogroup STM32F0_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
uint16_t SVM_Angle_Test;
uint8_t Test_Hall;
	
/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */

void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/**
  * @brief  This function handles External lines 4 to 15 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_15_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line11) != RESET)
  {  
    /* Clear the EXTI line 11 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line11);
  } 
  if(EXTI_GetITStatus(EXTI_Line12) != RESET)
  {
    /* Clear the EXTI line 12 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line12);							//Reset Angle
  }
}
/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
	/***********?????¡ä???D??************/  
	if((TIM_GetFlagStatus(TIM3,TIM_FLAG_Trigger)) != RESET )
	{
		TIM_ClearITPendingBit(TIM3, TIM_FLAG_Trigger);
		HallPLLCtrl();	

	}

	if((TIM_GetFlagStatus(TIM3,TIM_FLAG_CC4)) != RESET )
	{
		TIM_ClearITPendingBit(TIM3, TIM_FLAG_CC4);	
 	}
}

void TIM2_IRQHandler(void)
{
	/* Clear TIM2 Capture compare interrupt pending bit */
	TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  /* Clear TIM1 COM pending bit */
	if((TIM_GetFlagStatus(TIM1,TIM_FLAG_Update)) != RESET )
	{
		TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);	
	}

	if((TIM_GetFlagStatus(TIM1,TIM_FLAG_Break)) != RESET )
	{
		TIM_ClearITPendingBit(TIM1, TIM_FLAG_Break);
		Close_PWM();		
		TIM_CtrlPWMOutputs(TIM1, DISABLE);
		ShortI_Flag=1;
		Motor_State=WAIT;		
	}
}

void DMA1_Channel1_IRQHandler(void) 
{
	uint8_t ii;

	/* Test DMA1 TC flag */
	if((DMA_GetFlagStatus(DMA1_FLAG_TC1)) != RESET ) 
	{
		/* Clear DMA TC flag */
		DMA_ClearFlag(DMA1_FLAG_TC1);
			
		if(Motor_State==INIT)
		{
			ADCTemp_Init(ADC_Tab);
		}
		else
		{						
			//LED_ON();
			if(FOC_Flag)
			{
				for(ii=0;ii<6;ii++)  TPWM_Cnt[ii+1]++;
				
				PWM_Cnt++;
				
				if(Limit_angle<(ANGLE_60D+ANGLE_60D/8))
				{
					SVM_Angle=SVM_Angle+Delta_angle;				
					Limit_angle+=Delta_angle;
				} 	
				
//  				SVM_Angle_Test=0;
//  				IQ_Reference=0;
// 				ID_Reference=1000;
// 				Test_Hall=HALL_DATA();				
			
				SVPWM_2ShuntGetPhaseCurrentValues(&Curr_a_b);
				Clarke(&Curr_alfa_beta,&Curr_a_b);
				Park(&Curr_q_d,&Curr_alfa_beta,SVM_Angle); 	//			
	#ifdef OPEN_I								
				Volt_q_d.qV_Component1=VQ_Reference;
	#else
				Volt_q_d.qV_Component1=PID_Regulator(IQ_Reference,Curr_q_d.qI_Component1,&IQ_PID_t);				//			
	#endif
				Volt_q_d.qV_Component2=PID_Regulator(ID_Reference,Curr_q_d.qI_Component2,&ID_PID_t);				

				RevPark_Circle_Limitation(&Volt_q_d);
				Rev_Park(&Volt_alfa_beta,&Volt_q_d);
				SVPWM_2ShuntCalcDutyCycles(&Volt_alfa_beta);											  				
			} else PWM_Cnt=1000;
			//LED_OFF();
			DAC_SetChannel1Data(DAC_Align_12b_R,SVM_Angle/16);	
		}			
		
		if(++T2ms_Temp>T2MSTEMP)			//2ms
		{	
			T2ms_Temp=0;
			T2ms_Flag=1;
		}	
		if(++T100ms_Temp>T100MSTEMP)	//100ms 
		{				
			T100ms_Temp=0;
			T100ms_Flag=1;
		}else{}

	}
}
/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_IRQHandler(void)
{
		if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
		{
			USART_ClearFlag(USART1, USART_IT_RXNE);			
    	/* Read one byte from the receive data register */
			UART_FromPc();
  	}

  	if(USART_GetITStatus(USART1, USART_IT_TC) == SET)
  	{       
    	/* Clear the USART1 transmit interrupt */
			USART_ClearFlag(USART1, USART_IT_TC);
  	} 
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
