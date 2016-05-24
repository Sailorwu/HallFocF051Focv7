#include"System.h"

void Delay_ms(uint16_t Timer)
{
	uint16_t i,j;
	for(j=Timer;j>0;j--)
	for(i=1000;i>0;i--);
}

void ADCTemp_Init(uint16_t* ADCnum)
{
	static uint8_t i=0;
	if(i>=8)
	{
		InitMeasCompCurr(hPhaseAOffset,hPhaseBOffset);
		VoltageOffset=VoltageOffset;
		
		SPOffset>>=5;						//10bit
		Motor_State=WAIT;
	}
	else
	{
		i++;
		VoltageOffset+=ADCnum[VDC_Channl];		
		hPhaseAOffset+=ADCnum[IA_Channl];
		hPhaseBOffset+=ADCnum[IB_Channl];
		SPOffset+=ADCnum[SP_Channl];
	}
}

int32_t SP_32LPF(LPF_32PARAMETERS * p_LPF_input, int32_t x_in,uint8_t Z)
{
	p_LPF_input->acc = p_LPF_input->acc_pre + ((x_in - p_LPF_input->acc_pre) / Z);  
	p_LPF_input->acc_pre = p_LPF_input->acc;
	return(p_LPF_input->acc);		
}

int16_t SP_16LPF(LPF_16PARAMETERS * p_LPF_input, int16_t x_in,uint8_t Z)
{
	int16_t i;
	i=(x_in - p_LPF_input->acc_pre) / Z;
	if(i==0) i=(x_in - p_LPF_input->acc_pre);
	p_LPF_input->acc = p_LPF_input->acc_pre + i;  
	p_LPF_input->acc_pre = p_LPF_input->acc;
	return(p_LPF_input->acc);		
}

/**
  * @brief  Configure PC6, PC7 and PC8 in Hall interrupt mode
  * @param  None
  * @retval None
  */
void EXTI4_15_Config(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
//	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Connect EXTI11 Line to PC11 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource11);
  /* Connect EXTI12 Line to PC12 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource12);

  /* Configure EXTI11 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;			//EXTI_Trigger_Rising_Falling
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Configure EXTI12 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI4_15 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void GPIO_Iinitialization(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

  /* GPIOC Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* USART1 Pins configuration **************************************************/
	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void DAC_Iinitialization(void)
{
	DAC_InitTypeDef    DAC_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* Enable GPIOA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);	
	/* Configure PA.04 (DAC_OUT1) in analog mode -------------------------*/
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* Enable DAC clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
 
	/* DAC channel1 Configuration */
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;

	/* DAC Channel1 Init */
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	 
	/* Enable DAC Channel1 */
	DAC_Cmd(DAC_Channel_1, ENABLE);
}

void Open_PWM(void)
{
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);	
}

void Close_PWM(void)
{
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);	
}
