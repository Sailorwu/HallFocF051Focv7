#include "TMER_CFG.h"

void Delay(uint32_t nCount)
{
  for(;nCount!=0;nCount--);
}

void TMER_Iinitialization(void)
{
	//GPIO
	GPIO_InitTypeDef  GPIO_InitStructure;
	//Interrupt
	NVIC_InitTypeDef NVIC_InitStructure;
		//Tmer
	TIM_BDTRInitTypeDef  TIM_BDTRInitStructure;				   //����ɲ���ṹ���������
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;                      //����ṹ�����

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	//
	//---------------------------------TMER3 HALL------------------------------
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_0);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_0);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_0);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 47;				   					//TIM������ʼ��
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period =65535;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);     

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;            //ѡ��ͨ��1
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //���������ز���  
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_TRC ; //����ͨ��Ϊ���룬��ӳ�䵽����
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;       //���벶��Ԥ��Ƶֵ
	TIM_ICInitStructure.TIM_ICFilter = 10;                       //�����˲�����������

	TIM_ICInit(TIM3, &TIM_ICInitStructure);                     //����ͨ������
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing; 		    //TIM���ͨ����ʼ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;             
	TIM_OCInitStructure.TIM_Pulse =65535; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      

	TIM_OC4Init(TIM3,&TIM_OCInitStructure);

	TIM_SelectHallSensor(TIM3,ENABLE);                          //ʹ��TIMx�Ļ����������ӿ�

	TIM_SelectInputTrigger(TIM3, TIM_TS_TI1F_ED);               //���봥��Դѡ��   

	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);             //��ģʽѡ��

	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);//����ģʽѡ��        

//	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_OC2Ref );      //ѡ���������ģʽ(TRGO��)
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;		//��TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
 
	TIM_ITConfig(TIM3, TIM_IT_Trigger|TIM_IT_CC4, ENABLE);   //����ʱ���ж�
	TIM_ClearFlag(TIM3, TIM_FLAG_Trigger);
	TIM_ClearFlag(TIM3, TIM_FLAG_CC4);

	TIM_Cmd(TIM3,ENABLE);		
	
/**************************************************************************
//---------------------------------TMER1 PWM-------------------------------
***************************************************************************/
	PWM_Period=((SystemCoreClock / (2*PWM_FREQ) ) - 1);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
  /* TIM2 Configuration */
  TIM_DeInit(TIM1);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);
	/* GPIOA Configuration: Channel 1, 2, 3, 4 and Channel 1N as alternate function push-pull */
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* GPIOB Configuration: Channel 2N and 3N as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14| GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_2);					//TRIP
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_2); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_2);

	/*----------------------------------------------------------------------- */
	/* Compute the value to be set in ARR regiter to generate signal frequency at 12 Khz */
	
	TIM_TimeBaseStructure.TIM_Prescaler = 0;					   //TIM������ʼ��
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;//����������ģʽ
	TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;

	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 		   //TIM���ͨ����ʼ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;                  
	TIM_OCInitStructure.TIM_Pulse =200; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;         
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;          

	TIM_OC1Init(TIM1,&TIM_OCInitStructure); 
	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
	TIM_OC3Init(TIM1,&TIM_OCInitStructure);

  TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 		   //TIM���ͨ��4��ʼ������������AD����
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                   
	TIM_OCInitStructure.TIM_Pulse =PWM_PERIOD-24; //100; 		//
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 

	TIM_OC4Init(TIM1,&TIM_OCInitStructure); 
	
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;	//����ɲ����ʼ��
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF; 
	TIM_BDTRInitStructure.TIM_DeadTime = DEADTIME;							//96-2us
	TIM_BDTRInitStructure.TIM_Break =TIM_Break_Enable;	 // TIM_Break_Disable;//
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

	TIM_BDTRConfig(TIM1,&TIM_BDTRInitStructure);

	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);   //ʹ�ܲ���ȽϼĴ���Ԥװ�أ�ͨ��1��

	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);	 //ʹ�ܲ���ȽϼĴ���Ԥװ�أ�ͨ��2��

	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);	 //ʹ�ܲ���ȽϼĴ���Ԥװ�أ�ͨ��3��

  /* Enable the TIM1 Trigger and commutation interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
	
	TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
	
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM1, TIM_IT_Break, ENABLE);
	
  TIM_ClearFlag(TIM1, TIM_FLAG_Update); 							// ���TIM2�ĸ��±�־λ ��
	TIM_ClearFlag(TIM1, TIM_FLAG_Break);
	TIM_ClearFlag(TIM1, TIM_FLAG_COM);
  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);	
}

