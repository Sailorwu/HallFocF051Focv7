#include "ADC_CFG.h"

uint16_t ADC_Tab[4];

void ADC_GPIO_init(void)  
{  
	//GPIO
	GPIO_InitTypeDef  GPIO_InitStructure; 
    //GPIOA                                                         //PA-0~2,6用作ADC  
	/* Configure ADC Channel11 as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}  


void ADC_config(void)  
{  
	//ADC
	ADC_InitTypeDef     ADC_InitStructure;
	/* ADC1 DeInit */  
	ADC_DeInit(ADC1);

	 /* ADC1 Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
//	RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div4);

	/* ADCs DeInit */  
	ADC_DeInit(ADC1);	
	
	/* Initialize ADC structure */
	ADC_StructInit(&ADC_InitStructure);
	
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
	ADC_InitStructure.ADC_ExternalTrigConvEdge =ADC_ExternalTrigConvEdge_Rising ; //ADC_ExternalTrigConvEdge_Falling ;//
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC4 ;//ADC_ExternalTrigConv_T1_TRGO;//  
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
	ADC_Init(ADC1, &ADC_InitStructure); 
	

  ADC_OverrunModeCmd(ADC1, ENABLE); 

	ADC_ChannelConfig(ADC1, ADC_Channel_2|ADC_Channel_3|ADC_Channel_13 | ADC_Channel_14 , ADC_SampleTime_1_5Cycles); 


	/* ADC Calibration */
	ADC_GetCalibrationFactor(ADC1);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);     
	
	/* Wait the ADCEN falg */
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN)); 
	
	/* Enable ADC_DMA */
	ADC_DMACmd(ADC1, ENABLE); 
	
	/* ADC DMA request in circular mode */
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
	
	/* ADC1 regular Software Start Conv */ 
	ADC_StartOfConversion(ADC1);	
	
}  
  

  
void ADC_DMA_init(void)  
{  
	//DMA
	DMA_InitTypeDef   DMA_InitStructure;
	//Interrupt
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* DMA1 clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
	
	/* DMA1 Channel1 Config */
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_Tab[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 4;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	//DMA interrupt
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);				//Enable DMA1 interrupt 
// 	DMA_ClearFlag(DMA1_FLAG_TC1);
	/* DMA1 Channel1 enable */
	DMA_Cmd(DMA1_Channel1, ENABLE);
}  



void ADC_Iinitialization(void)  
{  
    ADC_GPIO_init();  
    ADC_config();               // 注意此处的初始化顺序，否则采样传输的数据容易出现数据错位的结果  
    ADC_DMA_init();             //   
}  

