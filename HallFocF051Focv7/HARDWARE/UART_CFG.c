#include "UART_CFG.h"
uint8_t	RxBuffer[20],TxBuffer[20];
uint8_t TxDat_len,RxDat_len,UART_Dleay_temp;
uint8_t TXFinishFlag,RXFinishFlag;
uint8_t SetRUN_direction,RUN_direction,SetQZ_Num;

void Send_data(void)
{
	static uint16_t TxLen = 0;
	static uint16_t TxIdx = 0;
	
	if( (0 == TxLen) && (TxDat_len>0) )
	{
		TxLen = TxDat_len;
		TxIdx = 0;
	}	
	// start a protocol frame
	if((USART_GetITStatus(USART1, USART_IT_TXE) == SET)&& (TxLen) )	// if buffer full and frame send over
	{
		// put data into SCIbuf
		USART_SendData(USART1, TxBuffer[TxIdx]);
		TxLen--;
		TxIdx++;
	}	
	if( TxLen == 0 )
	{
		TxDat_len = 0;
		TXFinishFlag = 1;
	}
//*****************RX Data*********************	
	if(RXFinishFlag)
	{
		RXFinishFlag=0;
	}
}



void UART_FromPc(void)
{
	static uint16_t RxIdx = 0;
	static uint8_t stage=0;
	uint8_t rx_data;	
	// get data from FIFO buffer
	rx_data = USART_ReceiveData(USART1);
	if(UART_Dleay_temp>5) stage=0;
		
	switch(stage)
	{
		case 0:											// get the length				
			RxDat_len = rx_data;
			RxBuffer[0] = rx_data;
			stage = 1; 
			RxIdx = 1;	
			if((rx_data==0)||(rx_data>=0x0F)) stage=0;
			RXFinishFlag=0;
		break;
		case 1:
			RxBuffer[RxIdx] = rx_data;
			RxIdx++;
	
			if(RxIdx >= RxDat_len)
			{
				stage = 0;
				RXFinishFlag=1;
			}					
		break;
		default :
			// do nothing
			RxIdx++;
			if(RxIdx >= RxDat_len)
			{
				stage = 0;
			}				
		break;
	}
	UART_Dleay_temp=0;					//Reset wart time
}

void UART_Iinitialization(void)
{
	//GPIO
	GPIO_InitTypeDef  GPIO_InitStructure;
	//Interrupt
	NVIC_InitTypeDef NVIC_InitStructure;
	//UART
	USART_InitTypeDef USART_InitStructure;	

	/* Enable USART1 APB clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
 
	/* Configure the HSI as USART clock */
	RCC_USARTCLKConfig(RCC_USART1CLK_HSI);
	
	/* Connect pin to Periph */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_0);    
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_0); 

	/* Configure pins as AF pushpull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	/* USARTx configured as follow:
	- BaudRate = 9600 baud  
	- Word Length = 8 Bits
	- Stop Bit = 1 Stop Bit
	- Parity = No Parity
	- Hardware flow control disabled (RTS and CTS signals)
	- Receive and transmit enabled
	*/

	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	//USART_StructInit(&USART_InitStructure);
	
	USART_Init(USART1, &USART_InitStructure);

	/* USART1 IRQ Channel configuration */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 
//	USART_ITConfig(USART1, USART_IT_TC, ENABLE); 

	USART_ClearFlag(USART1, USART_IT_RXNE);	
	USART_Cmd(USART1, ENABLE);
}


