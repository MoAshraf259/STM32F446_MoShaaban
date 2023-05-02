#include "stm32f446_uart_diver.h"



void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
  }else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	   //over sampling by 16
	  usartdiv = ((25 * PCLKx) / (4 *BaudRate));
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  pUSARTx->BRR = tempreg;
}


void USART_Init(USART_Handle_t *pUSARTHandle)
{

	/***************This part is Enabling the USART ******************/
	//Enabling the USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx,ENABLE);

	/***************This part is configured for the mode ******************/

	//This is to choose the mode of the USART as receive only or transmitter only or full duplex
	if(pUSARTHandle->USART_Config.USART_Mode==USART_MODE_ONLY_TX)
	{
		pUSARTHandle->pUSARTx->CR1 |= (1<<USART_CR1_TE);
		pUSARTHandle->pUSARTx->CR1 &= ~(1<<USART_CR1_RE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode==USART_MODE_ONLY_RX)
	{
		pUSARTHandle->pUSARTx->CR1 &= ~(1<<USART_CR1_TE);
		pUSARTHandle->pUSARTx->CR1 |= (1<<USART_CR1_RE);
	}
	else if(pUSARTHandle->USART_Config.USART_Mode==USART_MODE_TXRX)
	{
		pUSARTHandle->pUSARTx->CR1 |= (1<<USART_CR1_TE);
		pUSARTHandle->pUSARTx->CR1 |= (1<<USART_CR1_RE);
	}
/*************************************DONE**********************************************/

	/********************This part to configure the parity Control ! *******************/

	if(pUSARTHandle->USART_Config.USART_ParityControl==USART_PARITY_DISABLE)
	{
		pUSARTHandle->pUSARTx->CR1 &= ~(1<<USART_CR1_PCE);
	}
	else if (pUSARTHandle->USART_Config.USART_ParityControl==USART_PARITY_EN_ODD)
	{
		pUSARTHandle->pUSARTx->CR1 |= (1<<USART_CR1_PCE);
		pUSARTHandle->pUSARTx->CR1 |= (1<<USART_CR1_PS);
	}
	else if(pUSARTHandle->USART_Config.USART_ParityControl==USART_PARITY_EN_EVEN)
	{
		pUSARTHandle->pUSARTx->CR1 |= (1<<USART_CR1_PCE);
		pUSARTHandle->pUSARTx->CR1 &= ~(1<<USART_CR1_PS);
	}
	/*************************************DONE**********************************************/




	/*******************This part is to Configure the word length *************/
	if(pUSARTHandle->USART_Config.USART_WordLength==USART_WORDLEN_8BITS)
	{
		pUSARTHandle->pUSARTx->CR1 &= ~(1<<USART_CR1_M);
	}
	else if(pUSARTHandle->USART_Config.USART_WordLength==USART_WORDLEN_9BITS)
	{
		pUSARTHandle->pUSARTx->CR1 |= (1<<USART_CR1_M);
	}
	/*************************************DONE**********************************************/

	/*************************This part is to configure the number of stop bits *********/
		pUSARTHandle->pUSARTx->CR2 |= (pUSARTHandle->USART_Config.USART_NoOfStopBits<<USART_CR2_STOP);

		/*************************************DONE**********************************************/
			/**********************HARDWARE Flow control configuration ******************/
	if(pUSARTHandle->USART_Config.USART_HWFlowControl== USART_HW_FLOW_CTRL_CTS)
	{
		pUSARTHandle->pUSARTx->CR3 |= (1<<USART_CR3_CTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl== USART_HW_FLOW_CTRL_CTS_RTS)
	{
		pUSARTHandle->pUSARTx->CR3 |= (1<<USART_CR3_CTSE);
		pUSARTHandle->pUSARTx->CR3 |= (1<<USART_CR3_RTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl== USART_HW_FLOW_CTRL_RTS)
	{
		pUSARTHandle->pUSARTx->CR3 |= (1<<USART_CR3_RTSE);
	}
	/*************************************DONE**********************************************/

	 USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata; //This pointer is created to type cast the original buffer to 16 bit

	for(uint32_t i=0;i<Len;i++)
	{
		//Wait until the TXE is set which refers to (Transmission complete)
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_TXE));

		//Check whether the word length is 9 bits or 8 bits
		if(pUSARTHandle->USART_Config.USART_WordLength==USART_WORDLEN_9BITS)
		{
			//Type cast the pointer to point at 2 bytes instead of just 1 byte to include the 9bits
			pdata=(uint16_t*)pTxBuffer;
			//Load the data into the DR as 16 bits but mask the last 7 bits as they maybe a garbage value
			pUSARTHandle->pUSARTx->DR=(*pdata & (uint16_t)0x1FF);

			if(pUSARTHandle->USART_Config.USART_ParityControl==USART_PARITY_DISABLE)
			{
				//Increase the buffer address two times as you have shifted 2 bytes to the shift register
				//Because the parity is disabled the 9s bit is a user data bit used by software
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Increase the buffer address only one time as the PARITY IS HERE ENABLED !
				//So the the parity will take the 9s bit as a parity bit by hardware
				pTxBuffer++;
			}
		}
		else//This part is for the WORD LENGTH of 8BITS !
		{
			//Normal 8 bits data so we just send it without casting and only one ++ to the buffer as if
			//the parity is enable or disabled it will always be only 8 bits !
			pUSARTHandle->pUSARTx->DR =(*pTxBuffer  & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}

	//Wait until the whole transmission is complete
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_TC));
}


void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{


	while(Len!=0){

		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_RXNE));

		if(pUSARTHandle->USART_Config.USART_WordLength==USART_WORDLEN_9BITS)
		{



			if(pUSARTHandle->USART_Config.USART_ParityControl==USART_PARITY_DISABLE)
			{
				*((uint16_t*)pRxBuffer)= pUSARTHandle->pUSARTx->DR & (uint16_t)(0x1FF);
				pRxBuffer++;
				pRxBuffer++;
				Len--;
			}
			else
			{
				*pRxBuffer=pUSARTHandle->pUSARTx->DR & (uint16_t)(0xFF);
				pRxBuffer++;
				Len--;
			}
		}
		else if(pUSARTHandle->USART_Config.USART_WordLength==USART_WORDLEN_8BITS)
		{
			if(pUSARTHandle->USART_Config.USART_ParityControl==USART_PARITY_DISABLE)
			{
				*pRxBuffer=pUSARTHandle->pUSARTx->DR &(0xFF);
				pRxBuffer++;
				Len--;
			}
			else
			{
				*pRxBuffer=pUSARTHandle->pUSARTx->DR &(0x7F);
				pRxBuffer++;
				Len--;
			}
		}

	}
}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
		//Here we send the data via interrupt
	uint8_t data;
	return data;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{

}

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		if(pUSARTx==USART1)
		{
			USART1_PCLK_EN();
		}
		else if (pUSARTx==USART2)
		{
			USART2_PCLK_EN();
		}
		else if (pUSARTx==USART3)
		{
			USART3_PCLK_EN();
		}
		else if (pUSARTx==UART4)
		{
			UART4_PCLK_EN();
		}
		else if (pUSARTx==UART5)
		{
			UART5_PCLK_EN();
		}
		else if (pUSARTx==USART6)
		{
			USART6_PCLK_EN();
		}
	}

}


void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi==ENABLE){
		pUSARTx->CR1 |= (1<<USART_CR1_UE);// Enabling the UE Bit to enable the USART Peripheral
	}
	else{
		pUSARTx->CR1 &= ~(1<<USART_CR1_UE);// Disabling the UE Bit to enable the USART Peripheral

	}
}
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	if(pUSARTx->SR &(1<<FlagName)){
		return SET;
	}
	else{
		return RESET;
	}
}


void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~(1<<StatusFlagName);
}

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				//program ISER0 register
				*NVIC_ISER0 |= ( 1 << IRQNumber );

			}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
			{
				//program ISER1 register
				*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 64 && IRQNumber < 96 )
			{
				//program ISER2 register //64 to 95
				*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
			}
		}else
		{
			if(IRQNumber <= 31)
			{
				//program ICER0 register
				*NVIC_ICER0 |= ( 1 << IRQNumber );
			}else if(IRQNumber > 31 && IRQNumber < 64 )
			{
				//program ICER1 register
				*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 6 && IRQNumber < 96 )
			{
				//program ICER2 register
				*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
			}
		}

}
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
		uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section  = IRQNumber %4 ;

		uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

		*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}


