/*
 * LCD.c
 *
 *  Created on: Apr 30, 2023
 *      Author: moham
 */



#include "LCD.h"
static void mdelay(uint32_t cnt);
static void udelay(uint32_t cnt);

void LCD_Init(void)
{
	GPIO_Handle_t GPIO_LCD;
	GPIO_LCD.pGPIOx=LCD_GPIO_PORT;
	GPIO_LCD.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO_LCD.GPIO_PinConfig.GPIO_PinNumber=LCD_PIN_4;
	GPIO_LCD.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_LCD.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GPIO_LCD.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIO_Init(&GPIO_LCD);

	GPIO_LCD.GPIO_PinConfig.GPIO_PinNumber=LCD_PIN_5;
	GPIO_Init(&GPIO_LCD);

	GPIO_LCD.GPIO_PinConfig.GPIO_PinNumber=LCD_PIN_6;
	GPIO_Init(&GPIO_LCD);

	GPIO_LCD.GPIO_PinConfig.GPIO_PinNumber=LCD_PIN_7;
	GPIO_Init(&GPIO_LCD);

	GPIO_LCD.pGPIOx=Control_Port; //The controlling bits PORT

	GPIO_LCD.GPIO_PinConfig.GPIO_PinNumber=LCD_RS;
	GPIO_Init(&GPIO_LCD);

	GPIO_LCD.GPIO_PinConfig.GPIO_PinNumber=LCD_RW;
	GPIO_Init(&GPIO_LCD);

	GPIO_LCD.GPIO_PinConfig.GPIO_PinNumber=LCD_EN;
	GPIO_Init(&GPIO_LCD);

	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_4, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_5, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_6, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_7, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_RS, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_RW, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_EN, RESET);

	mdelay(40);

#if LCD_MODE ==_8BIT_MODE
{
	LCD_SendCMD(0x38); //Function set
	mdelay(100);

	LCD_SendCMD(0x0E); //Display on or OFF
	mdelay(100);

	LCD_SendCMD(0x01);//Clear the LCD
	mdelay(100);

}
#elif (LCD_MODE ==_4BIT_MODE)
{
	LCD_SendCMD(0x3);

	mdelay(5);



	udelay(150);

	LCD_SendCMD(0x3);

	udelay(50);
	LCD_SendCMD(0x3);

	udelay(50);
	LCD_SendCMD(0x2);

	udelay(50);
	LCD_SendCMD(LCD_CMD_4DL_2N_5X8F);

	udelay(50);
	LCD_SendCMD(LCD_CMD_DON_CURON);

	udelay(50);
	LCD_SendCMD(LCD_CMD_DIS_CLEAR);

	mdelay(50);
	LCD_SendCMD(LCD_CMD_INCADD);
}
#endif
}

void LCD_SendCMD(uint8_t cmd)
{

#if LCD_MODE==_8BIT_MODE
	GPIO_WriteToOutputPin(GPIOC, LCD_RS, LOW);
	GPIO_WriteToOutputPin(GPIOC, LCD_RW, LOW);


	GPIO_WriteToOutputPort(GPIOC, cmd);

	GPIO_WriteToOutputPin(GPIOC,LCD_EN,HIGH);
	mdelay(100);
	GPIO_WriteToOutputPin(GPIOC,LCD_EN,LOW);

#elif LCD_MODE== _4BIT_MODE

	GPIO_WriteToOutputPin(Control_Port,LCD_RS,LOW);
	GPIO_WriteToOutputPin(Control_Port,LCD_RW,LOW);

	write_4_bits(cmd>>4);

	GPIO_WriteToOutputPin(Control_Port,LCD_EN,HIGH);
	udelay(100);
	GPIO_WriteToOutputPin(Control_Port,LCD_EN,LOW);

	write_4_bits(cmd & 0x0F);
	udelay(100);
	GPIO_WriteToOutputPin(Control_Port,LCD_EN,HIGH);

	udelay(100);
		GPIO_WriteToOutputPin(Control_Port,LCD_EN,LOW);



#endif
}

void LCD_SendChar(uint8_t character)
{
#if(LCD_MODE==_8BIT_MODE)
	GPIO_WriteToOutputPin(Control_Port,LCD_RS,HIGH);
	GPIO_WriteToOutputPin(Control_Port,LCD_RW,LOW);

	GPIO_WriteToPort(LCD_GPIO_PORT,character);

	GPIO_WriteToOutputPin(Control_Port,LCD_EN,HIGH);
	mdelay(100);
	GPIO_WriteToOutputPin(Control_Port,LCD_EN,LOW);

#elif(mode ==_4BIT_MODE)

		GPIO_WriteToOutputPin(GPIOC,LCD_RS,HIGH);
		GPIO_WriteToOutputPin(GPIOC,LCD_RW,LOW);

		write_4_bits(character>>4);

		GPIO_WriteToOutputPin(GPIOC,LCD_EN,HIGH);
		udelay(50);
		GPIO_WriteToOutputPin(GPIOC,LCD_EN,LOW);
		udelay(50);



		write_4_bits(character & 0x0F);

		GPIO_WriteToOutputPin(GPIOC,LCD_EN,HIGH);;
		udelay(50);
		GPIO_WriteToOutputPin(GPIOC,LCD_EN,LOW);
		udelay(50);

#endif
}



void LCD_SendString(uint8_t *pTxBuffer)
{
	while(*pTxBuffer != '\0')
	{
		LCD_SendChar(*(pTxBuffer));

		pTxBuffer++;
	}
}

void LCD_SendSpecialCharacter(uint8_t *pPattern,uint8_t location,uint8_t row,uint8_t col)
{
	uint8_t i;
	    if(location<8)
	    {
	     LCD_SendCMD(0x40 + (location*8));  /* Command 0x40 and onwards forces
	                                      the device to point CGRAM address */
	    	//LCD_SendCMD(64);
	       for(i=0;i<8;i++)
	       {// Write 8 byte for generation of 1 character
	           LCD_SendChar(pPattern[i]);
	       }
	    }
	    lcd_set_cursor(row,col);
	    LCD_SendChar(location);
}

void clr_lcd(void)
{
#if LCD_MODE == _8BIT_MODE
	LCD_SendCMD(0x1);
#elif LCD_MODE == _4BIT_MODE
	LCD_SendCMD(LCD_CMD_DIS_CLEAR);
#endif
}

void write_4_bits(uint8_t value)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_4, (value>>0)& 0x1);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_5, (value>>1)& 0x1);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_6, (value>>2)& 0x1);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_7, (value>>3)& 0x1);
}

void lcd_set_cursor(uint8_t row, uint8_t column)
{
  column--;
  switch (row)
  {
    case 1:
      /* Set cursor to 1st row address and add index*/
      LCD_SendCMD((column |= 0x80));
      break;
    case 2:
      /* Set cursor to 2nd row address and add index*/
    	LCD_SendCMD((column |= 0xC0));
      break;
    default:
      break;
  }
}

void LCD_SendNumber(uint16_t number)
{
	uint8_t temp[8];
	uint8_t counter=0;
	uint8_t i;
	for(i=0;i<8;i++)
	{
		temp[i]=number%10;
		number=number/10;
		if (number == 0){
		break;
		}
		counter++;
	}

	for(i=counter;i>=0;i--)
	{
		if(i==0)
		{
			switch(temp[i])
			{
			case 1:
				LCD_SendChar('1');
				break;
			case 2:
				LCD_SendChar('2');
				break;
			case 3:
				LCD_SendChar('3');
				break;
			case 4:
				LCD_SendChar('4');
				break;
			case 5:
				LCD_SendChar('5');
				break;
			case 6:
				LCD_SendChar('6');
				break;
			case 7:
				LCD_SendChar('7');
				break;
			case 8:
				LCD_SendChar('8');
				break;
			case 9:
				LCD_SendChar('9');
				break;
			case 0:
				LCD_SendChar('0');
				break;

			default:
				break;
			}
			break;
		}
		else{
		switch(temp[i])
		{
		case 1:
			LCD_SendChar('1');
			break;
		case 2:
			LCD_SendChar('2');
			break;
		case 3:
			LCD_SendChar('3');
			break;
		case 4:
			LCD_SendChar('4');
			break;
		case 5:
			LCD_SendChar('5');
			break;
		case 6:
			LCD_SendChar('6');
			break;
		case 7:
			LCD_SendChar('7');
			break;
		case 8:
			LCD_SendChar('8');
			break;
		case 9:
			LCD_SendChar('9');
			break;
		case 0:
			LCD_SendChar('0');
			break;

		default:
			break;
		}
		}


	}

}

static void mdelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1000); i++);
}

static void udelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1); i++);
}
