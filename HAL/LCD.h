/*
 * LCD.h
 *
 *  Created on: Apr 30, 2023
 *      Author: moham
 */

#ifndef LCD_H_
#define LCD_H_

#include "stm32f446.h"
#define HIGH	1
#define LOW		0


#define Control_Port	GPIOC
#define LCD_GPIO_PORT 	GPIOC

/*#define LCD_PIN_0	GPIO_PIN_NO_0
#define LCD_PIN_1	GPIO_PIN_NO_1
#define LCD_PIN_2	GPIO_PIN_NO_2
#define LCD_PIN_3	GPIO_PIN_NO_3*/

#define LCD_PIN_4	GPIO_PIN_NO_3 //Used for the 4 bits mode !
#define LCD_PIN_5	GPIO_PIN_NO_4
#define LCD_PIN_6	GPIO_PIN_NO_5
#define LCD_PIN_7	GPIO_PIN_NO_6

#define LCD_RS	GPIO_PIN_NO_0
#define LCD_RW	GPIO_PIN_NO_1
#define LCD_EN	GPIO_PIN_NO_2

#define _4BIT_MODE		0
#define _8BIT_MODE		1
#define LCD_MODE 	0


#define LCD_CMD_4DL_2N_5X8F  		0x28
#define LCD_CMD_DON_CURON    		0x0E
#define LCD_CMD_INCADD       		0x06
#define LCD_CMD_DIS_CLEAR    		0X01
#define LCD_CMD_DIS_RETURN_HOME  	0x02





void LCD_Init(void);

void LCD_SendCMD(uint8_t cmd);

void LCD_SendChar(uint8_t character);

void LCD_SendString(uint8_t *pTxBuffer);

void LCD_SendSpecialCharacter(uint8_t *pPattern,uint8_t location,uint8_t row,uint8_t col);

void clr_lcd(void);

void write_4_bits(uint8_t value);

void lcd_set_cursor(uint8_t row, uint8_t column);
void LCD_SendNumber(uint16_t number);


#endif /* LCD_H_ */
