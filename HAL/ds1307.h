/*
 * ds1307.h
 *
 *  Created on: Apr 8, 2023
 *      Author: moham
 */

#ifndef DS1307_H_
#define DS1307_H_
#include "stm32f446.h"

/*Application configurable items*/
#define DS1307_I2C			I2C1
#define DS1307_I2C_PORT		GPIOB
#define DS1307_I2C_SDA		GPIO_PIN_NO_7
#define DS1307_I2C_SCL		GPIO_PIN_NO_6
#define DS1307_I2C_SPEED	I2C_SCL_SPEED_SM
#define DS1307_I2C_PUPD		GPIO_PIN_PU
/*Register addresses */
#define DS1307_ADDR_SEC		0x00
#define DS1307_ADDR_MIN		0x01
#define DS1307_ADDR_HOUR	0x02
#define DS1307_ADDR_DAY		0x03
#define DS1307_ADDR_DATE	0x04
#define DS1307_ADDR_MONTH	0x05
#define DS1307_ADDR_YEAR	0x06


#define TIMER_FORMAT_12AM	0
#define TIMER_FORMAT_12PM	1
#define TIMER_FORMAT_24		2

#define DS1307_SLAVE_ADDR	0x68

#define Saturday		1
#define Sunday			2
#define Monday			3
#define Tuesday			4
#define Wednesday		5
#define Thursday		6
#define Friday			7

typedef struct
{
	uint8_t date;
	uint8_t month;
	uint8_t day;
	uint8_t year;
}RTC_Date_t;

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t time_format;
}RTC_Time_t;


/*Functions prototype */
uint8_t DS1307_Init(void);
void DS1307_SetCurrentTime(RTC_Time_t *pRTC_Time);
void DS1307_GetCurrentTime(RTC_Time_t *pRTC_Time);

void DS1307_SetCurrentDate(RTC_Date_t *pRTC_Date);
void DS1307_GetCurrentDate(RTC_Date_t *pRTC_Date);
#endif /* DS1307_H_ */
