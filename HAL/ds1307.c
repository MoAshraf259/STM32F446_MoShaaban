/*
 * ds1307.c
 *
 *  Created on: Apr 8, 2023
 *      Author: moham
 */
#include "ds1307.h"
#include "string.h"
#include "stdint.h"

static uint8_t BCDToBinary(uint8_t data);
static uint8_t BinaryToBCD(uint8_t data);
static void I2C_Pin_Config(void);
static void DS1307_I2C_Config(void);
static void DS1307_Write(uint8_t data,uint8_t RegAddr);
static uint8_t DS1307_Read(uint8_t RegAddr);


I2C_Handle_t g_DS1307_I2CHandle;


uint8_t DS1307_Init(void)
{
	//INIT the i2c pin
	I2C_Pin_Config();

	//INIT the i2c peripheral
	DS1307_I2C_Config();

	//Enable the I2C Peripheral Clock
	I2C_PeripheralControl(DS1307_I2C, ENABLE);

	//make CH(Clock Halt ) as 0
	DS1307_Write(0x00,DS1307_ADDR_SEC);

	//Read the CH Back
	uint8_t Clock_State =DS1307_Read(DS1307_ADDR_SEC);

	return (Clock_State>>7) &0x1;//If it returns 1 CH is 1 meaning failed
								 //else it`s zero meaning it succedded

}
void DS1307_SetCurrentTime(RTC_Time_t *pRTC_Time)
{
	uint8_t seconds,hours;
	seconds =BinaryToBCD( pRTC_Time->seconds);
	seconds &=~(1<<7);

	DS1307_Write(seconds, DS1307_ADDR_SEC);
	DS1307_Write(BinaryToBCD(pRTC_Time->minutes), DS1307_ADDR_MIN);

	hours =BinaryToBCD(pRTC_Time->hours);

	if(pRTC_Time->time_format==TIMER_FORMAT_24)
	{
		hours &=~ (1<<6);
	}
	else
	{
		hours |= (1<<6);
		hours = (pRTC_Time->time_format==TIMER_FORMAT_12PM)? hours |(1<<5) : hours &~(1<<5);
	}
	DS1307_Write(hours, DS1307_ADDR_HOUR);
}


void DS1307_SetCurrentDate(RTC_Date_t *pRTC_Date)
{
	DS1307_Write(BinaryToBCD(pRTC_Date->date), DS1307_ADDR_DATE);

	DS1307_Write(BinaryToBCD(pRTC_Date->day), DS1307_ADDR_DAY);

	DS1307_Write(BinaryToBCD(pRTC_Date->month), DS1307_ADDR_MONTH);

	DS1307_Write(BinaryToBCD(pRTC_Date->year), DS1307_ADDR_YEAR);
}

void DS1307_GetCurrentTime(RTC_Time_t *pRTC_Time)
{
	uint8_t seconds,hours;

	seconds = DS1307_Read(DS1307_ADDR_SEC);
	seconds &= ~(1<<7);

	pRTC_Time->seconds = BCDToBinary(seconds);

	pRTC_Time->minutes= BCDToBinary( DS1307_Read(DS1307_ADDR_MIN));

	hours = DS1307_Read(DS1307_ADDR_HOUR);

	if(hours & (1<<6))
	{
		if(hours & (1<<5))
		{
			pRTC_Time->time_format=TIMER_FORMAT_12PM;
		}
		else
		{
			pRTC_Time->time_format=TIMER_FORMAT_12AM;
		}
		hours &= ~(0x3<<5);//To clear the 5 and 6 bits
	}
	else
	{
		pRTC_Time->time_format=TIMER_FORMAT_24;
	}

	pRTC_Time->hours=BCDToBinary(hours);


}


void DS1307_GetCurrentDate(RTC_Date_t *pRTC_Date)
{
	pRTC_Date->date=BCDToBinary(DS1307_Read(DS1307_ADDR_DATE));

	pRTC_Date->day=BCDToBinary(DS1307_Read(DS1307_ADDR_DAY));

	pRTC_Date->month=BCDToBinary(DS1307_Read(DS1307_ADDR_MONTH));

	pRTC_Date->year=BCDToBinary(DS1307_Read(DS1307_ADDR_YEAR));
}

static void I2C_Pin_Config(void)
{
	GPIO_Handle_t I2C_SDA,I2C_SCL;
	memset(&I2C_SCL,0,sizeof(I2C_SCL));
	memset(&I2C_SDA,0,sizeof(I2C_SDA));

	I2C_SDA.pGPIOx=DS1307_I2C_PORT;
	I2C_SDA.GPIO_PinConfig.GPIO_PinAltFunMode=4;
	I2C_SDA.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	I2C_SDA.GPIO_PinConfig.GPIO_PinNumber=DS1307_I2C_SDA;
	I2C_SDA.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
	I2C_SDA.GPIO_PinConfig.GPIO_PinPuPdControl=DS1307_I2C_PUPD;
	I2C_SDA.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	GPIO_Init(&I2C_SDA);

	I2C_SCL.pGPIOx=DS1307_I2C_PORT;
	I2C_SCL.GPIO_PinConfig.GPIO_PinAltFunMode=4;
	I2C_SCL.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	I2C_SCL.GPIO_PinConfig.GPIO_PinNumber=DS1307_I2C_SCL;
	I2C_SCL.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
	I2C_SCL.GPIO_PinConfig.GPIO_PinPuPdControl=DS1307_I2C_PUPD;
	I2C_SCL.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	GPIO_Init(&I2C_SCL);
}

static void DS1307_I2C_Config(void)
{
	g_DS1307_I2CHandle.pI2Cx=DS1307_I2C;
	g_DS1307_I2CHandle.I2C_Config.I2C_AckControl=I2C_ACK_ENABLE;
	g_DS1307_I2CHandle.I2C_Config.I2C_SCLSpeed=DS1307_I2C_SPEED;

	I2C_Init(&g_DS1307_I2CHandle);
}


static void DS1307_Write(uint8_t data,uint8_t RegAddr)
{
	uint8_t Tx[2];
	Tx[0]=RegAddr;
	Tx[1]=data;

	I2C_MasterSendData(&g_DS1307_I2CHandle, Tx	, sizeof(Tx),DS1307_SLAVE_ADDR , 0);
}

static uint8_t DS1307_Read(uint8_t RegAddr)
{
	uint8_t Rx=0;

	I2C_MasterSendData(&g_DS1307_I2CHandle, &RegAddr, 1, DS1307_SLAVE_ADDR, 0);

	I2C_MasterReceiveData(&g_DS1307_I2CHandle, &Rx	, 1, DS1307_SLAVE_ADDR, 0);

	return Rx;
}

static uint8_t BCDToBinary(uint8_t data)
{
	uint8_t m,n,result;
	m=data>>4;
	m=m*10;

	n=data &(0x0F);
	result=m+n;

	return result;
}
static uint8_t BinaryToBCD(uint8_t data)
{
	uint8_t m,n,result;

	result=data;
	if(data>=10)
	{
		m=data/10;
		n=data%10;

		result=(m<<4)|n;
	}
	return result;
}
