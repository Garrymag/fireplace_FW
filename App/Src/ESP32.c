
//==============================================================================
//---------------------------------Includes]-------------------------------------
//==============================================================================
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"
#include "ESP32.h"
#include "main.h"
//==============================================================================
//---------------------------------Defines--------------------------------------
//==============================================================================
#define headerESP32 0xFF
#define minor_soft  2
#define major_soft 	1
#define soft_type  0

//==============================================================================
//--------------------------------Variables-------------------------------------
//==============================================================================
struct ESP32Data_P ESP32Data;
	uint8_t fuel_time_h = 0;
	uint8_t fuel_time_l = 0;
//==============================================================================
//--------------------------------Function-------------------------------------
//==============================================================================
void ESP32DmaInit(void)
{
	HAL_UART_Receive_DMA(&ESP32_UART,ESP32Data.RxData, 4);
}
void ESP32SendWakeUpCode(void)
{
	uint8_t data[3] = {0xAA,0xAA,0xAA};
	HAL_UART_Transmit(&ESP32_UART, data, 3, 0xFF);
}

void ESP32SendServiceCode(uint8_t data)
{	
	uint8_t csum = 0;
	uint8_t SendTx[3] = {headerESP32, data, csum};
		for (int k = 0; k < 2; k+=1)
		csum^= SendTx[k];
	SendTx[2] = csum;
		HAL_UART_Transmit(&ESP32_UART, SendTx, 3, 0xFF);

}
void ESP32SendWarningErrorCode(uint8_t cmd, uint8_t data)
{
	uint8_t csum = 0;
	uint8_t SendTx[4] = {headerESP32, cmd, data, 0};
	for(int k =0; k <3 ; k+=1)
		csum^=SendTx[k];
	SendTx[3] = csum;
	HAL_UART_Transmit(&ESP32_UART, SendTx, 4, 0xFF);

}
void ESP32SendDeviceInfo(uint8_t data[7])
{
	uint8_t csum = 0;
	uint8_t SendTx[13] = {headerESP32,ASK_DEV,soft_type,major_soft,minor_soft,data[0],data[1],data[2],data[3],data[4],data[5],data[6],0};
	for (int k = 0; k < 12; k+=1)
	{
		csum^=SendTx[k];
	}
	SendTx[12] = csum;
	HAL_UART_Transmit(&ESP32_UART, SendTx, 13, 0xFF);

}
void ESP32SendDeviceInfoTest(void)
{
	uint8_t csum = 0;
	uint8_t SendTx[13] = {headerESP32,ASK_DEV,soft_type,major_soft,minor_soft,3,3,0,0,0,2,2,csum};
	for (int k = 0; k < 11; k+=1)
	{
		csum^=SendTx[k];
	}
	SendTx[12] = csum;
	HAL_UART_Transmit(&ESP32_UART, SendTx, 13, 0xFF);

}

void ESP32SendStatus(void)
{
	uint8_t csum = 0;
	uint8_t SendTx[10] =
	{headerESP32,ASK_ST,ESP32Data.status,ESP32Data.burn_lvl,ESP32Data.heat_cool_st,ESP32Data.temp,ESP32Data.fuel,fuel_time_l,fuel_time_h,0};
	fuel_time_h = (uint8_t)(ESP32Data.fuel_time>>8);
	fuel_time_l = (uint8_t)(ESP32Data.fuel_time&0x00FF);
	for (int k = 0; k < 9; k+=1)
		csum^= SendTx[k];
	SendTx[9] = csum;
	HAL_UART_Transmit(&ESP32_UART, SendTx, 10, 0xFF);
}
void ReceiveESP32(void)
{
	ESP32Data.header = ESP32Data.RxData[0];
	if (ESP32Data.header == headerESP32)
	{
			ESP32Data.cmd = ESP32Data.RxData[1];

			ESP32Data.data = ESP32Data.RxData[2];
			ESP32Data.rx_csum = ESP32Data.RxData[3];
			ESP32Data.ByteCount = 3;
		for (int k = 0; k < ESP32Data.ByteCount; k +=1)
			ESP32Data.csum ^= ESP32Data.RxData[k];
		if (ESP32Data.rx_csum == ESP32Data.csum)
		{
			ESP32Data.csum = 0;
			ESP32Data.ValidPacket = true;
		}
		else
		{
			ESP32Data.csum = 0;
			ESP32Data.ValidPacket = false;
			ESP32Data.data = 0;
			ESP32Data.cmd = 0;
			ESP32SendServiceCode(CSUM_ERR);
		}
		for (int i = 0; i < 5; i+=1)
			ESP32Data.RxData[i] = 0;
	}
}
