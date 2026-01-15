
//==============================================================================
#include <stdbool.h>
#include "stm32f4xx_hal.h"
//==============================================================================
//---------------------------------Defines--------------------------------------
//==============================================================================
#define ESP32_UART huart1

#define ESP32_UART_BUFFER_SIZE 4
#define ESP32_TX_BUFFER 7

#define ASK_DEV 				0x01
#define ASK_ST					0x02
#define	HEAT_ON_ESP			0x10
#define BURN_LVL				0x20
#define COOL_ON					0x40
#define HEX_UPD					0xf0
#define WAR							0xf1
#define ERR							0xf2
#define CSUM_ERR				0xf4
#define ESP32_DEFAULT		0xf8
#define ESP32_SLEEP			0xFA
#define ESP32_WAKEUP		0xAA
#define ESP32_RESET			0xFB

#define STM32_WAIT			0x00
#define STANDBY_ST			0x01
#define HEAT_ST_ON			0x02
#define BURN_ST_ON			0x04
#define COOL_ST_ON			0x08
#define WAR_ST					0x10
#define ERR_ST					0x20

#define headerESP32 0xFF

//==============================================================================
//--------------------------------Variables-------------------------------------
//==============================================================================


struct ESP32Data_P 
{
//			uint8_t TxData[ESP32_TX_BUFFER];
			uint8_t RxData[4];
			uint8_t header;
			uint8_t cmd;
			uint8_t csum;
			uint8_t data;
			uint8_t rx_csum;
			bool ValidPacket;
			uint8_t status;
			uint8_t burn_lvl;
			uint8_t	heat_cool_st;
			uint8_t	temp;
			uint8_t fuel;
			uint16_t fuel_time;
				uint8_t ByteCount;

};
			

//==============================================================================
//--------------------------------FUNCTIONS-------------------------------------
//==============================================================================
void ESP32DmaInit(void);
void ESP32SendServiceCode(uint8_t data);
void ESP32SendStatus(void);
void ReceiveESP32(void);
void ESP32SendDeviceInfo(uint8_t data[7]);
void ESP32SendDeviceInfoTest(void);
void ESP32SendWakeUpCode(void);
void ESP32SendWarningErrorCode(uint8_t cmd, uint8_t data);

//==============================================================================
//---------------------------------END FILE-------------------------------------
//==============================================================================

