//==============================================================================
//---------------------------------Includes-------------------------------------
//==============================================================================
#include "DWIN.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"
//==============================================================================
//---------------------------------Defines--------------------------------------
//==============================================================================

#define headerDWIN_H 0x5A
#define headerDWIN_L 0xA5
//==============================================================================
//--------------------------------Variables-------------------------------------
//==============================================================================
struct readDataDWIN_P readDataDWIN;
//==============================================================================
//--------------------------------PROTOTYPE-------------------------------------
//==============================================================================
uint8_t getByte(unsigned char*b);
//==============================================================================
//--------------------------Device initialization-------------------------------
//==============================================================================
void dwinUartDmaInit()
{
  readDataDWIN.delayOverflowMaxTime = 3000;
  for (uint16_t i = 0; i <DWIN_UART_BUFFER_SIZE;i++){readDataDWIN.uartBuffer[i] = 0xffff;}
  HAL_UART_Receive_DMA(&DWIN_UART, (uint8_t *)&readDataDWIN.uartBuffer, DWIN_UART_BUFFER_SIZE);
}
//==============================================================================
//--------------------------------FUNCTIONS-------------------------------------
//==============================================================================

//==============================================================================
//---------------------------preset variable DWIN-------------------------------
//==============================================================================

//==============================================================================
//--------------------------Retrieving_a_data_byte------------------------------
//==============================================================================
uint8_t getByte(unsigned char*b)
{
  if (readDataDWIN.uartBuffer[readDataDWIN.uartCnt]!=0xffff)
  {
    *b=(readDataDWIN.uartBuffer[readDataDWIN.uartCnt] & 0xff);
    readDataDWIN.uartBuffer[readDataDWIN.uartCnt]=0xffff;
    readDataDWIN.uartCnt++;
    if (readDataDWIN.uartCnt == DWIN_UART_BUFFER_SIZE) readDataDWIN.uartCnt=0;
    return 1;
  } 
  else 
  {
	  return 0;
  }
}
//==============================================================================
//-------------------------------Recive_Data------------------------------------
//==============================================================================
void receivingDataPacket()
{ 
  uint16_t m = 0;
    if (!readDataDWIN.PacketReady)
      { 
        getByte(&readDataDWIN.GettingByte); 
        m = 0; 
          if (readDataDWIN.GettingByte == 0x5A)
          {
            readDataDWIN.rxData[m] = readDataDWIN.GettingByte; // start byte 1
            m++;
            readDataDWIN.timeoutDelay = 0;
            while ((getByte(&readDataDWIN.GettingByte)==0)&&(readDataDWIN.timeoutOverflow  != 1))
            {
            	readDataDWIN.timeoutDelay++;
            	if (readDataDWIN.timeoutDelay > readDataDWIN.delayOverflowMaxTime)readDataDWIN.timeoutOverflow = 1;
            }
            readDataDWIN.rxData[m] = readDataDWIN.GettingByte; // start byte 2 
            m++;
            readDataDWIN.timeoutDelay = 0;
            while ((getByte(&readDataDWIN.GettingByte)==0)&&(readDataDWIN.timeoutOverflow  != 1))
            {
            	readDataDWIN.timeoutDelay++;
            	if (readDataDWIN.timeoutDelay > readDataDWIN.delayOverflowMaxTime)readDataDWIN.timeoutOverflow = 1;
            }
            readDataDWIN.rxData[m] = readDataDWIN.GettingByte; // Data packet length
            readDataDWIN.lengthRxPacket = readDataDWIN.rxData[m];
            m++;
            readDataDWIN.timeoutDelay = 0;
            for(uint16_t i = 0;i < readDataDWIN.lengthRxPacket; i++) // Fill the buffer with data
            {
              while ((getByte(&readDataDWIN.GettingByte)==0)&&(readDataDWIN.timeoutOverflow  != 1))
              {
            	  readDataDWIN.timeoutDelay++;
            	  if (readDataDWIN.timeoutDelay > readDataDWIN.delayOverflowMaxTime)readDataDWIN.timeoutOverflow = 1;
              }
              readDataDWIN.timeoutDelay = 0;
              readDataDWIN.rxData[m] = readDataDWIN.GettingByte; // Retrieving data
              m++;
            }
          readDataDWIN.PacketReady = true; 
          } 
        readDataDWIN.GettingByte = 0; 
      } 
}
//==============================================================================
//------------------------------Parsing_Data------------------------------------
//==============================================================================
void parsingDWIN()
{
  receivingDataPacket();
  if (readDataDWIN.PacketReady)
  {
    readDataDWIN.parsingDataDWIN.header = (uint16_t)readDataDWIN.rxData[0]<<8 | (uint16_t)readDataDWIN.rxData[1];
    readDataDWIN.parsingDataDWIN.length = readDataDWIN.rxData[2];
    readDataDWIN.parsingDataDWIN.command = (commandDWIN)readDataDWIN.rxData[3];
    for (uint8_t i = 0; i < (readDataDWIN.lengthRxPacket - 1);i++)
    {
      readDataDWIN.parsingDataDWIN.data[i] = readDataDWIN.rxData[i+4];
    }
    readDataDWIN.PacketReady = false;
  }
  
}
//==============================================================================
//----------------------------DWIN GO TO PAGE-----------------------------------
//==============================================================================
void goToPageDWIN(uint16_t page)
{
  uint8_t str[10] = {headerDWIN_H,headerDWIN_L,0x07,0x82,0x00,0x84,0x5A,0x01,(page & 0xFF00)>>8,page & 0xFF};

  HAL_UART_Transmit(&DWIN_UART, str, sizeof(str),5);    
}
//==============================================================================
//------------------------------Read Variable-----------------------------------
//==============================================================================

void readVariableDWIN(uint16_t adress, uint8_t lenHalfWord)
{
  uint8_t str[10] = {headerDWIN_H,headerDWIN_L,0x04,0x83,(adress & 0xFF00)>>8,adress & 0xFF,lenHalfWord};

  HAL_UART_Transmit(&DWIN_UART, str, 7,5);    
}
//==============================================================================
//-------------------------------Write LED--------------------------------------
//==============================================================================

void writeLedDWIN(uint8_t ledWork, uint8_t ledSleep)
{
  uint8_t str[10] = {0x00};
  str[0] = headerDWIN_H;
  str[1] = headerDWIN_L;
  str[2] = 0x05;
  str[3] = 0x82;
  str[4] = 0x00;
  str[5] = 0x82;
  str[6] = ledWork;
  str[7] = ledSleep;

  HAL_UART_Transmit(&DWIN_UART, str, 8,0xff);    
}
//==============================================================================
//----------------------------Write Variable------------------------------------
//==============================================================================

void writeVariableDWIN(uint8_t len, uint16_t adress, uint8_t* data)
{
  uint8_t str[250] = {0x00};
  str[0] = headerDWIN_H;
  str[1] = headerDWIN_L;
  str[2] = len;
  str[3] = writeVariable;
  str[4] = (adress & 0xFF00)>>8;
  str[5] = adress & 0xFF;
  for (uint8_t i = 0; i < (len - 3);i++)
{
  str[i+6] = data[i];
}

  HAL_UART_Transmit(&DWIN_UART, str, (len + 3),0xff);    
}

//==============================================================================
//------------------------Write Variable Half Word------------------------------
//==============================================================================

void writeHalfWordDWIN(uint16_t adress, uint16_t data)
{
  uint8_t str[9] = {0x00};
  str[0] = headerDWIN_H;
  str[1] = headerDWIN_L;
  str[2] = 0x05;
  str[3] = writeVariable;
  str[4] = (adress & 0xFF00)>>8;
  str[5] = adress & 0xFF;
  str[6] = (data & 0xFF00)>>8;
  str[7] = data & 0xFF;

  HAL_UART_Transmit(&DWIN_UART, str, 8,0xff);    
}
//==============================================================================
//--------------------------Write Variable Word---------------------------------
//==============================================================================

void writeWordDWIN(uint16_t adress, uint32_t data)
{
  uint8_t str[11] = {0x00};
  str[0] = headerDWIN_H;
  str[1] = headerDWIN_L;
  str[2] = 0x07;
  str[3] = writeVariable;
  str[4] = (adress & 0xFF00)>>8;
  str[5] = adress & 0xFF;
  str[6] = (data & 0xFF000000)>>24;
  str[7] = (data & 0x00FF0000)>>16;
  str[8] = (data & 0x0000FF00)>>8;
  str[9] =  data & 0x000000FF;
  
  HAL_UART_Transmit(&DWIN_UART, str, 10,0xff);    
}
//==============================================================================
//-------------------------Write Variable 2 Word--------------------------------
//==============================================================================

void writeDoubleWordDWIN(uint16_t adress, uint64_t data)
{
  uint8_t str[15] = {0x00};
  str[0] = headerDWIN_H;
  str[1] = headerDWIN_L;
  str[2] = 0x11;
  str[3] = writeVariable;
  str[4] = (adress & 0xFF00)>>8;
  str[5] = adress & 0xFF;
  str[6] = (data & 0xFF00000000000000)>>56;
  str[7] = (data & 0x00FF000000000000)>>48;
  str[8] = (data & 0x0000FF0000000000)>>40;
  str[9] =  data & 0x000000FF00000000>>32;
  str[10] = (data & 0x00000000FF000000)>>24;
  str[11] = (data & 0x0000000000FF0000)>>16;
  str[12] = (data & 0x000000000000FF00)>>8;
  str[13] =  data & 0x00000000000000FF;
  

  HAL_UART_Transmit(&DWIN_UART, str, 14,0xff);    
}


void buzzerTouchOnDWIN(void)
{																																									//3B
	 uint8_t str[10] = {headerDWIN_H,headerDWIN_L,0x07,0x82,0x00,0x80,0x5A,0x00,0x00,0x3B};
	 HAL_UART_Transmit(&DWIN_UART, str, 10,0xFF);    

}
void buzzerTouchOffDWIN(void)
{																																									//33 
	 uint8_t str[10] = {headerDWIN_H,headerDWIN_L,0x07,0x82,0x00,0x80,0x5A,0x00,0x00,0x33};
	 HAL_UART_Transmit(&DWIN_UART, str, 10,0xFF);    

}

void setLedDWIN(uint8_t bright)
{
	uint8_t str[10] = {headerDWIN_H,headerDWIN_L,0x07,0x82,0x00,0x82,bright,0x32,0x03,0xE8};
	HAL_UART_Transmit(&DWIN_UART, str, 10,0xFF);    

}

void readPageDWIN(void)
{
		uint8_t str[10] = {headerDWIN_H,headerDWIN_L,0x04,0x83,0x00,0x14,0x1};
		HAL_UART_Transmit(&DWIN_UART, str, 7,0xFF);    

}
void setIconDWIN(uint8_t ICL, uint8_t icon, uint8_t X, uint8_t Y)
{
	uint8_t str[15] = {0x00};
	str[0] = headerDWIN_H;
  str[1] = headerDWIN_L;
  str[2] = 0x0B;
  str[3] = 0x82;
  str[4] = 0;
  str[5] = 0xD5;
  str[6] = 0x5A;
  str[7] = ICL;
	str[8] = 0;
	str[9] = icon;
	str[10] = 0;
	str[11] = X;
	str[12] = 0;
	str[13] = Y;
	
	HAL_UART_Transmit(&DWIN_UART, str, sizeof(str),0xFF);
}
//==============================================================================
//---------------------------------END FILE-------------------------------------
//==============================================================================
