/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SERV1_Pin GPIO_PIN_5
#define SERV1_GPIO_Port GPIOE
#define SERV2_Pin GPIO_PIN_6
#define SERV2_GPIO_Port GPIOE
#define MOTOR_Pin GPIO_PIN_0
#define MOTOR_GPIO_Port GPIOC
#define IR1_Pin GPIO_PIN_1
#define IR1_GPIO_Port GPIOC
#define IR2_Pin GPIO_PIN_2
#define IR2_GPIO_Port GPIOC
#define IR3_Pin GPIO_PIN_3
#define IR3_GPIO_Port GPIOC
#define LEAK1_Pin GPIO_PIN_0
#define LEAK1_GPIO_Port GPIOA
#define LEAK2_Pin GPIO_PIN_1
#define LEAK2_GPIO_Port GPIOA
#define FUEL_Pin GPIO_PIN_2
#define FUEL_GPIO_Port GPIOA
#define T1_Pin GPIO_PIN_3
#define T1_GPIO_Port GPIOA
#define T2_Pin GPIO_PIN_4
#define T2_GPIO_Port GPIOA
#define T3_Pin GPIO_PIN_5
#define T3_GPIO_Port GPIOA
#define T4_Pin GPIO_PIN_6
#define T4_GPIO_Port GPIOA
#define T5_Pin GPIO_PIN_7
#define T5_GPIO_Port GPIOA
#define T6_Pin GPIO_PIN_4
#define T6_GPIO_Port GPIOC
#define T7_Pin GPIO_PIN_5
#define T7_GPIO_Port GPIOC
#define CS_Pin GPIO_PIN_12
#define CS_GPIO_Port GPIOB
#define MP3_TX_Pin GPIO_PIN_8
#define MP3_TX_GPIO_Port GPIOD
#define MP3_RX_Pin GPIO_PIN_9
#define MP3_RX_GPIO_Port GPIOD
#define MP3_BUSY_Pin GPIO_PIN_10
#define MP3_BUSY_GPIO_Port GPIOD
#define RX433_Pin GPIO_PIN_15
#define RX433_GPIO_Port GPIOD
#define COM_TX_Pin GPIO_PIN_6
#define COM_TX_GPIO_Port GPIOC
#define COM_RX_Pin GPIO_PIN_7
#define COM_RX_GPIO_Port GPIOC
#define FAN_OUT_Pin GPIO_PIN_8
#define FAN_OUT_GPIO_Port GPIOC
#define Buzzer_Pin GPIO_PIN_9
#define Buzzer_GPIO_Port GPIOC
#define TMC_EN_Pin GPIO_PIN_8
#define TMC_EN_GPIO_Port GPIOA
#define ESP_TX_Pin GPIO_PIN_9
#define ESP_TX_GPIO_Port GPIOA
#define ESP_RX_Pin GPIO_PIN_10
#define ESP_RX_GPIO_Port GPIOA
#define TMC_STEP_Pin GPIO_PIN_11
#define TMC_STEP_GPIO_Port GPIOA
#define TMC_DIR_Pin GPIO_PIN_12
#define TMC_DIR_GPIO_Port GPIOA
#define TFT_TX_Pin GPIO_PIN_10
#define TFT_TX_GPIO_Port GPIOC
#define TFT_RX_Pin GPIO_PIN_11
#define TFT_RX_GPIO_Port GPIOC
#define IRCAP_ST_Pin GPIO_PIN_1
#define IRCAP_ST_GPIO_Port GPIOD
#define CASE_ST_Pin GPIO_PIN_2
#define CASE_ST_GPIO_Port GPIOD
#define PWR_ST_Pin GPIO_PIN_3
#define PWR_ST_GPIO_Port GPIOD
#define FUEL_ST_Pin GPIO_PIN_4
#define FUEL_ST_GPIO_Port GPIOD
#define PWR2_ST_Pin GPIO_PIN_5
#define PWR2_ST_GPIO_Port GPIOD
#define REM_ST_Pin GPIO_PIN_6
#define REM_ST_GPIO_Port GPIOD
#define REM_FLAME_Pin GPIO_PIN_7
#define REM_FLAME_GPIO_Port GPIOD
#define PLUG_OUT_Pin GPIO_PIN_3
#define PLUG_OUT_GPIO_Port GPIOB
#define HE1_OUT_Pin GPIO_PIN_4
#define HE1_OUT_GPIO_Port GPIOB
#define HE2_OUT_Pin GPIO_PIN_5
#define HE2_OUT_GPIO_Port GPIOB
#define HE3_OUT_Pin GPIO_PIN_6
#define HE3_OUT_GPIO_Port GPIOB
#define HE4_OUT_Pin GPIO_PIN_7
#define HE4_OUT_GPIO_Port GPIOB
#define HE5_OUT_Pin GPIO_PIN_8
#define HE5_OUT_GPIO_Port GPIOB
#define HE6_OUT_Pin GPIO_PIN_9
#define HE6_OUT_GPIO_Port GPIOB
#define PWR_ON_OFF_Pin GPIO_PIN_0
#define PWR_ON_OFF_GPIO_Port GPIOE
#define PUMP_OUT_Pin GPIO_PIN_1
#define PUMP_OUT_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
