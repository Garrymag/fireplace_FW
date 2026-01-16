/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2022 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "ntc_table.h"
#include "DWIN_APP.h"
#include "DWIN.h"
#include "ESP32.h"
#include "RX433.h"
#include "w25qxx.h"
#include "DFPLAYER_MINI.h"
#include "vl53l0x_api.h"
#include "vl53l0x_api_calibration.h"
#include "vl53l0x_api_core.h"
#include "vl53l0x_api_ranging.h"
#include "FuelLevel.h"
#include "settings.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define для АЦП
#define FUEL 0
#define LEAK1 1
#define LEAK2 2
#define IR1 3
#define IR2 4
#define IR3 5
#define T_IN 6
#define T1 7
#define T2 8
#define T3 9
#define T4 10
#define T5 11
#define T6 12
#define MOTOR 13
///////////////////////////////////////////////////////////
const uint16_t IR_DELAY_BURN = 60;	   // Задержка переключения При горении
const uint16_t IR_DELAY_PREBURN = 180; // Задержка переключения При розжиге
// Задержка руки
const uint32_t IR_COUNT_DEF = 300;				 // Все режимы, кроме
const uint32_t IR_COUNT_COOL = 1700;			 // Остывание
const uint32_t IR_COUNT_PREBURN = 200000;		 // Розжиг
const uint32_t IR_COUNT_FLAME_DETECTED = 300000; // Обнаружили пламя при розжиге
//////////////////////////////////////////////////////////////////////////////////////
const uint16_t PreHeatDuty = 18000; // мощность нагревателя в процессе разогрева (по умолчанию 18 000)

const uint8_t PLUG_FAULT_CNT = 5; // задержка до аварии свечи, цыфра умножить на 40 сек.

// Коэффициент высоты измерительного бачка
float k_disp_fuel = 1.1f; // коэф высоты бака k_disp_fuel = (LOW_FUEL - MAX_FUEL) / (20.0 * кэфф запаса)  коэф запаса в премиуме 1.158
// #define FUEL_LVL_K 2.8f

// Все коэфф фильтров с учетом частоты опроса 10 гц

// Коэффициент фильтрации топлива. Чемм больше, тем меньше фильтрует
const float kf_fuel_fueling = 0.03f;   // при заправке
const float kf_fuel_burning = 0.0001f; // прир горении
const float kf_fuel_std = 0.005f;	   // в остальных режимах

const float K_FILT_TEMP = 0.01f;
const float K_FILT = 0.1f;
float K_FILT_CAP = 0.5f; // коэфф фильтра потенциометра лючка

#define FAIL_TOPCAP_MAX 20
// const float CAP_DX_DT_MAX = 10.0; // макс величина dx/dt для дельта фильтра потенциометра лючка

// топливные коэффициенты (для правильного отображения уровня)
#define FUEL_BIAS_K 2.0f
#define FUEL_BIAS_K_ 0.7f

#define K_FROAD 1.0f // коэффициент вранья для времени горения

#define MAX_FUEL 80.0f	// shalet 65 полный бак
#define LOW_FUEL 130.0f // 100 	//50 пустой бак

#define FUEL_ZAP_STOP 2.0f // Запас топлива до выключения
#define FUEL_ZAP_WARN 1.0f // Запас до warning
const float FUEL_STOP = LOW_FUEL - FUEL_ZAP_STOP;
const float FUEL_WARN = LOW_FUEL - FUEL_ZAP_WARN;

// const float a_disp_fuel = (FUEL_BIAS_K - FUEL_BIAS_K_) / (LOW_FUEL - MAX_FUEL);
// const float b_disp_fuel = FUEL_BIAS_K - a_disp_fuel * LOW_FUEL;
float FUEL_LVL_K = (LOW_FUEL - MAX_FUEL) / 20.0f; //

float fuel_buff_disp = 0.0f; //

float filt_temp = 0.0f;

// define уровней для ацп
#define T_LVL_HEATER 100.0f
#define T_LVL_5 74.0f
#define T_LVL_4 73.0f
#define T_LVL_3 72.0f
#define T_LVL_2 71.0f
#define T_LVL_1 70.0f
#define T_LVL_BURN_3 72.0f // 73
#define T_LVL_BURN_2 67.0f // shalet 60
#define T_LVL_HEATER_START 80.0f
#define T_STARTUP 45.0f // температруа испарителей при котрой камин считается остывшим

uint8_t max_vapor_index = 0; // максимальный индекс датчика температуры испарителя - зависит от длины камина(количества испарителей)
float test_max_vapor_T = 0.0f;
float max_vapor_T = 0.0f; // максимальная темепратура всех испарителей
uint8_t cool_scale = 0;	  // время до остывания/нагрева

/*    При нагреве изначально задаются
heat_cool_scale = 0; шкала 0 это палки
				time_heat_cool = 5;	 время 5 минут
			потом во время нагрева ищем попадание максимальной температуры в диапазон и если нашли
		то переключаем шкалу(только вверх от нуля или от текущей шкалы)
		например в таблице диапазон 20 - 52 градуса: если определяем, что температура между этими числами,
		и если текущая шкала = 0, то переключаем шкалу на 1 и время на 4.
		следующий диапазон 52 - 54	градуса: если определяем, что температура между этими числами,
		и если текущая шкала = 0 или 1 то переключаем шкалу на 2 и время на 3.
		и так далее по всем 5 диапазонам*/
/*const uint8_t heat_state[6][] = {// 0-Temp, 1-Time
									 {20,      4},
									 {52,      3},
																		 {54,      2},
																		 {58,      1},
																		 {62,      0},
																		 {70,      0}
};
*/
const uint8_t heat_state[6][3] = { // 0-Temp, 1-Time
	{45, 10, 4},
	{52, 10, 3},
	{54, 10, 2},
	{58, 10, 1},
	{62, 10, 1},
	{70, 10, 0}};

/*    При охлаждении изначально задаются
		heat_cool_scale
				time_heat_cool
		по таблице waitcool_state
			потом во время охлаждения ищем попадание максимальной температуры в диапазон и если нашли
		то переключаем шкалу(только вниз от текущей шкалы)
		например в таблице диапазон 67 - 71 градуса: если определяем, что температура между этими числами,
		и если текущая шкала = 5, то переключаем шкалу на 4 и время на 7.
		следующий диапазон 63 - 67	градуса: если определяем, что температура между этими числами,
		и если текущая шкала = 4 то переключаем шкалу на 3 и время на 5.
		и так далее по всем 5 диапазонам*/
const uint8_t cool_state[11][3] = { // 0-Temp, 1-Time
	{0, 10, 1},
	{47, 10, 3},
	{52, 10, 5},
	{54, 10, 6},
	{57, 10, 8},
	{60, 1, 0},
	{62, 1, 1},
	{65, 1, 2},
	{67, 1, 3},
	{69, 1, 4},
	{71, 1, 5}};

/*    При охлаждении по этой таблице определяем
	  стартовые значения(один раз)
		heat_cool_scale
				time_heat_cool
		если температура меньше первой, то
		heat_cool_scale = 0
				time_heat_cool = 0 */
/*const uint8_t waitcool_state[10][3] = {// 0-Temp, 1-Time
									 {46,      10,     3},
									 {48,      10,     6},
																		 {50,      10,     9},
																		 {53,      1,      2},
																		 {55,      1,      5},
																		 {58,      1,      8},
									 {61,      2,      1},
																		 {64,      2,      3},
																		 {68,      2,      6},
																		 {72,      3,      0}
};*/
const uint8_t waitcool_state[10][3] = { // 0-Temp, 1-Time
	{46, 10, 2},
	{48, 10, 4},
	{50, 10, 6},
	{53, 10, 7},
	{55, 10, 9},
	{58, 1, 0},
	{61, 1, 1},
	{64, 1, 1},
	{68, 1, 3},
	{72, 1, 5}};

const uint8_t prefer_handy = 1; // если 0 то значения уровней шим пламени, макс. и жел. открытия лючка берем из флеш иначе - отсюда

const uint8_t SENSE_MAX_OPEN_HANDY = 8;							 // максимальное открытие лючка
const uint8_t SENSE_OPEN_HANDY = 18;							 // желательное открытие лючка
const uint8_t D_MOTOR_SENS = 34;								 // Разница верх и ниж положения лючка (шаги лючка)
const uint8_t FLAME_LEVELS_HANDY[5] = {100, 130, 160, 190, 240}; // уровни пламени
const uint8_t T_LVL_LOW_HANDY = 105;							 // подогрев сковородки
const uint8_t T_LVL_START_HANDY = 70;							 // начало розжига
const uint8_t T_LVL_START_LOW_HANDY = 60;						 // температура сковородки

// Скважность на вентиляторы в режиме горения 0-200
#define FAN_DUTY_BURN 20

// Define для статусов камина
#define STANDBY 0
#define PRE_HEAT 1
#define HEAT 2
#define PRE_BURN 3
#define BURN 4
#define BURN_AFTER_5 5
#define COOLING 6
#define FUELING 7
#define FULL_FUELING 8
#define WAIT 9
#define WAIT_COOL 10
#define WARNING 11
#define WAIT_WAR 12
#define FAULT 13




// define для музыки
#define DROVA 0x01
#define FOREST 0x02
#define RAIN 0x03
#define RATATA 0x04
#define MIR 0x05

// define для кулеров
// #define 	FAN_START				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3)
// #define 	FAN_STOP				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3)
// #define 	FAN_TIM					TIM3->CCR3
#define FAN_START HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3)
#define FAN_STOP HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3)
#define FAN_TIM TIM8->CCR3

// define для электродов

#define PLUG_TIM TIM2->CCR2
void PLUG_START(void)
{
	PLUG_TIM = 0;
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim2);
}

void PLUG_STOP(void)
{
	HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_Base_Stop_IT(&htim2);
}

//#define TIMEOUT_PLUG_STOP HAL_TIM_Base_Stop_IT(&htim14)

/* void TIMEOUT_PLUG_START(void)
{
	__HAL_TIM_CLEAR_FLAG(&htim14, TIM_SR_UIF);
	HAL_TIM_Base_Start_IT(&htim14);
}
 */
// define для насоса
#define PUMP_START HAL_GPIO_WritePin(GPIOE, PUMP_OUT_Pin, GPIO_PIN_RESET)
#define PUMP_STOP HAL_GPIO_WritePin(GPIOE, PUMP_OUT_Pin, GPIO_PIN_SET)
/*shalet void PUMP_START(void)
{
	HAL_GPIO_WritePin(GPIOE, PUMP_OUT_5V_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,PUMP_OUT_Pin,GPIO_PIN_RESET);
}
void PUMP_STOP(void)
{
	HAL_GPIO_WritePin(GPIOE, PUMP_OUT_5V_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,PUMP_OUT_Pin,GPIO_PIN_SET);
}*/
// define для управления  нагревателями
#define HE1_START HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1)
#define HE2_START HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2)
#define HE3_START HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1)
#define HE4_START HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_2)
#define HE5_START HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3)
#define HE6_START HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4)

#define HE1_TIM TIM3->CCR1
#define HE2_TIM TIM3->CCR2
#define HE3_TIM TIM4->CCR1
#define HE4_TIM TIM4->CCR2
#define HE5_TIM TIM4->CCR3
#define HE6_TIM TIM4->CCR4

#define HE1_STOP HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1)
#define HE2_STOP HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_2)
#define HE3_STOP HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1)
#define HE4_STOP HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_2)
#define HE5_STOP HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_3)
#define HE6_STOP HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_4)

#define PID_DUTY_CYCLE_MAX 30000.0f

#define MUTE_ON HAL_GPIO_WritePin(GPIOE, PWR_ON_OFF_Pin, GPIO_PIN_SET)
#define MUTE_OFF HAL_GPIO_WritePin(GPIOE, PWR_ON_OFF_Pin, GPIO_PIN_RESET)

// define ошибок
#define LOW_FUEL_WAR 0x01
#define OPEN_FUEL_WAR 0x02
#define LEAK2_WAR 0x04
#define LEAK1_WAR 0x08

#define OPEN_ELCAMINO 0x01
#define HE_ERR 0x02
#define PLUG_ERR 0x04
#define IR_ERR 0x08
#define TSENS_ERR 0x10
#define LEAK1_PISS 0x20
#define HI_T_WAR 0x40
#define LOW_T_WAR 0x80

// Define значений для критических ошибок и предупреждений
#define H_LOW_LVL_FUEL 25.0f
#define LOW_LVL_FUEL 35.0f
#define T_LVL_ERR 130.0f	// Высокий порог температуры  испарителя
#define T_HI_LVL_WAR 60.0f	// Высокий порог температуры, при которой нельзяа запускать камин
#define T_LOW_LVL_WAR 10.0f // Низкий порог температуры, при которой нельзя запускать камин
#define LEAK1_LVL_WAR 85.0f // shalet 90.0f			//Уровень ошибки протечки бензобака
#define LEAK2_LVL_WAR 85.0f // Уровень ошибки протечки топлива через люк
#define LEAK1_LVL_NON 95.0f
#define LEAK2_LVL_NON 95.0f
#define HI_LVL_ERR 85.0f	// Уровень ошибки по КЗ
#define HI_T_LVL_ERR 300.0f // Уровень ошибки по КЗ температуры
#define LOW_T_LVL_ERR 7.0f	// Уровень ошибки по обрыву температуры
#define LOW_LVL_ERR 0x0000	// Уровень ошибки по обрыву
#define IR_LVL_BURN 40.7f	// shalet 50.7							// Уровень �?К датчика, когда камин разгорелся

// Define для пин-кода
#define USE_PIN_ON 0x02
#define USE_PIN_433_ON 0x20
#define NON 0
#define NEW_SETTING_PIN 1
#define NEW_SETTING_PIN_433 2
#define ENTER_SETTING_PIN 3
#define ENTER_SETTING_PIN_433 4
#define LOCK 5
#define UNLOCK 6

// define для звуков
#define SYSTEM_SOUND_ON 0x02
#define SYSTEM_SOUND_OFF 0x01
#define TOUCH_SOUND_ON 0x02
#define TOUCH_SOUND_OFF 0x01
// define для памяти
#define MEM_ARR 32

// 1.34f // shalet 1.00f
//  define для мотора открытия крышки //shalet not present
#define DIR_FORWARD HAL_GPIO_WritePin(GPIOA, TMC_DIR_Pin, GPIO_PIN_SET)
#define DIR_BACKWARD HAL_GPIO_WritePin(GPIOA, TMC_DIR_Pin, GPIO_PIN_RESET)
#define MOTOR_START HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4)
#define MOTOR_STOP HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4)
#define MOTOR_EN HAL_GPIO_WritePin(GPIOA, TMC_EN_Pin, GPIO_PIN_RESET)
#define MOTOR_DIS HAL_GPIO_WritePin(GPIOA, TMC_EN_Pin, GPIO_PIN_SET)
#define OPEN_MOTOR 25.0f
#define OPEN_CLOSE_TIMER 100

/* USER CODE END PV */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CMD_CAP_STOP 0
#define CMD_CAP_OPEN_CLOSE 1
#define LONG_KEY_TIME 12	// 1200 мс
#define PAUSE_AFTER_LONG 10 /// 1000 мс
#define SHORT_KEY_TIME 1	// 100 мс
#define NO_KEY_TIME 4		// 400 мс
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float k_filter_fuel = 0.05;		// shalet not present		// коэф филтра среднего скользящего
uint8_t pwr_st = 0;				// флаг включения камина
uint8_t bright_st = 0;			// shalet not present		//флаг включение подсветки
uint16_t fail_topcap_count = 0; // shalet not present			// счетчик закрытия при блокировки люка
uint8_t close_topcap_count = 0; // shalet not present			// Счетчик закрытия
uint16_t count_5min = 301;		// shalet not present				//счетчик 5 минут
uint8_t elcamino_lenght = 10;	// shalet not present			//переменная длины камина
uint8_t mp3_start_flag = 0;		// Это флаг на инициализацию MP3 модуля, когда подается основное питание
uint8_t ctrl_key_time = 0;		// флаг удержание кнопки
uint8_t up_key_time = 0;		// флаг удержание кнопки
uint8_t down_key_time = 0;		// флаг удержание кнопки
uint8_t nokey_time = 0;
uint8_t pause_after_long = 0;
uint8_t cmd_cap_remote = CMD_CAP_STOP;
uint8_t hand_cap_ctrl = 1;
uint8_t rx_int_flag = 0;
uint8_t lol_flag = 0;			  // ЭТО САМЫЙ ВАЖНЫЙ ФЛАГ
uint8_t adc_flag = 0;			  // Флаг коллбека АЦП
uint8_t adc_ask = 0;			  // флаг запроса с сервисной программы
uint8_t rx433_flag = 0;			  // флаг коллбэка 433
uint8_t plug_tim_flag = 0;		  // флаг старта таймера на росжиг свечки
uint8_t rc_service_data[8] = {0}; // Команды с сервисной программы
uint8_t tx_service_data = 0;	  // Команды на сервисную программу
uint16_t elcamino_addr = 0;		  // переменная с адресом переменных дислпея
uint8_t elcamino_st = 0;		  // Состояние камина
uint8_t fault_st = 0;			  // Состояние ошибок
uint8_t memory[MEM_ARR] = {0};	  // буффер чтении из памяти
uint8_t warning_st = 0;			  // состояние предупреждений
uint8_t service_st = 0;			  // Сервисный статус

uint16_t elcamino_cmd = 0;	   // команды с экрана на стм
uint8_t burn_lvl = 1;		   // уровень горения
uint8_t cancel_cool_count = 0; // отсчет времени отмены охлажденя
uint8_t heat_cool_scale = 0;   // Шкала охлаждения и нагрева
uint8_t inp_fuel = 21;
//  shalet float time_k[5] = {0.24,0.37,0.45,0.67,0.9};							// переменная для таймера топлива
// float time_k[5] = {0.23, 0.32, 0.39, 0.45, 0.49}; // массив коэффциентов горения
// float time_k[5] = {0.244, 0.37, 0.455, 0.66, 1.0};
// 420 352.5 285 217.5 150
float time_k[5] = {0.237, 0.282, 0.348, 0.457, 0.66};
uint16_t fuel_time = 0;		   // переменная для расчета времени горениия
uint16_t sleep_time_count = 0; // переменная для таймера сна
uint8_t sleep_time_st = 0;	   // статус таймера сна
float sleep_time_view = 0;

uint8_t count_pin = 0;										 // счетчик цифр пинкода
uint8_t pin_st = 0;											 // Статус пин кода
uint8_t pin_cfg = 0;										 // конфигурация пина
uint16_t pincode = 0;										 // Значение пин кода
uint16_t pincode_temp = 0;									 // значение вводимого пинкода
uint8_t pin_arr[4] = {0};									 // Массив цифр пинкода с экрана//
uint16_t num_pin_addr[4] = {0x5064, 0x5066, 0x5068, 0x5070}; // Массив адресов, в которых отображаюся цифры на экране
uint16_t SERVICE_PIN = 0x5A51;
// PW для  мотора //shalet not present
uint32_t max_ir = 0;
uint16_t ir_delay = 0;
uint16_t ir_delay_max = 60;
uint32_t ir_count = 0;
uint32_t ir_delay_count = 0;
uint32_t ir_burn_count[5] = {30000, 30000, 30000, 30000, 30000}; // Задержка сработки датчика руки при горении

uint16_t motor_flag = 0;
uint16_t motor_sens_max_open = 30;
uint16_t motor_sens_open = 30;
uint16_t motor_sens_close = 80;
uint8_t cover_open_close = 0;

uint8_t fuel_st = 0;
uint8_t fuel = 0;
float buff_fuel = 0.0f;
float filter_fuel = 0.0f;

uint8_t min = 0;
uint8_t hour = 0;
uint8_t play_st;
uint8_t rf_data[3];

uint8_t rf_data_svd[3];

int i;
extern struct readDataDWIN_P readDataDWIN;
extern struct ESP32Data_P ESP32Data;
// PW для PID
// shalet uint16_t burn_pwm[5] = {8600,12500,14800,16500,19500};
uint16_t burn_pwm[5] = {11000, 13500, 16000, 18000, 20000}; // Массив максимального шима
uint8_t ask_temp[9] = {0};									// Массив температур уровня горения из сервиса
uint8_t t_lvl_start_low = 60;								// shalet 69							// температура нижнего подогрева на момент запуска
uint8_t t_lvl_start = 64;									// shalet 63									// температура испартеля на момент старта
uint8_t t_lvl_low = 105;									// температура нижнего подогрева в рабочем режиме
// PW для ADC
volatile uint16_t adc[14] = {0}; // Данные с АЦП
uint8_t adc_service[15] = {0};	 // Данные с АЦП для сервисной проги
float adc_f[14] = {0};			 // Массив с преобразованными данными с АЦП
float filter_adc_f[14] = {0};	 // Массив для фильтрации АЦП

uint8_t serial_number[7] = {0};
uint8_t Message[64];
uint8_t MessageLen;
VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_Dev_t vl53l0x_c; // center module
VL53L0X_DEV Dev = &vl53l0x_c;
uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;

uint8_t flag_sleep_int = 0; // флаг пропуска ISR 10 таймера (sleep)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void memory_init()
{
	HAL_Init();
	SystemClock_Config();
	HAL_Delay(7000); // shalet 6000
	//	buzzerTouchOnDWIN();
	W25qxx_ReadBytes(memory, 0, MEM_ARR); // считывание значений настроек из памяти
	if (memory[0] != 0xFF)
	{
		elcamino_lenght = memory[0]; // shalet not present
		max_vapor_index = T1;
		if (elcamino_lenght >= 10)
			max_vapor_index = T2;
		if (elcamino_lenght >= 15)
			max_vapor_index = T3;
		if (elcamino_lenght >= 20)
			max_vapor_index = T4;
		if (elcamino_lenght >= 25)
			max_vapor_index = T5;
		if (elcamino_lenght == 30)
			max_vapor_index = T6;
	}
	if (memory[1] == 0xFF) // shalet commented
		fault_st = 0;
	else
		fault_st = 0; /// memory[1]; // запись в переменную ошибок ошибки по вскрытию камина
	if ((fault_st & (OPEN_ELCAMINO)) == OPEN_ELCAMINO)
	{
		goToPageDWIN(OPEN_ELCAMINO_SCREEN);
	}
	if (memory[2] == 0x02) // настройка работы звука тач дисплея
	{
		buzzerTouchOnDWIN();
		writeHalfWordDWIN(DISP_SOUND_ADDR, 2);
	}
	else
	{
		buzzerTouchOffDWIN();
		writeHalfWordDWIN(DISP_SOUND_ADDR, 1);
	}
	if (memory[3] == SYSTEM_SOUND_ON) // Сохранение настройки звуков камина в переменную
	{
		writeHalfWordDWIN(SYS_SOUND_ADDR, 2);
		HAL_TIM_Base_Init(&htim8);
	}
	else
	{
		writeHalfWordDWIN(SYS_SOUND_ADDR, 1);
		HAL_TIM_Base_DeInit(&htim8);
	}
	if (memory[4] != 0xFF)
		setLedDWIN(memory[4]); // настройка яркости
	else
		setLedDWIN(0x64);

	pin_cfg = memory[5];
	if (memory[5] == 0xFF)
		pin_cfg = 0x00;
	if ((pin_cfg & (USE_PIN_ON)) == USE_PIN_ON)
		writeHalfWordDWIN(PIN_USE_ADDR, ON);
	else
		writeHalfWordDWIN(PIN_USE_ADDR, OFF);
	if ((pin_cfg & (USE_PIN_433_ON)) == USE_PIN_433_ON)
		writeHalfWordDWIN(PIN_USE_433_ADDR, ON);
	else
		writeHalfWordDWIN(PIN_USE_433_ADDR, OFF);
	if (memory[6] != 0xFF)
		burn_lvl = memory[6]; // Настройка уровня горения
	else
		burn_lvl = 1;
	writeHalfWordDWIN(FLAME_LVL_ADDR, burn_lvl);
	pincode = memory[7] << 8 | memory[8]; // Сохранение пинкода в переменную
	for (int i = 0; i < 7; i += 1)
	{
		serial_number[i] = memory[i + 9];
	}
	if (memory[16] != 0xFF)
	{
		for (int i = 0; i < 5; i += 1)
		{
			burn_pwm[i] = memory[16 + i] * 100;
		}
		t_lvl_low = memory[21];
		t_lvl_start = memory[22];
		t_lvl_start_low = memory[23];
	}
	if (memory[24] != 0xff)
	{
		DF_SetVolume(memory[24]);
		writeHalfWordDWIN(MP3_VOLUME_ADDR, memory[24]);
	}
	else
	{
		DF_SetVolume(15);
		writeHalfWordDWIN(MP3_VOLUME_ADDR, 15);
	}
	if (memory[25] == ON)
	{
		DF_SetTrack(DROVA); // отправка по уарт играть дрова
		writeHalfWordDWIN(FOREST_MP3_ADDR, OFF);
		writeHalfWordDWIN(RAIN_MP3_ADDR, OFF);
		writeHalfWordDWIN(DROVA_MP3_ADDR, ON);
		DF_Pause();
	}
	if (memory[26] == ON)
	{
		DF_SetTrack(FOREST); // отправка по уарт играть дрова
		writeHalfWordDWIN(FOREST_MP3_ADDR, ON);
		writeHalfWordDWIN(RAIN_MP3_ADDR, OFF);
		writeHalfWordDWIN(DROVA_MP3_ADDR, OFF);
		DF_Pause();
		;
	}
	if (memory[27] == ON)
	{
		DF_SetTrack(RAIN); // отправка по уарт играть дрова
		writeHalfWordDWIN(FOREST_MP3_ADDR, OFF);
		writeHalfWordDWIN(RAIN_MP3_ADDR, ON);
		writeHalfWordDWIN(DROVA_MP3_ADDR, OFF);
		DF_Pause();
	}
	if (memory[28] != 0xFF)
	{ // shalet dumb
		motor_sens_max_open = memory[28];
		motor_sens_close = motor_sens_max_open + D_MOTOR_SENS;
	}

	if (memory[29] != 0xFF)
	{
		motor_sens_open = memory[29];
	}

	if (memory[30] != 0xFF)
	{
		pwr_st = memory[30];
	}
	if (pwr_st == 1)
	{
		ESP32SendWakeUpCode();
		__HAL_TIM_CLEAR_FLAG(&htim9, TIM_SR_UIF);
		HAL_TIM_Base_Start_IT(&htim9); // если питание идет от сети - запускаем таймер на рестарт ESP
	}

	if (prefer_handy == 1)
	{
		motor_sens_max_open = SENSE_MAX_OPEN_HANDY;
		motor_sens_close = SENSE_MAX_OPEN_HANDY + D_MOTOR_SENS;
		motor_sens_open = SENSE_OPEN_HANDY;
		for (int i = 0; i < 5; i += 1)
		{
			burn_pwm[i] = FLAME_LEVELS_HANDY[i] * 100;
		}
		t_lvl_low = T_LVL_LOW_HANDY;
		t_lvl_start = T_LVL_START_HANDY;
		t_lvl_start_low = T_LVL_START_LOW_HANDY;
	}

	writeHalfWordDWIN(SLEEP_ON_ADDR, 1);
	writeHalfWordDWIN(PIN_LOCK_ADDR, 2);
	BUZZ_START();
	HAL_Delay(300); // shalet 500
	BUZZ_STOP();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_ADC2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	// shalet not present
	// shalet MUTE_ON;
	HE1_TIM = burn_pwm[3];
	HE2_TIM = burn_pwm[3];
	HE3_TIM = burn_pwm[3];
	HE4_TIM = burn_pwm[3];
	HE5_TIM = burn_pwm[3];
	HE6_TIM = burn_pwm[3];
	ESP32DmaInit();
	W25qxx_Init();
	dwinUartDmaInit();
	HAL_UART_Receive_IT(&huart6, rc_service_data, 8);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc, 14);
	__HAL_TIM_CLEAR_FLAG(&htim6, TIM_SR_UIF);
	HAL_TIM_Base_Start_IT(&htim6);
	__HAL_TIM_CLEAR_FLAG(&htim5, TIM_SR_UIF); // тест
	HAL_TIM_Base_Start_IT(&htim5);			  // тест

	Dev->I2cHandle = &hi2c2;
	Dev->I2cDevAddr = 0x52;
	VL53L0X_WaitDeviceBooted(Dev);
	VL53L0X_DataInit(Dev);
	VL53L0X_StaticInit(Dev);
	VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
	VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
	VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	VL53L0X_StartMeasurement(Dev);
	VL53L0X_GetMeasurementDataReady(Dev, &fuel_st);
	ESP32Data.status = STM32_WAIT; // shalet not present
	ESP32SendStatus();			   // shalet not present
	DF_Init(15);
	FuelLevel_Init(); // Initialize fuel level module
	memory_init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{
		//=================== Обработка кнопки включения камина и HW Reset=====================//
		//	{
		//		if ((HAL_GPIO_ReadPin(GPIOD,PWR_ST_Pin) == GPIO_PIN_SET)&&(pwr_st == 0))										// Если нажата кнопка включения и камин был выключен
		//		{
		//			__enable_irq ();																																					// Включаем прерывания
		//			pwr_st = 1;																																								// Ставим флаг "питание вкл"
		//			fault_st = 0;																																							// Скидываем ошибки
		//			memory[30] = pwr_st;																																			// Записываем флаг в память, что питание включено
		//			W25qxx_EraseSector(0);																																		// Стираем весь сектор памяти EEPROM
		//			W25qxx_WritePage(memory,0,0,MEM_ARR);																											// Записываем заного весь буффер памяти в EEPROM
		//			MOTOR_STOP;
		//			MOTOR_DIS;
		//			memory_init();																																						// инициализируем память для вывода параметров на экран
		//			elcamino_st = STANDBY;
		//		}
		//		if ((HAL_GPIO_ReadPin(GPIOD,CASE_ST_Pin) == 1)&&(pwr_st == 0))															//Если камин был разобран и питание было выключено
		//		{
		//			ESP32SendServiceCode(ESP32_SLEEP);																												// Отправляем ESP в сон
		//			__enable_irq ();																																					// Включаем прерывания
		//			fault_st |= OPEN_ELCAMINO;																																// Ставим ошибку - камин вскрыт
		//			memory[1] = fault_st;																																			// Записываем в буфер памяти ошибку о вскрытии камина
		//			W25qxx_EraseSector(0);																																		//Стираем весь сектор памяти EEPROM
		//			W25qxx_WritePage(memory,0,0,MEM_ARR);																											//Записываем заного весь буфер памяти в EEPROM
		//			__disable_irq ();																																					// Выключаем прерывания
		//			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFE);											// Отправляем МК в стопмод
		//		}
		//		if ((HAL_GPIO_ReadPin(GPIOD,PWR_ST_Pin) == GPIO_PIN_RESET)&&(pwr_st == 1))									// Если нажата кнопка выключения и камин был включен
		//		{
		//			ESP32SendServiceCode(ESP32_SLEEP);																												// Отправляем ESP в сон
		//			pwr_st = 0;																																								// Ставим флаг "питание выкл"
		//			memory[30] = pwr_st;																																			// Записываем флаг в буфер памяти
		//			W25qxx_EraseSector(0);																																		// Стираем весь сектор памяти EEPROM
		//			W25qxx_WritePage(memory,0,0,MEM_ARR);																											// Записываем заного весь буфер памяти в EEPROM
		//			__disable_irq ();																																					// Выключаем прерывания
		//			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFE);											// Отправляем МК в стопмод
		//		}
		//	}
		//===================Прием и обработка команд с экрана ==================//
		{
			parsingDWIN();																					  // Парсим то, что пришло с экрана
			elcamino_addr = readDataDWIN.parsingDataDWIN.data[0] << 8 | readDataDWIN.parsingDataDWIN.data[1]; // Записываем адрес переменной с экрана в переменную МК
			elcamino_cmd = readDataDWIN.parsingDataDWIN.data[4];											  // Записываем команду с экрана в переменну МК
			if (elcamino_addr == CMD_ADDR)																	  // Если это адрес для команд управления
			{
				switch (elcamino_cmd) // Парсим команды управления
				{
				case (HEAT_CMD): // Если пришла команда о нагреве
				{
					if (elcamino_st == WAIT) // �? если статус камина - ожидание команды
					{
						elcamino_st = PRE_HEAT;
					}
				}
				break;
				case (PREFUEL_CMD):
				{
					k_filter_fuel = 0.1f; // shalet not present
					TIM11->ARR = 3000;	  // shalet 3845
					if (filter_fuel < MAX_FUEL)
					{
						// k_filter_fuel = 0.05f; // shalet not present
						goToPageDWIN(FULL_FUEL_SCREEN);
/* 						TIM11->ARR = 3500;
						__HAL_TIM_CLEAR_FLAG(&htim11, TIM_SR_UIF); // shalet not present
						HAL_TIM_Base_Start_IT(&htim11);			   // Таймер 5с на перекачку из шланга */
					}
					else
					{
						elcamino_st = FUELING;
						goToPageDWIN(PREFUEL_SCREEN);
					}
				}
				break;
				case (START_FUEL_CMD):
				{
					PUMP_START;
					elcamino_cmd = 0;
				}
				break;
				case (STOP_FUEL_CMD):
				{
					PUMP_STOP;
				}
				break;
				case (END_FUEL_CMD):
				{
					if (warning_st == 0)
					{
						elcamino_st = WAIT;
						ESP32Data.status = STANDBY_ST;
						ESP32SendStatus();
					}
					else
						elcamino_st = WAIT_WAR;
					k_filter_fuel = 0.05f; // shalet not present
					PUMP_START;
					__HAL_TIM_CLEAR_FLAG(&htim11, TIM_SR_UIF);
					HAL_TIM_Base_Start_IT(&htim11); // Таймер 5с на перекачку из шланга
				}
				break;
				case (BACK_FROM_MENU_CMD): // Если команда возврата из меню
				{
					k_filter_fuel = 0.05f; // shalet not present
					if ((pin_st == NON) || (pin_st == UNLOCK) || ((pin_cfg & USE_PIN_433_ON) == USE_PIN_433_ON) || ((pin_cfg & USE_PIN_433_ON) == !USE_PIN_433_ON))
					{
						if ((elcamino_st == STANDBY) || (elcamino_st == WAIT))
							goToPageDWIN(STANDBY_SCREEN);
						if (elcamino_st == BURN)
							goToPageDWIN(BURN_SCREEN);
						if ((elcamino_st == HEAT) || (elcamino_st == PRE_BURN))
							goToPageDWIN(HEAT_SCREEN);
						if (elcamino_st == BURN_AFTER_5)
							goToPageDWIN(BURN_5_SCREEN);
						if (elcamino_st == WAIT_WAR)
							goToPageDWIN(WARNING_SCREEN);
						for (i = 0; i < 4; i = i + 1)
						{
							pin_arr[i] = 0;
							writeHalfWordDWIN(num_pin_addr[i], 0);
						}
						count_pin = 0;
					}
					if ((pin_st == ENTER_SETTING_PIN) || (pin_st == ENTER_SETTING_PIN_433))
					{
						goToPageDWIN(PIN_SETTING_SCREEN);
						if (pin_st == ENTER_SETTING_PIN)
							writeHalfWordDWIN(PIN_USE_ADDR, ON);
						if (pin_st == ENTER_SETTING_PIN_433)
							writeHalfWordDWIN(PIN_USE_433_ADDR, ON);
						for (i = 0; i < 4; i = i + 1)
						{
							pin_arr[i] = 0;
							writeHalfWordDWIN(num_pin_addr[i], 0);
						}
						count_pin = 0;
					}
					if ((pin_st == LOCK) && ((pin_cfg & USE_PIN_ON) == USE_PIN_ON))
					{
						if ((elcamino_st == STANDBY) || (elcamino_st == WAIT))
							goToPageDWIN(LOCK_STANDBY_SCREEN);
						if (elcamino_st == BURN)
							goToPageDWIN(LOCK_BURN_SCREEN);
						if ((elcamino_st == HEAT) || (elcamino_st == PRE_BURN))
							goToPageDWIN(LOCK_HEAT_SCREEN);
						if (elcamino_st == BURN_AFTER_5)
							goToPageDWIN(LOCK_BURN_5_SCREEN);
						if (elcamino_st == WAIT_WAR)
							goToPageDWIN(LOCK_WARNING_SCREEN);
						for (i = 0; i < 4; i = i + 1)
						{
							pin_arr[i] = 0;
							writeHalfWordDWIN(num_pin_addr[i], 0);
						}
						count_pin = 0;
					}
					if (elcamino_st == FUELING)
					{
						if (warning_st == 0)
						{
							goToPageDWIN(STANDBY_SCREEN);
							elcamino_st = WAIT;
						}
						else
						{
							goToPageDWIN(WARNING_SCREEN);
							elcamino_st = WAIT_WAR;
						}
					}
					if ((sleep_time_st == OFF) || (sleep_time_st == 0))
					{
						writeHalfWordDWIN(SLEEP_TIME_ADDR, 0);
						sleep_time_count = 0;
					}
				}
				break;
				case (BACK_FROM_PIN_ENTER_CMD):
				{
					goToPageDWIN(PIN_SETTING_SCREEN);
					if (pin_st == NEW_SETTING_PIN)
						writeHalfWordDWIN(PIN_USE_ADDR, OFF);
					if (pin_st == NEW_SETTING_PIN_433)
						writeHalfWordDWIN(PIN_USE_433_ADDR, OFF);
					for (i = 0; i < 4; i = i + 1)
					{
						pin_arr[i] = 0;
						writeHalfWordDWIN(num_pin_addr[i], 0);
					}
					count_pin = 0;
					pin_st = NON;
				}
				break;
				case (PRECOOL_CMD): // Если пришла команда об запросе охлаждения камина
				{
					__HAL_TIM_CLEAR_FLAG(&htim13, TIM_SR_UIF);
					HAL_TIM_Base_Start_IT(&htim13);							  // Запуск таймера на 1с
					cancel_cool_count = 10;									  // Выставляем счетчик отсчета по 1с
					writeHalfWordDWIN(COUNT_PRECOOL_ADDR, cancel_cool_count); // Записываем на экран значение счетчика (10с по умолчанию)
				}
				break;
				case (CANCEL_COOL_CMD): // Если пришла команда отмены охлажения
				{
					HAL_TIM_Base_Stop_IT(&htim13); // Останов таймера
					cancel_cool_count = 0;		   // Обнуление счетчика
					if (elcamino_st == HEAT)	   // Смотрим статус камина перед охлажением
						goToPageDWIN(HEAT_SCREEN); // Переходим экран статуса
					if (elcamino_st == BURN)
						goToPageDWIN(BURN_SCREEN);
					if (elcamino_st == BURN_AFTER_5)
						goToPageDWIN(BURN_5_SCREEN);
				}
				break;
				case (COOL_CMD): // Если пришла команда охлаждения камина
				{
					elcamino_st = COOLING;		  // Выставляем статус охлаждения
					goToPageDWIN(COOLING_SCREEN); // Переходим на экран охлаждения камина
					HAL_TIM_Base_Stop(&htim13);	  // Останов таймера запроса охлаждения
				}
				break;
				case (LOCK_ELCAMINO_CMD):
				{
					if (((pin_cfg & USE_PIN_433_ON) == USE_PIN_433_ON) && (pin_st == LOCK))
						goToPageDWIN(ENTER_PIN_SCREEN);
					if (((pin_cfg & USE_PIN_433_ON) == USE_PIN_433_ON) && (pin_st == UNLOCK))
					{
						rx433_flag = 0;
						rf_data[2] = 0;
						HAL_TIM_Base_Stop_IT(&htim6);
						pin_st = LOCK;
						writeHalfWordDWIN(PIN_LOCK_ADDR, 1);
					}
					if ((pin_cfg & USE_PIN_ON) == USE_PIN_ON) // Если активен - переход на заблокированный экран
					{
						pin_st = LOCK;
						if ((elcamino_st == STANDBY) || (elcamino_st == WAIT))
							goToPageDWIN(LOCK_STANDBY_SCREEN);
						if (elcamino_st == BURN)
							goToPageDWIN(LOCK_BURN_SCREEN);
						if ((elcamino_st == HEAT) || (elcamino_st == PRE_BURN))
							goToPageDWIN(LOCK_HEAT_SCREEN);
						if (elcamino_st == BURN_AFTER_5)
							goToPageDWIN(LOCK_BURN_5_SCREEN);
						if (elcamino_st == WAIT_WAR)
							goToPageDWIN(LOCK_WARNING_SCREEN);
						writeHalfWordDWIN(PIN_LOCK_ADDR, 1); // �?зменение замочка на закблокированный
					}
					if (pin_cfg == 0)
					{
						goToPageDWIN(FAST_PIN_CHANGE_SCREEN);
					}
				}
				break;
				case (UNLOCK_ELCAMINO_CMD):
				{
					goToPageDWIN(ENTER_PIN_SCREEN); // Переход на экран запроса пинкода
				}
				break;
				case (SHOW_WAR_CMD): // Если пришла команда показать предупреждения камина
				{
					if (warning_st == LOW_FUEL_WAR)		   // Смотрим какое предупреждение было зарегистрировано
						goToPageDWIN(LOW_FUEL_WAR_SCREEN); // Переходим на экран этого прежупреждения
					if (warning_st == LEAK2_WAR)
						goToPageDWIN(LEAK2_WAR_SCREEN);
				}
				break;
				case (CLEAR_PIN_CMD): // Очистка пин-кода
				{
					HAL_TIM_Base_Start_IT(&htim6);
					pincode = 0;						// Сбрасываем пин-код
					pin_cfg = 0;						// Сбрасываем флаги пинкода
					writeHalfWordDWIN(PIN_USE_ADDR, 1); // �?зменяем состояние ползунков на экране
					writeHalfWordDWIN(PIN_USE_433_ADDR, 1);
					writeHalfWordDWIN(PIN_LOCK_ADDR, 2);
					memory[5] = pin_cfg;
					W25qxx_EraseSector(0);					 // Стираем сектор в EEPROM
					W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Записываем буфер в EEPROM
				}
				break;
				case (SLEEP_COUNT_PLUS_CMD):
				{
					if (sleep_time_count <= 300)
						sleep_time_count = sleep_time_count + 60;
					else
						sleep_time_count = 360;
					sleep_time_view = fmod(sleep_time_count, 10.0f);
					if (sleep_time_view != 0)
						writeHalfWordDWIN(SLEEP_TIME_ADDR, (sleep_time_count / 10) + 1);
					else
						writeHalfWordDWIN(SLEEP_TIME_ADDR, (sleep_time_count / 10));
				}
				break;
				case (SLEEP_COUNT_MINUS_CMD):
				{
					if (sleep_time_count > 60)
						sleep_time_count = sleep_time_count - 60;
					else
					{
						if ((sleep_time_st == OFF) || (sleep_time_st == 0))
							sleep_time_count = 0;
					}
					sleep_time_view = fmod(sleep_time_count, 10.0f);
					if (sleep_time_view != 0)
						writeHalfWordDWIN(SLEEP_TIME_ADDR, (sleep_time_count / 10) + 1);
					else
						writeHalfWordDWIN(SLEEP_TIME_ADDR, (sleep_time_count / 10));
				}
				break;
				case (RATATA_CMD): // Заставили убрать хохму ((((
				{
					DF_SetTrack(RATATA);
					play_st = ON; // shalet MUTE_OFF
					writeHalfWordDWIN(MP3_PLAY_ADDR, ON);
					lol_flag = 1;
				}
				break;
					//				case (MIR_CMD):
					//				{
					//					DF_SetTrack(MIR);
					//					writeHalfWordDWIN(MP3_PLAY_ADDR,ON);
					//					lol_flag = 1;
					//				}
					//				break;
				case (GOTO_SLEEP_MENU_CMD):
				{
					goToPageDWIN(SLEEP_MENU_SCREEN);
				}
				break;
				case (GOTO_WIFI_MENU_CMD):
				{
					goToPageDWIN(WIFI_RESET_SCREEN);
				}
				case (RESET_WIFI_CMD):
				{
					ESP32SendServiceCode(ESP32_DEFAULT);
				}
				}
				readDataDWIN.parsingDataDWIN.data[4] = 0;
				readDataDWIN.parsingDataDWIN.data[0] = 0;
			}

			if (elcamino_addr == DISP_SOUND_ADDR) // Если адрес для команд управления звуком тача дисплея
			{
				if (elcamino_cmd == 0x02)				 // Если пришла команда включить звук тача
					buzzerTouchOnDWIN();				 // Отправляем команду на включения звука тача
				else									 // Если иная ( по умолчанию команды могут приходить на такие состояния 01 и 02
					buzzerTouchOffDWIN();				 // Отправляем команду на выключение звука тач
				memory[2] = elcamino_cmd;				 // Сохраняем в буффер для EEPROM
				W25qxx_EraseSector(0);					 // Стираем весь сектор памяти EEPROM
				W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Записываем заного весь буффер памяти в EEPROM
				elcamino_cmd = 0;						 // Обнуляем команду
			}
			if (elcamino_addr == SYS_SOUND_ADDR) // Если адрес для команд управления системным звуком
			{
				if (elcamino_cmd == SYSTEM_SOUND_ON)	 // Если пришла команда на включение звука
					HAL_TIM_Base_Init(&htim8);			 // �?нициализируем таймер пищалки
				else									 // Если команда на отключения звука пищалки
					HAL_TIM_Base_DeInit(&htim8);		 // Деиницализируем таймер пищалки
				memory[3] = elcamino_cmd;				 // Записываем значение в буффер для EEPROM
				W25qxx_EraseSector(0);					 // Стираем сектор памяти EEPROM
				W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Записываем буффер в EEPROM
				elcamino_cmd = 0;						 // Обнуляем команду
			}

			if (elcamino_addr == FLAME_LVL_ADDR) // Если адрес для команд управления уровня пламенем
			{
				if (burn_lvl <= elcamino_cmd) // shalet not present
				{
					ir_delay = 1;
					ir_delay_max = IR_DELAY_BURN;
				}
				else
					ir_delay = 0;
				burn_lvl = elcamino_cmd;				 // Записываем в переменную выставленное значение уровня пламени
				memory[6] = burn_lvl;					 // Записываем значение в буфер для EEPROM
				W25qxx_EraseSector(0);					 // Стираем сектор памяти EEPROm
				W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Записываем буфер в EEPROM
				elcamino_cmd = 0;						 // Обнуляем команду
			}

			if (elcamino_addr == BRIGHT_ADDR) // Если адрес для управления яркостью дисплея
			{
				memory[4] = readDataDWIN.parsingDataDWIN.data[3]; // Записываем значение яркости в буфер EEPROM
				W25qxx_EraseSector(0);							  // Стираем сектор в EEPROM
				W25qxx_WritePage(memory, 0, 0, MEM_ARR);		  // Записываем буфер в EEPROM
				elcamino_addr = 0;								  // Обнуляем команду
			}

			if (elcamino_addr == PIN_USE_ADDR) // Если адрес для управление использованием пинкодом
			{
				if (elcamino_cmd == ON) // Если пришла команда включить пин-код
				{
					if ((pin_cfg & USE_PIN_433_ON) == USE_PIN_433_ON) // Если был включена функция использовать пин на ключ
					{
						pin_cfg |= USE_PIN_ON;					 // То просто включаем функцию использовать пин-код
						memory[5] = pin_cfg;					 // Записываем значение в буфер EEPROM
						W25qxx_EraseSector(0);					 // Стираем сектор памяти EEPROM
						W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Записываем буфер в EEPROM
					}
					else // Если функция в целом не была включена
					{
						goToPageDWIN(ENTER_NEW_PIN_SCREEN); // Переходим на экран установки пинкода
						pin_st = NEW_SETTING_PIN;			// ставим флаг установки пин-кода
					}
				}
				else // Если пришла команда не использовать пин код
				{
					goToPageDWIN(ENTER_PIN_SCREEN); // Переходим экран ввода пин-кода
					pin_st = ENTER_SETTING_PIN;		// Ставим флаг использования пин-кода
				}
			}

			if (elcamino_addr == PIN_USE_433_ADDR) // Если адрес для управления использования пин-кода на пульт
			{
				if (elcamino_cmd == ON) // Если команда на включение пин-кода пульта
				{
					if ((pin_cfg & USE_PIN_ON) == USE_PIN_ON) // Если был включена функция "�?спользование пин-кода"
					{
						pin_cfg |= USE_PIN_433_ON;				 // Ставим флаг на использование пин-кода пульта
						memory[5] = pin_cfg;					 // Записываем значение в буфер EEPROM
						W25qxx_EraseSector(0);					 // Стираем сектор памяти EEPROM
						W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Записываем буфер в EEPROM
					}
					else // Если не была включена функция пин-кода
					{
						goToPageDWIN(ENTER_NEW_PIN_SCREEN); // Переходим на экран ввода нового пин-кода
						pin_st = NEW_SETTING_PIN_433;		// Статус меняется на ввод нового пинкода
					}
				}
				else // Если команда на выключение пин-кода пульта
				{
					goToPageDWIN(ENTER_PIN_SCREEN);
					pin_st = ENTER_SETTING_PIN_433;
				}
			}
			if (elcamino_addr == MP3_PLAY_ADDR)
			{
				if ((elcamino_cmd == ON) && (lol_flag == 0) && ((memory[25] == 2) || (memory[26] == 2) || (memory[27] == 2)))
				{
					play_st = ON;
				}
				else
				{
					if (memory[25] == 2)
						DF_SetTrack(DROVA);
					if (memory[26] == 2)
						DF_SetTrack(FOREST);
					if (memory[27] == 2)
						DF_SetTrack(RAIN);
					DF_Pause(); // выключение музыки
					play_st = OFF;
					MUTE_ON;
					lol_flag = 0;
					writeHalfWordDWIN(MP3_PLAY_ADDR, OFF);
				}
				elcamino_cmd = 0;
			}
			if (elcamino_addr == MP3_VOLUME_ADDR)
			{

				DF_SetVolume(elcamino_cmd);
				memory[24] = elcamino_cmd;				 // Записываем значение в буфер для EEPROM
				W25qxx_EraseSector(0);					 // Стираем сектор памяти EEPROm
				W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Записываем буфер в EEPROM
				elcamino_cmd = 0;
			}
			if (elcamino_addr == DROVA_MP3_ADDR)
			{
				if (elcamino_cmd == ON)
				{
					DF_SetTrack(DROVA); // отправка по уарт играть дрова
					DF_Pause();
					play_st = ON;
					writeHalfWordDWIN(FOREST_MP3_ADDR, OFF);
					memory[26] = OFF;
					writeHalfWordDWIN(RAIN_MP3_ADDR, OFF);
					memory[27] = OFF;
					lol_flag = 0;
				}
				else
				{
					MUTE_ON;
					DF_Pause(); // ставим на паузу
					writeHalfWordDWIN(MP3_PLAY_ADDR, OFF);
				}
				memory[25] = elcamino_cmd;				 // Записываем значение в буфер для EEPROM
				W25qxx_EraseSector(0);					 // Стираем сектор памяти EEPROm
				W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Записываем буфер в EEPROM
				elcamino_cmd = 0;
			}
			if (elcamino_addr == FOREST_MP3_ADDR)
			{
				if (elcamino_cmd == ON)
				{
					DF_SetTrack(FOREST); // отправка по уарт играть дрова
					DF_Pause();
					play_st = ON;
					writeHalfWordDWIN(DROVA_MP3_ADDR, OFF);
					memory[25] = OFF;
					writeHalfWordDWIN(RAIN_MP3_ADDR, OFF);
					memory[27] = OFF;
					lol_flag = 0;
				}
				else
				{
					MUTE_ON;
					DF_Pause(); // ставим на паузу
					writeHalfWordDWIN(MP3_PLAY_ADDR, OFF);
				}
				memory[26] = elcamino_cmd;				 // Записываем значение в буфер для EEPROM
				W25qxx_EraseSector(0);					 // Стираем сектор памяти EEPROm
				W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Записываем буфер в EEPROM
				elcamino_cmd = 0;
			}
			if (elcamino_addr == RAIN_MP3_ADDR)
			{
				if (elcamino_cmd == ON)
				{
					DF_SetTrack(RAIN);
					DF_Pause();
					play_st = ON;
					writeHalfWordDWIN(FOREST_MP3_ADDR, OFF);
					memory[25] = OFF;
					writeHalfWordDWIN(DROVA_MP3_ADDR, OFF);
					memory[26] = OFF;
					lol_flag = 0;
				}
				else
				{
					MUTE_ON;
					DF_Pause(); // ставим на паузу
					writeHalfWordDWIN(MP3_PLAY_ADDR, OFF);
				}
				memory[27] = elcamino_cmd;				 // Записываем значение в буфер для EEPROM
				W25qxx_EraseSector(0);					 // Стираем сектор памяти EEPROm
				W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Записываем буфер в EEPROM
				elcamino_cmd = 0;
			}
			if (elcamino_addr == SLEEP_ON_ADDR)
			{
				if (sleep_time_count != 0)
				{
					if (elcamino_cmd == ON)
						sleep_time_st = ON;
					else
						sleep_time_st = OFF;
				}
				else
					writeHalfWordDWIN(SLEEP_ON_ADDR, OFF);
			}
			if (elcamino_addr != 0x4F4B) // shalet not present
			{
				close_topcap_count = 0;
			}
		}
		//========== Обработка, прием и отправка команд с ESP32====================//
		{
			switch (ESP32Data.cmd)
			{
			case (HEAT_ON_ESP):
			{
				if (elcamino_st == WAIT)
					elcamino_st = PRE_HEAT;
			}
			break;
			case (BURN_LVL):
			{
				if (burn_lvl <= ESP32Data.data) // shalet not present
				{
					ir_delay = 1;
					ir_delay_max = IR_DELAY_BURN;
				}
				burn_lvl = ESP32Data.data;
				writeHalfWordDWIN(FLAME_LVL_ADDR, burn_lvl);
				memory[6] = burn_lvl;  // Записываем значение в буфер для EEPROM
				W25qxx_EraseSector(0); // Стираем сектор памяти EEPROm
				W25qxx_WritePage(memory, 0, 0, MEM_ARR);
				BUZZ_START();
				HAL_Delay(100);
				BUZZ_STOP();
			}
			break;
			case (COOL_ON):
			{
				goToPageDWIN(COOLING_SCREEN);
				elcamino_st = COOLING;
			}
			break;
			case (ASK_ST):
				ESP32SendStatus();
			case (HEX_UPD):
			{
				if (elcamino_st == WAIT)
					ESP32SendServiceCode(HEX_UPD);
			}
			break;
			case (ASK_DEV):
			{
				ESP32SendDeviceInfo(serial_number);
			}
			break;
			}
			ESP32Data.cmd = 0;
		}
		//=========================Обработка ПпН кода============================
		{
			if (elcamino_addr == PIN_ENTER_ADDR)
			{
				readDataDWIN.parsingDataDWIN.data[1] = 0;
				elcamino_addr = 0;

				if ((elcamino_cmd != 0x0F) && (elcamino_cmd != 0x0B) && (count_pin < 4))
				{
					pin_arr[count_pin] = elcamino_cmd;
					writeHalfWordDWIN(num_pin_addr[count_pin], elcamino_cmd);
					count_pin = count_pin + 1;
				}
				if ((elcamino_cmd == 0x0B) && (count_pin > 0))
				{
					count_pin = count_pin - 1;
					pin_arr[count_pin] = 0;
					writeHalfWordDWIN(num_pin_addr[count_pin], 0);
				}
				if ((elcamino_cmd == 0x0F) && (count_pin == 4)) // Если нажата кнопка ок и введены все 4 цифры - сравниваем с пинкодом в памяти
				{
					pincode_temp |= pin_arr[0] << 12 | pin_arr[1] << 8 | pin_arr[2] << 4 | pin_arr[3];
					if (pin_st == NEW_SETTING_PIN) // Если флаг нового пин-кода
					{
						pincode = pincode_temp;				// Записываем в переменную пинкода - введеный пин-код
						pin_cfg |= USE_PIN_ON;				// Ставим флаг использования пин-кода
						writeHalfWordDWIN(PIN_USE_ADDR, 2); // Ставим переклчатель на использование пинкода обычного
						goToPageDWIN(PIN_CHANGE_SCREEN);	// Переход на экран "Пин установлен"
						memory[7] = pincode >> 8;
						memory[8] = pincode;
					}
					if (pin_st == NEW_SETTING_PIN_433)
					{
						pincode = pincode_temp;					// Записываем в переменную пинкода - введеный пин-код
						pin_cfg |= USE_PIN_433_ON;				// Ставим флаг использования пин-кода
						writeHalfWordDWIN(PIN_USE_433_ADDR, 2); // Ставим переклчатель на использование пинкода обычного
						goToPageDWIN(PIN_CHANGE_SCREEN);		// Переход на экран "Пин установлен"
						memory[7] = pincode >> 8;
						memory[8] = pincode;
					}
					if (pin_st == ENTER_SETTING_PIN)
					{
						if ((pincode_temp == pincode) || (pincode_temp == SERVICE_PIN))
						{
							pin_cfg &= ~USE_PIN_ON;
							;									  // Снимаем флаг использования пин-
							writeHalfWordDWIN(PIN_USE_ADDR, OFF); // Снимаем переклчатель на использование пинкода обычного
							goToPageDWIN(PIN_UNLOCK_SCREEN);
						}
						else
						{
							goToPageDWIN(WRONG_PIN_SCREEN); // Переходим на экран "Неверный пинкод"
							writeHalfWordDWIN(PIN_USE_ADDR, ON);
						}
					}
					if (pin_st == ENTER_SETTING_PIN_433)
					{
						if ((pincode_temp == pincode) || (pincode_temp == SERVICE_PIN))
						{
							writeHalfWordDWIN(PIN_LOCK_ADDR, 2); // Отображаем открытый замочек пин-кода
							pin_cfg &= ~USE_PIN_433_ON;
							;										  // Снимаем флаг использования пин-кода пульта
							writeHalfWordDWIN(PIN_USE_433_ADDR, OFF); // Снимаем переклчатель на использование пинкода обычного
							goToPageDWIN(PIN_UNLOCK_SCREEN);
							HAL_TIM_Base_Start_IT(&htim6);
						}
						else
						{
							goToPageDWIN(WRONG_PIN_SCREEN); // Переходим на экран "Неверный пинкод"
							writeHalfWordDWIN(PIN_USE_433_ADDR, ON);
						}
					}
					if ((pin_st != NON) && (pin_st != UNLOCK) && (pin_st != LOCK))
					{
						memory[5] = pin_cfg;					 // Записываем значение в буфер EEPROM
						W25qxx_EraseSector(0);					 // Стираем сектор памяти EEPROM
						W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Записываем буфер в EEPROM
					}
					if ((pin_st == NON) || (pin_st == LOCK))
					{
						if (pincode_temp == SERVICE_PIN)
							goToPageDWIN(PIN_UNLOCK_SCREEN);
						else
						{
							if (pincode_temp == pincode)
								goToPageDWIN(PIN_UNLOCK_SCREEN);
							else
								goToPageDWIN(WRONG_PIN_SCREEN);
						}
					}
					__HAL_TIM_CLEAR_FLAG(&htim7, TIM_SR_UIF);
					HAL_TIM_Base_Start_IT(&htim7); // в таймере проверить верность пинкода, чтобы перейти на нужный экран
				}
			}
		}

		//=================Обработка данных с АЦП и концевиков===================//
		if ((adc_flag == 1) || (elcamino_st == STANDBY))
		{
			if (fuel_st == 1)
			{
				VL53L0X_GetRangingMeasurementData(Dev, &RangingData);
				if (elcamino_st == STANDBY)
					filter_fuel = RangingData.RangeMilliMeter;
				fuel_st = 0;
				// ---------------------------------------------------------------------
				if ((elcamino_st == BURN) || (elcamino_st == BURN_AFTER_5))

					k_filter_fuel = kf_fuel_burning;

				else if (elcamino_st == FUELING)
					k_filter_fuel = kf_fuel_fueling;

				else
					k_filter_fuel = kf_fuel_std;
				// ---------------------------------------------------------------------
				filter_fuel += (RangingData.RangeMilliMeter - filter_fuel) * k_filter_fuel; // shalet hardcode 0.05f
				VL53L0X_ClearInterruptMask(Dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
			}

			for (int j = 0; j < 6; j += 1)
			{
				adc_f[j] = (adc[j] * 100) / 4095;
				if (elcamino_st == STANDBY)
					filter_adc_f[j] = adc_f[j];
				filter_adc_f[j] += (adc_f[j] - filter_adc_f[j]) * K_FILT;
			}
			for (int k = 6; k < 13; k++)
			{
				adc_f[k] = (calc_temperature((adc[k]))) * 0.1;
				if (elcamino_st == STANDBY)
					filter_adc_f[k] = adc_f[k];
				filter_adc_f[k] += (adc_f[k] - filter_adc_f[k]) * K_FILT_TEMP;
			}
			adc_f[MOTOR] = (calc_temperature((adc[MOTOR]))) * 0.1;
			filt_temp += (adc_f[MOTOR] - filt_temp) * K_FILT_CAP;
			// if (fabs(filt_temp = filter_adc_f[MOTOR]) < CAP_DX_DT_MAX)
			filter_adc_f[MOTOR] = filt_temp;

			// вычисление максимальной температуры испарителей

			max_vapor_T = filter_adc_f[T1];
			for (i = T1; i <= max_vapor_index; i++)
			{
				if (filter_adc_f[i] > max_vapor_T)
					max_vapor_T = filter_adc_f[i];
			}
			// max_vapor_T = test_max_vapor_T;

			adc_flag = 0;
		}

		//================================Обработка предупреждений=================================//
		if (service_st == 0)
		{

			if (filter_adc_f[LEAK1] < LEAK1_LVL_WAR) // shalet LEAK1																							// Если топливо больше нужного и оно перелилось на датчик протечки
			{
				warning_st |= LEAK1_WAR; // То ставим флаг перелива топлива
			}

			if (filter_adc_f[LEAK1] > LEAK1_LVL_NON)
			{
				warning_st &= ~LEAK1_WAR; // Если датчик высох - убираем флаг
			}

			if (filter_adc_f[LEAK2] < LEAK2_LVL_WAR) // shalet LEAK1																							// Если топливо больше нужного и оно перелилось на датчик протечки
			{
				warning_st |= LEAK2_WAR; // То ставим флаг перелива топлива
			}

			if (filter_adc_f[LEAK2] > LEAK2_LVL_NON)
			{
				warning_st &= ~LEAK2_WAR; // Если датчик высох - убираем флаг
			}

			if (filter_fuel >= FUEL_WARN) // if (inp_fuel < 1)
			{
				warning_st |= LOW_FUEL_WAR;
			}
			else if (inp_fuel > 1)
			{
				warning_st &= ~LOW_FUEL_WAR;
			}
			//=================================Обработка ошибок=======================================//
			/*if ((HAL_GPIO_ReadPin(GPIOD, CASE_ST_Pin) == 1) || ((fault_st & (OPEN_ELCAMINO)) == OPEN_ELCAMINO)) // shalet not present
			{
				fault_st |= OPEN_ELCAMINO; // РАБОТАЕТ
				goToPageDWIN(OPEN_ELCAMINO_SCREEN);
				memory[1] = fault_st;
				W25qxx_EraseSector(0);
				W25qxx_WritePage(memory, 0, 0, MEM_ARR);
			}*/
			if (fault_st == 0) // Это условие для того, чтобы не прыгать по экранам, когда есть ошибка
			{
				if (filter_adc_f[T_IN] < T_LOW_LVL_WAR) // Если температура внутри камина < 10 граудсов
				{
					fault_st |= LOW_T_WAR;			// Ставим флаг ошибки по минимальной температуре внутри камниа
					goToPageDWIN(LOW_T_WAR_SCREEN); // Переходим на экран ошибки по минимальной температуре внутри камниа
				}
				if ((filter_adc_f[T_IN] > T_HI_LVL_WAR) || (max_vapor_T > T_LVL_ERR)) // Если внутри камина сильно горячо >60 градусов или испаритель перегрелся
				{
					fault_st |= HI_T_WAR;		   // То ставим флаг ошибки по превышению температуры внутри камина
					goToPageDWIN(HI_T_WAR_SCREEN); // Переходим на экран ошибки по температуре внутри камина
				}
				if ((filter_adc_f[T_IN] > HI_T_LVL_ERR) || (filter_adc_f[T_IN] < -LOW_T_LVL_ERR) || (filter_adc_f[T1] > HI_T_LVL_ERR) || (filter_adc_f[T1] < -LOW_T_LVL_ERR))
				{
					fault_st |= TSENS_ERR; // работает
					goToPageDWIN(TSENS_ERR_SCREEN);
				}
				if (((filter_adc_f[T2] > HI_T_LVL_ERR) || (filter_adc_f[T2] < -LOW_T_LVL_ERR)) && (elcamino_lenght >= 10))
				{
					fault_st |= TSENS_ERR;
					goToPageDWIN(TSENS_ERR_SCREEN);
				}
				if (((filter_adc_f[T3] > HI_T_LVL_ERR) || (filter_adc_f[T3] < -LOW_T_LVL_ERR)) && (elcamino_lenght >= 15))
				{
					fault_st |= TSENS_ERR;
					goToPageDWIN(TSENS_ERR_SCREEN);
				}
				// shalet not present
				if ((elcamino_lenght >= 20) && ((filter_adc_f[T4] > HI_T_LVL_ERR) || (filter_adc_f[T4] < -LOW_T_LVL_ERR)))
				{
					fault_st |= TSENS_ERR;
					goToPageDWIN(TSENS_ERR_SCREEN);
				}
				// shalet not present
				if (((filter_adc_f[T5] > HI_T_LVL_ERR) || (filter_adc_f[T5] < -LOW_LVL_ERR)) && (elcamino_lenght >= 25))
				{
					fault_st |= TSENS_ERR;
					goToPageDWIN(TSENS_ERR_SCREEN);
				}
				// shalet not present
				if ((elcamino_lenght == 30) && ((filter_adc_f[T6] > HI_T_LVL_ERR) || (filter_adc_f[T6] < -LOW_T_LVL_ERR)))
				{
					fault_st |= TSENS_ERR;
					goToPageDWIN(TSENS_ERR_SCREEN);
				}
				if (plug_tim_flag == PLUG_FAULT_CNT) // shalet &&(filter_adc_f[IR1] > IR_LVL_BURN))
				{
					plug_tim_flag = 0;
					fault_st |= PLUG_ERR;
					goToPageDWIN(PLUG_ERR_SCREEN);
					PLUG_STOP();
					TIMEOUT_PLUG_STOP;
				}
				if ((elcamino_st == PRE_BURN) && (((filter_adc_f[T1] < filter_adc_f[T_IN]) && (elcamino_lenght >= 5)) || ((filter_adc_f[T2] < filter_adc_f[T_IN]) && (elcamino_lenght >= 10)) ||
												  ((filter_adc_f[T3] < filter_adc_f[T_IN]) && (elcamino_lenght >= 15)) || ((filter_adc_f[T4] < filter_adc_f[T_IN]) && (elcamino_lenght >= 20)) || ((filter_adc_f[T5] < filter_adc_f[T_IN]) && (elcamino_lenght >= 25)) ||
												  ((filter_adc_f[T6] < filter_adc_f[T_IN]) && (elcamino_lenght >= 30))))
				{
					fault_st |= HE_ERR;
					goToPageDWIN(HE_ERR_SCREEN);
				}
			}
		}

		//============================Работа камина ==========================//
		{
			// Логика работы моторчика крышки //shalet not present
			if ((elcamino_st != FUELING))
			{
				if ((HAL_GPIO_ReadPin(GPIOD, IRCAP_ST_Pin) == GPIO_PIN_RESET) && (hand_cap_ctrl == 1))
				{
					ir_count += 1;
				}
				if ((HAL_GPIO_ReadPin(GPIOD, IRCAP_ST_Pin) == GPIO_PIN_SET))
				{
					if (max_ir < ir_count)
						max_ir = ir_count;
					ir_count = 0;
				}
				if (elcamino_st == BURN)
				{
					if ((ir_delay == 0) || (ir_delay == ir_delay_max))
					{
						ir_delay_count = ir_burn_count[burn_lvl - 1];
						ir_delay = 0;
					}
				}

				if ((((ir_count == IR_COUNT_DEF) && (elcamino_st != BURN) && (elcamino_st != PRE_BURN) && (elcamino_st != WAIT_COOL)) ||
					 ((elcamino_st == BURN) && (ir_count == ir_delay_count)) ||
					 ((elcamino_st == WAIT_COOL) && (ir_count == IR_COUNT_COOL)) ||
					 ((elcamino_st == PRE_BURN) && (ir_count == IR_COUNT_PREBURN)) || cmd_cap_remote) && // задержка датчика руки при розжиге (по умолчанию 200000)
					(motor_flag == 0))
				{
					motor_flag = 1;
					cmd_cap_remote = CMD_CAP_STOP;
					MOTOR_EN;
					if (cover_open_close)
					{
						DIR_BACKWARD;
						HAL_Delay(10);
						MOTOR_START;
					}
					else
					{
						/*if (adc_f[MOTOR] >= motor_sens_close)
						{*/
						DIR_FORWARD;
						HAL_Delay(10);
						MOTOR_START;
						close_topcap_count = 0;
						//	}
					}
				}
				if ((close_topcap_count == 40) && (fail_topcap_count != 12))
				{
					DIR_BACKWARD;
					HAL_Delay(10);
					MOTOR_START;
					// motor_flag = 1;
				}
				if (fail_topcap_count == FAIL_TOPCAP_MAX)
				{

					DIR_FORWARD;
					HAL_Delay(10);
					MOTOR_START;
					// motor_flag = 1;
					close_topcap_count = 0;
					fail_topcap_count = 0;
				}
				if ((HAL_GPIO_ReadPin(GPIOA, TMC_DIR_Pin) == GPIO_PIN_RESET) &&
					(filter_adc_f[MOTOR] >= motor_sens_close))
				{
					motor_flag = 0;
					fail_topcap_count = 0;
					close_topcap_count = 0;
					cover_open_close = 0;
					MOTOR_STOP;
					MOTOR_DIS;
				}
				if ((HAL_GPIO_ReadPin(GPIOA, TMC_DIR_Pin) == GPIO_PIN_SET) &&
					(filter_adc_f[MOTOR] <= motor_sens_open))
				{
					motor_flag = 0;
					fail_topcap_count = 0;
					cover_open_close = 1;

					MOTOR_STOP;
				}
			}
			else
			{
				cmd_cap_remote = CMD_CAP_STOP;
			}

			if ((filter_adc_f[MOTOR] <= motor_sens_open) && (bright_st == 1)) // Если крышка экрана открытой и был флаг, что она закрывалась
			{
				bright_st = 0;		   // Ставим флаг окрытой крышка дисплея камина
				setLedDWIN(memory[4]); // Посылаем команду на дисплей о выставлении яркости с дисплея
			}

			if ((filter_adc_f[MOTOR] >= motor_sens_close) && (bright_st == 0)) // Если крышка дисплея закрыта и и был флаг о том, что крышка была открыта
			{
				bright_st = 1; // Выставляем флаг закрытой крышки
				setLedDWIN(0); // Убираем ярккость в ноль
			}

			if (elcamino_st == WARNING) // Если статус камина "Предупреждение"
			{
				if (warning_st == LOW_FUEL_WAR)							  // Если предупреждение о низком уровне топлива
					goToPageDWIN(LOW_FUEL_WAR_SCREEN);					  // Переход на экран предупреждения о низком уровне топлива
				if (warning_st == (LEAK1_WAR | LEAK2_WAR))				  // Если предупреждение о переливе топлива
					goToPageDWIN(LEAK2_WAR_SCREEN);						  // Переход на экран предупреждения о переливе
				if (warning_st == LEAK1_WAR)							  // Если предупреждение о переливе топлива
					goToPageDWIN(LEAK2_WAR_SCREEN);						  // Переход на экран предупреждения о переливе
				if (warning_st == (LOW_FUEL_WAR | LEAK2_WAR | LEAK1_WAR)) // Если есть оба предупреждения
					goToPageDWIN(LOW_FUEL_WAR_SCREEN);					  // То выдаеся предупреждение о низком уровне топлива
				elcamino_st = WAIT_WAR;									  // Статус камина меняется на ожидание в предупреждении
				ESP32SendWarningErrorCode(WAR, warning_st);
				ESP32Data.status = WAR_ST;
			}

			if ((elcamino_st == WAIT_WAR) && (warning_st == 0)) // Если статус камина ожидание в предупреждении и нет флагов предупреждения
			{
				elcamino_st = STANDBY; // То статус камина - готовность
				ESP32Data.status = STANDBY_ST;
				ESP32SendStatus();
			}

			if ((max_vapor_T /*adc_f[T1]*/ > T_STARTUP) && (elcamino_st == STANDBY))
			{
				elcamino_st = COOLING;
				goToPageDWIN(COOLING_SCREEN);
				/*shalet
				writeHalfWordDWIN(HEAT_COOL_ADDR,heat_cool_scale);
				writeHalfWordDWIN(HEAT_COOL_TIME_ADDR,9);*/
				ESP32Data.status = COOL_ST_ON;
			}

			if ((filter_fuel < MAX_FUEL) && (elcamino_st == FUELING))
			{
				// k_filter_fuel = 0.05f; // shalet not present
				PUMP_STOP;
				inp_fuel = 20;
				goToPageDWIN(END_FUEL_SCREEN);
			}

			if ((elcamino_st == STANDBY) && (warning_st == 0) && (fault_st == 0)) // Если камин готов к работе и нет никаких  предупреждений и ошибок
			{
				goToPageDWIN(STANDBY_SCREEN);
				FAN_STOP;
				PUMP_STOP;
				if ((pin_cfg & USE_PIN_ON) == USE_PIN_ON)
				{
					goToPageDWIN(LOCK_STANDBY_SCREEN);
					pin_st = LOCK;
					writeHalfWordDWIN(PIN_LOCK_ADDR, 1);
				}
				if ((pin_cfg & USE_PIN_433_ON) == USE_PIN_433_ON)
				{
					pin_st = LOCK;
					rx433_flag = 0;
					rf_data[2] = 0;
					HAL_TIM_Base_Stop_IT(&htim6);
					writeHalfWordDWIN(PIN_LOCK_ADDR, 1);
				}
				elcamino_st = WAIT;
				ESP32Data.status = STANDBY_ST;
			}

			if (elcamino_st == PRE_HEAT)
			{
				elcamino_st = HEAT; // �?зменяем статус камина на "�?дет нагрев"
				ESP32Data.status = HEAT_ST_ON;
				ESP32SendStatus();
				//				FAN_TIM = 30000;
				//				FAN_START;

				HE1_TIM = PreHeatDuty;
				HE2_TIM = PreHeatDuty;
				HE3_TIM = PreHeatDuty;
				HE4_TIM = PreHeatDuty;
				HE5_TIM = PreHeatDuty;
				HE6_TIM = PreHeatDuty;

				HE1_START;
				if (elcamino_lenght >= 10)
					HE2_START;
				if (elcamino_lenght >= 15)
					HE3_START;
				if (elcamino_lenght >= 20)
					HE4_START;
				if (elcamino_lenght >= 25)
					HE5_START;
				if (elcamino_lenght == 30)
					HE6_START;
				if ((pin_st == UNLOCK) || (pin_st == NON))
					goToPageDWIN(HEAT_SCREEN);
				if (pin_st == LOCK)
					goToPageDWIN(LOCK_HEAT_SCREEN);
				BUZZ_START(); // Запускаем пищалку для обощначения команды
				HAL_Delay(500);
				BUZZ_STOP();
				heat_cool_scale = 0;

				writeHalfWordDWIN(HEAT_COOL_ADDR, heat_cool_scale);
				writeHalfWordDWIN(HEAT_COOL_TIME_ADDR_H, 10);
				writeHalfWordDWIN(HEAT_COOL_TIME_ADDR_L, 5);
			}

			if (elcamino_st == HEAT)
			{

				for (uint8_t i = 0; i < 5; i++)
				{
					if ((max_vapor_T > heat_state[i][0]) &&
						(max_vapor_T < heat_state[i + 1][0]) &&
						((heat_cool_scale == 0) || (heat_cool_scale == i)))
					{
						heat_cool_scale = i + 1;
						writeHalfWordDWIN(HEAT_COOL_ADDR, heat_cool_scale);
						writeHalfWordDWIN(HEAT_COOL_TIME_ADDR_H, heat_state[i][1]);
						writeHalfWordDWIN(HEAT_COOL_TIME_ADDR_L, heat_state[i][2]);
						break;
					}
				}

				ESP32Data.heat_cool_st = heat_cool_scale;
				/*shalet
								if (filter_adc_f[T1] > t_lvl_start_low)
				{
					HE2_START;
					if (chalet_zone >= 2)
						HE4_START;
					if (chalet_zone == 3)
						HE6_START;
					HE2_TIM = 20000;
					HE4_TIM = 20000;
					HE6_TIM = 20000;
				}*/
				if (max_vapor_T /*filter_adc_f[T1]*/ >= t_lvl_start) // shalet &&(filter_adc_f[T2] >= t_lvl_start)
				{
					// shalet present	FAN_TIM = 30000;
					// shalet present	FAN_START;

					HE1_TIM = (uint16_t)burn_pwm[1];
					HE2_TIM = (uint16_t)burn_pwm[1];
					HE3_TIM = (uint16_t)burn_pwm[1];
					HE4_TIM = (uint16_t)burn_pwm[1];
					HE5_TIM = (uint16_t)burn_pwm[1];
					HE6_TIM = (uint16_t)burn_pwm[1];

					elcamino_st = PRE_BURN;
					BUZZ_START();
					HAL_Delay(300);
					BUZZ_STOP();
					PLUG_START();
				}
			}

			if (elcamino_st == PRE_BURN)
			{

				if (PLUG_TIM > 20000)
					TIMEOUT_PLUG_START();
				if ((plug_tim_flag < PLUG_FAULT_CNT) && ((filter_adc_f[IR1] <= IR_LVL_BURN)))
				{
					ir_delay_count = IR_COUNT_FLAME_DETECTED; // shalet not present задержка датчика руки после сработки датчика пламени
					ir_delay = 1;							  // shalet not present
					ir_delay_max = IR_DELAY_PREBURN;
					TIMEOUT_PLUG_STOP; // shalet not present
					PLUG_STOP();
					plug_tim_flag = 0;
					HAL_TIM_Base_Stop_IT(&htim14); // shalet not present
					FAN_TIM = FAN_DUTY_BURN;	   // 15000; 10%
					FAN_START;					   // shalet not present
					elcamino_st = BURN;
					ESP32Data.status = BURN_ST_ON;
					ESP32SendStatus();
					if ((pin_st == UNLOCK) || (pin_st == NON))
						goToPageDWIN(BURN_SCREEN);
					if (pin_st == LOCK)
						goToPageDWIN(LOCK_BURN_SCREEN);
					BUZZ_START(); // shalet not present
					HAL_Delay(300);
					BUZZ_STOP();
				}
			}

			// таймер сна
			if ((elcamino_st == BURN) || (elcamino_st == BURN_AFTER_5)) // Если камин горит
			{
				if (sleep_time_st == ON) // Был включен таймер сна
				{
					__HAL_TIM_CLEAR_FLAG(&htim10, TIM_SR_UIF);
					HAL_TIM_Base_Start_IT(&htim10); // Запускаем таймер на отсчет
					sleep_time_st = 3;				// Обнуляем статус
				}
				if (sleep_time_st == OFF)
				{								   // Если выключен таймер сна или его выключили
					HAL_TIM_Base_Stop_IT(&htim10); // Останавливаем таймер на отсчет
					sleep_time_st = 0;			   // Обнуляем статус
					sleep_time_count = 0;
					writeHalfWordDWIN(SLEEP_TIME_ADDR, 0);
					writeHalfWordDWIN(SLEEP_ON_ADDR, 1);
				}
				if (play_st == ON)
				{
					MUTE_OFF;
					DF_Playback();
					DF_Repeat();
					writeHalfWordDWIN(MP3_PLAY_ADDR, ON);
					play_st = OFF;
				}
			}
			// PID регулирование //shalet present

			if ((elcamino_st == BURN) || (elcamino_st == BURN_AFTER_5))
			{ // shalet
				HE1_TIM = (uint16_t)burn_pwm[burn_lvl - 1];
				HE2_TIM = (uint16_t)burn_pwm[burn_lvl - 1];
				HE3_TIM = (uint16_t)burn_pwm[burn_lvl - 1];
				HE4_TIM = (uint16_t)burn_pwm[burn_lvl - 1];
				HE5_TIM = (uint16_t)burn_pwm[burn_lvl - 1];
				HE6_TIM = (uint16_t)burn_pwm[burn_lvl - 1];
				ESP32Data.burn_lvl = burn_lvl;
			}

			// Отмена режимов из за ошибок, либо по запросу пользователя
			{
				if ((sleep_time_count == 0) && (sleep_time_st == 3))
				{
					elcamino_st = COOLING;
					goToPageDWIN(COOLING_SCREEN);
				}

				if ((filter_fuel > FUEL_STOP) && ((elcamino_st == HEAT) ||
												  (elcamino_st == BURN) || (elcamino_st == PRE_BURN)))
				{
					elcamino_st = COOLING;
					goToPageDWIN(COOLING_FUEL_SCREEN);
					// shalet writeHalfWordDWIN(HEAT_COOL_TIME_ADDR,10);
				}

				if ((fault_st != 0) && ((elcamino_st == HEAT) || (elcamino_st == BURN) || (elcamino_st == PRE_BURN)))
				{
					ESP32SendWarningErrorCode(ERR, fault_st);
					elcamino_st = COOLING;
					ESP32Data.status = ERR_ST;
				}

				if (elcamino_st == COOLING)
				{
					BUZZ_START();	// shalet not present
					HAL_Delay(300); // shalet not present
					BUZZ_STOP();	// shalet not present
					heat_cool_scale = 0;
					cool_scale = 0;
					for (uint8_t i = 0; i < 10; i++)
					{
						if (max_vapor_T > waitcool_state[i][0])
						{
							heat_cool_scale = i + 1;
						}
					}
					cool_scale = ((heat_cool_scale / 2) >= 1) ? (heat_cool_scale / 2) : 1;
					writeHalfWordDWIN(HEAT_COOL_TIME_ADDR_H, waitcool_state[heat_cool_scale - 1][1]);
					writeHalfWordDWIN(HEAT_COOL_TIME_ADDR_L, waitcool_state[heat_cool_scale - 1][2]);
					writeHalfWordDWIN(HEAT_COOL_ADDR, cool_scale);

					ESP32Data.heat_cool_st = cool_scale;
					ESP32Data.status = COOL_ST_ON;
					ESP32SendStatus();
					HAL_TIM_Base_Stop_IT(&htim10);
					DF_Pause();
					writeHalfWordDWIN(MP3_PLAY_ADDR, OFF);
					HE1_STOP;
					HE2_STOP;
					HE3_STOP;
					HE4_STOP;
					HE5_STOP;
					HE6_STOP;
					PLUG_STOP();
					FAN_TIM = 200; // 30000;
					FAN_START;
					plug_tim_flag = 0;
					sleep_time_st = 0; // Обнуляем статус
					sleep_time_count = 0;
					writeHalfWordDWIN(SLEEP_TIME_ADDR, 0);
					writeHalfWordDWIN(SLEEP_ON_ADDR, 1);
					elcamino_st = WAIT_COOL;
					MUTE_ON;	  // shalet not present
					ir_delay = 1; // shalet not present
					ir_delay_max = IR_DELAY_BURN;
					//				ir_delay_count = 200;
				}
				if (elcamino_st == WAIT_COOL)
				{ // shalet T2

					for (uint8_t i = 0; i < 10; i++)
					{
						if ((max_vapor_T > cool_state[i][0]) &&
							(max_vapor_T < cool_state[i + 1][0]) &&
							(heat_cool_scale == (i + 1)))
						{
							heat_cool_scale = i;
							cool_scale = ((heat_cool_scale / 2) >= 1) ? (heat_cool_scale / 2) : 1;
							writeHalfWordDWIN(HEAT_COOL_ADDR, cool_scale);
							writeHalfWordDWIN(HEAT_COOL_TIME_ADDR_H, cool_state[i][1]);
							writeHalfWordDWIN(HEAT_COOL_TIME_ADDR_L, cool_state[i][2]);
							break;
						}
					}

					ESP32Data.heat_cool_st = cool_scale;
					ESP32SendStatus();
					if (max_vapor_T < T_STARTUP)
					{
						if (fault_st == 0)
						{
							plug_tim_flag = 0; // shalet not present
							BUZZ_START();
							HAL_Delay(300);
							BUZZ_STOP();
							FAN_STOP;
							elcamino_st = WAIT;
							ESP32Data.status = STANDBY_ST;
							if (((pin_cfg & USE_PIN_ON) == USE_PIN_ON) && (pin_st == NON))
							{
								goToPageDWIN(LOCK_STANDBY_SCREEN);
								pin_st = LOCK;
								writeHalfWordDWIN(PIN_LOCK_ADDR, 1);
							}
							if (((pin_cfg & USE_PIN_433_ON) == USE_PIN_433_ON) && (pin_st == NON))
							{
								pin_st = LOCK;
								rx433_flag = 0;
								rf_data[2] = 0;
								HAL_TIM_Base_Stop_IT(&htim6);
								writeHalfWordDWIN(PIN_LOCK_ADDR, 1);
							}
							if ((pin_st == UNLOCK) || (pin_st == NON))
								goToPageDWIN(STANDBY_SCREEN);
							if (pin_st == LOCK)
								goToPageDWIN(LOCK_STANDBY_SCREEN);
						}
						if (fault_st != 0)
							FAN_STOP;
					}
				}
				if ((warning_st != 0) && (elcamino_st != WAIT_WAR) && (elcamino_st != FUELING) && (elcamino_st != COOLING) && (elcamino_st != WAIT_COOL)) //&&((elcamino_st == STANDBY)||(elcamino_st == WAIT)))					// Обработка предупреждений с выходом на экран. Работает
					elcamino_st = WARNING;
			}
		}

		//=========================Обработка команд с пульта=====================//
		if (rx433_flag == 1)
		{
			rx433_flag = 0;
			Receive_433();
			__HAL_TIM_CLEAR_FLAG(&htim6, TIM_SR_UIF);
			HAL_TIM_Base_Start_IT(&htim6);

			rf_data_svd[0] = rf_data[0];
			rf_data_svd[1] = rf_data[1];
			rf_data_svd[2] = rf_data[2];

			if (rx_int_flag)
			{

				switch (rf_data[2] & 0x0F)
				{

				case CNTR_KEY:
					nokey_time = 0;
					if (!pause_after_long)
					{
						if (++ctrl_key_time >= LONG_KEY_TIME)
						{
							pause_after_long = PAUSE_AFTER_LONG;
							if (elcamino_st == WAIT)
								elcamino_st = PRE_HEAT;
							else if ((elcamino_st == HEAT) || (elcamino_st == BURN))
							{
								BUZZ_START(); // Запускаем пищалку для обощначения команды
								HAL_Delay(100);
								BUZZ_STOP();
								elcamino_st = COOLING;
								goToPageDWIN(COOLING_SCREEN);
							}
							ctrl_key_time = 0;
						}
					}

					break;

				case UP_KEY:
					nokey_time = 0;
					if (++up_key_time > LONG_KEY_TIME)
					{
						pause_after_long = PAUSE_AFTER_LONG;
						cmd_cap_remote = CMD_CAP_OPEN_CLOSE;
						BUZZ_START(); // Запускаем пищалку для обощначения команды
						HAL_Delay(100);
						BUZZ_STOP();
						up_key_time = 0;
					}
					break;

				case DOWN_KEY:
					nokey_time = 0;
					if (++down_key_time > LONG_KEY_TIME)
					{
						pause_after_long = PAUSE_AFTER_LONG;
						BUZZ_START(); // Запускаем пищалку для обощначения команды
						HAL_Delay(100);
						BUZZ_STOP();
						hand_cap_ctrl ^= 1;
						down_key_time = 0;
					}
					break;

				default:
					if (!pause_after_long)
					{
						if (++nokey_time > NO_KEY_TIME)
						{
							ctrl_key_time = 0;
							if (up_key_time >= SHORT_KEY_TIME)
							{

								if (burn_lvl < 5)
								{
									ir_delay = 0; // shalet not present
									max_ir = 0;	  // shalet not present

									burn_lvl++;

									ESP32Data.burn_lvl = burn_lvl;
									writeHalfWordDWIN(FLAME_LVL_ADDR, burn_lvl);
									memory[6] = burn_lvl;  // Записываем значение в буфер для EEPROM
									W25qxx_EraseSector(0); // Стираем сектор памяти EEPROm
									W25qxx_WritePage(memory, 0, 0, MEM_ARR);
								}
								BUZZ_START(); // Запускаем пищалку для обощначения команды
								HAL_Delay(100);
								BUZZ_STOP();
								up_key_time = 0;
							}
							if (down_key_time >= SHORT_KEY_TIME)
							{
								if (burn_lvl > 1)
								{
									ir_delay = 1; // shalet not present
									ir_delay_max = IR_DELAY_BURN;
									max_ir = 0; // shalet not present
									burn_lvl--;
									ESP32Data.burn_lvl = burn_lvl;
									writeHalfWordDWIN(FLAME_LVL_ADDR, burn_lvl);
									memory[6] = burn_lvl;  // Записываем значение в буфер для EEPROM
									W25qxx_EraseSector(0); // Стираем сектор памяти EEPROm
									W25qxx_WritePage(memory, 0, 0, MEM_ARR);
								}
								down_key_time = 0;
								BUZZ_START(); // Запускаем пищалку для обощначения команды
								HAL_Delay(100);
								BUZZ_STOP();
							}
							nokey_time = 0;
						}
					}
				}
				if (pause_after_long)
					pause_after_long--;
				rx_int_flag = 0;
				rf_data[2] = 0;
			}
		}
		//========================Обработка сервисной программы==================//
		if (service_st == 1) // Включен сервисный режим
		{
			close_topcap_count = 0;		// shalet not present
			switch (rc_service_data[0]) // Если пришел запрос о настройке длинны камина
			{
			case HALF_M:
			{
				elcamino_lenght = 5;				  // Настраиваем длину в 2 метра
				W25qxx_ReadBytes(memory, 0, MEM_ARR); // считывание значений настроек из памяти
				W25qxx_EraseSector(0);				  // Очистка памяти
				memory[0] = elcamino_lenght;
				W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Запись конфига обратно
				tx_service_data = ANS_HALF_M;
				HAL_UART_Transmit(&huart1, &tx_service_data, 1, 10);
				rc_service_data[0] = 0;
			}
			break;
			case ONE_M:
			{
				elcamino_lenght = 10;				  // Настраиваем  длину в 1 метр
				W25qxx_ReadBytes(memory, 0, MEM_ARR); // считывание значений настроек из памяти
				W25qxx_EraseSector(0);				  // Очистка памяти
				memory[0] = elcamino_lenght;
				W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Запись конфига обратно
				tx_service_data = ANS_ONE_M;
				HAL_UART_Transmit(&huart1, &tx_service_data, 1, 10);
				rc_service_data[0] = 0;
				// запись во FLASH значения
			}
			break;
			case ONEHALF_M:
			{
				elcamino_lenght = 15;				  // Настраиваем длину в 1.5 метра
				W25qxx_ReadBytes(memory, 0, MEM_ARR); // считывание значений настроек из памяти
				W25qxx_EraseSector(0);				  // Очистка памяти
				memory[0] = elcamino_lenght;
				W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Запись конфига обратно
				tx_service_data = ANS_ONEHALF_M;
				HAL_UART_Transmit(&huart1, &tx_service_data, 1, 10);
				rc_service_data[0] = 0;
			}
			break;
			case TWO_M:
			{
				elcamino_lenght = 20;				  // Настраиваем длину в 2 метра
				W25qxx_ReadBytes(memory, 0, MEM_ARR); // считывание значений настроек из памяти
				W25qxx_EraseSector(0);				  // Очистка памяти
				memory[0] = elcamino_lenght;
				W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Запись конфига обратно
				tx_service_data = ANS_TWO_M;
				HAL_UART_Transmit(&huart1, &tx_service_data, 1, 10);
				rc_service_data[0] = 0;
				// запись во FLASH значения
			}
			break;
			case TWOHALF_M:
			{
				elcamino_lenght = 25;				  // Настраиваем длину в 2 метра
				W25qxx_ReadBytes(memory, 0, MEM_ARR); // считывание значений настроек из памяти
				W25qxx_EraseSector(0);				  // Очистка памяти
				memory[0] = elcamino_lenght;
				W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Запись конфига обратно
				tx_service_data = ANS_TWOHALF_M;
				HAL_UART_Transmit(&huart1, &tx_service_data, 1, 10);
				rc_service_data[0] = 0;
			}
			break;

			case THREE_M:
			{
				elcamino_lenght = 30;				  // Настраиваем длину в 3 метра
				W25qxx_ReadBytes(memory, 0, MEM_ARR); // считывание значений настроек из памяти
				W25qxx_EraseSector(0);				  // Очистка памяти
				memory[0] = elcamino_lenght;
				W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Запись конфига обратно
				tx_service_data = ANS_TH_M;
				HAL_UART_Transmit(&huart1, &tx_service_data, 1, 10);
				rc_service_data[0] = 0;
				// запись во FLASH значения
			}
			break;
			}

			if (rc_service_data[0] == WR_LVL_1)
			{
				rc_service_data[0] = 0;
				burn_pwm[0] = rc_service_data[1] * 100;
			}
			if (rc_service_data[0] == WR_LVL_2)
			{
				rc_service_data[0] = 0;
				burn_pwm[1] = rc_service_data[1] * 100;
			}
			if (rc_service_data[0] == WR_LVL_3)
			{
				rc_service_data[0] = 0;
				burn_pwm[2] = rc_service_data[1] * 100;
			}
			if (rc_service_data[0] == WR_LVL_4)
			{
				rc_service_data[0] = 0;
				burn_pwm[3] = rc_service_data[1] * 100;
			}
			if (rc_service_data[0] == WR_LVL_5)
			{
				rc_service_data[0] = 0;
				burn_pwm[4] = rc_service_data[1] * 100;
			}
			if (rc_service_data[0] == WR_LOW_T)
			{
				rc_service_data[0] = 0;
				t_lvl_low = rc_service_data[1];
			}
			if (rc_service_data[0] == WR_START_LOW_T)
			{
				rc_service_data[0] = 0;
				t_lvl_start_low = rc_service_data[1];
			}
			if (rc_service_data[0] == WR_START_HEAT_T)
			{
				rc_service_data[0] = 0;
				t_lvl_start = rc_service_data[1];
			}
			if (rc_service_data[0] == ASK_TEMP_LVL) // shalet not present
			{
				rc_service_data[0] = 0;
				for (int l = 0; l < 5; l = l + 1)
				{
					ask_temp[l + 1] = (uint8_t)(burn_pwm[l] / 100);
				}
				ask_temp[6] = t_lvl_low;
				ask_temp[7] = t_lvl_start_low;
				ask_temp[8] = t_lvl_start;
				ask_temp[0] = ASK_TEMP_LVL;
				HAL_UART_Transmit(&huart6, ask_temp, 9, 100);
			}
			if (rc_service_data[0] == WRITE_SN)
			{
				rc_service_data[0] = 0;
				for (int i = 0; i < 7; i += 1)
				{
					serial_number[i] = rc_service_data[i + 1];
				}
				W25qxx_ReadBytes(memory, 0, MEM_ARR); // считывание значений настроек из памяти
				W25qxx_EraseSector(0);				  // Очистка памяти
				for (int p = 0; p < 7; p += 1)
				{
					memory[p + 9] = serial_number[p];
				}
				W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Запись конфига обратно
			}
			if (rc_service_data[0] == READ_SN)
			{
				rc_service_data[0] = 0;
				tx_service_data = READ_SN;
				HAL_UART_Transmit(&huart6, &tx_service_data, 1, 10);
				HAL_UART_Transmit(&huart6, serial_number, 7, 10);
			}
			if (rc_service_data[0] == WR_TEMP_MEM)
			{
				rc_service_data[0] = 0;
				W25qxx_ReadBytes(memory, 0, MEM_ARR); // считывание значений настроек из памяти
				W25qxx_EraseSector(0);				  // Очистка памяти
				for (int i = 0; i < 5; i += 1)
				{
					memory[16 + i] = burn_pwm[i] / 100;
				}
				memory[21] = t_lvl_low;
				memory[22] = t_lvl_start;
				memory[23] = t_lvl_start_low;
				W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Запись конфига обратно
			}
			if (rc_service_data[0] == ADC_ASK_ON) // Если пришел запрос о данных с датчиков АЦП и обработка АЦП завершена
			{
				adc_ask = 1;
				rc_service_data[0] = 0;
			}
			if (rc_service_data[0] == ADC_ASK_OFF)
			{
				adc_ask = 0;
				rc_service_data[0] = 0;
			}
			if (rc_service_data[0] == HID_ON) // Если пришел запрос о закрытие лючка
			{
				PLUG_START();
				PLUG_TIM = 1000;
				HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);

				rc_service_data[0] = 0;
				tx_service_data = ANS_HID_ON;
				HAL_UART_Transmit(&huart6, &tx_service_data, 1, 10); // shalet 15
			}
			if (rc_service_data[0] == HID_OFF) // Если пришел запрос об открытие лючка
			{
				PLUG_STOP(); // Включаем сервопривод на открытие лючка
				rc_service_data[0] = 0;
				tx_service_data = ANS_HID_OFF;
				HAL_UART_Transmit(&huart6, &tx_service_data, 1, 10); // shalet 15
			}
			if (rc_service_data[0] == HEAT_ON) // Если пришел запрос на включение нагревателей
			{
				HE1_TIM = 20000;
				HE2_TIM = 20000;
				HE3_TIM = 20000;
				HE4_TIM = 20000;
				HE5_TIM = 20000;
				HE6_TIM = 20000;

				HE1_START;
				HE2_START; // Включаем все нагреватели
				HE3_START;
				HE4_START;
				HE5_START;
				HE6_START;

				rc_service_data[0] = 0;
				tx_service_data = ANS_HEAT_ON;
				HAL_UART_Transmit(&huart6, &tx_service_data, 1, 10); // shalet 15
			}
			if (rc_service_data[0] == HEAT_OFF) // Если пришел запрос на выключение нагревателей
			{
				HE1_STOP;
				HE2_STOP;
				HE3_STOP;
				HE4_STOP;
				HE5_STOP;
				HE6_STOP;

				rc_service_data[0] = 0;
				tx_service_data = ANS_HEAT_OFF;
				HAL_UART_Transmit(&huart6, &tx_service_data, 1, 10); // shalet 15
			}
			if (rc_service_data[0] == PUMP_ON)
			{
				PUMP_START;
				rc_service_data[0] = 0;
				tx_service_data = ANS_PUMP_ON;
				HAL_UART_Transmit(&huart6, &tx_service_data, 1, 10); // shalet 15
			}
			if (rc_service_data[0] == PUMP_OFF)
			{
				PUMP_STOP;
				rc_service_data[0] = 0;
				tx_service_data = ANS_PUMP_OFF;
				HAL_UART_Transmit(&huart6, &tx_service_data, 1, 10); // shalet 15
			}
			if (rc_service_data[0] == FAN_ON)
			{
				FAN_TIM = 200; // 30000;
				FAN_START;
				rc_service_data[0] = 0;
				tx_service_data = ANS_FAN_ON;
				HAL_UART_Transmit(&huart6, &tx_service_data, 1, 10); // shalet 15
			}
			if (rc_service_data[0] == FAN_OFF)
			{
				FAN_STOP;
				rc_service_data[0] = 0;
				tx_service_data = ANS_FAN_OFF;
				HAL_UART_Transmit(&huart6, &tx_service_data, 1, 10); // shalet 15
			}
			if (rc_service_data[0] == BUZZ_ON)
			{
				BUZZ_START();
				rc_service_data[0] = 0;
				tx_service_data = ANS_BUZZ_ON;
				HAL_UART_Transmit(&huart6, &tx_service_data, 1, 10); // shalet 15
			}
			if (rc_service_data[0] == BUZZ_OFF)
			{
				BUZZ_STOP();
				rc_service_data[0] = 0;
				tx_service_data = ANS_BUZZ_OFF;
				HAL_UART_Transmit(&huart6, &tx_service_data, 1, 10); // shalet 15
			}
			if (rc_service_data[0] == CAL_ADC_MAX_MOTOR) // shalet not present
			{
				rc_service_data[0] = 0;
				memory[28] = filter_adc_f[MOTOR];
				motor_sens_max_open = memory[28];
				motor_sens_close = motor_sens_max_open + D_MOTOR_SENS;
				W25qxx_EraseSector(0);					 // Стираем весь сектор памяти EEPROM
				W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Записываем заного весь буффер памяти в EEPROM
				tx_service_data = ANS_CAL_ADC_MAX;
				HAL_UART_Transmit(&huart6, &tx_service_data, 1, 10);
			}
			if (rc_service_data[0] == CAL_ADC_MOTOR)
			{
				rc_service_data[0] = 0;
				memory[29] = filter_adc_f[MOTOR];
				motor_sens_open = memory[29];
				W25qxx_EraseSector(0);					 // Стираем весь сектор памяти EEPROM
				W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Записываем заного весь буффер памяти в EEPROM
				tx_service_data = ANS_CAL_ADC;
				HAL_UART_Transmit(&huart6, &tx_service_data, 1, 10);
			}
			if (rc_service_data[0] == ASK_FAULT)
			{
				rc_service_data[0] = ASK_FAULT;
				rc_service_data[1] = fault_st;
				HAL_UART_Transmit(&huart6, rc_service_data, 2, 10); // shalet 8
				rc_service_data[0] = 0;
			}
			if (rc_service_data[0] == CLEAR_OPEN_ELCAMINO)
			{
				W25qxx_ReadBytes(memory, 0, MEM_ARR); // считывание значений настроек из памяти
				W25qxx_EraseSector(0);				  // Очистка памяти
				memory[1] = 0;						  // Обнуление ошибки вскрытия камина
				fault_st &= ~OPEN_ELCAMINO;
				W25qxx_WritePage(memory, 0, 0, MEM_ARR); // Запись конфига обратно
				rc_service_data[0] = 0;
			}
			if (rc_service_data[0] == CLEAR_SETTINGS)
			{
				rc_service_data[0] = 0;
				W25qxx_EraseSector(0);
				W25qxx_ReadBytes(memory, 0, MEM_ARR);
				HAL_UART_Transmit(&huart6, memory, MEM_ARR, 10);
			}
			if (rc_service_data[0] == SERVICE_OFF) // shalet not present											//Если пришел запрос о выходе из сервисного режима
			{
				service_st = 0; // Скидываем флаг
				rc_service_data[0] = 0;
				tx_service_data = ANS_SERVICE_OFF;
				HAL_UART_Transmit(&huart6, &tx_service_data, 1, 10);
			}
			HAL_UART_Receive_IT(&huart6, rc_service_data, 8);
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart6)
	{
		if (rc_service_data[0] == SERVICE_ON)
		{
			rc_service_data[0] = 0;
			service_st = 1;
			tx_service_data = ANS_SERVICE_ON;
			HAL_UART_Transmit(&huart6, &tx_service_data, 1, 10); // shalet 15
		}
		HAL_UART_Receive_IT(&huart6, rc_service_data, 8);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2) // shalet not present
	{
		PLUG_TIM = PLUG_TIM + 10;
	}
	if (htim == &htim5)
	{
		VL53L0X_GetMeasurementDataReady(Dev, &fuel_st);
		adc_flag = 1;
		rx_int_flag = 1;
		// shalet not present
		if ((HAL_GPIO_ReadPin(GPIOA, TMC_DIR_Pin) == GPIO_PIN_RESET) && (filter_adc_f[MOTOR] > motor_sens_open))
			fail_topcap_count += 1;
	}

	if (htim == &htim7)
	{
		HAL_TIM_Base_Stop_IT(&htim7);
		count_pin = 0;
		if (pin_st == LOCK)
		{
			if (pincode_temp == SERVICE_PIN)
			{
				writeHalfWordDWIN(PIN_LOCK_ADDR, 2);
				if ((elcamino_st == STANDBY) || (elcamino_st == WAIT))
					goToPageDWIN(STANDBY_SCREEN);
				if (elcamino_st == BURN)
					goToPageDWIN(BURN_SCREEN);
				if (elcamino_st == HEAT)
					goToPageDWIN(HEAT_SCREEN);
				if (elcamino_st == BURN_AFTER_5)
					goToPageDWIN(BURN_5_SCREEN);
				if (elcamino_st == WAIT_WAR)
					goToPageDWIN(WARNING_SCREEN);
				pin_st = UNLOCK;
				rx433_flag = 0;
				rf_data[2] = 0;
				if ((pin_cfg & USE_PIN_433_ON) == USE_PIN_433_ON)
					HAL_TIM_Base_Start_IT(&htim6);
			}
			else
			{
				if (pincode_temp == pincode)
				{
					writeHalfWordDWIN(PIN_LOCK_ADDR, 2);
					if ((elcamino_st == STANDBY) || (elcamino_st == WAIT))
						goToPageDWIN(STANDBY_SCREEN);
					if (elcamino_st == BURN)
						goToPageDWIN(BURN_SCREEN);
					if ((elcamino_st == HEAT) || (elcamino_st == PRE_BURN))
						goToPageDWIN(HEAT_SCREEN);
					if (elcamino_st == BURN_AFTER_5)
						goToPageDWIN(BURN_5_SCREEN);
					if (elcamino_st == WAIT_WAR)
						goToPageDWIN(WARNING_SCREEN);
					pin_st = UNLOCK;
					rx433_flag = 0;
					rf_data[2] = 0;
					if ((pin_cfg & USE_PIN_433_ON) == USE_PIN_433_ON)
						HAL_TIM_Base_Start_IT(&htim6);
				}
				else
				{
					writeHalfWordDWIN(PIN_LOCK_ADDR, 1);
					if ((elcamino_st == STANDBY) || (elcamino_st == WAIT))
						goToPageDWIN(LOCK_STANDBY_SCREEN);
					if (elcamino_st == BURN)
						goToPageDWIN(LOCK_BURN_SCREEN);
					if ((elcamino_st == HEAT) || (elcamino_st == PRE_BURN))
						goToPageDWIN(LOCK_HEAT_SCREEN);
					if (elcamino_st == BURN_AFTER_5)
						goToPageDWIN(LOCK_BURN_5_SCREEN);
					if (elcamino_st == WAIT_WAR)
						goToPageDWIN(LOCK_WARNING_SCREEN);
				}
			}
		}
		if ((pin_st == ENTER_SETTING_PIN) || (pin_st == ENTER_SETTING_PIN_433) || (pin_st == NEW_SETTING_PIN) || (pin_st == NEW_SETTING_PIN_433))
		{
			goToPageDWIN(PIN_SETTING_SCREEN);
			pin_st = UNLOCK;
		}
		pincode_temp = 0;
		for (i = 0; i < 4; i = i + 1)
		{
			pin_arr[i] = 0;
			writeHalfWordDWIN(num_pin_addr[i], 0);
		}
	}

	if (htim == &htim6)
	{
		rx433_flag = 1;
		HAL_TIM_Base_Stop_IT(&htim6);
	}
	if (htim == &htim9)
	{
		ESP32SendServiceCode(ESP32_RESET);
		HAL_TIM_Base_Stop_IT(&htim9);
	}
	if (htim == &htim10)
	{
		if (flag_sleep_int)
		{
			if (sleep_time_st == 3)
			{

				sleep_time_count = sleep_time_count - 1;
				sleep_time_view = fmod(sleep_time_count, 10.0f);
				if (sleep_time_view != 0)
					writeHalfWordDWIN(SLEEP_TIME_ADDR, (sleep_time_count / 10) + 1);
				else
					writeHalfWordDWIN(SLEEP_TIME_ADDR, (sleep_time_count / 10));
			}
			flag_sleep_int = 0;
		}
		else
		{
			flag_sleep_int = 1;
		}
	}
	if (htim == &htim11)
	{
		HAL_TIM_Base_Stop_IT(&htim11);
		PUMP_STOP;
		if (warning_st == 0)
			goToPageDWIN(STANDBY_SCREEN);
		else
			goToPageDWIN(WARNING_SCREEN);
	}
	if (htim == &htim12)
	{
		if ((ir_delay > 0) && (ir_delay < ir_delay_max)) // shalet not present
			ir_delay += 1;
		if ((filter_adc_f[MOTOR] <= motor_sens_open) && (elcamino_st != FUELING)) // shalet not present
			close_topcap_count += 1;
		ReceiveESP32();

		if (filter_fuel > LOW_FUEL)
			filter_fuel = LOW_FUEL;

		/*if (((fuel_buff_disp < filter_fuel)) && (elcamino_st != FUELING))
			fuel_buff_disp = filter_fuel;
		if ((fuel_buff_disp > filter_fuel) && (elcamino_st == FUELING))
			fuel_buff_disp = filter_fuel;*/

		// k_disp_fuel = (filter_fuel * a_disp_fuel + b_disp_fuel) * FUEL_LVL_K;

		//	buff_fuel = (LOW_FUEL - filter_fuel/*fuel_buff_disp*/) / k_disp_fuel;

		buff_fuel = (LOW_FUEL - filter_fuel) / FUEL_LVL_K; // shalet 1.5f

		/*if (((inp_fuel > buff_fuel)) && (elcamino_st != FUELING))
			inp_fuel = buff_fuel;
		if ((inp_fuel < buff_fuel) && (elcamino_st == FUELING))*/

		/*if(elcamino_st == FUELING)
			inp_fuel = floorf(buff_fuel);
		else
			inp_fuel = ceilf(buff_fuel);*/

		inp_fuel = roundf(buff_fuel * k_disp_fuel);

		if (inp_fuel > 20)
			inp_fuel = 20;
		writeHalfWordDWIN(FUEL_ADDR, inp_fuel);
		fuel_time = ((inp_fuel * 5) / time_k[burn_lvl - 1]) * K_FROAD;
		ESP32Data.fuel_time = fuel_time;
		ESP32Data.fuel = (uint8_t)inp_fuel * 5;
		ESP32SendStatus();
		hour = fuel_time / 60;
		if (hour > 9)
			hour = 0;
		min = roundf((fuel_time % 60) / 10.0f);
		if (min > 5)
			min = 5;
		writeHalfWordDWIN(HOUR_FUEL_ADDR, hour);
		writeHalfWordDWIN(MIN_FUEL_ADDR, min);
		if (adc_ask == 1)
		{
			adc_service[0] = ADC_ASK_ON;
			adc_service[1] = (uint8_t)filter_fuel;
			for (int k = 1; k < 15; k = k + 1)
			{
				adc_service[k + 1] = (uint8_t)filter_adc_f[k];
			}
			HAL_UART_Transmit(&huart6, adc_service, 15, 0xFF);
		}
		if (memory[2] == 0x02) // настройка работы звука тач дисплея
			buzzerTouchOnDWIN();
		else
			buzzerTouchOffDWIN();
	}
	if (htim == &htim13)
	{
		cancel_cool_count = cancel_cool_count - 1;
		writeHalfWordDWIN(COUNT_PRECOOL_ADDR, cancel_cool_count);
		if (cancel_cool_count < 10)
			writeHalfWordDWIN(COUNT_SEC_ICON_ADDR, 3);
		if ((cancel_cool_count <= 4) && (cancel_cool_count > 1))
			writeHalfWordDWIN(COUNT_SEC_ICON_ADDR, 2);
		if (cancel_cool_count == 1)
			writeHalfWordDWIN(COUNT_SEC_ICON_ADDR, 1);

		if (cancel_cool_count == 0)
		{
			HAL_TIM_Base_Stop(&htim13);
			elcamino_st = COOLING;
			goToPageDWIN(COOLING_SCREEN);
		}
	}
	if (htim == &htim14)
	{
		plug_tim_flag += 1;
		if (plug_tim_flag == PLUG_FAULT_CNT) // shalet 25
			PLUG_STOP();
		//			plug_tim_flag = 2;				// таймаут свечи наступил
		//			PLUG_STOP();
	}
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
