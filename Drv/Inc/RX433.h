

#include "stm32f4xx.h"                  
//#include "delay.h"
#include "usart.h"
 
#ifdef __cplusplus
extern "C" {
#endif
	

	void Receive_433(void);					// функция обработки приема
	uint32_t GetAddr_433 (void); // Получаем адрес устройства
	 uint8_t GetKey_433 (void); // Получаем номер ключа 1527M
	 void Rst_rf_data (void); // Обнуляем rf_data после вызова функции GetKey_315M ()
	
#ifdef __cplusplus
}
#endif
	
