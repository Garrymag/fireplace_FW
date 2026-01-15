#include "RX433.h"	


 // ******************* Область определения данных ************************** ******* //
 uint8_t RF; // Уровень интерфейса
 uint8_t a_code1, a_code2, a_code3; // Первый код дистанционного управления
 uint8_t rf_ok1; // Первое назначение кода удаленного управления выполнено успешно
 uint8_t b_code1, b_code2, b_code3; // Второй код дистанционного управления
 uint8_t rf_ok2; // Второе назначение кода дистанционного управления выполнено успешно
 uint8_t t_code1, t_code2, t_code3; // Временный код дистанционного управления
 uint8_t last_state; // Последний код, 0 - низкий, 1 - высокий
 uint8_t hh_w, ll_w; // Ширина верхнего и нижнего уровня
 uint8_t flag_syn; // Бит флага кода синхронизации, установлен в 1, чтобы указать, что код синхронизации был получен, и установлен в 0, чтобы указать, что код синхронизации не получен
extern uint8_t rf_data [3]; // Последний полученный код дистанционного управления
 uint8_t ma_x; // какой код получен
 uint16_t s; // Период между получением первого кода и второго кода не может превышать s
uint8_t rf_ok=1;
//*************************************************************//

 
void Receive_433(void)  
{
	    {
        RF = HAL_GPIO_ReadPin(GPIOD, RX433_Pin);
	//printf("%d", RF);
	 // Получение низкого уровня, ll_w добавляется сам по себе, а предыдущее состояние устанавливается на низкий уровень
        if (!RF){ 
            ll_w++;                         
            last_state = 0;
	//printf("low!\r");
        }    
	 // Получение высокого уровня, hh_w добавляет
	 // Последний бит всех кодов синхронизации, последний бит каждого кода изменяется с низкого на высокий.
        else { 
            hh_w++;
	 // Если последнее состояние низкое, нарастающий фронт от низкого до высокого	
	//printf("hh_w =%d!\r\n", hh_w);
            if (!last_state)  
            {   
	//printf("ll_w =%d!\r\n", ll_w);
	 // Если код синхронизации не получен, определяем время начала кода синхронизации
                if (((hh_w>=1)&&(hh_w<=5))&&((ll_w>=90)&&(ll_w<=140))){ 
                    flag_syn = 1 ;
                    ma_x = 0;
                                         t_code1 = 0; t_code2 = 0; t_code3 = 0; // инициализация 
	//printf("\r\n\r\n");									
                }
	 // Если код синхронизации был получен и получен 3 CLK bit 0
                else if ((flag_syn)&&((ll_w >= 7)&&(ll_w <= 14))){   
                    ma_x++; 
	//printf("L\r\n");
	 // Если полученный код больше 23
                    if(ma_x>23){
	 // прием rf_ok1 не завершен
                        if(!rf_ok1){ 
	 // Флаг кодировки получен впервые
                            a_code1=t_code1;
                            a_code2=t_code2;
                            a_code3=t_code3;													
                                                       // можно декодировать                                                         
                            rf_ok1=1;                    
                            flag_syn=0; 
		 // Отсчет времени начинается
		s=1000;
                         }
		 // Если rf_ok1 получен успешно, сохраняем данные во вторую группу
                        else{
                            b_code1=t_code1;
                            b_code2=t_code2;
                            b_code3=t_code3;  																																 
		            rf_ok2=1;                     
		            flag_syn=0;                                                                        
                       }
                    }
                  }  
                  else if ((flag_syn)&&((ll_w>=2)&& (ll_w<=6))) { 
		//printf("H\r\n");	
		 switch (ma_x)
		 { 
		     case 0 : { t_code1=t_code1 | 0x80; break;} 
                     case 1 : { t_code1=t_code1 | 0x40;break; }
                     case 2 : { t_code1=t_code1 | 0x20;break; }
                     case 3 : { t_code1=t_code1 | 0x10;break; }
                     case 4 : { t_code2=t_code2 | 0x08;break; }
		     case 5 : { t_code2=t_code2 | 0x04;break; }
		     case 6 : { t_code2=t_code2 | 0x02;break; }
		     case 7 : { t_code2=t_code2 | 0x01;break; }
		     case 8 : { t_code2=t_code2 | 0x80;break; }
		     case 9 : { t_code2=t_code2 | 0x40;break; }
		     case 10: { t_code2=t_code2 | 0x20;break; }
		     case 11: { t_code2=t_code2 | 0x10;break; }
		     case 12: { t_code2=t_code2 | 0x08;break; }
		     case 13: { t_code2=t_code2 | 0x04;break; }
		     case 14: { t_code2=t_code2 | 0x02;break; }
		     case 15: { t_code2=t_code2 | 0x01;break; }
		     case 16: { t_code3=t_code3 | 0x80;break; }
		     case 17: { t_code3=t_code3 | 0x40; break; }
		     case 18: { t_code3=t_code3 | 0x20;break; }
		     case 19: { t_code3=t_code3 | 0x10;break; }
		     case 20: { t_code3=t_code3 | 0x08;break; }
		     case 21: { t_code3=t_code3 | 0x04;break; }
		     case 22: { t_code3=t_code3 | 0x02;break; }
		     case 23: { t_code3=t_code3 | 0x01;              
                                  if(!rf_ok1)
                                  {
                                      a_code3=t_code3;
                                      a_code2=t_code2;
                                      s=100;
                                  }
                                  else
                                  {
                                      b_code3=t_code3;
                                      b_code2=t_code2;
                                      b_code1=t_code1;
				}   
			               flag_syn=0;
                                       break;  	
                          }
                    } 
                    ma_x++; 
                 }
                 else{ ma_x=0; flag_syn=0;t_code3=0;t_code2=0; t_code1=0;ll_w=0; }                                    
                    ll_w=0;hh_w=1; 
             }          
                 last_state=1; 
		//printf("ma_x = %d", ma_x);						 
				}
	 if(rf_ok1)  
		{
		//printf("ok1\r\n");
		s--;
		if(!s) rf_ok1=0;
		if(rf_ok2) {
			if((a_code1==b_code1)&&(a_code2==b_code2)&&(a_code3==b_code3)){
				rf_ok=1;
				rf_ok1=0;
				rf_ok2=0;                    
					}
			else{
				rf_ok=0;
				rf_ok1=0;
				rf_ok2=0;
					}          
				}                   
		}
 
    if((rf_ok))     
    {   //printf("ok2\r\n");
				rf_ok=0; 
        rf_data[0]=a_code1;
        rf_data[1]=a_code2;
        rf_data[2]=a_code3;
	//printf("rf_data[2] =%d\r\n",rf_data[2]);
			}
    }
		
}
 
uint32_t GetAddr_433
(){		
	uint32_t result = ((rf_data[0] << 16) + (rf_data[1] << 8) + (rf_data[2] & 0xF0) )>> 4;
	//printf("%d\r\n",result);
	return result;
}
	
uint8_t GetKey()
{
	uint8_t result = (rf_data[2] & 0x0F);
	//printf("result = %d\r\n",result);
	return result;
}	
 
void Rst_rf_data()
	{
		rf_data[2]= 0x00;
	}