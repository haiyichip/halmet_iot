#ifndef __DHT11_H
#define __DHT11_H
#include "main.h"                  // Device header



#define dht11_high HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)
#define dht11_low HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)
#define Read_Data HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)

void delay_us(u32 delay);
void DHT11_GPIO_Init_OUT(void);
void DHT11_GPIO_Init_IN(void);
void DHT11_Start(void);
unsigned char DHT11_REC_Byte(void);
void DHT11_REC_Data(void);



#endif

