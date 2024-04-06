#include  "dht11.h"

//接收数据
unsigned int rec_data[4];

//延时1us
void delay_us(u32 delay)
{
	
}

//初始化PBA4为stm32输出
void DH11_GPIO_Init_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	
	GPIO_InitStructure.Pin   = GPIO_PIN_4;
	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP; //推挽输出
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

}

//初始化PBA4为stm32输入
void DH11_GPIO_Init_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure={0};
	
	GPIO_InitStructure.Pin   = GPIO_PIN_4;
	GPIO_InitStructure.Mode  = GPIO_MODE_INPUT; //输入
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

}



//主机发送开始信号
void DHT11_Start(void)
{
	DH11_GPIO_Init_OUT();
	
	dht11_high; //先拉高总线
	osDelay(1);
	
	dht11_low; //拉低总线至少18ms
	osDelay(20);
	
	dht11_high; //再拉高总线
	osDelay(1);
	
	DH11_GPIO_Init_IN(); //配置为输入模式
}


//获取一个字节的数据
char DHT11_Rec_Byte(void)
{
	unsigned char i = 0;
	unsigned char data;
	
	for(i=0;i<8;i++)
	{
		while(Read_Data==0); //等待低电平结束7
		delay_us(30); //26~28us
		
		data <<= 1; //左移
		
		if( Read_Data == 1 ) //????30us???????????1
		{
			data |= 1; //??+1
		}
		
		while( Read_Data == 1 ); //???????,???????
	}
	
	return data;
}



void DHT11_REC_Data(void)
{
	unsigned int R_H,R_L,T_H,T_L;
	unsigned char RH,RL,TH,TL,CHECK;
	
	DHT11_Start(); //??????
	dht11_high; //????
	
	if( Read_Data == 0 ) //??DHT11????
	{
		while( Read_Data == 0); //???????,???????
		while( Read_Data == 1); //???????,???????
		
		R_H = DHT11_Rec_Byte();
		R_L = DHT11_Rec_Byte();
		T_H = DHT11_Rec_Byte();
		T_L = DHT11_Rec_Byte();
		CHECK = DHT11_Rec_Byte(); //??5???
		
		dht11_low; //????bit???????,DHT11???? 50us
		delay_us(55); //????55us
		dht11_high; //??????????????????
		
		if(R_H + R_L + T_H + T_L == CHECK) //??????,??????????????
		{
			RH = R_H;
			RL = R_L;
			TH = T_H;
			TL = T_L;
		}
	}
	rec_data[0] = RH;
	rec_data[1] = RL;
	rec_data[2] = TH;
	rec_data[3] = TL;
}

