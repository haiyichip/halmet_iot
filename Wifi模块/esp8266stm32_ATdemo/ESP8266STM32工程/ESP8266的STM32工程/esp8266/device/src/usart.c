#include "usart.h"
#include "esp8266.h"
#include "tcp.h"


#pragma import(__use_no_semihosting)             
//��׼����Ҫ֧�ֵĺ���
struct __FILE 
{
	int handle;
};

FILE __stdout;       
//����_sys_exit()�Ա��⹤���ڰ�����״̬
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc����
//�����Ҫ����MCU������ϣ��printf���ĸ����������ȷ�� __WAIT_TODO__
int fputc(int ch, FILE *f)
{
	//ע�⣺USART_FLAG_TXE�Ǽ�鷢�ͻ������Ƿ�Ϊ�գ����Ҫ�ڷ���ǰ��飬������������߷���Ч�ʣ����������ߵ�ʱ����ܵ������һ���ַ���ʧ
	//USART_FLAG_TC�Ǽ�鷢����ɱ�־������ڷ��ͺ��飬����������˯�߶�ʧ�ַ����⣬����Ч�ʵͣ����͹����з��ͻ������Ѿ�Ϊ���ˣ����Խ�����һ�������ˣ�������ΪҪ�ȴ�������ɣ�����Ч�ʵͣ�
	//��Ҫ����һ���ã�һ����Ч�����
	
	//ѭ���ȴ�ֱ�����ͻ�����Ϊ��(TX Empty)��ʱ���Է������ݵ�������
  while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
  {}
	USART_SendData(USART3, (uint8_t) ch);

  /* ѭ���ȴ�ֱ�����ͽ���*/
  while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET){}

	return ch;
}




void uart2_Init(u32 bound)//����2  ����ΪPA2  PA3
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	    //ʹ��ָ���˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//��ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//��ʼ��GPIO
	
	//Usart2 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         
  NVIC_Init(&NVIC_InitStructure); 

	
	//USART2����
	USART_InitStructure.USART_BaudRate = bound;	//���ô��ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//�ֳ�Ϊ8
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	//1��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;	//����żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure); //����USART����
	
	USART_ITConfig(USART2, USART_IT_RXNE|USART_IT_IDLE, ENABLE);//�����˽����жϺ����߿����ж�
	
	USART_Cmd(USART2, ENABLE);                    //ʹ��USART
}
void USART2_IRQHandler(void)
{   
    u8 ucCh;

    if(USART_GetITStatus( USART2, USART_IT_RXNE ) != RESET )
    {
        ucCh  = USART_ReceiveData( USART2 );
			  USART_SendData(USART3,ucCh);
        if(ESP8266_Fram_Record_Struct .InfBit .FramLength < ( RX_BUF_MAX_LEN - 1 ) ) 
        {

            ESP8266_Fram_Record_Struct .Data_RX_BUF[ ESP8266_Fram_Record_Struct .InfBit .FramLength ++ ]  = ucCh;   
        }                      
    }

    if( USART_GetITStatus( USART2, USART_IT_IDLE ) == SET )                                         //������߿���
    {
        ESP8266_Fram_Record_Struct .InfBit .FramFinishFlag = 1;

        ucCh = USART_ReceiveData( USART2 );                                                              //�������������жϱ�־λ���ȶ�USART_SR,Ȼ���USART_DR��
				USART_SendData(USART3,ucCh);
        TcpClosedFlag = strstr ( ESP8266_Fram_Record_Struct .Data_RX_BUF, "CLOSED\r\n" ) ? 1 : 0;

    }   

}


void uart3_Init(u32 bound)//����3  ����ΪPB10  PB11
{

    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //?�䨮?��?������?3?
    GPIO_Init(GPIOB, &GPIO_InitStructure);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO

    //Usart3 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      //��Ӧ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //USART_IRQnͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure); //��ʼ��NVIC

    //USART3 ����
    USART_InitStructure.USART_BaudRate = bound;//������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//���ݳ���
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//ֹͣλ1
    USART_InitStructure.USART_Parity = USART_Parity_No;//У��λ��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//Ӳ����������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //ʹ�ܴ��ڵĽ��պͷ��͹���
    USART_Init(USART3, &USART_InitStructure); //��ʼ������

    USART_ITConfig(USART3, USART_IT_RXNE|USART_IT_IDLE, ENABLE);//�����˽����жϺ����߿����ж�

    USART_Cmd(USART3, ENABLE);      //��������ʹ��    
}

void USART3_IRQHandler( void )
{   
    u8 ucCh;

    if(USART_GetITStatus( USART3, USART_IT_RXNE ) != RESET )
    {
        ucCh  = USART_ReceiveData( USART3 );

        if(ESP8266_Fram_Record_Struct .InfBit .FramLength < ( RX_BUF_MAX_LEN - 1 ) ) 
        {
            //�����һλ������λ
            ESP8266_Fram_Record_Struct .Data_RX_BUF[ ESP8266_Fram_Record_Struct .InfBit .FramLength ++ ]  = ucCh;   
        }                      
    }

    if( USART_GetITStatus( USART3, USART_IT_IDLE ) == SET )                              //������߿���
    {
        ESP8266_Fram_Record_Struct .InfBit .FramFinishFlag = 1;
        ucCh = USART_ReceiveData( USART3 );                                                              //�������������жϱ�־λ���ȶ�USART_SR,Ȼ���USART_DR��
        TcpClosedFlag = strstr ( ESP8266_Fram_Record_Struct .Data_RX_BUF, "CLOSED\r\n" ) ? 1 : 0;

    }   

}
