#include "mqtt.h"
#include "esp8266.h"


void ESP8266_STA_MQTTClient_Test(void)
{
	  char str[1024];
		printf("��������ESP8266����\r\n");
    ESP8266_AT_Test();//�ָ�����Ĭ��ģʽ
    ESP8266_Net_Mode_Choose(STA);
    while(!ESP8266_JoinAP(User_ESP8266_SSID, User_ESP8266_PWD));
	  ESP8266_MQTTUSERCFG(User_ESP8266_client_id,User_ESP8266_username,User_ESP8266_password);
	  ESP8266_MQTTCONN( User_ESP8266_MQTTServer_IP, User_ESP8266_MQTTServer_PORT);
	  ESP8266_MQTTSUB( User_ESP8266_MQTTServer_Topic);
		printf("\r\nMQTT�������");
		while(1)
		{
			  sprintf(str,"aithinker");//��ʽ�������ַ�����MQTT������
			  MQTT_SendString (User_ESP8266_MQTTServer_Topic,str);//�������ݵ�MQTT������
		}
}
