#ifndef _LORA_APP_H_
#define _LORA_APP_H_

#include "sys.h"
#include "lora_cfg.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//ATK-LORA-01ģ�鹦������		  
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2016/4/1
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved	
//********************************************************************************
//��

#define LORA_AUX  PBin(1)    //LORAģ��״̬����
#define LORA_MD0  PBout(8)  //LORAģ���������

extern _LoRa_CFG LoRa_CFG;
extern u8 Lora_mode;

u8 LoRa_Init(void);
void Aux_Int(u8 mode);
void LoRa_Set(void);
void LoRa_SendData(void);
void LoRa_SendData1(void);

void LoRa_ReceData(void);
void LoRa_Process(void);

void LoRa_Process1(void);
void Lora_Test(void);


#endif

