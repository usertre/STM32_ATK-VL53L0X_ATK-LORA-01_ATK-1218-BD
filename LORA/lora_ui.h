#ifndef __LORA_UI_H__
#define __LORA_UI_H__	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//ATK-LORA-01ģ��UI�͵ײ㴦������		  
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2016/4/1
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved	
//********************************************************************************
//��

void lora_at_response(u8 mode);	
u8* lora_check_cmd(u8 *str);
u8 lora_send_cmd(u8 *cmd,u8 *ack,u16 waittime);
extern u32 obj_addr;
extern u8 obj_chn;
void Menu_ui(void);
void Menu_cfg(u8 num);
void Process_ui(void);
void Dire_Set(void);
void Diree_Set(void);
extern u32 obj_addr;
extern u8 obj_chn;
#endif





