#ifndef __USART2_H
#define __USART2_H	 
#include "sys.h"  
#include "usart2.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEK STM32������
//����2��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/3/29
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   

#define USART2_MAX_RECV_LEN		1024				//�����ջ����ֽ���
#define USART2_MAX_SEND_LEN		1024				//����ͻ����ֽ���
#define USART2_RX_EN 			1					//0,������;1,����.

extern u8  USART2_RX_BUF[USART2_MAX_RECV_LEN]; 		//���ջ���,���USART2_MAX_RECV_LEN�ֽ�
extern u8  USART2_TX_BUF[USART2_MAX_SEND_LEN]; 		//���ͻ���,���USART2_MAX_SEND_LEN�ֽ�
extern u16 USART2_RX_STA;   						//��������״̬

void USART2_Init(u32 bound);				//����2��ʼ�� 

void TIM7_Set(u8 sta);

void TIM7_SetARR(u16 period);

void TIM7_Init(u16 arr,u16 psc);

void UART_DMA_Config1(DMA_Channel_TypeDef*DMA_CHx,u32 cpar,u32 cmar);


void UART_DMA_Enable1(DMA_Channel_TypeDef*DMA_CHx,u8 len);

void u2_printf(char* fmt, ...);
#endif












