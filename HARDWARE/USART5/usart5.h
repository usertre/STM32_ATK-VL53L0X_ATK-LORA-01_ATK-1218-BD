/**
  ****************************************************************************************************
  * @file    	usart2.h
  * @author		����ԭ���Ŷ�(ALIENTEK)
  * @version    V1.0
  * @date		2020-04-17
  * @brief   	����2��������
  * @license   	Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
  ****************************************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:����ԭ�� STM32������
  * ������Ƶ:www.yuanzige.com
  * ������̳:www.openedv.com
  * ��˾��ַ:www.alientek.com
  * �����ַ:openedv.taobao.com
  *
  * �޸�˵��
  * V1.0 2020-04-17
  * ��һ�η���
  *
  ****************************************************************************************************
  */

#ifndef _UART4_H_
#define _UART4_H_


#include "sys.h"
void uart4_init(uint32_t bound);

uint8_t uart4_sendData(uint8_t *data, uint16_t len);
uint16_t uart4_getRxData(uint8_t *buf, uint16_t len);


#endif /* _USART2_H_ */

/*******************************END OF FILE************************************/


