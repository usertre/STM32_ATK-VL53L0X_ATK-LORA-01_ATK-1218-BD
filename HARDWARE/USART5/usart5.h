/**
  ****************************************************************************************************
  * @file    	usart2.h
  * @author		正点原子团队(ALIENTEK)
  * @version    V1.0
  * @date		2020-04-17
  * @brief   	串口2驱动代码
  * @license   	Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
  ****************************************************************************************************
  * @attention
  *
  * 实验平台:正点原子 STM32开发板
  * 在线视频:www.yuanzige.com
  * 技术论坛:www.openedv.com
  * 公司网址:www.alientek.com
  * 购买地址:openedv.taobao.com
  *
  * 修改说明
  * V1.0 2020-04-17
  * 第一次发布
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


