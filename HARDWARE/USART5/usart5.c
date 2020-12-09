/**
  ****************************************************************************************************
  * @file    	usart2.c
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


#include "sys.h"
#include "usart2.h"
#include "ringbuffer.h"



#define UART2_RX_BUFFER_SIZE	256
uint8_t uart2RxBuffer[UART2_RX_BUFFER_SIZE];

ringbuffer_t uart2RxFifo;
UART_HandleTypeDef uart2_handler; 		/*UART句柄*/




/**
 * @brief       串口X初始化
 * @param       无
 * @retval      无
 */
void usart2_init(uint32_t bound)
{
    GPIO_InitTypeDef gpio_initure;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    gpio_initure.Pin = GPIO_PIN_2;
    gpio_initure.Mode = GPIO_MODE_AF_PP;
    gpio_initure.Pull = GPIO_PULLUP;
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio_initure);

    gpio_initure.Pin = GPIO_PIN_3;
    gpio_initure.Mode = GPIO_MODE_AF_INPUT;
    HAL_GPIO_Init(GPIOA, &gpio_initure);

    /* USART 初始化设置 */
    uart2_handler.Instance = USART2;
    uart2_handler.Init.BaudRate = bound;                                     /*波特率*/
    uart2_handler.Init.WordLength = UART_WORDLENGTH_8B;                      /*字长为8位数据格式*/
    uart2_handler.Init.StopBits = UART_STOPBITS_1;                           /*一个停止位*/
    uart2_handler.Init.Parity = UART_PARITY_NONE;                            /*无奇偶校验位*/
    uart2_handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;                      /*无硬件流控*/
    uart2_handler.Init.Mode = UART_MODE_TX_RX;                               /*收发模式*/
    HAL_UART_Init(&uart2_handler);                                           /*HAL_UART_Init()会使能UART2*/

    __HAL_UART_ENABLE_IT(&uart2_handler, UART_IT_RXNE);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 3);

    ringbuffer_init(&uart2RxFifo, uart2RxBuffer, UART2_RX_BUFFER_SIZE);
}


/**
  * @brief  串口2发送
  * @param  data: 发送的数据
  * @param  len: 数据长度
  * @retval uint8_t: 0成功 其他：失败
  */
uint8_t usart2_sendData(uint8_t *data, uint16_t len)
{
	return HAL_UART_Transmit(&uart2_handler, data, len, 500);
}


/**
  * @brief  获取串口2接收fifo的数据
  * @param  buf: 存放的缓冲区
  * @param  len: 需要获取的长度
  * @retval uint16_t: 实际获取到的长度 0表示没有数据可获取
  */
uint16_t usart2_getRxData(uint8_t *buf, uint16_t len)
{
    return ringbuffer_out(&uart2RxFifo, buf, len);
}


/**
 * @brief       串口X中断服务函数
 * @param       无
 * @retval      无
 */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&uart2_handler);

    if (__HAL_UART_GET_FLAG(&uart2_handler, UART_FLAG_RXNE) != RESET) /*!< 接收非空中断 */
    {
        uint8_t res = uart2_handler.Instance->DR;
        ringbuffer_in_check(&uart2RxFifo, (uint8_t *)&res, 1); /*!< 将接收到的数据放入FIFO */
    }
}




/*******************************END OF FILE************************************/



