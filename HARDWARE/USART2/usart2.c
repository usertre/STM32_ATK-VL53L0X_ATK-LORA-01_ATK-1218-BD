#include "delay.h"
#include "usart2.h"
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"
#include "lora_cfg.h"
#include "lora_app.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//����2��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/3/29
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   

//���ڷ��ͻ����� 	
__align(8) u8 USART3_TX_BUF[USART3_MAX_SEND_LEN]; 	//���ͻ���,���USART2_MAX_SEND_LEN�ֽ�
#ifdef USART3_RX_EN   								//���ʹ���˽���   	  
//���ڽ��ջ����� 	
u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//���ջ���,���USART2_MAX_RECV_LEN���ֽ�.


//������ģʽ�£� timer=10ms
//ͨ���жϽ�������2���ַ�֮���ʱ������timer�������ǲ���һ������������.
//���2���ַ����ռ������timer,����Ϊ����1����������.Ҳ���ǳ���timerû�н��յ�
//�κ�����,���ʾ�˴ν������.
//���յ�������״̬
//[15]:0,û�н��յ�����;1,���յ���һ������.
//[14:0]:���յ������ݳ���
u16 USART3_RX_STA=0;   	 
void USART3_IRQHandler(void)
{
	u8 res;	    
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//���յ�����
	{	 
		res =USART_ReceiveData(USART3);
		if((USART3_RX_STA&(1<<15))==0)
		{		
			if(USART3_RX_STA<USART3_MAX_RECV_LEN)		//�����Խ�������
			{
				if(!Lora_mode)
				{
					TIM_SetCounter(TIM4,0); //���������      
					if(USART3_RX_STA==0)TIM4_Set(1);	 	//ʹ�ܶ�ʱ��4���ж� 
				}
				USART3_RX_BUF[USART3_RX_STA++]=res;		//��¼���յ���ֵ	 
			}else 
			{
				USART3_RX_STA|=1<<15;					//ǿ�Ʊ�ǽ������
			}
	   }		
	}  											 
}   

USART_InitTypeDef USART_InitStructure;

//��ʼ��IO ����2
//pclk1:PCLK1ʱ��Ƶ��(Mhz)
//bound:������	  
void usart3_init(u32 bound)
{ 	
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);// GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);

 	USART_DeInit(USART3);                                //��λ����1
		 //USART2_TX   PA.2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;            //PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	     //�����������
    GPIO_Init(GPIOB, &GPIO_InitStructure);               //��ʼ��PA2
   
    //USART2_RX	  PA.3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;            //PA.3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;        //��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);               //��ʼ��PA3
	
	USART_InitStructure.USART_BaudRate = bound;                                    //һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;                            //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	               //�շ�ģʽ
   
	USART_Init(USART3, &USART_InitStructure); //��ʼ������2
  
	//����������
    //USART2->BRR=(pclk1*1000000)/(bound);// ����������	 
	//USART2->CR1|=0X200C;  	//1λֹͣ,��У��λ.
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  	                   //ʹ�ܴ���2��DMA����
	UART_DMA_Config(DMA1_Channel2,(u32)&USART3->DR,(u32)USART3_TX_BUF);//DMA1ͨ��7,����Ϊ����2,�洢��ΪUSART2_TX_BUF 
	USART_Cmd(USART3, ENABLE);                                         //ʹ�ܴ��� 
	
#ifdef USART3_RX_EN		  	//���ʹ���˽���
	//ʹ�ܽ����ж�
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�����ж�   
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	TIM4_Init(99,7199);		//10ms�ж�
	USART3_RX_STA=0;		//����
	TIM4_Set(0);			//�رն�ʱ��4
#endif	 									  	
}
//����2,printf ����
//ȷ��һ�η������ݲ�����USART2_MAX_SEND_LEN�ֽ�
void u3_printf(char* fmt,...)  
{  
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART3_TX_BUF,fmt,ap);
	va_end(ap);
	while(DMA1_Channel2->CNDTR!=0);	//�ȴ�ͨ��7�������   
	UART_DMA_Enable(DMA1_Channel2,strlen((const char*)USART3_TX_BUF)); 	//ͨ��dma���ͳ�ȥ
}

//��ʱ��4�жϷ������		    
void TIM4_IRQHandler(void)
{ 	
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//�Ǹ����ж�
	{	 			   

		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //���TIMx�����жϱ�־    
	    USART3_RX_STA|=1<<15; //��ǽ������
		TIM4_Set(0);		   //�ر�TIM4  
	}   
}
//����TIM4�Ŀ���
//sta:0���ر�;1,����;
void TIM4_Set(u8 sta)
{
	if(sta)
	{ 
		TIM_SetCounter(TIM4,0);  //���������
		TIM_Cmd(TIM4, ENABLE);   //ʹ��TIMx	
	}else TIM_Cmd(TIM4, DISABLE);//�رն�ʱ��4
}
//����TIM4Ԥװ������ֵ
void TIM4_SetARR(u16 period)
{
     TIM_SetCounter(TIM4,0); //���������
	 TIM4->ARR&=0x00;        //����Ԥװ������ֵΪ0
	 TIM4->ARR|= period;     //����Ԥװ������ֵ
}
//ͨ�ö�ʱ���жϳ�ʼ��
//����ʼ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��		 
void TIM4_Init(u16 arr,u16 psc)
{	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʱ��ʹ��//TIM4ʱ��ʹ��    
	
	//��ʱ��TIM3��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM4�ж�,��������ж�

	 	  
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
}
#endif		 
///////////////////////////////////////USART2 DMA�������ò���//////////////////////////////////	   		    
//DMA1�ĸ�ͨ������
//����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
//�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
//DMA_CHx:DMAͨ��CHx
//cpar:�����ַ
//cmar:�洢����ַ    
void UART_DMA_Config(DMA_Channel_TypeDef*DMA_CHx,u32 cpar,u32 cmar)
{
	DMA_InitTypeDef DMA_InitStructure;
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMA����
    DMA_DeInit(DMA_CHx);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA����ADC����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
	DMA_InitStructure.DMA_BufferSize = 0;  //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //��������������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMAͨ�� xӵ�������ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���	
} 
//����һ��DMA����
void UART_DMA_Enable(DMA_Channel_TypeDef*DMA_CHx,u16 len)
{
	DMA_Cmd(DMA_CHx, DISABLE );  //�ر� ָʾ��ͨ��        
	DMA_SetCurrDataCounter(DMA_CHx,len);//DMAͨ����DMA����Ĵ�С	
	DMA_Cmd(DMA_CHx, ENABLE);           //����DMA����
}	   
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 									 


//����2�����ʺ�У��λ����
//bps:�����ʣ�1200~115200��
//parity:У��λ���ޡ�ż���棩
void usart3_set(u8 bps,u8 parity)
{
    static u32 bound=0;
	
	switch(bps)
	{
		case LORA_TTLBPS_1200:   bound=1200;     break;
		case LORA_TTLBPS_2400:   bound=2400;     break;
		case LORA_TTLBPS_4800:   bound=4800;     break;
		case LORA_TTLBPS_9600:   bound=9600;     break;
		case LORA_TTLBPS_19200:  bound=19200;    break;
		case LORA_TTLBPS_38400:  bound=38400;    break;
		case LORA_TTLBPS_57600:  bound=57600;    break;
		case LORA_TTLBPS_115200: bound=115200;   break;
	}
    
	USART_Cmd(USART3, DISABLE); //�رմ��� 
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 
	
	if(parity==LORA_TTLPAR_8N1)//��У��
	{
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;    
		USART_InitStructure.USART_Parity = USART_Parity_No;
	}else if(parity==LORA_TTLPAR_8E1)//żУ��
	{
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;    
		USART_InitStructure.USART_Parity = USART_Parity_Even;
	}else if(parity==LORA_TTLPAR_8O1)//��У��
	{
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;    
		USART_InitStructure.USART_Parity = USART_Parity_Odd;
	}
	USART_Init(USART3, &USART_InitStructure); //��ʼ������2
    USART_Cmd(USART3, ENABLE); //ʹ�ܴ��� 
	
}
 
//���ڽ���ʹ�ܿ���
//enable:0,�ر� 1,��
void usart3_rx(u8 enable)
{
	 USART_Cmd(USART3, DISABLE); //ʧ�ܴ��� 
	
	 if(enable)
	 {
		 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�ģʽ
	 }else
	 {
		 USART_InitStructure.USART_Mode = USART_Mode_Tx;//ֻ���� 
	 }
	 
	 USART_Init(USART3, &USART_InitStructure); //��ʼ������3
     USART_Cmd(USART3, ENABLE); //ʹ�ܴ��� 
	
}


















