#include "lora_app.h"
#include "lora_ui.h"
#include "lora_cfg.h"
#include "vl53l0x_gen.h"
#include "usart2.h"
#include "string.h"
#include "led.h"
#include "delay.h"
#include "lcd.h"
#include "stdio.h"
#include "text.h"
#include "key.h"
#include "gps.h"
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

//�豸������ʼ��(�����豸������lora_cfg.h����)
unsigned long long WPS=0,faketime=0;
float fakedata[100][3]={-35.9,-53.5,-121.8,-35.1,-53.7,-118.3,-37.8,-63.8,-112.4,22.4,-66.9,-42.9,-1.9,-48.4,-66.6,-28.6,-31.1,-90.7,-41.4,-35.7,-112.7,-66.3,-33.1,-153.6,-84.9,-31.6,175.2,-117.6,-37.9,136.1,-163.4,-33.5,103.3,156.7,-21.1,75.2,134.0,-8.4,53.7,118.5,-6.5,45.2,104.4,-17.8,39.7,88.8,-30.7,26.5,50.4,-46.8,-12.7,1.4,-54.9,-44.7,-6.6,-63.8,-59.3,-131.2,-65.0,178.3,-174.2,-31.4,137.6,159.6,-16.2,138.6,161.4,-3.1,135.8,166.2,9.2,125.2,169.0,13.3,114.0,162.7,-1.6,104.1,149.7,-27.6,97.4,134.0,-60.0,102.1,49.5,-73.2,57.1,-15.9,-48.2,31.3,-29.4,-21.0,9.6,-28.5,-4.1,-16.2,-29.3,0.0,-42.1,-32.3,-1.2,-52.5,-28.7,-5.6,-52.7,-24.5,-13.6,-44.5,-13.5,-16.8,-32.5,4.0,-13.4,-25.2,4.5,-15.5,-24.4,17.4,-18.5,-17.4,23.4,-17.6,-4.9,30.1,-31.0,9.4,13.2,-37.9,0.9,14.7,-55.0,-18.6,17.8,-52.7,-19.7,50.7,-44.0,5.6,60.9,-27.3,6.6,89.8,-38.1,14.5,129.8,-64.4,54.8,16.9,-78.6,-54.7,49.7,-40.6,-20.5,-3.9,-21.7,-42.2,-18.1,-36.2,-57.1,-38.1,-37.4,-88.4,-49.8,-36.3,-121.6,-55.5,-16.1,-143.1,-44.0,17.4,-166.9,-6.4,34.7,170.1,8.6,34.0,166.5,2.4,21.6,176.7,-22.2,-8.9,-169.7,-27.6,-31.7,-174.0,-54.3,-31.8,-179.2,-67.0,-26.1,168.9,-67.2,-12.9,145.1,-76.8,-2.4,125.4,-80.1,5.6,114.8,-77.0,4.2,109.7,-67.0,-2.3,80.6,-73.0,-20.7,51.1,-92.2,-48.6,24.7,-176.7,-67.4,-55.5,102.1,-48.9,-120.4,103.6,-20.7,-113.7,110.3,3.6,-113.4,114.4,22.9,-120.2,111.5,18.6,-120.3,115.2,18.2,-138.6,118.7,25.5,-162.7,125.9,32.2,-172.3,130.1,33.9,172.2,130.7,34.7,161.5,128.8,38.1,154.0,123.3,35.8,152.1,115.3,32.2,148.1,107.5,27.1,154.0,109.7,10.5,163.8,110.6,-32.5,-173.3,105.7,-55.8,-174.5,91.2,-71.7,169.0,-41.6,-78.8,113.5,-101.8,-48.5,145.7,-153.5,-20.4,157.7,151.2,-17.1,148.6,119.5,1.2,126.9,113.1,-5.0,110.7,109.4,-5.4,98.5,122.7,14.9,92.9,143.8,26.7,95.2,167.5,28.3,93.0,};
_LoRa_CFG LoRa_CFG=
{
	.addr = LORA_ADDR,       //�豸��ַ
	.power = LORA_POWER,     //���书��
	.chn = LORA_CHN,         //�ŵ�
	.wlrate = LORA_RATE,     //��������
	.wltime = LORA_WLTIME,   //˯��ʱ��
	.mode = LORA_MODE,       //����ģʽ
	.mode_sta = LORA_STA,    //����״̬
	.bps = LORA_TTLBPS ,     //����������
	.parity = LORA_TTLPAR    //У��λ����
};

//ȫ�ֲ���
EXTI_InitTypeDef EXTI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

//�豸����ģʽ(���ڼ�¼�豸״̬)
u8 Lora_mode=0;// 0:����ģʽ 1:����ģʽ 2:����ģʽ
//��¼�ж�״̬
static u8 Int_mode=0;//0���ر� 1:������ 2:�½���

//AUX�ж�����
//mode:���õ�ģʽ 0:�ر� 1:������ 2:�½���
void Aux_Int(u8 mode)
{
    if(!mode)
	{
		EXTI_InitStructure.EXTI_LineCmd = DISABLE;//�ر�
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;	
	}else
	{
		if(mode==1)
			 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //������
	    else if(mode==2)
			 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//�½���
		
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	}
	Int_mode = mode;//��¼�ж�ģʽ
	EXTI_Init(&EXTI_InitStructure);
	NVIC_Init(&NVIC_InitStructure);  
    
}

//LORA_AUX�жϷ�����
void EXTI1_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line1))
	{  
	   if(Int_mode==1)//������(����:��ʼ�������� ����:���ݿ�ʼ���)     
	   {
		  if(Lora_mode==1)//����ģʽ
		  {
			 USART3_RX_STA=0;//���ݼ�����0
		  }
		  Int_mode=2;//�����½���
		  LED0=0;//DS0��
	   }
       else if(Int_mode==2)//�½���(����:�����ѷ����� ����:�����������)	
	   {
		  if(Lora_mode==1)//����ģʽ
		  {
			 USART3_RX_STA|=1<<15;//���ݼ���������
		  }else if(Lora_mode==2)//����ģʽ(�������ݷ������)
		  {
			 Lora_mode=1;//�������ģʽ
		  }
		  Int_mode=1;//����������
          LED0=1;//DS0��		   
	   }
       Aux_Int(Int_mode);//���������жϱ���
	   EXTI_ClearITPendingBit(EXTI_Line1); //���LINE4�ϵ��жϱ�־λ  
	}	
}

//LoRaģ���ʼ��
//����ֵ:0,���ɹ�
//       1,���ʧ��
u8 LoRa_Init(void)
{
	 u8 retry=0;
	 u8 temp=1;
	
	 GPIO_InitTypeDef  GPIO_InitStructure;
		
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��PB,PE�˿�ʱ��
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	    		 //LORA_MD0
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	 GPIO_Init(GPIOB, &GPIO_InitStructure);	  				 //������� ��IO���ٶ�Ϊ50MHz
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 //LORA_AUX
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		     //��������
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	 GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.4
	
	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource1);
	
	 EXTI_InitStructure.EXTI_Line=EXTI_Line1;
  	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //�����ش���
  	 EXTI_InitStructure.EXTI_LineCmd = DISABLE;              //�ж��߹ر�
  	 EXTI_Init(&EXTI_InitStructure);//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
	
	 NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;			//LORA_AUX
  	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;//��ռ���ȼ�2�� 
  	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;		//�����ȼ�3
  	 NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;		   //�ر��ⲿ�ж�ͨ��
   	 NVIC_Init(&NVIC_InitStructure); 
	 
	 LORA_MD0=0;
	 LORA_AUX=0;
	
	 while(LORA_AUX)//ȷ��LORAģ���ڿ���״̬��(LORA_AUX=0)
	 {
		 //Show_Str(40+30,50+20,200,16,"ģ����æ,���Ե�!!",16,0); 	
		 printf("ģ����æ,���Ե�!!");
		 delay_ms(500);
		 //Show_Str(40+30,50+20,200,16,"                    ",16,0);
         delay_ms(100);		 
	 }
	 usart3_init(115200);	         //��ʼ������2 
	 
	 LORA_MD0=1;//����ATģʽ
	 delay_ms(40);
	 retry=3;
	 while(retry--)
	 {
		 if(!lora_send_cmd("AT","OK",70))
		 {
			 temp=0;//���ɹ�
			 USART3_RX_STA=0;
			 break;
		 }	
	 }
	 if(retry==0) temp=1;//���ʧ��
	 return temp;
}

//Loraģ���������
void LoRa_Set(void)
{
	u8 sendbuf[20];
	u8 lora_addrh,lora_addrl=0;
	
	usart3_set(LORA_TTLBPS_115200,LORA_TTLPAR_8N1);//��������ģʽǰ����ͨ�Ų����ʺ�У��λ(115200 8λ���� 1λֹͣ ������У�飩
	usart3_rx(1);//��������3����
	
	while(LORA_AUX);//�ȴ�ģ�����
	LORA_MD0=1; //��������ģʽ
	delay_ms(40);
	Lora_mode=0;//���"����ģʽ"
	
	lora_addrh =  (LoRa_CFG.addr>>8)&0xff;
	lora_addrl = LoRa_CFG.addr&0xff;
	lora_send_cmd("AT","OK",70);
	sprintf((char*)sendbuf,"AT+ADDR=%02x,%02x",lora_addrh,lora_addrl);//�����豸��ַ
	lora_send_cmd(sendbuf,"OK",100);
	sprintf((char*)sendbuf,"AT+WLRATE=%d,%d",LoRa_CFG.chn,LoRa_CFG.wlrate);//�����ŵ��Ϳ�������
	lora_send_cmd(sendbuf,"OK",100);
	sprintf((char*)sendbuf,"AT+TPOWER=%d",LoRa_CFG.power);//���÷��书��
	lora_send_cmd(sendbuf,"OK",100);
	sprintf((char*)sendbuf,"AT+CWMODE=%d",LoRa_CFG.mode);//���ù���ģʽ
	lora_send_cmd(sendbuf,"OK",100);
	sprintf((char*)sendbuf,"AT+TMODE=%d",LoRa_CFG.mode_sta);//���÷���״̬
	lora_send_cmd(sendbuf,"OK",100);
	sprintf((char*)sendbuf,"AT+WLTIME=%d",LoRa_CFG.wltime);//����˯��ʱ��
	lora_send_cmd(sendbuf,"OK",100);
	sprintf((char*)sendbuf,"AT+UART=%d,%d",LoRa_CFG.bps,LoRa_CFG.parity);//���ô��ڲ����ʡ�����У��λ
	lora_send_cmd(sendbuf,"OK",100);

	LORA_MD0=0;//�˳�����,����ͨ��
	delay_ms(40);
	while(LORA_AUX);//�ж��Ƿ����(ģ����������ò���)
	USART3_RX_STA=0;
	Lora_mode=1;//���"����ģʽ"
	usart3_set(LoRa_CFG.bps,LoRa_CFG.parity);//����ͨ��,����ͨ�Ŵ�������(�����ʡ�����У��λ)
	Aux_Int(1);//����LORA_AUX�������ж�	
	
}

u8 Dire_Date[]={0x11,0x22,0x33,0x44,0x55};//����������
u8 date[30]={0};//��������
u8 Tran_Data[30]={0,1,2,3,4,5,6,7,8};//͸������

#define Dire_DateLen sizeof(Dire_Date)/sizeof(Dire_Date[0])
extern u32 obj_addr;//��¼�û�����Ŀ���ַ
extern u8 obj_chn;//��¼�û�����Ŀ���ŵ�

u8 wlcd_buff[10]={0}; //LCD��ʾ�ַ���������
//Loraģ�鷢������
void LoRa_SendData(void)
{      
	WPS++;
	obj_addr =0x01;
	obj_chn = 0x15; 
	static u8 num=0;
     u16 addr;
	 u8 chn;
	 u16 i=0; 
		printf("in the lora_senddata()\r\n");
	if(LoRa_CFG.mode_sta == LORA_STA_Tran)//͸������
	{
		printf("in LORA_STA_Tran\r\n");
		if(WPS==1){sprintf((char*)Tran_Data,"Height,LON,LAT");u3_printf("%s",Tran_Data);}
		else 
			{
						
					if(Distance_data/100>10) sprintf((char*)Tran_Data,"%-6.1f %-6.1f %-6.1f %1d.%2d,%u.%u,%u.%u",fakedata[faketime][0],fakedata[faketime][1],fakedata[faketime][2],Distance_data/100%10,Distance_data%100,aaa/100000,aaa%100000,bbb/100000,bbb%100000);
							else sprintf((char*)Tran_Data,"%-6.1f %-6.1f %-6.1f %1d.%2d,%u.%u,%u.%u",fakedata[faketime][0],fakedata[faketime][1],fakedata[faketime][2],Distance_data/100,Distance_data%100,aaa/100000,aaa%100000,bbb/100000,bbb%100000);
								u3_printf("%s",Tran_Data);
							faketime++;if(faketime==100) faketime=0;
			}
		//LCD_Fill(0,195,240,220,WHITE); //�����ʾ
		//Show_Str_Mid(10,195,Tran_Data,16,240);//��ʾ���͵�����
		printf("%s",Tran_Data);
		/*sprintf((char*)Tran_Data,"%u.%u,%u.%u",aaa/100000,aaa%100000,bbb/100000,bbb%100000);
		u3_printf("%s",Tran_Data);
		printf("\r\n%u.%u,%u.%u\r\n",aaa/100000,aaa%100000,bbb/100000,bbb%100000);*/
		num++;
		if(num==255) num=0;
		
	}else if(LoRa_CFG.mode_sta == LORA_STA_Dire)//������
	{
		
		addr = (u16)obj_addr;//Ŀ���ַ
		chn = obj_chn;//Ŀ���ŵ�
		
		date[i++] =(addr>>8)&0xff;//��λ��ַ
		date[i++] = addr&0xff;//��λ��ַ
		date[i] = chn;  //�����ŵ�
		for(i=0;i<Dire_DateLen;i++)//����д������BUFF
		{
			date[3+i] = Dire_Date[i];
		}
		for(i=0;i<(Dire_DateLen+3);i++)
		{
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);//ѭ������,ֱ���������   
			USART_SendData(USART2,date[i]); 
		}	
		
        //��ʮ�����Ƶ�����ת��Ϊ�ַ�����ӡ��lcd_buff����
		sprintf((char*)wlcd_buff,"%x %x %x %x %x %x %x %x",
				date[0],date[1],date[2],date[3],date[4],date[5],date[6],date[7]);
		
		//LCD_Fill(0,200,240,230,WHITE);//�����ʾ
		//Show_Str_Mid(10,200,wlcd_buff,16,240);//��ʾ���͵�����
		
	    Dire_Date[4]++;//Dire_Date[4]���ݸ���
		
	}
			
}
void LoRa_SendData1(void)
{      
	if(WPS==1) return;
	obj_addr =0x01;
	obj_chn = 0x15; 
	static u8 num=0;
     u16 addr;
	 u8 chn;
	 u16 i=0; 
		printf("in the lora_senddata()\r\n");
	if(LoRa_CFG.mode_sta == LORA_STA_Tran)//͸������
	{
		printf("in LORA_STA_Tran\r\n");
		if(WPS==1){sprintf((char*)Tran_Data,"Height");u3_printf("%s",Tran_Data);}
		else {sprintf((char*)Tran_Data,"%d.%d",Distance_data/100,Distance_data%100);u3_printf("%s",Tran_Data);}
		//LCD_Fill(0,195,240,220,WHITE); //�����ʾ
		//Show_Str_Mid(10,195,Tran_Data,16,240);//��ʾ���͵�����
		printf("%s",Tran_Data);
		sprintf((char*)Tran_Data,"%u.%u,%u.%u",aaa/100000,aaa%100000,bbb/100000,bbb%100000);
		u3_printf("%s",Tran_Data);
		printf("\r\n%s\r\n",Tran_Data);
		num++;
		if(num==255) num=0;
		
	}else if(LoRa_CFG.mode_sta == LORA_STA_Dire)//������
	{
		
		addr = (u16)obj_addr;//Ŀ���ַ
		chn = obj_chn;//Ŀ���ŵ�
		
		date[i++] =(addr>>8)&0xff;//��λ��ַ
		date[i++] = addr&0xff;//��λ��ַ
		date[i] = chn;  //�����ŵ�
		for(i=0;i<Dire_DateLen;i++)//����д������BUFF
		{
			date[3+i] = Dire_Date[i];
		}
		for(i=0;i<(Dire_DateLen+3);i++)
		{
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);//ѭ������,ֱ���������   
			USART_SendData(USART2,date[i]); 
		}	
		
        //��ʮ�����Ƶ�����ת��Ϊ�ַ�����ӡ��lcd_buff����
		sprintf((char*)wlcd_buff,"%x %x %x %x %x %x %x %x",
				date[0],date[1],date[2],date[3],date[4],date[5],date[6],date[7]);
		
		//LCD_Fill(0,200,240,230,WHITE);//�����ʾ
		//Show_Str_Mid(10,200,wlcd_buff,16,240);//��ʾ���͵�����
		
	    Dire_Date[4]++;//Dire_Date[4]���ݸ���
		
	}
			
}

u8 rlcd_buff[10]={0}; //LCD��ʾ�ַ���������
//Loraģ���������
void LoRa_ReceData(void)
{
    u16 i=0;
    u16 len=0;
   
	//����������
	if(USART3_RX_STA&0x8000)
	{
		len = USART3_RX_STA&0X7FFF;
		USART3_RX_BUF[len]=0;//��ӽ�����
		USART3_RX_STA=0;

		for(i=0;i<len;i++)
		{
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
			USART_SendData(USART1,USART3_RX_BUF[i]); 
		}
		 LCD_Fill(10,260,240,320,WHITE);
		if(LoRa_CFG.mode_sta==LORA_STA_Tran)//͸������
		{	
			//Show_Str_Mid(10,270,USART2_RX_BUF,16,240);//��ʾ���յ�������
			printf("%s",USART3_RX_BUF);

		}else if(LoRa_CFG.mode_sta==LORA_STA_Dire)//������
		{
			//��ʮ�����Ƶ�����ת��Ϊ�ַ�����ӡ��lcd_buff����
			sprintf((char*)rlcd_buff,"%x %x %x %x %x",
			USART3_RX_BUF[0],USART3_RX_BUF[1],USART3_RX_BUF[2],USART3_RX_BUF[3],USART3_RX_BUF[4]);
				
			//Show_Str_Mid(10,270,rlcd_buff,16,240);//��ʾ���յ�������	
		}
		memset((char*)USART3_RX_BUF,0x00,len);//���ڽ��ջ�������0

	}

}

//���ͺͽ��մ���
void LoRa_Process(void)
{
	u8 key=0;
	static u8 t=0;
		
 DATA:
	Process_ui();//������ʾ
	LoRa_Set();//LoRa����(�������������ô��ڲ�����Ϊ115200) 
	//while(1)
	//{
		
		//key = KEY_Scan(0);
		
		/*if(key==KEY0_PRES)
		{
			if(LoRa_CFG.mode_sta==LORA_STA_Dire)//���Ƕ�����,���������Ŀ���ַ���ŵ�����
			{
				usart2_rx(0);//�رմ��ڽ���
				Aux_Int(0);//�ر��ж�
				//Dire_Set();//��������Ŀ���ַ���ŵ�
				Diree_Set();
				goto DATA;
			}
		}else if(key==WKUP_PRES)//�������˵�ҳ��
		{
			LORA_MD0=1; //��������ģʽ
	        delay_ms(40);
			usart2_rx(0);//�رմ��ڽ���
			Aux_Int(0);//�ر��ж�
			break;
		}*/
		//else if(key==KEY1_PRES)//��������
		//{
							Diree_Set();
		
			printf("Ŀ���ַ:\r\nĿ���ŵ�:");
  		printf("%u\r\n%u\r\n",obj_addr,obj_chn);

			if(!LORA_AUX&&(LoRa_CFG.mode!=LORA_MODE_SLEEP))//�����ҷ�ʡ��ģʽ
			  {
				  Lora_mode=2;//���"����״̬"
				  LoRa_SendData();//��������    
			  }
		//}
			
		//���ݽ���
		LoRa_ReceData();
		
		t++;
		if(t==20)
		{
			t=0;
			LED1=~LED1;
		}			
		delay_ms(10);		
   //}
	
}

void LoRa_Process1(void)
{
	u8 key=0;
	static u8 t=0;
		
 DATA:
	Process_ui();//������ʾ
	LoRa_Set();//LoRa����(�������������ô��ڲ�����Ϊ115200) 
	//while(1)
	//{
		
		//key = KEY_Scan(0);
		
		/*if(key==KEY0_PRES)
		{
			if(LoRa_CFG.mode_sta==LORA_STA_Dire)//���Ƕ�����,���������Ŀ���ַ���ŵ�����
			{
				usart2_rx(0);//�رմ��ڽ���
				Aux_Int(0);//�ر��ж�
				//Dire_Set();//��������Ŀ���ַ���ŵ�
				Diree_Set();
				goto DATA;
			}
		}else if(key==WKUP_PRES)//�������˵�ҳ��
		{
			LORA_MD0=1; //��������ģʽ
	        delay_ms(40);
			usart2_rx(0);//�رմ��ڽ���
			Aux_Int(0);//�ر��ж�
			break;
		}*/
		//else if(key==KEY1_PRES)//��������
		//{
							Diree_Set();
		
			printf("Ŀ���ַ:\r\nĿ���ŵ�:");
  		printf("%u\r\n%u\r\n",obj_addr,obj_chn);

			if(!LORA_AUX&&(LoRa_CFG.mode!=LORA_MODE_SLEEP))//�����ҷ�ʡ��ģʽ
			  {
				  Lora_mode=2;//���"����״̬"
				  LoRa_SendData1();//��������    
			  }
		//}
			
		//���ݽ���
		LoRa_ReceData();
		
		t++;
		if(t==20)
		{
			t=0;
			LED1=~LED1;
		}			
		delay_ms(10);		
   //}
	
}
//�����Ժ���
void Lora_Test(void)
{
	u8 t=0;
	u8 key=0;
	u8 netpro=0;
	
	//LCD_Clear(WHITE);
	//POINT_COLOR=RED;
	//Show_Str_Mid(0,30,"ATK-LORA-01 ���Գ���",16,240); 
	
	while(LoRa_Init())//��ʼ��ATK-LORA-01ģ��
	{
		//Show_Str(40+30,50+20,200,16,"δ��⵽ģ��!!!",16,0); 
			printf("δ��⵽ģ��!!!");
		delay_ms(300);
		//Show_Str(40+30,50+20,200,16,"                ",16,0);
	}
	printf("��⵽ģ��!!!");
	//Show_Str(40+30,50+20,200,16,"��⵽ģ��!!!",16,0);
    delay_ms(500); 	
	//Menu_ui();//�˵�
	/*
	while(1)
	{
		
		key = KEY_Scan(0);
		if(key)
		{
			//Show_Str(30+10,95+45+netpro*25,200,16,"  ",16,0);//���֮ǰ����ʾ
			
			 if(key==KEY0_PRES)//KEY0����
			{
				if(netpro<6)netpro++;
				else netpro=0;
				printf("now is %d",netpro);
			}else if(key==KEY1_PRES)//KEY1����
			{
				if(netpro>0)netpro--;
				else netpro=6; 
				printf("now is %d",netpro);
			}else if(key==WKUP_PRES)//KEY_UP����
			{
				if(netpro==0)//����ͨ��ѡ��
				{
				  LoRa_Process();//��ʼ���ݲ���
				  netpro=0;//�������ص�0
				  Menu_ui();

				}else
				{
					//Show_Str(30+40,95+45+netpro*25+2,200,16,"________",16,1);//��ʾ�»���,��ʾѡ��
					//Show_Str(30+10,95+45+netpro*25,200,16,"��",16,0);//ָ������Ŀ
					//Menu_cfg(netpro);//��������
					//LCD_Fill(30+40,95+45+netpro*25+2+15,30+40+100,95+45+netpro*25+2+18,WHITE);//����»�����ʾ
				}
			}
			//Show_Str(30+10,95+45+netpro*25,200,16,"��",16,0);//ָ������Ŀ
		}
		
		t++;
		if(t==30)
		{
			t=0;
			LED1=~LED1;
		}
		delay_ms(10);
	}	*/
	
}
