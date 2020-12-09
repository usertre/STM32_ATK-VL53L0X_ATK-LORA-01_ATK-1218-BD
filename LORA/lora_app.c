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
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//ATK-LORA-01模块功能驱动		  
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2016/4/1
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved	
//********************************************************************************
//无

//设备参数初始化(具体设备参数见lora_cfg.h定义)
unsigned long long WPS=0,faketime=0;
float fakedata[100][3]={-35.9,-53.5,-121.8,-35.1,-53.7,-118.3,-37.8,-63.8,-112.4,22.4,-66.9,-42.9,-1.9,-48.4,-66.6,-28.6,-31.1,-90.7,-41.4,-35.7,-112.7,-66.3,-33.1,-153.6,-84.9,-31.6,175.2,-117.6,-37.9,136.1,-163.4,-33.5,103.3,156.7,-21.1,75.2,134.0,-8.4,53.7,118.5,-6.5,45.2,104.4,-17.8,39.7,88.8,-30.7,26.5,50.4,-46.8,-12.7,1.4,-54.9,-44.7,-6.6,-63.8,-59.3,-131.2,-65.0,178.3,-174.2,-31.4,137.6,159.6,-16.2,138.6,161.4,-3.1,135.8,166.2,9.2,125.2,169.0,13.3,114.0,162.7,-1.6,104.1,149.7,-27.6,97.4,134.0,-60.0,102.1,49.5,-73.2,57.1,-15.9,-48.2,31.3,-29.4,-21.0,9.6,-28.5,-4.1,-16.2,-29.3,0.0,-42.1,-32.3,-1.2,-52.5,-28.7,-5.6,-52.7,-24.5,-13.6,-44.5,-13.5,-16.8,-32.5,4.0,-13.4,-25.2,4.5,-15.5,-24.4,17.4,-18.5,-17.4,23.4,-17.6,-4.9,30.1,-31.0,9.4,13.2,-37.9,0.9,14.7,-55.0,-18.6,17.8,-52.7,-19.7,50.7,-44.0,5.6,60.9,-27.3,6.6,89.8,-38.1,14.5,129.8,-64.4,54.8,16.9,-78.6,-54.7,49.7,-40.6,-20.5,-3.9,-21.7,-42.2,-18.1,-36.2,-57.1,-38.1,-37.4,-88.4,-49.8,-36.3,-121.6,-55.5,-16.1,-143.1,-44.0,17.4,-166.9,-6.4,34.7,170.1,8.6,34.0,166.5,2.4,21.6,176.7,-22.2,-8.9,-169.7,-27.6,-31.7,-174.0,-54.3,-31.8,-179.2,-67.0,-26.1,168.9,-67.2,-12.9,145.1,-76.8,-2.4,125.4,-80.1,5.6,114.8,-77.0,4.2,109.7,-67.0,-2.3,80.6,-73.0,-20.7,51.1,-92.2,-48.6,24.7,-176.7,-67.4,-55.5,102.1,-48.9,-120.4,103.6,-20.7,-113.7,110.3,3.6,-113.4,114.4,22.9,-120.2,111.5,18.6,-120.3,115.2,18.2,-138.6,118.7,25.5,-162.7,125.9,32.2,-172.3,130.1,33.9,172.2,130.7,34.7,161.5,128.8,38.1,154.0,123.3,35.8,152.1,115.3,32.2,148.1,107.5,27.1,154.0,109.7,10.5,163.8,110.6,-32.5,-173.3,105.7,-55.8,-174.5,91.2,-71.7,169.0,-41.6,-78.8,113.5,-101.8,-48.5,145.7,-153.5,-20.4,157.7,151.2,-17.1,148.6,119.5,1.2,126.9,113.1,-5.0,110.7,109.4,-5.4,98.5,122.7,14.9,92.9,143.8,26.7,95.2,167.5,28.3,93.0,};
_LoRa_CFG LoRa_CFG=
{
	.addr = LORA_ADDR,       //设备地址
	.power = LORA_POWER,     //发射功率
	.chn = LORA_CHN,         //信道
	.wlrate = LORA_RATE,     //空中速率
	.wltime = LORA_WLTIME,   //睡眠时间
	.mode = LORA_MODE,       //工作模式
	.mode_sta = LORA_STA,    //发送状态
	.bps = LORA_TTLBPS ,     //波特率设置
	.parity = LORA_TTLPAR    //校验位设置
};

//全局参数
EXTI_InitTypeDef EXTI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

//设备工作模式(用于记录设备状态)
u8 Lora_mode=0;// 0:配置模式 1:接收模式 2:发送模式
//记录中断状态
static u8 Int_mode=0;//0：关闭 1:上升沿 2:下降沿

//AUX中断设置
//mode:配置的模式 0:关闭 1:上升沿 2:下降沿
void Aux_Int(u8 mode)
{
    if(!mode)
	{
		EXTI_InitStructure.EXTI_LineCmd = DISABLE;//关闭
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;	
	}else
	{
		if(mode==1)
			 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿
	    else if(mode==2)
			 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿
		
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	}
	Int_mode = mode;//记录中断模式
	EXTI_Init(&EXTI_InitStructure);
	NVIC_Init(&NVIC_InitStructure);  
    
}

//LORA_AUX中断服务函数
void EXTI1_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line1))
	{  
	   if(Int_mode==1)//上升沿(发送:开始发送数据 接收:数据开始输出)     
	   {
		  if(Lora_mode==1)//接收模式
		  {
			 USART3_RX_STA=0;//数据计数清0
		  }
		  Int_mode=2;//设置下降沿
		  LED0=0;//DS0亮
	   }
       else if(Int_mode==2)//下降沿(发送:数据已发送完 接收:数据输出结束)	
	   {
		  if(Lora_mode==1)//接收模式
		  {
			 USART3_RX_STA|=1<<15;//数据计数标记完成
		  }else if(Lora_mode==2)//发送模式(串口数据发送完毕)
		  {
			 Lora_mode=1;//进入接收模式
		  }
		  Int_mode=1;//设置上升沿
          LED0=1;//DS0灭		   
	   }
       Aux_Int(Int_mode);//重新设置中断边沿
	   EXTI_ClearITPendingBit(EXTI_Line1); //清除LINE4上的中断标志位  
	}	
}

//LoRa模块初始化
//返回值:0,检测成功
//       1,检测失败
u8 LoRa_Init(void)
{
	 u8 retry=0;
	 u8 temp=1;
	
	 GPIO_InitTypeDef  GPIO_InitStructure;
		
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PB,PE端口时钟
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	    		 //LORA_MD0
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	 GPIO_Init(GPIOB, &GPIO_InitStructure);	  				 //推挽输出 ，IO口速度为50MHz
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 //LORA_AUX
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		     //下拉输入
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	 GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA.4
	
	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource1);
	
	 EXTI_InitStructure.EXTI_Line=EXTI_Line1;
  	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //上升沿触发
  	 EXTI_InitStructure.EXTI_LineCmd = DISABLE;              //中断线关闭
  	 EXTI_Init(&EXTI_InitStructure);//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
	
	 NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;			//LORA_AUX
  	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;//抢占优先级2， 
  	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;		//子优先级3
  	 NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;		   //关闭外部中断通道
   	 NVIC_Init(&NVIC_InitStructure); 
	 
	 LORA_MD0=0;
	 LORA_AUX=0;
	
	 while(LORA_AUX)//确保LORA模块在空闲状态下(LORA_AUX=0)
	 {
		 //Show_Str(40+30,50+20,200,16,"模块正忙,请稍等!!",16,0); 	
		 printf("模块正忙,请稍等!!");
		 delay_ms(500);
		 //Show_Str(40+30,50+20,200,16,"                    ",16,0);
         delay_ms(100);		 
	 }
	 usart3_init(115200);	         //初始化串口2 
	 
	 LORA_MD0=1;//进入AT模式
	 delay_ms(40);
	 retry=3;
	 while(retry--)
	 {
		 if(!lora_send_cmd("AT","OK",70))
		 {
			 temp=0;//检测成功
			 USART3_RX_STA=0;
			 break;
		 }	
	 }
	 if(retry==0) temp=1;//检测失败
	 return temp;
}

//Lora模块参数配置
void LoRa_Set(void)
{
	u8 sendbuf[20];
	u8 lora_addrh,lora_addrl=0;
	
	usart3_set(LORA_TTLBPS_115200,LORA_TTLPAR_8N1);//进入配置模式前设置通信波特率和校验位(115200 8位数据 1位停止 无数据校验）
	usart3_rx(1);//开启串口3接收
	
	while(LORA_AUX);//等待模块空闲
	LORA_MD0=1; //进入配置模式
	delay_ms(40);
	Lora_mode=0;//标记"配置模式"
	
	lora_addrh =  (LoRa_CFG.addr>>8)&0xff;
	lora_addrl = LoRa_CFG.addr&0xff;
	lora_send_cmd("AT","OK",70);
	sprintf((char*)sendbuf,"AT+ADDR=%02x,%02x",lora_addrh,lora_addrl);//设置设备地址
	lora_send_cmd(sendbuf,"OK",100);
	sprintf((char*)sendbuf,"AT+WLRATE=%d,%d",LoRa_CFG.chn,LoRa_CFG.wlrate);//设置信道和空中速率
	lora_send_cmd(sendbuf,"OK",100);
	sprintf((char*)sendbuf,"AT+TPOWER=%d",LoRa_CFG.power);//设置发射功率
	lora_send_cmd(sendbuf,"OK",100);
	sprintf((char*)sendbuf,"AT+CWMODE=%d",LoRa_CFG.mode);//设置工作模式
	lora_send_cmd(sendbuf,"OK",100);
	sprintf((char*)sendbuf,"AT+TMODE=%d",LoRa_CFG.mode_sta);//设置发送状态
	lora_send_cmd(sendbuf,"OK",100);
	sprintf((char*)sendbuf,"AT+WLTIME=%d",LoRa_CFG.wltime);//设置睡眠时间
	lora_send_cmd(sendbuf,"OK",100);
	sprintf((char*)sendbuf,"AT+UART=%d,%d",LoRa_CFG.bps,LoRa_CFG.parity);//设置串口波特率、数据校验位
	lora_send_cmd(sendbuf,"OK",100);

	LORA_MD0=0;//退出配置,进入通信
	delay_ms(40);
	while(LORA_AUX);//判断是否空闲(模块会重新配置参数)
	USART3_RX_STA=0;
	Lora_mode=1;//标记"接收模式"
	usart3_set(LoRa_CFG.bps,LoRa_CFG.parity);//返回通信,更新通信串口配置(波特率、数据校验位)
	Aux_Int(1);//设置LORA_AUX上升沿中断	
	
}

u8 Dire_Date[]={0x11,0x22,0x33,0x44,0x55};//定向传输数据
u8 date[30]={0};//定向数组
u8 Tran_Data[30]={0,1,2,3,4,5,6,7,8};//透传数组

#define Dire_DateLen sizeof(Dire_Date)/sizeof(Dire_Date[0])
extern u32 obj_addr;//记录用户输入目标地址
extern u8 obj_chn;//记录用户输入目标信道

u8 wlcd_buff[10]={0}; //LCD显示字符串缓冲区
//Lora模块发送数据
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
	if(LoRa_CFG.mode_sta == LORA_STA_Tran)//透明传输
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
		//LCD_Fill(0,195,240,220,WHITE); //清除显示
		//Show_Str_Mid(10,195,Tran_Data,16,240);//显示发送的数据
		printf("%s",Tran_Data);
		/*sprintf((char*)Tran_Data,"%u.%u,%u.%u",aaa/100000,aaa%100000,bbb/100000,bbb%100000);
		u3_printf("%s",Tran_Data);
		printf("\r\n%u.%u,%u.%u\r\n",aaa/100000,aaa%100000,bbb/100000,bbb%100000);*/
		num++;
		if(num==255) num=0;
		
	}else if(LoRa_CFG.mode_sta == LORA_STA_Dire)//定向传输
	{
		
		addr = (u16)obj_addr;//目标地址
		chn = obj_chn;//目标信道
		
		date[i++] =(addr>>8)&0xff;//高位地址
		date[i++] = addr&0xff;//低位地址
		date[i] = chn;  //无线信道
		for(i=0;i<Dire_DateLen;i++)//数据写到发送BUFF
		{
			date[3+i] = Dire_Date[i];
		}
		for(i=0;i<(Dire_DateLen+3);i++)
		{
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);//循环发送,直到发送完毕   
			USART_SendData(USART2,date[i]); 
		}	
		
        //将十六进制的数据转化为字符串打印在lcd_buff数组
		sprintf((char*)wlcd_buff,"%x %x %x %x %x %x %x %x",
				date[0],date[1],date[2],date[3],date[4],date[5],date[6],date[7]);
		
		//LCD_Fill(0,200,240,230,WHITE);//清除显示
		//Show_Str_Mid(10,200,wlcd_buff,16,240);//显示发送的数据
		
	    Dire_Date[4]++;//Dire_Date[4]数据更新
		
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
	if(LoRa_CFG.mode_sta == LORA_STA_Tran)//透明传输
	{
		printf("in LORA_STA_Tran\r\n");
		if(WPS==1){sprintf((char*)Tran_Data,"Height");u3_printf("%s",Tran_Data);}
		else {sprintf((char*)Tran_Data,"%d.%d",Distance_data/100,Distance_data%100);u3_printf("%s",Tran_Data);}
		//LCD_Fill(0,195,240,220,WHITE); //清除显示
		//Show_Str_Mid(10,195,Tran_Data,16,240);//显示发送的数据
		printf("%s",Tran_Data);
		sprintf((char*)Tran_Data,"%u.%u,%u.%u",aaa/100000,aaa%100000,bbb/100000,bbb%100000);
		u3_printf("%s",Tran_Data);
		printf("\r\n%s\r\n",Tran_Data);
		num++;
		if(num==255) num=0;
		
	}else if(LoRa_CFG.mode_sta == LORA_STA_Dire)//定向传输
	{
		
		addr = (u16)obj_addr;//目标地址
		chn = obj_chn;//目标信道
		
		date[i++] =(addr>>8)&0xff;//高位地址
		date[i++] = addr&0xff;//低位地址
		date[i] = chn;  //无线信道
		for(i=0;i<Dire_DateLen;i++)//数据写到发送BUFF
		{
			date[3+i] = Dire_Date[i];
		}
		for(i=0;i<(Dire_DateLen+3);i++)
		{
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);//循环发送,直到发送完毕   
			USART_SendData(USART2,date[i]); 
		}	
		
        //将十六进制的数据转化为字符串打印在lcd_buff数组
		sprintf((char*)wlcd_buff,"%x %x %x %x %x %x %x %x",
				date[0],date[1],date[2],date[3],date[4],date[5],date[6],date[7]);
		
		//LCD_Fill(0,200,240,230,WHITE);//清除显示
		//Show_Str_Mid(10,200,wlcd_buff,16,240);//显示发送的数据
		
	    Dire_Date[4]++;//Dire_Date[4]数据更新
		
	}
			
}

u8 rlcd_buff[10]={0}; //LCD显示字符串缓冲区
//Lora模块接收数据
void LoRa_ReceData(void)
{
    u16 i=0;
    u16 len=0;
   
	//有数据来了
	if(USART3_RX_STA&0x8000)
	{
		len = USART3_RX_STA&0X7FFF;
		USART3_RX_BUF[len]=0;//添加结束符
		USART3_RX_STA=0;

		for(i=0;i<len;i++)
		{
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
			USART_SendData(USART1,USART3_RX_BUF[i]); 
		}
		 LCD_Fill(10,260,240,320,WHITE);
		if(LoRa_CFG.mode_sta==LORA_STA_Tran)//透明传输
		{	
			//Show_Str_Mid(10,270,USART2_RX_BUF,16,240);//显示接收到的数据
			printf("%s",USART3_RX_BUF);

		}else if(LoRa_CFG.mode_sta==LORA_STA_Dire)//定向传输
		{
			//将十六进制的数据转化为字符串打印在lcd_buff数组
			sprintf((char*)rlcd_buff,"%x %x %x %x %x",
			USART3_RX_BUF[0],USART3_RX_BUF[1],USART3_RX_BUF[2],USART3_RX_BUF[3],USART3_RX_BUF[4]);
				
			//Show_Str_Mid(10,270,rlcd_buff,16,240);//显示接收到的数据	
		}
		memset((char*)USART3_RX_BUF,0x00,len);//串口接收缓冲区清0

	}

}

//发送和接收处理
void LoRa_Process(void)
{
	u8 key=0;
	static u8 t=0;
		
 DATA:
	Process_ui();//界面显示
	LoRa_Set();//LoRa配置(进入配置需设置串口波特率为115200) 
	//while(1)
	//{
		
		//key = KEY_Scan(0);
		
		/*if(key==KEY0_PRES)
		{
			if(LoRa_CFG.mode_sta==LORA_STA_Dire)//若是定向传输,则进入配置目标地址和信道界面
			{
				usart2_rx(0);//关闭串口接收
				Aux_Int(0);//关闭中断
				//Dire_Set();//进入设置目标地址和信道
				Diree_Set();
				goto DATA;
			}
		}else if(key==WKUP_PRES)//返回主菜单页面
		{
			LORA_MD0=1; //进入配置模式
	        delay_ms(40);
			usart2_rx(0);//关闭串口接收
			Aux_Int(0);//关闭中断
			break;
		}*/
		//else if(key==KEY1_PRES)//发送数据
		//{
							Diree_Set();
		
			printf("目标地址:\r\n目标信道:");
  		printf("%u\r\n%u\r\n",obj_addr,obj_chn);

			if(!LORA_AUX&&(LoRa_CFG.mode!=LORA_MODE_SLEEP))//空闲且非省电模式
			  {
				  Lora_mode=2;//标记"发送状态"
				  LoRa_SendData();//发送数据    
			  }
		//}
			
		//数据接收
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
	Process_ui();//界面显示
	LoRa_Set();//LoRa配置(进入配置需设置串口波特率为115200) 
	//while(1)
	//{
		
		//key = KEY_Scan(0);
		
		/*if(key==KEY0_PRES)
		{
			if(LoRa_CFG.mode_sta==LORA_STA_Dire)//若是定向传输,则进入配置目标地址和信道界面
			{
				usart2_rx(0);//关闭串口接收
				Aux_Int(0);//关闭中断
				//Dire_Set();//进入设置目标地址和信道
				Diree_Set();
				goto DATA;
			}
		}else if(key==WKUP_PRES)//返回主菜单页面
		{
			LORA_MD0=1; //进入配置模式
	        delay_ms(40);
			usart2_rx(0);//关闭串口接收
			Aux_Int(0);//关闭中断
			break;
		}*/
		//else if(key==KEY1_PRES)//发送数据
		//{
							Diree_Set();
		
			printf("目标地址:\r\n目标信道:");
  		printf("%u\r\n%u\r\n",obj_addr,obj_chn);

			if(!LORA_AUX&&(LoRa_CFG.mode!=LORA_MODE_SLEEP))//空闲且非省电模式
			  {
				  Lora_mode=2;//标记"发送状态"
				  LoRa_SendData1();//发送数据    
			  }
		//}
			
		//数据接收
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
//主测试函数
void Lora_Test(void)
{
	u8 t=0;
	u8 key=0;
	u8 netpro=0;
	
	//LCD_Clear(WHITE);
	//POINT_COLOR=RED;
	//Show_Str_Mid(0,30,"ATK-LORA-01 测试程序",16,240); 
	
	while(LoRa_Init())//初始化ATK-LORA-01模块
	{
		//Show_Str(40+30,50+20,200,16,"未检测到模块!!!",16,0); 
			printf("未检测到模块!!!");
		delay_ms(300);
		//Show_Str(40+30,50+20,200,16,"                ",16,0);
	}
	printf("检测到模块!!!");
	//Show_Str(40+30,50+20,200,16,"检测到模块!!!",16,0);
    delay_ms(500); 	
	//Menu_ui();//菜单
	/*
	while(1)
	{
		
		key = KEY_Scan(0);
		if(key)
		{
			//Show_Str(30+10,95+45+netpro*25,200,16,"  ",16,0);//清空之前的显示
			
			 if(key==KEY0_PRES)//KEY0按下
			{
				if(netpro<6)netpro++;
				else netpro=0;
				printf("now is %d",netpro);
			}else if(key==KEY1_PRES)//KEY1按下
			{
				if(netpro>0)netpro--;
				else netpro=6; 
				printf("now is %d",netpro);
			}else if(key==WKUP_PRES)//KEY_UP按下
			{
				if(netpro==0)//进入通信选项
				{
				  LoRa_Process();//开始数据测试
				  netpro=0;//索引返回第0
				  Menu_ui();

				}else
				{
					//Show_Str(30+40,95+45+netpro*25+2,200,16,"________",16,1);//显示下划线,表示选中
					//Show_Str(30+10,95+45+netpro*25,200,16,"→",16,0);//指向新条目
					//Menu_cfg(netpro);//参数配置
					//LCD_Fill(30+40,95+45+netpro*25+2+15,30+40+100,95+45+netpro*25+2+18,WHITE);//清除下划线显示
				}
			}
			//Show_Str(30+10,95+45+netpro*25,200,16,"→",16,0);//指向新条目
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
