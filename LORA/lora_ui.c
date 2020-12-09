#include "lora_ui.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"   	 
#include "key.h"	 	 	 	 	 
#include "lcd.h" 	  
#include "flash.h" 	 
#include "touch.h" 	 
#include "malloc.h"
#include "string.h"    
#include "text.h"		
#include "usart2.h" 
#include "ff.h"
#include "lora_cfg.h"
#include "lora_app.h"

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

//usmart֧�ֲ��� 
//���յ���ATָ��Ӧ�����ݷ��ظ����Դ���
//mode:0,������USART2_RX_STA;
//     1,����USART2_RX_STA;
void lora_at_response(u8 mode)
{
	if(USART3_RX_STA&0X8000)		//���յ�һ��������
	{ 
		USART3_RX_BUF[USART3_RX_STA&0X7FFF]=0;//��ӽ�����
		printf("%s",USART3_RX_BUF);	//���͵�����
		if(mode)USART3_RX_STA=0;
	} 
}

//lora���������,�����յ���Ӧ��
//str:�ڴ���Ӧ����
//����ֵ:0,û�еõ��ڴ���Ӧ����
//����,�ڴ�Ӧ������λ��(str��λ��)
u8* lora_check_cmd(u8 *str)
{
	char *strx=0;
	if(USART3_RX_STA&0X8000)		//���յ�һ��������
	{ 
		USART3_RX_BUF[USART3_RX_STA&0X7FFF]=0;//��ӽ�����
		strx=strstr((const char*)USART3_RX_BUF,(const char*)str);
	} 
	return (u8*)strx;
}
//lora��������
//cmd:���͵������ַ���(����Ҫ��ӻس���),��cmd<0XFF��ʱ��,��������(���緢��0X1A),���ڵ�ʱ�����ַ���.
//ack:�ڴ���Ӧ����,���Ϊ��,���ʾ����Ҫ�ȴ�Ӧ��
//waittime:�ȴ�ʱ��(��λ:10ms)
//����ֵ:0,���ͳɹ�(�õ����ڴ���Ӧ����)
//       1,����ʧ��
u8 lora_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
{
	u8 res=0; 
	USART3_RX_STA=0;
	if((u32)cmd<=0XFF)
	{
		while(DMA1_Channel2->CNDTR!=0);	//�ȴ�ͨ��7�������   
		USART2->DR=(u32)cmd;
	}else u3_printf("%s\r\n",cmd);//��������

	if(ack&&waittime)		//��Ҫ�ȴ�Ӧ��
	{
	   while(--waittime)	//�ȴ�����ʱ
	   { 
		  delay_ms(10);
		  if(USART3_RX_STA&0X8000)//���յ��ڴ���Ӧ����
		  {
			  if(lora_check_cmd(ack))
			  {
				 USART3_RX_STA=0;
				 break;//�õ���Ч���� 
			  }
		  } 
	   }
	   if(waittime==0)res=1; 
	}
	return res;
} 

//��1���ַ�ת��Ϊ16��������
//chr:�ַ�,0~9/A~F/a~F
//����ֵ:chr��Ӧ��16������ֵ
u8 chr2hex(u8 chr)
{
	if(chr>='0'&&chr<='9')return chr-'0';
	if(chr>='A'&&chr<='F')return (chr-'A'+10);
	if(chr>='a'&&chr<='f')return (chr-'a'+10); 
	return 0;
}

//�ַ�ת��16��������
//test[] �洢λ��
//buffer[] ��ת��������
//len�� ����
void char2hex_buff(u8 test[],u8 buff[],u8 len)
{
	u8 i=0;
	for(i=0;i<len;i++)
	{
 		test[i] = chr2hex(buff[i]);
	}
}

//��16��������ת��������
u32 hexbuff_num(u8 test[],u8 buff[],u8 len)
{
	u8 i=0;
	u32 num=0; 
	
	for(i=0;i<len;i++) test[i] = chr2hex(buff[i]);
    if(len==5)  num = test[0]*10000+test[1]*1000+test[2]*100+test[3]*10+test[4];
	else if(len==4) num = test[0]*1000+test[1]*100+test[2]*10+test[3];		
	else if(len==3) num = test[0]*100+test[1]*10+test[2];		
	else if(len==2) num = test[0]*10+test[1];
	else if(len==1) num = test[0];
	
	return num;
	
}

//��������λ��
//��:100,3λ����10,1λ��
u8 num_count(u32 num)
{
	u32 date=0;
	u8 count=0;
	
	date = num;
	do
	{
		count++;
		date/=10;
	}while(date>0);
	
	return count;
}

//�������
const u8* kbd_tbl1[11]={"1","2","3","4","5","6","7","8","9","0","DEL"};
u8** kbd_tbl;
u8* kbd_fn_tbl[2]={"ȷ��","����"};
//���ؼ��̽��棨�ߴ�Ϊ240*140��
//x,y:������ʼ���꣨320*240�ֱ��ʵ�ʱ��x����Ϊ0��
void lora_load_keyboard(u16 x,u16 y,u8 **kbtbl)
{
	u16 i;
	POINT_COLOR=RED;
	kbd_tbl=kbtbl;
	/*LCD_Fill(x,y,x+240,y+140,WHITE);
	LCD_DrawRectangle(x,y,x+240,y+140);						   	 
	LCD_DrawRectangle(x+80,y,x+160,y+112);	
	LCD_DrawRectangle(x,y+28,x+240,y+56);
	LCD_DrawRectangle(x,y+84,x+240,y+112);
	POINT_COLOR=BLUE;*/
	/*for(i=0;i<13;i++)
	{
		if(i<11)//Show_Str_Mid(x+(i%3)*80,y+6+28*(i/3),(u8*)kbd_tbl[i],16,80);
		else if(i==11);
			//Show_Str_Mid(x+(i%3)*80,y+6+28*(i/3),kbd_fn_tbl[i-11],16,80); 
		else;
		//	Show_Str_Mid(x+(i%3)*80+80,y+6+28*(i/3),kbd_fn_tbl[i-11],16,80); 
	}  		*/ 					   
}

//����״̬����
//x,y:��������
//key:��ֵ��0~15��
//sta:״̬��0���ɿ���1�����£�
void lora_key_staset(u16 x,u16 y,u8 keyx,u8 sta)
{		  
	/*u16 i=keyx/3,j=keyx%3;
	if(keyx>15)return;
	if(sta)
	{
		if(keyx>11) 
		   LCD_Fill(2,y+i*28+1,238,318,GREEN);
		else LCD_Fill(x+j*80+1,y+i*28+1,x+j*80+78,y+i*28+26,GREEN);
	}
	else
	{
		if(keyx>11)
			 LCD_Fill(2,y+i*28+1,238,318,WHITE);
		else LCD_Fill(x+j*80+1,y+i*28+1,x+j*80+78,y+i*28+26,WHITE);
	}	 
    if((i==3)&&(j==2))
		//Show_Str_Mid(x+j*80,y+6+28*i,(u8*)kbd_fn_tbl[0],16,80);
	else if((i==4)&&(j<3)) 
	  //  Show_Str_Mid(80,180+6+28*i,kbd_fn_tbl[1],16,80); 
    else 
       // Show_Str_Mid(x+j*80,y+6+28*i,(u8*)kbd_tbl[keyx],16,80);			
	*/
}

//�õ�������������
//x,y:��������
//����ֵ��������ֵ��1~15��Ч��0,��Ч��
static u8 key_x=0;//0,û���κΰ������£�1~15��1~15�Ű�������
u8 lora_get_keynum(u16 x,u16 y)
{
	u16 i,j;
	u8 key=0;
	//tp_dev.scan(0); 		 
	/*if(tp_dev.sta&TP_PRES_DOWN)	//������������
	{	
		for(i=0;i<5;i++)
		{
			for(j=0;j<3;j++)
			{
			 	if(tp_dev.x[0]<(x+j*80+80)&&tp_dev.x[0]>(x+j*80)&&tp_dev.y[0]<(y+i*28+28)&&tp_dev.y[0]>(y+i*28))
				{	
					key=i*3+j+1;	 
					break;	 		   
				}
			}
			if(key)
			{	   
				if(key_x==key)key=0;
				else 
				{
					lora_key_staset(x,y,key_x-1,0);
					key_x=key;
					lora_key_staset(x,y,key_x-1,1);
				}
				break;
			}
		}  
	}else if(key_x) 
	{
		lora_key_staset(x,y,key_x-1,0);
		key_x=0;
	} */
	return key; 
}

u8* cnttbl_mode[3]={"һ��ģʽ","����ģʽ","ʡ��ģʽ"};
u8* cnttbl_sta[2]={"͸������","������"};
u8* cnttbl_power[4]={"11dBm","14dBm","17dBm","20dBm"};
u8* cnttbl_rate[6]={"0.3kbps","1.2kbps","2.4kbps","4.8kbps","9.6kbps","19.2kbps"};

//������UI
void Menu_ui(void)
{
	u8 num=0;
	
	//LCD_Clear(WHITE);
	//POINT_COLOR=RED;
	printf("ATK-LORA-01 ���Գ���\r\n");
	printf("KEY0:��һ��\r\nKEY1:��һ��\r\nWK_UP:ȷ��/ȡ��\r\n");
	printf("����ͨ��\r\nģ���ַ\r\nͨ���ŵ���\r\n�������ʣ�\r\n���书�ʣ�\r\n����ģʽ��\r\n����״̬��\r\n");
	//Show_Str_Mid(0,30,"ATK-LORA-01 ���Գ���",16,240); 
	POINT_COLOR=BLUE;
	/*Show_Str(40+30,50+20,200,16,"KEY0:��һ��",16,0); 				    	 
	Show_Str(40+30,70+20,200,16,"KEY1:��һ��",16,0);				    	 
	Show_Str(40+30,90+20,200,16,"WK_UP:ȷ��/ȡ��",16,0); 
	POINT_COLOR=RED;
	Show_Str(40+30,45+95,200,16,"����ͨ��",16,0);
	Show_Str(40+30,70+95,200,16,"ģ���ַ��",16,0);
	Show_Str(40+30,95+95,200,16,"ͨ���ŵ���",16,0);
	Show_Str(40+30,120+95,200,16,"�������ʣ�",16,0);
	Show_Str(40+30,145+95,200,16,"���书�ʣ�",16,0);
	Show_Str(40+30,170+95,200,16,"����ģʽ��",16,0);
	Show_Str(40+30,195+95,200,16,"����״̬��",16,0);
	POINT_COLOR=BLUE;
	Show_Str(30+10,45+95,200,16,"��",16,0); */
	num = num_count(LoRa_CFG.addr);
	//LCD_ShowNum(40+110+10,70+95,LoRa_CFG.addr,num,16);
	num = num_count(LoRa_CFG.chn);
			printf("%u\r\n%u\r\n%s\r\n%s\r\n%s\r\n%s\r\n",LoRa_CFG.addr,LoRa_CFG.chn,(u8*)cnttbl_rate[LoRa_CFG.wlrate],(u8*)cnttbl_power[LoRa_CFG.power],(u8*)cnttbl_mode[LoRa_CFG.mode],(u8*)cnttbl_sta[LoRa_CFG.mode_sta]);
	printf("when the netpro is 0,process\r\n");
	/*LCD_ShowxNum(40+110+10,95+95,LoRa_CFG.chn,num,16,0);
	Show_Str(40+110+10,120+95,200,16,(u8*)cnttbl_rate[LoRa_CFG.wlrate],16,0);
	Show_Str(40+110+10,145+95,200,16,(u8*)cnttbl_power[LoRa_CFG.power],16,0);
	Show_Str(40+110+10,170+95,200,16,(u8*)cnttbl_mode[LoRa_CFG.mode],16,0);
	Show_Str(40+110+10,195+95,200,16,(u8*)cnttbl_sta[LoRa_CFG.mode_sta],16,0);*/
}

//�������������
void Menu_cfg(u8 num)
{
    u8 key=0;
	u8 t=0;
	u8 count=0;
	
	POINT_COLOR=BLUE;
    while(1)
    {
	  key = KEY_Scan(0);
	  
	  if(key==KEY1_PRES)//����+ 
	  {
		  switch(num)
		  {
			  case 1: LoRa_CFG.addr++; break;//��ַ
	          case 2: LoRa_CFG.chn++;//�ŵ�
			          if(LoRa_CFG.chn>31) LoRa_CFG.chn=0;
          			  break;
			  case 3: LoRa_CFG.wlrate++;//��������
			          if(LoRa_CFG.wlrate>5) LoRa_CFG.wlrate=0;
			          break;
			  case 4: LoRa_CFG.power++;//���书��
			          if(LoRa_CFG.power>3)  LoRa_CFG.power=0;
			          break;	   
			  case 5: LoRa_CFG.mode++;//����ģʽ
			          if(LoRa_CFG.mode>2) LoRa_CFG.mode=0;
         			  break;
              case 6: LoRa_CFG.mode_sta++;//����״̬
			          if(LoRa_CFG.mode_sta>1) LoRa_CFG.mode_sta=0;
			          break;		   
		  }
	  }else if(key==KEY0_PRES)//����-
	  {
		  switch(num)
		  {
			  case 1: LoRa_CFG.addr--; break;//��ַ
	          case 2: LoRa_CFG.chn--;//�ŵ�
			          if(LoRa_CFG.chn==255) LoRa_CFG.chn=31;
			          break;
			  case 3: LoRa_CFG.wlrate--;//�������� 
			         if(LoRa_CFG.wlrate==255) LoRa_CFG.wlrate=5;
			          break;
			  case 4: LoRa_CFG.power--;//���书��
			          if(LoRa_CFG.power==255)  LoRa_CFG.power=3;
        			  break;	   
			  case 5: LoRa_CFG.mode--;//����ģʽ
                      if(LoRa_CFG.mode==255) LoRa_CFG.mode=2; 			  
			          break;
              case 6: LoRa_CFG.mode_sta--;//����״̬
			          if(LoRa_CFG.mode_sta>1) LoRa_CFG.mode_sta=1;
        			  break;			   
		  }
	  }else if(key==WKUP_PRES)  break;//�˳�

	  
	  //������ʾ
	  if(key==KEY1_PRES||key==KEY0_PRES)
	  {
		  switch(num)
		  {
			  case 1: 
					count = num_count(LoRa_CFG.addr);
					//Show_Str(40+110+10,70+95,200,16,"                ",16,0);
					//LCD_ShowNum(40+110+10,70+95,LoRa_CFG.addr,count,16);
					break;
			 case 2: 
					count = num_count(LoRa_CFG.chn);
			       // Show_Str(40+110+10,95+95,200,16,"                ",16,0);
			        //LCD_ShowxNum(40+110+10,95+95,LoRa_CFG.chn,count,16,0);
					break;
			 case 3:
				  //  Show_Str(40+110+10,120+95,200,16,"             ",16,0);
				  //  Show_Str(40+110+10,120+95,200,16,(u8*)cnttbl_rate[LoRa_CFG.wlrate],16,0);
					break;
			 case 4:
				  //  Show_Str(40+110+10,145+95,200,16,(u8*)cnttbl_power[LoRa_CFG.power],16,0);
					break;
			 case 5: 
				   // Show_Str(40+110+10,170+95,200,16,(u8*)cnttbl_mode[LoRa_CFG.mode],16,0);
				    break;
			 case 6:
				    //Show_Str(40+110+10,195+95,200,16,(u8*)cnttbl_sta[LoRa_CFG.mode_sta],16,0);
				    break;       
		  }
	  }
	  t++;
	  if(t==30)
	  {
		  t=0;
		  LED1=~LED1;
	  }
	  delay_ms(10);   
  }
		
}


u32 obj_addr=0;//��¼Ŀ���ַ���û�ͨ��������������ʱ������99999,������Ҫ����u32���ͣ�
u8 obj_chn=0;//��¼Ŀ���ŵ�
const u8* set_mode[2]={"����Ŀ���ַ:","����Ŀ���ŵ�"};
u8 callbuf[5]={0};//�洢�û���������ֵ

//����Ŀ���ַ��Ŀ���ŵ�(������)
void Dire_Set(void)
{
	u8 t=0;
	u8 chn=0;
	u32 addr=0;
	u8 key=0;
	u8 test[5]={0};
	u8 addrnumlen; //��ַ����,�5����(65535) 
	u8 chnnumlen;  //�ŵ�����,�2����(31)
	u8 mode=1;//0:���˵�ҳ�� 1:����Ŀ���ַ 2:����Ŀ���ŵ�
	
	LCD_Clear(WHITE);
 RESTART:
	addrnumlen=0;
	chnnumlen=0;
    POINT_COLOR=BLACK;	
	//Show_Str(40,70,200,16,(u8*)set_mode[0],16,0);
    //Show_Str(40+106,70,200,16,"          ",16,0);  	
	lora_load_keyboard(0,180,(u8**)kbd_tbl1);
	while(1)
	{
		key = lora_get_keynum(0,180);//��ȡ��������ֵ
		
		if(key)
		{
			if(key<11)
			{
				if(mode==1)//����Ŀ���ַ��
				{   
					if(addrnumlen<5)
					{
						if(addrnumlen==1&&callbuf[0]=='0') callbuf[0]=kbd_tbl[key-1][0];
						else callbuf[addrnumlen++]=kbd_tbl[key-1][0];
					}
				}else if(mode==2)//����Ŀ���ŵ�
				{
					if(chnnumlen<2)
					{
						if(chnnumlen==1&&callbuf[0]=='0') callbuf[0]=kbd_tbl[key-1][0];
						else callbuf[chnnumlen++]=kbd_tbl[key-1][0];
					}
				}
			}
			else
			{
				if(key==11)//ɾ������ 
				{
					if(mode==1)//Ŀ���ַ����
					{
						if(addrnumlen) addrnumlen--;
					}else if(mode==2)//Ŀ���ŵ�����
					{
						if(chnnumlen) chnnumlen--;
					}
				}
				else if(key==12)//ȷ������ 
				{
					if(mode==1)//Ŀ���ַ����
					{
						if(addrnumlen)
						{
							addr = hexbuff_num(test,callbuf,addrnumlen);//�ַ�ת����
							if(addr>65535)//�趨��ַԽ��
							{
						//		POINT_COLOR=RED;
							//	Show_Str(40,100,200,16,"��ַ������Χ,���������ã�",16,0); 
								delay_ms(500);
								delay_ms(500);
							//	Show_Str(40,100,200,16,"                        ",16,0); 
								addrnumlen=0;
								POINT_COLOR=BLUE;
							}else //��������
							{
								obj_addr = addr;
								LCD_Fill(40+106,70,239,70+16,WHITE);
								mode=2;//����Ŀ���ŵ����ý���
								POINT_COLOR=BLACK;
							//	Show_Str(40,70,200,16,(u8*)set_mode[1],16,0); 
								POINT_COLOR=BLUE;
							}
						}else //Ŀ���ַû�趨
						{
							    POINT_COLOR=RED;
							   // Show_Str(40,100,200,16,"�����õ�ַ��",16,0); 
								delay_ms(500);
								delay_ms(500);
							//	Show_Str(40,100,200,16,"                        ",16,0); 
							  //  POINT_COLOR=BLUE;
						}
						
					}else if(mode==2)//Ŀ���ŵ�����
					{
						if(chnnumlen)
						{
							chn = hexbuff_num(test,callbuf,chnnumlen);//�ַ�ת����
							if(chn>31)
							{
								POINT_COLOR=RED;
							//	Show_Str(40,100,200,16,"�ŵ�������Χ,���������ã�",16,0); 
								delay_ms(500);
								delay_ms(200);
							//	Show_Str(40,100,200,16,"                        ",16,0); 
								chnnumlen=0;
							//	POINT_COLOR=BLUE;
							}else
							{
								//Ŀ���ŵ����óɹ�,�򷵻ز��Խ���
								obj_chn = chn;//�������õ�Ŀ���ŵ�
								break;
							}
						}else//Ŀ���ŵ�û�趨
						{
							 //   POINT_COLOR=RED;
							 //   Show_Str(40,100,200,16,"�������ŵ���",16,0); 
								delay_ms(500);
								delay_ms(200);
							//	Show_Str(40,100,200,16,"                        ",16,0); 
							    POINT_COLOR=BLUE;
						}
					}
				}else //���ذ���
				{
					mode--;//�����ϼ�
					if(mode==1) goto RESTART;//����Ŀ���ַ����ҳ��
					if(mode==0) break; //�������˵�
				}
				
			 }
			    //���ݸ�����ʾ
	            if(mode==1) callbuf[addrnumlen]=0; 
			    else if(mode==2) callbuf[chnnumlen]=0; 
			//	LCD_Fill(40+106,70,239,70+16,WHITE);
			//	Show_Str(40+106,70,200,16,callbuf,16,0);  
		}
		t++;
		if(t==30)
		{
			t=0;
			LED1=~LED1;
		}
		delay_ms(10);
	}
	
		
}

//���Խ���UI
void Process_ui(void)
{
	u8 num=0;
	
	//LCD_Clear(WHITE);
	//POINT_COLOR=BLACK;
	printf("ģ���ַ\r\nͨ���ŵ���\r\n�������ʣ�\r\n���书�ʣ�\r\n����ģʽ��\r\n����״̬��\r\n");
	/*Show_Str(10,20,200,12,"ģ���ַ��",12,0);
	Show_Str(10,40,200,12,"ͨ���ŵ���",12,0);
	Show_Str(10,60,200,12,"�������ʣ�",12,0);
	Show_Str(10,80,200,12,"���书�ʣ�",12,0);
	Show_Str(10,100,200,12,"����ģʽ��",12,0);
	Show_Str(10,120,200,12,"����״̬��",12,0);
	POINT_COLOR=RED;*/
	num = num_count(LoRa_CFG.addr);
	//LCD_ShowNum(10+60,20,LoRa_CFG.addr,num,12);
	num = num_count(LoRa_CFG.chn);
	/*LCD_ShowxNum(10+60,40,LoRa_CFG.chn,num,12,0);
	Show_Str(10+60,60,200,12,(u8*)cnttbl_rate[LoRa_CFG.wlrate],12,0);
	Show_Str(10+60,80,200,12,(u8*)cnttbl_power[LoRa_CFG.power],12,0);
	Show_Str(10+60,100,200,12,(u8*)cnttbl_mode[LoRa_CFG.mode],12,0);
	Show_Str(10+60,120,200,12,(u8*)cnttbl_sta[LoRa_CFG.mode_sta],12,0);
	POINT_COLOR=BLUE;*/
			printf("%u\r\n%u\r\n%s\r\n%s\r\n%s\r\n%s\r\n",LoRa_CFG.addr,LoRa_CFG.chn,(u8*)cnttbl_rate[LoRa_CFG.wlrate],(u8*)cnttbl_power[LoRa_CFG.power],(u8*)cnttbl_mode[LoRa_CFG.mode],(u8*)cnttbl_sta[LoRa_CFG.mode_sta]);

	/*Show_Str(10,150,200,16,"���ݷ��ͣ�",16,0);
	Show_Str(10,240,200,16,"���ݽ��գ�",16,0);
	Show_Str(130+15,20,200,12,"KEY_UP: ���˵�",12,0);
	Show_Str(130+15,40,200,12,"KEY1: ��������",12,0);*/
		
		//printf("���ݷ��ͣ�\r\n���ݽ��գ�\r\nKEY_UP: ���˵�\r\nKEY1: ��������\r\n");

	if(LoRa_CFG.mode_sta==LORA_STA_Dire)//���Ƕ�����
	{
	  //Show_Str(130+15,60,200,12,"KEY0: ���ò���",12,0);
		printf("KEY0: ���ò���");
	  //POINT_COLOR=BLACK;
	  //Show_Str(130+15,80,200,12,"Ŀ���ַ:",12,0);
      //Show_Str(130+15,100,200,12,"Ŀ���ŵ�:",12,0);
		printf("Ŀ���ַ:\r\nĿ���ŵ�:");
	  //POINT_COLOR=RED;
	  num = num_count(obj_addr);
     // Show_Str(130+15+55,80,200,12,"                ",12,0);//���֮ǰ����ʾ
	  //LCD_ShowNum(130+15+55,80,obj_addr,num,12);  
	  num = num_count(obj_chn);
      //LCD_ShowxNum(130+15+55,100,obj_chn,num,12,0);		
		printf("%u\r\n%u\r\n",obj_addr,obj_chn);
		
	}
	
}
void Diree_Set(void)
{
	u8 t=0;
	obj_addr =0x01;
	obj_chn = 0x15;//�������õ�Ŀ���ŵ�23;
	printf("Ŀ���ַ:\r\nĿ���ŵ�:");
  		printf("%u\r\n%u\r\n",obj_addr,obj_chn);
}
