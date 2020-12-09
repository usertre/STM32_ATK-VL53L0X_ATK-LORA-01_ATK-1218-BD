#include "vl53l0x.h"
#include "lora_app.h"
#include "lora_ui.h"
#include "gps.h"
#include "usart4.h"




#include "usart5.h"
#include "imu9011.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK MiniV3 STM32������
//VL53L0X-���ܲ��� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2017/7/1
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

VL53L0X_Dev_t vl53l0x_dev;//�豸I2C���ݲ���
VL53L0X_DeviceInfo_t vl53l0x_dev_info;//�豸ID�汾��Ϣ
uint8_t AjustOK=0;//У׼��־λ

//VL53L0X������ģʽ����
//0��Ĭ��;1:�߾���;2:������;3:����
mode_data Mode_data[]=
{
    {(FixPoint1616_t)(0.25*65536), 
	 (FixPoint1616_t)(18*65536),
	 33000,
	 14,
	 10},//Ĭ��
		
	{(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(18*65536),
	 200000, 
	 14,
	 10},//�߾���
		
    {(FixPoint1616_t)(0.1*65536) ,
	 (FixPoint1616_t)(60*65536),
	 33000,
	 18,
	 14},//������
	
    {(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(32*65536),
	 20000,
	 14,
	 10},//����
		
};

//API������Ϣ��ӡ
//Status�����鿴VL53L0X_Error�����Ķ���
void print_pal_error(VL53L0X_Error Status)
{
	
	char buf[VL53L0X_MAX_STRING_LENGTH];
	
	VL53L0X_GetPalErrorString(Status,buf);//����Status״̬��ȡ������Ϣ�ַ���
	
    printf("API Status: %i : %s\r\n",Status, buf);//��ӡ״̬�ʹ�����Ϣ
	
}

//ģʽ�ַ�����ʾ
//mode:0-Ĭ��;1-�߾���;2-������;3-����
void mode_string(u8 mode,char *buf)
{
	switch(mode)
	{
		case Default_Mode: strcpy(buf,"Default");        break;
		case HIGH_ACCURACY: strcpy(buf,"High Accuracy"); break;
		case LONG_RANGE: strcpy(buf,"Long Range");       break;
		case HIGH_SPEED: strcpy(buf,"High Speed");       break;
	}

}

//����VL53L0X�豸I2C��ַ
//dev:�豸I2C�����ṹ��
//newaddr:�豸��I2C��ַ
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev,uint8_t newaddr)
{
	uint16_t Id;
	uint8_t FinalAddress;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	u8 sta=0x00;
	
	FinalAddress = newaddr;
	
	if(FinalAddress==dev->I2cDevAddr)//���豸I2C��ַ��ɵ�ַһ��,ֱ���˳�
		return VL53L0X_ERROR_NONE;
	//�ڽ��е�һ���Ĵ�������֮ǰ����I2C��׼ģʽ(400Khz)
	Status = VL53L0X_WrByte(dev,0x88,0x00);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		sta=0x01;//����I2C��׼ģʽ����
		goto set_error;
	}
	//����ʹ��Ĭ�ϵ�0x52��ַ��ȡһ���Ĵ���
	Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		sta=0x02;//��ȡ�Ĵ�������
		goto set_error;
	}
	if(Id == 0xEEAA)
	{
		//�����豸�µ�I2C��ַ
		Status = VL53L0X_SetDeviceAddress(dev,FinalAddress);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x03;//����I2C��ַ����
			goto set_error;
		}
		//�޸Ĳ����ṹ���I2C��ַ
		dev->I2cDevAddr = FinalAddress;
		//����µ�I2C��ַ��д�Ƿ�����
		Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x04;//��I2C��ַ��д����
			goto set_error;
		}	
	}
	set_error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
		print_pal_error(Status);//��ӡ������Ϣ
	}
	if(sta!=0)
	  printf("sta:0x%x\r\n",sta);
	return Status;
}

//vl53l0x��λ����
//dev:�豸I2C�����ṹ��
void vl53l0x_reset(VL53L0X_Dev_t *dev)
{
	uint8_t addr;
	addr = dev->I2cDevAddr;//�����豸ԭI2C��ַ
    VL53L0X_Xshut=0;//ʧ��VL53L0X
	delay_ms(30);
	VL53L0X_Xshut=1;//ʹ��VL53L0X,�ô��������ڹ���(I2C��ַ��ָ�Ĭ��0X52)
	delay_ms(30);	
	dev->I2cDevAddr=0x52;
	vl53l0x_Addr_set(dev,addr);//����VL53L0X������ԭ���ϵ�ǰԭI2C��ַ
	VL53L0X_DataInit(dev);	
}

//��ʼ��vl53l0x
//dev:�豸I2C�����ṹ��
VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_Dev_t *pMyDevice = dev;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);   //��ʹ������IO PORTAʱ�� 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	           // �˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);				   //�����趨������ʼ��GPIOA

	pMyDevice->I2cDevAddr = VL53L0X_Addr;//I2C��ַ(�ϵ�Ĭ��0x52)
	pMyDevice->comms_type = 1;           //I2Cͨ��ģʽ
	pMyDevice->comms_speed_khz = 400;    //I2Cͨ������
	
	VL53L0X_i2c_init();//��ʼ��IIC����
	
	VL53L0X_Xshut=0;//ʧ��VL53L0X
	delay_ms(30);
	VL53L0X_Xshut=1;//ʹ��VL53L0X,�ô��������ڹ���
	delay_ms(30);
	
    vl53l0x_Addr_set(pMyDevice,0x54);//����VL53L0X������I2C��ַ
    if(Status!=VL53L0X_ERROR_NONE) goto error;
	Status = VL53L0X_DataInit(pMyDevice);//�豸��ʼ��
	if(Status!=VL53L0X_ERROR_NONE) goto error;
	delay_ms(2);
	Status = VL53L0X_GetDeviceInfo(pMyDevice,&vl53l0x_dev_info);//��ȡ�豸ID��Ϣ
    if(Status!=VL53L0X_ERROR_NONE) goto error;
	
	//AT24CXX_Read(0,(u8*)&Vl53l0x_data,sizeof(_vl53l0x_adjust));//��ȡ24c02�����У׼����,����У׼ Vl53l0x_data.adjustok==0xAA
	//if(Vl53l0x_data.adjustok==0xAA)//��У׼
	  AjustOK=1;	
	//else //ûУ׼	
	  //AjustOK=0;
	
	error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
		print_pal_error(Status);//��ӡ������Ϣ
		return Status;
	}
  	
	return Status;
}

//���˵�����
void vl53l0x_mtest_ui(void)
{
	 //POINT_COLOR=BLUE;//��������Ϊ��ɫ
	 //LCD_Fill(30,170,300,300,WHITE);
	 //LCD_ShowString(30,170,200,16,16,"KEY_UP: Calibration mode");//У׼ģʽ
	 //LCD_ShowString(30,190,200,16,16,"KEY1:   General mode");    //��ͨ����ģʽ
	 //LCD_ShowString(30,210,200,16,16,"KEY0:   Interrupt mode");  //�жϲ���ģʽ
	printf("KEY_UP: Calibration mode");
	printf("KEY1:   General mode");
	printf("KEY0:   Interrupt mode");
}

//VL53L0X�����Գ���
void vl53l0x_test(void)
{   
	 u8 i=0;
	 u8 key=0;
	 while(vl53l0x_init(&vl53l0x_dev))//vl53l0x��ʼ��
	 {
		//LCD_ShowString(30,140,200,16,16,"VL53L0X Error!!!");
		 printf("VL53L0X Error!!!");//
	    delay_ms(500);
		//LCD_ShowString(30,140,200,16,16,"                ");
		  printf("");//
		delay_ms(500);
		LED0=!LED0;//DS0��˸
	 }
	 printf("VL53L0X OK\r\n");//
	 //LCD_ShowString(30,140,200,16,16,"VL53L0X OK");
	 
	 
	 
	 Lora_Test();
	 Process_ui();//������ʾ
	 LoRa_Set();//LoRa����(�������������ô��ڲ�����Ϊ115200) 
	 		//u8 obj_chn;
	 //u32 obj_addr;
	 		Diree_Set();

			//printf("Ŀ���ַ:\r\nĿ���ŵ�:");
  		//printf("%u\r\n%u\r\n",obj_addr,obj_chn);
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 if(SkyTra_Cfg_Rate(5)!=0)	//���ö�λ��Ϣ�����ٶ�Ϊ5Hz,˳���ж�GPSģ���Ƿ���λ. 
	{
   	//LCD_ShowString(30,120,200,16,16,"S1216F8-BD Setting...");
		printf("S1216F8-BD Setting...");
		do
		{
			USART2_Init(9600);			//��ʼ������3������Ϊ9600
	  	SkyTra_Cfg_Prt(3);			//��������ģ��Ĳ�����Ϊ38400
			USART2_Init(38400);			//��ʼ������3������Ϊ38400
      key=SkyTra_Cfg_Tp(100000);	//������Ϊ100ms
		}while(SkyTra_Cfg_Rate(5)!=0&&key!=0);//����SkyTraF8-BD�ĸ�������Ϊ5Hz
	  //LCD_ShowString(30,120,200,16,16,"S1216F8-BD Set Done!!");
		printf("S1216F8-BD Set Done!!");
		delay_ms(500);
		//LCD_Fill(30,120,30+200,120+16,WHITE);//�����ʾ 
	}
	 //gps_test();
	 
	 
	 
	 



	//uart4_init(115200);
	 //imu901_init();
	 
	 
	 
	
	
	
	
	
	
	 
	 vl53l0x_mtest_ui();//���˵���ʾ
	 while(1)
	 {
		 
		 key = KEY_Scan(0);
		 if(key)
		 {
			  switch(key)
			  {
				  case WKUP_PRES:  vl53l0x_calibration_test(&vl53l0x_dev);   break;//У׼ģʽ
				  case KEY1_PRES:  vl53l0x_general_test(&vl53l0x_dev);       break;//��ͨ����ģʽ
				  case KEY0_PRES:  vl53l0x_interrupt_test(&vl53l0x_dev);     break;//�жϲ���ģʽ  
			  }
			  vl53l0x_mtest_ui();
		 }
		 i++;
		 if(i==5)
		 {
			 i=0;
			 LED0=!LED0;
		 }
		 delay_ms(50);
		 
	 }
}

//----------���º���ΪUSMART����------------//

//��ȡvl53l0x������ID��Ϣ
void vl53l0x_info(void)
{
	printf("\r\n-------vl53l0x�������豸��Ϣ-------\r\n\r\n");
	printf("  Name:%s\r\n",vl53l0x_dev_info.Name);
	printf("  Addr:0x%x\r\n",vl53l0x_dev.I2cDevAddr);
	printf("  ProductId:%s\r\n",vl53l0x_dev_info.ProductId);
	printf("  RevisionMajor:0x%x\r\n",vl53l0x_dev_info.ProductRevisionMajor);
	printf("  RevisionMinor:0x%x\r\n",vl53l0x_dev_info.ProductRevisionMinor);
	printf("\r\n-----------------------------------\r\n");
}

//��ȡһ�β�����������
//modeģʽ���� 0:Ĭ��;1:�߾���;2:������;3:����
void One_measurement(u8 mode)
{
	vl53l0x_set_mode(&vl53l0x_dev,mode);
	VL53L0X_PerformSingleRangingMeasurement(&vl53l0x_dev,&vl53l0x_data);
	printf("\r\n d: %4d mm.\r\n",vl53l0x_data.RangeMilliMeter);
		
}
//The following is written by WPS
void calibration_test(void)//calibration
{
	vl53l0x_calibration_test(&vl53l0x_dev);
}
