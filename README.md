# STM32_ATK-VL53L0X_ATK-LORA-01_ATK-1218-BD
通过LORA发送VL53L0X测得的高度数据和GPS模块测得的经纬度
stm32:stm32f103mini   LORA：透明传输
目前有点小毛病，24C02我好像改代码没改好，个人疏忽。VL53L0X每次上电都需要重新校准一次


VL53L0X:
	PA12(SCL),PA11(SDA),PA5(XSH),PA4(INT)
	
LORA:
	VCC<------------>5V
	GND<------------>GND 
	TXD<------------>PB11
	RXD<------------>PB10
	AUX<------------>PB1   
	MD0<------------>PB8     
	
GPS:
	RX---PA2
	TX---PA3
