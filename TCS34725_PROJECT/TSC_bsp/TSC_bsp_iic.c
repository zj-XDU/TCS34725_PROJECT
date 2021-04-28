#include "TSC_bsp_iic.h"


/**
  * 函数功能: I2C总线位延迟，最快400KHz
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
  
  uint32_t in,out,time;
static void TSC_I2C_Delay(void)
{
	uint8_t i;

	/*　
	 	下面的时间是通过逻辑分析仪测试得到的。
		CPU主频72MHz时，在内部Flash运行, MDK工程不优化
		循环次数为10时，SCL频率 = 205KHz 
		循环次数为7时，SCL频率 = 347KHz， SCL高电平时间1.5us，SCL低电平时间2.87us 
	 	循环次数为5时，SCL频率 = 421KHz， SCL高电平时间1.25us，SCL低电平时间2.375us 
        
    IAR工程编译效率高，不能设置为7
	*/
	 in=HAL_GetTick();
	for (i = 0; i < 30; i++);
	out=HAL_GetTick();
	time = out-in;
}

//TCS34725 I2C初始化
/*void TCS34725_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure; 
	__HAL_RCC_GPIOI_CLK_ENABLE();   //使能GPIOI时钟
	
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull = GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed = GPIO_SPEED_FAST;     //快速 
	
    GPIO_Initure.Pin = TCS_SDA_PIN;
	HAL_GPIO_Init(TCS_SDA_GPIO, &GPIO_Initure);
	GPIO_Initure.Pin = TCS_SCL_PIN;
	HAL_GPIO_Init(TCS_SCL_GPIO, &GPIO_Initure);
	
	TCS_SDA_SET;
	TCS_SCL_SET;
	
	delay_ms(10);
}*/

//产生I2C起始信号
void TCS34725_I2C_Start()
{
	//TCS_SDA_DIR_OUT();//sda线输出
	TCS_SDA_SET;
	TCS_SCL_SET;
	TSC_I2C_Delay();
	TCS_SDA_RESET;//START:when CLK is high,DATA change form high to low 
	TSC_I2C_Delay();
	TCS_SCL_RESET;//钳住I2C总线，准备发送或接收数据 
}

//产生I2C停止信号
void TCS34725_I2C_Stop()
{
	//TCS_SDA_DIR_OUT();//sda线输出
	TCS_SCL_RESET;
	TCS_SDA_RESET;//STOP:when CLK is high DATA change form low to high
	TSC_I2C_Delay();
	TCS_SCL_SET;
	TCS_SDA_SET;//发送I2C总线结束信号
	TSC_I2C_Delay();							   	
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t TCS34725_I2C_Wait_ACK()
{
	uint8_t timeOut = 0;
	
	//TCS_SDA_DIR_IN();//SDA设置为输入  
	TCS_SDA_SET; TSC_I2C_Delay();
	TCS_SCL_SET; TSC_I2C_Delay();
	while(TCS_SDA_READ)
	{
		timeOut++;
		if(timeOut > 250)
		{
			TCS34725_I2C_Stop();
			return 1;
		}
	}
	TCS_SCL_RESET;//时钟输出0
			
	return 0;	
} 

//产生ACK应答
void TCS34725_I2C_ACK()
{
	TCS_SCL_RESET;
	//TCS_SDA_DIR_OUT();//sda线输出
	TCS_SDA_RESET;
	TSC_I2C_Delay();
	TCS_SCL_SET;
	TSC_I2C_Delay();
	TCS_SCL_RESET;
}

//不产生ACK应答		    
void TCS34725_I2C_NACK()
{
	TCS_SCL_RESET;
	//TCS_SDA_DIR_OUT();//sda线输出
	TCS_SDA_SET;
	TSC_I2C_Delay();
	TCS_SCL_SET;
	TSC_I2C_Delay();
	TCS_SCL_RESET;
}	

//I2C发送一个字节		  
void TCS34725_I2C_Send_Byte(uint8_t byte)
{
	uint8_t i = 0;
	
	//TCS_SDA_DIR_OUT();//sda线输出
	TCS_SCL_RESET;//拉低时钟开始数据传输
	for(i = 0; i < 8; i++)
	{
		((byte & 0x80) >> 7) == 0x01 ? TCS_SDA_SET : TCS_SDA_RESET;
		byte <<= 1;
		TSC_I2C_Delay();
		TCS_SCL_SET;
		TSC_I2C_Delay();
		TCS_SCL_RESET;
		TSC_I2C_Delay();
	} 
} 

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t TCS34725_I2C_Read_Byte(uint8_t ack)
{
	uint8_t i,receive = 0;
	
	
	//TCS_SDA_DIR_IN();
	for(i = 0; i < 8; i++)
	{
		TCS_SCL_RESET;
		TSC_I2C_Delay();
		TCS_SCL_SET;
		receive <<= 1;
		if(TCS_SDA_READ) receive++;
		TSC_I2C_Delay();
	}
	if (!ack) TCS34725_I2C_NACK();//发送nACK
	else TCS34725_I2C_ACK(); //发送ACK 
	
	return receive;
}


//TSC写	reg寄存器 data数据
void TCS34725_I2C_Write(uint8_t slaveAddress, 
						uint8_t* dataBuffer,
						uint8_t bytesNumber, 
						uint8_t stopBit)
{
	unsigned char i = 0;
	
	TCS34725_I2C_Start();
	TCS34725_I2C_Send_Byte((slaveAddress << 1) | 0x00);	   //发送从机地址写命令
	TCS34725_I2C_Wait_ACK();
	for(i = 0; i < bytesNumber; i++)
	{
		TCS34725_I2C_Send_Byte(*(dataBuffer + i));
		TCS34725_I2C_Wait_ACK();
	}
	if(stopBit == 1) TCS34725_I2C_Stop();
}




//TSC读取8位数据
void TCS34725_I2C_Read(uint8_t slaveAddress, 
						uint8_t* dataBuffer, 
						uint8_t bytesNumber, 
						uint8_t stopBit)
{
	unsigned char i = 0;
	
	TCS34725_I2C_Start();
	TCS34725_I2C_Send_Byte((slaveAddress << 1) | 0x01);	   //发送从机地址读命令
	TCS34725_I2C_Wait_ACK();
	for(i = 0; i < bytesNumber; i++)
	{
		if(i == bytesNumber - 1)
		{
			*(dataBuffer + i) = TCS34725_I2C_Read_Byte(0);//读取的最后一个字节发送NACK
		}
		else
		{
			*(dataBuffer + i) = TCS34725_I2C_Read_Byte(1);
		}
	}
	if(stopBit == 1) TCS34725_I2C_Stop();
	
	
}



void TCS34725_Write(unsigned char subAddr, unsigned char* dataBuffer, unsigned char bytesNumber)
{
    unsigned char sendBuffer[10] = {0, };
    unsigned char byte = 0;
    
    sendBuffer[0] = subAddr | TCS34725_COMMAND_BIT;
    for(byte = 1; byte <= bytesNumber; byte++)
    {
        sendBuffer[byte] = dataBuffer[byte - 1];
    }
	TCS34725_I2C_Write(TCS34725_ADDRESS, sendBuffer, bytesNumber + 1, 1);
}




void TCS34725_Read(unsigned char subAddr, unsigned char* dataBuffer, unsigned char bytesNumber)
{
	subAddr |= TCS34725_COMMAND_BIT;
	
	TCS34725_I2C_Write(TCS34725_ADDRESS, (unsigned char*)&subAddr, 1, 0);
	TCS34725_I2C_Read(TCS34725_ADDRESS, dataBuffer, bytesNumber, 1);
}

/***************************************************************************//**
 * @brief TCS34725设置积分时间
 *
 * @return None
*******************************************************************************/
void TCS34725_SetIntegrationTime(uint8_t time)
{
	unsigned char cmd = time;
	
	TCS34725_Write(TCS34725_ATIME, &cmd, 1);
}
/***************************************************************************//**
 * @brief TCS34725设置增益
 *
 * @return None
*******************************************************************************/
void TCS34725_SetGain(uint8_t gain)
{
	unsigned char cmd = gain;
	
	TCS34725_Write(TCS34725_CONTROL, &cmd, 1);
}
/***************************************************************************//**
 * @brief TCS34725设置寄存器
 *
 * @return None
*******************************************************************************/
void TCS34725_Setup(void)
{
	TCS34725_SetIntegrationTime(TCS34725_INTEGRATIONTIME_240MS);
	TCS34725_SetGain(TCS34725_GAIN_4X);
}

/***************************************************************************//**
 * @brief TCS34725使能
 *
 * @return None
*******************************************************************************/
void TCS34725_Enable(void)
{
	unsigned char cmd = TCS34725_ENABLE_PON;
	
	TCS34725_Write(TCS34725_ENABLE, &cmd, 1);
	//delay_ms(3);//延时应该放在设置AEN之后
	cmd = TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN;
	TCS34725_Write(TCS34725_ENABLE, &cmd, 1);
	HAL_Delay(3);
}
/***************************************************************************//**
 * @brief TCS34725失能
 *
 * @return None
*******************************************************************************/
void TCS34725_Disable(void)
{
	unsigned char cmd = 0x00;
	
	TCS34725_Read(TCS34725_ENABLE, &cmd, 1);
	cmd = cmd & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
	TCS34725_Write(TCS34725_ENABLE, &cmd, 1);
}
/***************************************************************************//**
 * @brief TCS34725获取单个通道数据
 *
 * @return data - 该通道的转换值
 
*******************************************************************************/

uint16_t TCS34725_GetChannelData(unsigned char reg)
{
	uint16_t data = 0;
	unsigned char tmp[2] = {0,0};
	TCS34725_Read(reg, tmp, 2);
	data = ((uint16_t)tmp[1] << 8) | tmp[0];
	
	return data;
}
/***************************************************************************//**
 * @brief TCS34725获取各个通道数据
 *
 * @return 1 - 转换完成，数据可用
 *   	   0 - 转换未完成，数据不可用
*******************************************************************************/
uint8_t TCS34725_GetRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
	unsigned char status[1] = {0};

	status[0] = TCS34725_STATUS_AVALID;
	
	TCS34725_Read(TCS34725_STATUS, status, 1);
	
	if(status[0] & TCS34725_STATUS_AVALID)
	{
		*c = TCS34725_GetChannelData(TCS34725_CDATAL);	
		*r = TCS34725_GetChannelData(TCS34725_RDATAL);	
		*g = TCS34725_GetChannelData(TCS34725_GDATAL);	
		*b = TCS34725_GetChannelData(TCS34725_BDATAL);
		return 1;
	}
	return 0;
}
