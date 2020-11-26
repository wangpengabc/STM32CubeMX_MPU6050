#include "mpu6050.h"
 
struct ACCELSTRUCT accelStruct = {0,0,0};
struct GYROSTRUCT	gyroStruct = {0,0,0};
 
 
//IO方向设置
#define MPU_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define MPU_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

#define PC13_IN()  {GPIOC->CRH&=0XFF0FFFFF;GPIOC->CRH|=8<<20;}
#define PC13_OUT() {GPIOC->CRH&=0XFF0FFFFF;GPIOC->CRH|=3<<20;}
#define PC13_SET    PCout(13) //MPU SCL
#define PC13_READ   PCin(13) //MPU SDA	 

 
//IO操作函数	 
#define MPU_SCL    PBout(10) //MPU SCL
#define MPU_SDA    PBout(11) //MPU SDA	 
#define MPU_READ_SDA   PBin(11)  //输入SDA 
 
void DelayUs(uint32_t udelay)
{
  __IO uint32_t Delay = udelay * 8 / 8;//(SystemCoreClock / 8U / 1000000U)
    //见stm32f1xx_hal_rcc.c -- static void RCC_Delay(uint32_t mdelay)
  do
  {
    __NOP();
  }
  while (Delay --);

}


/**************************MPU5883 IIC驱动函数*********************************/
 
static void MPU5883IOInit(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

		/* GPIO Ports Clock Enable */
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

		/*Configure GPIO pin : PC13 */
		GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
	
    MPU_SCL = 1;//初始化均为浮空状态
    MPU_SDA = 1;
		
		HAL_Delay(500);
		
		MPU_SCL = 0;//初始化均为浮空状态
    MPU_SDA = 0;
		
		PC13_OUT();
		
		PC13_SET = 1;
		HAL_Delay(1000);
		PC13_SET = 0;
		HAL_Delay(1000);
		PC13_SET = 1;
		HAL_Delay(1000);
		PC13_SET = 0;
		HAL_Delay(1000);
		
}
 
 
 
//发送IIC起始信号
static void ComStart(void)
{
	MPU_SDA_OUT();     //sda线输出
    MPU_SDA=1;	  	  
    MPU_SCL=1;
    DelayUs(5);
    MPU_SDA=0;//START:when CLK is high,DATA change form high to low 
    DelayUs(5);
    MPU_SCL=0;//钳住I2C总线，准备发送或接收数据
}
//发送IIC停止信号
static void ComStop(void)
{
	MPU_SDA_OUT();//sda线输出
    MPU_SDA=0;//STOP:when CLK is high DATA change form low to high
    MPU_SCL=1;
    DelayUs(5);
    MPU_SDA=1;//发送I2C总线结束信号
    DelayUs(5);		
}
//等待ACK,为1代表无ACK 为0代表等到了ACK
static u8 ComWaitAck(void)
{
	u8 waitTime = 0;
	MPU_SDA_OUT();//sda线输出
	MPU_SDA = 1;
	DelayUs(5);
    MPU_SDA_IN();      //SDA设置为输入
	MPU_SCL=1;
	DelayUs(5);
	while(MPU_READ_SDA)
	{
		waitTime++;
		DelayUs(1);
		if(waitTime > MPU_ACK_WAIT_TIME)
		{
			ComStop();
			return 1;
		}
	}
	MPU_SCL = 0;
	return 0;
	
}
 
//static void ComSendAck(void)
//{
//	MPU_SCL = 0;
//	MPU_SDA_OUT();
//    MPU_SDA = 0;
//	DelayUs(2);
//    MPU_SCL = 1;
//    DelayUs(5);
//    MPU_SCL = 0;
//    DelayUs(5);
//}
 
static void ComSendNoAck(void)
{
	MPU_SCL = 0;
	MPU_SDA_OUT();
    MPU_SDA = 1;
	DelayUs(2);
    MPU_SCL = 1;
    DelayUs(5);
    MPU_SCL = 0;
    DelayUs(5);
}
//返回0 写入收到ACK 返回1写入未收到ACK
static u8 ComSendByte(u8 byte)
{
	u8 t;   
    MPU_SDA_OUT(); 	
    for(t=0;t<8;t++)
    {              
        MPU_SDA=(byte&0x80)>>7;
        byte<<=1; 	   
        MPU_SCL=1;
        DelayUs(5); 
        MPU_SCL=0;	
        DelayUs(5);
    }	 
    return ComWaitAck();
}
 
static void ComReadByte(u8* byte)
{
	u8 i,receive=0;
    MPU_SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
    {
        receive <<= 1;
        MPU_SCL=1; 
        DelayUs(5);
        if(MPU_READ_SDA)receive++;
        MPU_SCL=0; 
        DelayUs(5); 
    }					  
    *byte = receive;
}
 
/**************************MPU5883 IIC驱动函数*********************************/
 
 
//向MPU写入一个字节数据,失败返回1 成功返回0
u8 MPUWriteReg(u8 regValue,u8 setValue)
{
	u8 res;
    ComStart();                 	//起始信号
    res = ComSendByte(MPU_ADDR);    //发送设备地址+写信号
	if(res)
	{
//		#ifdef MPU_DEBUG
//		printf("file=%s,func=%s,line=%d\r\n",__FILE__,__FUNCTION__,__LINE__);
//		#endif
		return res;
	}
    res = ComSendByte(regValue);    //内部寄存器地址
	if(res)
	{
//		#ifdef MPU_DEBUG
//		printf("file=%s,func=%s,line=%d\r\n",__FILE__,__FUNCTION__,__LINE__);
//		#endif
		return res;
	}
    res = ComSendByte(setValue);    //内部寄存器数据
	if(res)
	{
//		#ifdef MPU_DEBUG
//		printf("file=%s,func=%s,line=%d\r\n",__FILE__,__FUNCTION__,__LINE__);
//		#endif
		return res;
	}
    ComStop();                   	//发送停止信号
	return res;
}
 
//**************************************
//从I2C设备读取一个字节数据 返回值 读取成功或失败
//**************************************
u8 MPUReadReg(u8 regAddr,u8* readValue)
{
    u8 res;
    ComStart();                 		//起始信号
    res = ComSendByte(MPU_ADDR);    	//发送设备地址+写信号
	if(res)
	{
//		#ifdef MPU_DEBUG
//		printf("file=%s,func=%s,line=%d\r\n",__FILE__,__FUNCTION__,__LINE__);
//		#endif
		return res;
	}
    res = ComSendByte(regAddr);     	//发送存储单元地址，从0开始	
	if(res)
	{
//		#ifdef MPU_DEBUG
//		printf("file=%s,func=%s,line=%d\r\n",__FILE__,__FUNCTION__,__LINE__);
//		#endif
		return res;
	}
    ComStart();                 		//起始信号
    res = ComSendByte(MPU_ADDR+1);  	//发送设备地址+读信号
	if(res)
	{
//		#ifdef MPU_DEBUG
//		printf("file=%s,func=%s,line=%d\r\n",__FILE__,__FUNCTION__,__LINE__);
//		#endif
		return res;
	}
    ComReadByte(readValue);     		//读出寄存器数据
    ComSendNoAck();               		//发送非应答信号
    ComStop();                  		//停止信号
    return res;
}
 
//MPU读取两个字节的数据
s16 MpuReadTwoByte(u8 addr)
{
    u8 H,L;
    MPUReadReg(addr,&H);
    MPUReadReg(addr+1,&L);
    return (s16)((((u16)H)<<8)+L);   //合成数据
}
 
/*
 *初始化，返回0代表失败 返回1代表成功
 **/
u8 MpuInit(void)
{
	u8 result;
	u8 id = 0;
    MPU5883IOInit();
	result = MPUReadReg(MPU6050_RA_WHO_AM_I,&id);
	if(result)	return result;	//IIC总线错误
	else 
	{
		id &= 0x7e;//除去最高位最低位
		id>>= 1;
		if(id != 0x34) return 1;	//获取到的芯片ID错误
	}
    //初始化成功，设置参数
    MPUWriteReg(MPU6050_RA_PWR_MGMT_1,0x01);			// 退出睡眠模式，设取样时钟为陀螺X轴。
    MPUWriteReg(MPU6050_RA_SMPLRT_DIV,0x04);			// 取样时钟4分频，1k/4，取样率为25Hz。
    MPUWriteReg(MPU6050_RA_CONFIG,2);				// 低通滤波，截止频率100Hz左右。
    MPUWriteReg(MPU6050_RA_GYRO_CONFIG,3<<3);			// 陀螺量程，2000dps
    MPUWriteReg(MPU6050_RA_ACCEL_CONFIG,2<<3);			// 加速度计量程，8g。
    MPUWriteReg(MPU6050_RA_INT_PIN_CFG,0x32);					// 中断信号为高电平，推挽输出，直到有读取操作才消失，直通辅助I2C。
    MPUWriteReg(MPU6050_RA_INT_ENABLE,0x01);					// 使用“数据准备好”中断。
    MPUWriteReg(MPU6050_RA_USER_CTRL,0x00);					// 不使用辅助I2C。
    return 0;
}
 
 
//获取相应的测量数据
void MpuGetData(void)
{
	s16 temp = 0;
    accelStruct.accelX = MpuReadTwoByte(MPU6050_RA_ACCEL_XOUT_H);
    accelStruct.accelY = MpuReadTwoByte(MPU6050_RA_ACCEL_YOUT_H);
    accelStruct.accelZ = MpuReadTwoByte(MPU6050_RA_ACCEL_ZOUT_H);
    gyroStruct.gyroX = MpuReadTwoByte(MPU6050_RA_GYRO_XOUT_H);
    gyroStruct.gyroY = MpuReadTwoByte(MPU6050_RA_GYRO_YOUT_H);
    gyroStruct.gyroZ = MpuReadTwoByte(MPU6050_RA_GYRO_ZOUT_H);
	temp = MpuReadTwoByte(MPU6050_RA_TEMP_OUT_H);
//	#ifdef MPU_DEBUG
//	printf("accel  x = %d  ,y =  %d  ,z = %d  \r\n",accelStruct.accelX,accelStruct.accelY,accelStruct.accelZ);
//	printf("gyro  x = %d  ,y =  %d  ,z = %d  \r\n",gyroStruct.gyroX,gyroStruct.gyroY,gyroStruct.gyroZ);
//	printf("temp is %0.3f \r\n",(((float)temp)/340.0 + 36.53));
//	#endif
}
