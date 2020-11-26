#ifndef __MPU6050_H_
#define __MPU6050_H_
 
//#include "common.h"
//#include "ioremap.h"
//#include "stm32f10x.h"
//#include "delay.h"
//#include "uart.h"

#include "main.h"
 
#define MPU_ACK_WAIT_TIME	200	//us
 
#define MPU6050_ADDRESS_AD0_LOW     0xD0 // AD0为低的时候设备的写地址
#define MPU6050_ADDRESS_AD0_HIGH    0XD1 // AD0为高的时候设备的写地址
#define	MPU_ADDR	0xD0	//IIC写入时的地址字节数据
 
 
#define MPU_DEBUG		1
 
//技术文档未公布的寄存器 主要用于官方DMP操作
#define MPU6050_RA_XG_OFFS_TC       0x00 //[bit7] PWR_MODE, [6:1] XG_OFFS_TC, [bit 0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
//bit7的定义,当设置为1,辅助I2C总线高电平是VDD。当设置为0,辅助I2C总线高电平是VLOGIC
 
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
 
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS 两个寄存器合在一起
#define MPU6050_RA_XA_OFFS_L_TC     0x07
 
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS 两个寄存器合在一起
#define MPU6050_RA_YA_OFFS_L_TC     0x09
 
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS 两个寄存器合在一起
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
 
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR 两个寄存器合在一起
#define MPU6050_RA_XG_OFFS_USRL     0x14
 
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR 两个寄存器合在一起
#define MPU6050_RA_YG_OFFS_USRL     0x16
 
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR 两个寄存器合在一起
#define MPU6050_RA_ZG_OFFS_USRL     0x18
 
/*陀螺仪的采样频率*/
/*传感器的寄存器输出,FIFO输出,DMP采样、运动检测、
 *零运动检测和自由落体检测都是基于采样率。
 *通过SMPLRT_DIV把陀螺仪输出率分频即可得到采样率
 *采样率=陀螺仪输出率/ (1 + SMPLRT_DIV)
 *禁用DLPF的情况下(DLPF_CFG = 0或7) ，陀螺仪输出率= 8 khz
 *在启用DLPF(见寄存器26)时，陀螺仪输出率= 1 khz
 *加速度传感器输出率是1 khz。这意味着,采样率大于1 khz时,
 *同一个加速度传感器的样品可能会多次输入到FIFO、DMP和传感器寄存器*/
#define MPU6050_RA_SMPLRT_DIV       0x19 //[0-7] 陀螺仪输出分频采样率
 
/*配置外部引脚采样和DLPF数字低通滤波器*/
#define MPU6050_RA_CONFIG           0x1A
//bit5-bit3  一个连接到FSYNC端口的外部信号可以通过配置EXT_SYNC_SET来采样
//			 也就是说,这里设置之后,FSYNC的电平0或1进入最终数据寄存器,具体如下
//			0 不使用 1 FSYNC电平进入所有数据寄存器 2 FSYNC电平进入GYRO_XOUT_L 3 FSYNC电平进入GYRO_YOUT_L
//			4 FSYNC电平进入GYRO_ZOUT_L 5 FSYNC电平进入ACCEL_XOUT_L 6 FSYNC电平进入ACCEL_YOUT_L
//			7 FSYNC电平进入SYNC_ACCEL_ZOUT_L
//bit2-bit0 数字低通滤波器 用于滤除高频干扰 高于这个频率的干扰被滤除掉
/*对应关系如下
 * *				  |   加速度传感器  |          陀螺仪
 * * DLPF_CFG |    带宽   |  延迟  |    带宽   |  延迟  | 采样率
 * -------------+--------+-------+--------+------+-------------
 * 0			| 260Hz     | 0ms    | 256Hz   | 0.98ms | 8kHz
 * 1			| 184Hz     | 2.0ms  | 188Hz   | 1.9ms  | 1kHz
 * 2			| 94Hz      | 3.0ms  | 98Hz    | 2.8ms  | 1kHz
 * 3			| 44Hz      | 4.9ms  | 42Hz    | 4.8ms  | 1kHz
 * 4			| 21Hz      | 8.5ms  | 20Hz    | 8.3ms  | 1kHz
 * 5			| 10Hz      | 13.8ms | 10Hz    | 13.4ms | 1kHz
 * 6			| 5Hz       | 19.0ms | 5Hz     | 18.6ms | 1kHz
 * 7			| Reserved  | Reserved | Reserved
 * */
 
 
/*陀螺仪的配置,主要是配置陀螺仪的量程与自检(通过相应的位7 6 5 开启自检)*/
#define MPU6050_RA_GYRO_CONFIG      0x1B
//bit4-bit3 量程设置如下
//			 0 = +/- 250 度/秒
//			 1 = +/- 500 度/秒
//			 2 = +/- 1000 度/秒
//			 3 = +/- 2000 度/秒*/
 
/*加速度计的配置,主要是配置加速度计的量程与自检(通过相应的位7 6 5 开启自检)
 *另外,还能配置系统的高通滤波器*/
#define MPU6050_RA_ACCEL_CONFIG     0x1C
//bit7 启动X自检 加速度计的自检
//bit6 启动Y自检
//bit5 启动Z自检
//bit4-bit3 加速度传感器的量程配置
//			 0 = +/- 2g
//			 1 = +/- 4g
//			 2 = +/- 8g
//			 3 = +/- 16g*/
//bit0到bit2 加速度传感器的高通滤波器
/*DHPF是在路径中连接于运动探测器(自由落体,运动阈值,零运动)的一个滤波器模块。
 *高通滤波器的输出值不在数据寄存器中
 *高通滤波器有三种模式：
 *重置:在一个样本中将滤波器输出值设为零。这有效的禁用了高通滤波器。这种模式可以快速切换滤波器的设置模式。
 *开启:高通滤波器能通过高于截止频率的信号
 *持续:触发后,过滤器持续当前采样。过滤器输出值是输入样本和持续样本之间的差异
 *设置值如下所示
 * ACCEL_HPF | 高通滤波模式| 截止频率
 * ----------+-------------+------------------
 * 0         | Reset       | None
 * 1         | On          | 5Hz
 * 2         | On          | 2.5Hz
 * 3         | On          | 1.25Hz
 * 4         | On          | 0.63Hz
 * 7         | Hold        | None
 */
 
#define MPU6050_RA_FF_THR           0x1D
/*自由落体加速度的阈值
 *这个寄存器为自由落体的阈值检测进行配置。
 *FF_THR的单位是1LSB = 2mg。当加速度传感器测量而得的三个轴的绝对值
 *都小于检测阈值时，就可以测得自由落体值。这种情况下，(加速度计每次检测到就+1以下,所以还要依靠加速度采样率)
 *自由落体时间计数器计数一次 (寄存器30)。当自由落体时间计数器达到
 *FF_DUR中规定的时间时，自由落体被中断(或发生自由落体中断)
 **/
 
#define MPU6050_RA_FF_DUR           0x1E
/*
 *自由落体加速度的时间阈值
* 这个寄存器为自由落体时间阈值计数器进行配置。
* 时间计数频率为1 khz,因此FF_DUR的单位是 1 LSB = 1毫秒。
* 当加速度器测量而得的绝对值都小于检测阈值时，
* 自由落体时间计数器计数一次。当自由落体时间计数器
* 达到该寄存器的规定时间时，自由落体被中断。
* (或发生自由落体中断)
* */
 
#define MPU6050_RA_MOT_THR          0x1F
/*
 *运动检测的加速度阈值
 *这个寄存器为运动中断的阈值检测进行配置。
 *MOT_THR的单位是 1LSB = 2mg。
 *当加速度器测量而得的绝对值都超过该运动检测的阈值时，
 *即可测得该运动。这一情况下，运动时间检测计数器计数一次。
 *当运动检测计数器达到MOT_DUR的规定时间时，运动检测被中断。
 * 运动中断表明了被检测的运动MOT_DETECT_STATUS (Register 97)的轴和极性。
 */
 
#define MPU6050_RA_MOT_DUR          0x20
/*
*运动检测时间的阈值。
*这个寄存器为运动中断的阈值检测进行配置。
*时间计数器计数频率为1 kHz ，因此MOT_THR的单位是 1LSB = 1ms。
*当加速度器测量而得的绝对值都超过该运动检测的阈值时(Register 31)，
*运动检测时间计数器计数一次。当运动检测计数器达到该寄存器规定的时间时，
*运动检测被中断。
 **/
 
#define MPU6050_RA_ZRMOT_THR        0x21
/*
*零运动检测加速度阈值。
* 这个寄存器为零运动中断检测进行配置。
* ZRMOT_THR的单位是1LSB = 2mg。
* 当加速度器测量而得的三个轴的绝对值都小于检测阈值时，
* 就可以测得零运动。这种情况下，零运动时间计数器计数一次 (寄存器34)。
* 当自零运动时间计数器达到ZRMOT_DUR (Register 34)中规定的时间时，零运动被中断。
* 与自由落体或运动检测不同的是，当零运动首次检测到以及当零运动检测不到时，零运动检测都被中断。
* 当零运动被检测到时,其状态将在MOT_DETECT_STATUS寄存器(寄存器97) 中显示出来。
* 当运动状态变为零运动状态被检测到时,状态位设置为1。当零运动状态变为运动状态被检测到时,
* 状态位设置为0。
 **/
 
#define MPU6050_RA_ZRMOT_DUR        0x22
/*
*零运动检测的时间阈值
* 这个寄存器为零运动中断检测进行时间计数器的配置。
* 时间计数器的计数频率为16 Hz,因此ZRMOT_DUR的单位是1 LSB = 64 ms。
* 当加速度器测量而得的绝对值都小于检测器的阈值(Register 33)时，
* 运动检测时间计数器计数一次。当零运动检测计数器达到该寄存器规定的时间时，
* 零运动检测被中断。
 **/
 
 
/*
 *设备的各种FIFO使能,包括温度 加速度 陀螺仪 从机
 *将相关的数据写入FIFO缓冲中
 **/
#define MPU6050_RA_FIFO_EN          0x23
//bit7 温度fifo使能
//bit6 陀螺仪Xfifo使能
//bit5 陀螺仪Yfifo使能
//bit4 陀螺仪Zfifo使能
//bit3 加速度传感器fifo使能
//bit2 外部从设备2fifo使能
//bit1 外部从设备1fifo使能
//bit0 外部从设备0fifo使能
 
#define MPU6050_RA_I2C_MST_CTRL     0x24
//配置单主机或者多主机下的IIC总线
//bit7 监视从设备总线,看总线是否可用 MULT_MST_EN设置为1时,MPU-60X0的总线仲裁检测逻辑被打开
//bit6 延迟数据就绪中断,直达从设备数据也进入主机再触发 相当于数据同步等待
//bit5 当设置为1时,与Slave3 相连的外部传感器数据(寄存器73 到寄存器 96)写入FIFO缓冲中,每次都写入
//bit4 主机读取一个从机到下一个从机读取之间的动作 为0 读取之间有一个restart,为1 下一次读取前会有一个重启,然后
//		一直读取直到切换写入或者切换设备
//bit3-bit0 配置MPU作为IIC主机时的时钟,基于MPU内部8M的分频
/* I2C_MST_CLK | I2C 主时钟速度 | 8MHz 时钟分频器
* ------------+------------------------+-------------------
* 0			    | 348kHz          | 23
* 1			    | 333kHz          | 24
* 2			    | 320kHz          | 25
* 3				| 308kHz          | 26
* 4				| 296kHz          | 27
* 5				| 286kHz          | 28
* 6				| 276kHz          | 29
* 7				| 267kHz          | 30
* 8				| 258kHz          | 31
* 9				| 500kHz          | 16
* 10			| 471kHz          | 17
* 11			| 444kHz          | 18
* 12			| 421kHz          | 19
* 13			| 400kHz          | 20
* 14			| 381kHz          | 21
* 15			| 364kHz          | 22
* */
 
 
 
/**************************MPU链接IIC从设备控制寄存器,没使用从机连接的基本不用考虑这些************************************/
/*指定slave (0-3)的I2C地址
* 注意Bit 7 (MSB)控制了读/写模式。如果设置了Bit 7,那么这是一个读取操作,
* 如果将其清除,那么这是一个编写操作。其余位(6-0)是slave设备的7-bit设备地址。
* 在读取模式中,读取结果是存储于最低可用的EXT_SENS_DATA寄存器中。
* MPU-6050支持全5个slave，但Slave 4有其特殊功能(getSlave4* 和setSlave4*)。
* 如寄存器25中所述，I2C数据转换通过采样率体现。用户负责确保I2C数据转换能够
* 在一个采样率周期内完成。
* I2C slave数据传输速率可根据采样率来减小。
* 减小的传输速率是由I2C_MST_DLY(寄存器52)所决定的。
* slave数据传输速率是否根据采样率来减小是由I2C_MST_DELAY_CTRL (寄存器103)所决定的。
* slave的处理指令是固定的。Slave的处理顺序是Slave 1, Slave 2, Slave 3 和 Slave 4。
* 如果某一个Slave被禁用了，那么它会被自动忽略。
* 每个slave可按采样率或降低的采样率来读取。在有些slave以采样率读取有些以减小
* 的采样率读取的情况下，slave的读取顺序依旧不变。然而，
* 如果一些slave的读取速率不能在特定循环中进行读取，那么它们会被自动忽略
* 更多降低的读取速率相关信息,请参阅寄存器52。
* Slave是否按采样率或降低的采样率来读取由寄存器103得Delay Enable位来决定
 **/
 
//从机0设置相关
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
//bit7 当前IIC 从设备0的操作,1为读取 0写入
//bit6-bit0 从机设备的地址
/* 要读取或者要写入的设备内部的寄存器地址,不管读取还是写入*/
#define MPU6050_RA_I2C_SLV0_REG     0x26
/*iic从机系统配置寄存器*/
#define MPU6050_RA_I2C_SLV0_CTRL    0x27
//bit7 启动或者禁止这个设备的IIC数据传送过程
//bit6 当设置为1时,字节交换启用。当启用字节交换时,词对的高低字节即可交换
//bit5 当 I2C_SLV0_REG_DIS 置 1，只能进行读取或者写入数据。当该位清 0，可以再读取
//		或写入数据之前写入一个寄存器地址。当指定从机设备内部的寄存器地址进行发送或接收
//		数据时，该位必须等于 0
//bit4	指定从寄存器收到的字符对的分组顺序。当该位清 0，寄存器地址
// 		0和 1, 2 和 3 的字节是分别成对（甚至，奇数寄存器地址 ） ，作为一个字符对。当该位置 1，
//		寄存器地址 1 和 2， 3 和 4 的字节是分别成对的，作为一个字符对
//bit3-bit0  指定从机 0 发送字符的长度。由Slave 0转换而来和转换至Slave 0的字节数,(IIC一次传输的长度)
// 			该位清 0，I2C_SLV0_EN 位自动置 0.
 
/*IIC SLAVE1配置寄存器,与0相同*/
 
#define MPU6050_RA_I2C_SLV1_ADDR    0x28
#define MPU6050_RA_I2C_SLV1_REG     0x29
#define MPU6050_RA_I2C_SLV1_CTRL    0x2A
 
/*IIC SLAVE2配置寄存器,与0相同*/
#define MPU6050_RA_I2C_SLV2_ADDR    0x2B
#define MPU6050_RA_I2C_SLV2_REG     0x2C
#define MPU6050_RA_I2C_SLV2_CTRL    0x2D
 
/*IIC SLAVE3配置寄存器,与0相同*/
#define MPU6050_RA_I2C_SLV3_ADDR    0x2E
#define MPU6050_RA_I2C_SLV3_REG     0x2F
#define MPU6050_RA_I2C_SLV3_CTRL    0x30
 
/*slave4的I2C地址 IIC4与前几个的寄存器定义有所不同*/
#define MPU6050_RA_I2C_SLV4_ADDR    0x31		//与IIC SLAVE1类似 
#define MPU6050_RA_I2C_SLV4_REG     0x32	/*slave4的当前内部寄存器*/
#define MPU6050_RA_I2C_SLV4_DO      0x33
	/*写于slave4的新字节这一寄存器可储存写于slave4的数据。
	* 如果I2C_SLV4_RW设置为1（设置为读取模式），那么该寄存器无法执行操作*/
#define MPU6050_RA_I2C_SLV4_CTRL    0x34
//当设置为1时，此位启用了slave4的转换操作。当设置为0时，则禁用该操作
#define MPU6050_I2C_SLV4_EN_BIT         7
//当设置为1时，此位启用了slave4事务完成的中断信号的生成。
// 当清除为0时，则禁用了该信号的生成。这一中断状态可在寄存器54中看到。
#define MPU6050_I2C_SLV4_INT_EN_BIT     6
//当设置为1时,只进行数据的读或写操作。当设置为0时,
// 在读写数据之前将编写一个寄存器地址。当指定寄存器地址在slave设备中时
// ，这应该等于0，而在该寄存器中会进行数据处理。
#define MPU6050_I2C_SLV4_REG_DIS_BIT    5
//采样率延迟,这为根据采样率减小的I2C slaves传输速率进行了配置。
// 当一个slave的传输速率是根据采样率而降低的,那么该slave是以每1 / (1 + I2C_MST_DLY) 个样本进行传输。
// 这一基本的采样率也是由SMPLRT_DIV (寄存器 25)和DLPF_CFG (寄存器26)所决定的的。
// slave传输速率是否根据采样率来减小是由I2C_MST_DELAY_CTRL (寄存器103)所决定的
#define MPU6050_I2C_SLV4_MST_DLY_BIT    4	//[4:0]
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH 5
/*slave4中可读取的最后可用字节*/
#define MPU6050_RA_I2C_SLV4_DI      0x35
 
/*
 * IIC辅助从机系统中断状态
 **/
#define MPU6050_RA_I2C_MST_STATUS   0x36
//bit7 此位反映了一个与MPU-60X0相连的外部设备的FSYNC中断状态。
//		当设置为1且在INT_PIN_CFG(寄存器55)中断言FSYNC_INT_EN时，中断产生。
//bit6 当slave4事务完成时，设备会自动设置为1 如果定义了INT_ENABLE中的I2C_MST_INT_EN则产生中断
//bit5 I2C主机失去辅助I2C总线（一个错误状态）的仲裁，此位自动设置为1.如果断言了INT_ENABLE寄存器
//		（寄存器56）中的I2C_MST_INT_EN位，则中断产生
//bit4	slave4的NACK状态
//bit3  slave3的NACK状态
//bit2  slave2的NACK状态
//bit1  slave1的NACK状态
//bit0  slave0的NACK状态
 
 
/*中断引脚配置寄存器*/
#define MPU6050_RA_INT_PIN_CFG      0x37
//bit7  中断的逻辑电平模式,高电平时，设置为0；低电平时，设置为1
//bit6  中断驱动模式,推拉模式设置为0，开漏模式设置为1.
//bit5  中断锁存模式.50us-pulse模式设置为0，latch-until-int-cleared模式设置为1
//bit4  中断锁存清除模式 status-read-only状态设置为0，any-register-read状态设置为1.
//bit3  FSYNC中断逻辑电平模式 0=active-high, 1=active-low
//bit2  FSYNC端口中断启用设置设置为0时禁用，设置为1时启用
//bit1  I2C支路启用状态,此位等于1且I2C_MST_EN (寄存器 106 位[5])等于0时,主机应用程序处理器能够直接访问MPU-60X0的辅助I2C总线
//		否则无论如何都不能直接访问
//bit0  当此位为1时，CLKOUT端口可以输出参考时钟。当此位为0时，输出禁用
 
 
/*部分中断使能*/
#define MPU6050_RA_INT_ENABLE       0x38
//bit7  自由落体中断使能
//bit6  运动检测中断使能
//bit5  零运动检测中断使能
//bit4  FIFO溢出中断使能
//bit3  IIC主机所有中断源使能
//bit0  数据就绪中断使能
 
 
/*DMP中断使能*/
#define MPU6050_RA_DMP_INT_STATUS   0x39
//不知道这些位的具体作用是什么,官方语焉不详,但是的确存在
#define MPU6050_DMPINT_4_BIT            4
#define MPU6050_DMPINT_3_BIT            3
#define MPU6050_DMPINT_2_BIT            2
#define MPU6050_DMPINT_1_BIT            1
#define MPU6050_DMPINT_0_BIT            0
 
/*DMP中断配置*/
#define MPU6050_RA_INT_STATUS       0x3A
//DMP中断位之一使能
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
//DMP中断位之二使能
#define MPU6050_INTERRUPT_DMP_INT_BIT       1
 
/*加速度X输出*/
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
 
/*加速度Y输出*/
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
 
/*加速度Z输出*/
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
 
/*温度值输出*/
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
 
/*陀螺仪X输出*/
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
 
/*陀螺仪Y输出*/
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
 
/*陀螺仪Z输出*/
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
 
/*从IIC从机上获取到的数据*/
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
 
//运动检测的状态
#define MPU6050_RA_MOT_DETECT_STATUS    0x61
//bit7  x轴反向运动检测中断状态
//bit6  x轴正向运动检测中断状态
//bit5  Y轴反向运动检测中断状态
//bit4  Y轴正向运动检测中断状态
//bit3  Z轴反向运动检测中断状态
//bit2  Z轴正向运动检测中断状态
//bit1
//bit0  零运动检测中断状态
//
 
 
/*写入到IIC从机中的数据,指定的slv数据输出容器*/
#define MPU6050_RA_I2C_SLV0_DO      0x63
#define MPU6050_RA_I2C_SLV1_DO      0x64
#define MPU6050_RA_I2C_SLV2_DO      0x65
#define MPU6050_RA_I2C_SLV3_DO      0x66
 
/*外部影子寄存器的配置,这个寄存器用于指定外部传感器数据影子的时间
*当启用了某一特定的slave，其传输速率就会减小。
*当一个slave的传输速率是根据采样率而降低的,那么该slave是以
*每1 / (1 + I2C_MST_DLY) 个样本进行传输。
*     1 / (1 + I2C_MST_DLY) Samples
* 这一基本的采样率也是由SMPLRT_DIV (寄存器 25)和DLPF_CFG (寄存器26)所决定的的。*/
#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
//DELAY_ES_SHADOW设置为1,跟随外部传感器数据影子将会延迟到所有的数据接收完毕。
#define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
//slv4-0的配置
#define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0
 
/*用于陀螺仪，加速度计，温度传感器的模拟和数字信号通道的复位。
复位会还原模数转换信号通道和清除他们的上电配置*/
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68
//bit2  重置陀螺仪的信号路径
//bit1  重置加速度传感器的信号路径
//bit0  重置温度传感器的信号路径
 
 
/*获取加速度传感器启动延迟 还有滤波器的一些配置
* 加速度传感器数据路径为传感器寄存器、运动检测、
* 零运动检测和自由落体检测模块提供样本。在检测模块开始操作之前，
* 包含过滤器的信号路径必须用新样本来启用。
* 默认的4毫秒唤醒延迟时间可以加长3毫秒以上。在ACCEL_ON_DELAY中规定
* 这个延迟以1 LSB = 1 毫秒为单位。除非InvenSense另行指示，
* 用户可以选择任何大于零的值。*/
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
//具体的有效控制位
//bit5-bit4 [5:4]1-4ms 延时时间1-4ms选择
//bit3-bit2 自由落体检测计数器的减量配置。
// 			当指定数量的样本的加速度测量都满足其各自的阈值条件时，
//  		检测结果存储于自由落体检测模块中。当满足阈值条件时，
//  		相应的检测计数器递增1。用户可通过FF_COUNT配置不满足阈值条件来减量。
//  		减量率可根据下表进行设置：
			/* FF_COUNT | 计数器减量
			* ---------+------------------
			* 0				| 重置
			* 1				| 1
			* 2				| 2
			* 3				| 4
			* 当FF_COUNT配置为0(复位)时,任何不合格的样品都将计数器重置为0*/
//bit1-bit0  运动检测计数器的减量配置。
// 			当指定数量的样本的加速度测量都满足其各自的阈值条件时，
// 			检测结果存储于运动检测模块中。当满足阈值条件时，相应的检测计数器递增1。
// 			用户可通过MOT_COUNT配置不满足阈值条件来减量。减量率可根据下表进行设置：
// 			MOT_COUNT | 计数器减量
			/* ----------+------------------
			* 0				 | 重置
			* 1				 | 1
			* 2				 | 2
			* 3				 | 4
			* 当MOT_COUNT配置为0(复位)时,任何不合格的样品都将计数器重置为0*/
			
 
/*这个寄存器允许用户使能或使能 FIFO 缓冲区，
 *I2C 主机模式和主要 I2C 接口。FIFO 缓冲
区，I2C 主机，传感器信号通道和传感器寄存器也可以使用这个寄存器复位*/
#define MPU6050_RA_USER_CTRL        0x6A
//bit7  DMP禁止
//bit6  当此位设置为0,FIFO缓冲是禁用的
//bit5  当这个模式被启用,MPU-60X0即成为辅助I2C总线上的外部传感器slave设备的I2C主机
//		当此位被清除为0时,辅助I2C总线线路(AUX_DA and AUX_CL)理论上是由I2C总线
//		(SDA和SCL)驱动的。这是启用旁路模式的一个前提
//bit4  I2C转换至SPI模式(只允许MPU-6000)
//bit3  重置DMP模式,官方文档未说明的寄存器
//bit2  重置FIFO当设置为1时，此位将重置FIFO缓冲区，此时FIFO_EN等于0。触发重置后，此位将自动清为0
//bit1	重置I2C主机当设置为1时，此位将重置I2C主机，此时I2C_MST_EN等于0。触发重置后，此位将自动清为0
//bit0  重置所有传感器寄存器和信号路径 如果只重置信号路径（不重置传感器寄存器），请使用寄存器104
 
 
/*允许用户配置电源模式和时钟源。还提供了复位整个设备和禁用温度传感器的位*/
#define MPU6050_RA_PWR_MGMT_1       0x6B
//bit7  触发一个设备的完整重置。 触发重置后，一个~ 50 毫秒的小延迟是合理的
//bit6  寄存器的SLEEP位设置使设备处于非常低功率的休眠模式。
//bit5  唤醒周期启用状态当此位设为1且SLEEP禁用时.在休眠模式和唤醒模式间循环，以此从活跃的传感器中获取数据样本
//bit3  温度传感器启用状态控制内部温度传感器的使用
//bit2-bit0 设定时钟源设置,一个频率为8 mhz的内部振荡器,基于陀螺仪的时钟或外部信息源都可以被选为MPU-60X0的时钟源
			/* CLK_SEL | 时钟源
			* --------+--------------------------------------
			* 0			 | 内部振荡器
			* 1			 | PLL with X Gyro reference
			* 2			 | PLL with Y Gyro reference
			* 3			 | PLL with Z Gyro reference
			* 4			 | PLL with external 32.768kHz reference
			* 5			 | PLL with external 19.2MHz reference
			* 6			 | Reserved
			* 7			 | Stops the clock and keeps the timing generator in reset
			* */
 
 
/*这个寄存器允许用户配置加速度计在低功耗模式下唤起的频率。也允许用户让加速度计和
陀螺仪的个别轴进入待机模式。*/
#define MPU6050_RA_PWR_MGMT_2       0x6C
//bit7-bit6 Accel-Only低电量模式下的唤醒频率
			/* 通过把Power Management 1寄存器（寄存器107）中的PWRSEL设为1，
			* MPU-60X0可以处于Accerlerometer Only的低电量模式。在这种模式下,
			设备将关闭除了原I2C接口以外的所有设备，只留下accelerometer以固定时间
			间隔醒来进行测量。唤醒频率可用LP_WAKE_CTRL进行配置，如下表所示：
			* LP_WAKE_CTRL | 　唤醒频率
			* -------------+------------------
			* 0            | 1.25 Hz
			* 1            | 2.5 Hz
			* 2            | 5 Hz
			* 3            | 10 Hz
			* */
//bit5  备用的x轴加速度传感器启用状态,也就是进入待机模式
//bit4  备用的Y轴加速度传感器启用状态
//bit3  备用的Z轴加速度传感器启用状态
//bit2  备用的x轴陀螺仪启用状态
//bit1  备用的Y轴陀螺仪启用状态
//bit0  备用的Z轴陀螺仪启用状态
 
/*设定DMP模式下的bank*/
#define MPU6050_RA_BANK_SEL         0x6D
//DMP内存配置
#define MPU6050_BANKSEL_PRFTCH_EN_BIT       6
#define MPU6050_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU6050_BANKSEL_MEM_SEL_BIT         4
#define MPU6050_BANKSEL_MEM_SEL_LENGTH      5
//dmp内存地址设置
#define MPU6050_DMP_MEMORY_BANKS        8
#define MPU6050_DMP_MEMORY_BANK_SIZE    256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16
 
/*设定DMP模式下的起始地址*/
#define MPU6050_RA_MEM_START_ADDR   0x6E
/*一个字节的dmp数据缓存*/
#define MPU6050_RA_MEM_R_W          0x6F
/*DMP配置寄存器1*/
#define MPU6050_RA_DMP_CFG_1        0x70
/*DMP配置寄存器2*/
#define MPU6050_RA_DMP_CFG_2        0x71
 
/*当前FIFO缓冲区大小
* 这个值表明了存储于FIFO缓冲区的字节数。
* 而这个数字也是能从FIFO缓冲区读取的字节数，
* 它与存储在FIFO(寄存器35和36)中的传感器数据组所提供的可用样本数成正比。
* 两个寄存器一起构成一个16位数据*/
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
 
/*这个寄存器用于从FIFO缓冲区中读取和编写数据。数据在寄存器编号(从低到高)的指
 *令下编写入数据写入FIFO。如果所有的FIFO启用标志(见下文)都被启用了且
 *所有外部传感器数据寄存器(寄存器73至寄存器96)都与一个slave设备相连
 *,那么寄存器59到寄存器96的内容都将在采样率的指令下编写。
* 当传感器数据寄存器（寄存器59到寄存器96）的相关FIFO启用标志在FIFO_EN 寄存
* 器35)中都设为1时，它们的内容将被写入FIFO缓冲区。在I2C_MST_CTRL (寄存器 36)
* 中能找到一个与I2C Slave 3相连的额外的传感器数据寄存器标志。
* 如果FIFO缓冲区溢出,状态位FIFO_OFLOW_INT自动设置为1。
* 此位位于INT_STATUS (寄存器58)中。当FIFO缓冲区溢出时,最早的数据将会丢失
* 而新数据将被写入FIFO。如果FIFO缓冲区为空, 读取将返回原来从FIFO中读取的
* 最后一个字节，直到有可用的新数据。用户应检查FIFO_COUNT,以确保不在FIFO缓冲为空时读取。*/
#define MPU6050_RA_FIFO_R_W         0x74
 
/*寄存器是用来验证设备的身份的 默认值是0X34*/
#define MPU6050_RA_WHO_AM_I         0x75
//bit6-bit1 设备身份验证 0x34 最高位和最低位都剔除掉
 
 
#include "sys.h"
/* exact-width signed integer types */
typedef   signed          char s8;
typedef   signed short     int s16;
typedef   signed           int s32;


    /* exact-width unsigned integer types */
typedef unsigned           char u8;
typedef unsigned short     int u16;
typedef unsigned           int u32;

#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //Êä³ö 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //ÊäÈë 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //Êä³ö 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //ÊäÈë 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //Êä³ö 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //ÊäÈë 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //Êä³ö 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //ÊäÈë 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //Êä³ö 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //ÊäÈë

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //Êä³ö 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //ÊäÈë

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //Êä³ö 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //ÊäÈë

 
 
typedef struct ACCELSTRUCT
{
    s16 accelX;
    s16 accelY;
    s16 accelZ;
}ACCELSTRUCT;
 
typedef struct GYROSTRUCT
{
    s16 gyroX;
    s16 gyroY;
    s16 gyroZ;
}GYROSTRUCT;
 
extern struct ACCELSTRUCT       accelStruct ;
extern struct GYROSTRUCT	gyroStruct ;
 
 
u8 MpuInit(void);
 
void MpuGetData(void);
 
 
 
#endif
 
