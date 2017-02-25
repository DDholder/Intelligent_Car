/* IOI2C.c file
功能：
提供I2C接口操作API 。
使用IO模拟方式
------------------------------------
 */
#include "IO_I2C.h"
#include "include.h"
static void IIC_delay(volatile uint16_t time)
{
    while(time)
    {
        time--;
    }
}
  
/**************************实现函数********************************************
*函数原型:		void IIC_Init(void)
*功　　能:		初始化I2C对应的接口引脚。
*******************************************************************************/
void IIC_Init(void)
{			
    gpio_init  (PTE24, GPO, 1); //初始化SCL0
    gpio_init  (PTE25, GPO, 1); //初始化SDA0
    port_init_NoALT(PTE24,ODO | PULLUP);
    port_init_NoALT(PTE25,ODO | PULLUP);
}

/**************************实现函数********************************************
*函数原型:		void IIC_Start(void)
*功　　能:		产生IIC起始信号
*******************************************************************************/
void IIC_Start(void)
{
	IIC_SDA_DDR_OUT();     
	IIC_SDA_H();	  	  
	IIC_SCL_H();
	IIC_DELAY();
 	IIC_SDA_L();//START:when CLK is high,DATA change form high to low 
	IIC_DELAY();
	IIC_SCL_L();
}

/**************************实现函数********************************************
*函数原型:		void IIC_Stop(void)
*功　　能:	    //产生IIC停止信号
*******************************************************************************/	  
void IIC_Stop(void)
{
	IIC_SDA_DDR_OUT();
	IIC_SCL_L();
	IIC_SDA_L();//STOP:when CLK is high DATA change form low to high
 	IIC_DELAY();
	IIC_SCL_H(); 
	IIC_SDA_H();
	IIC_DELAY();							   	
}

/**************************实现函数********************************************
*函数原型:		u8 IIC_Wait_Ack(void)
*功　　能:	    等待应答信号到来 
//返回值：1，接收应答失败
//        0，接收应答成功
*******************************************************************************/
uint8 IIC_Wait_Ack(void)
{
	uint8 ucErrTime=0;
	IIC_SDA_DDR_IN();     
	IIC_SDA_H(); IIC_DELAY();	   
	IIC_SCL_H(); IIC_DELAY();	 
	while(SDA_IN())
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 1;
		}
	  IIC_DELAY();
	}
	IIC_SCL_L();//ê±?óê?3?0 	   
	return 0;  
} 

/**************************实现函数********************************************
*函数原型:		void IIC_Ack(void)
*功　　能:	    产生ACK应答
*******************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL_L();
	IIC_SDA_DDR_OUT();
	IIC_SDA_L();
	IIC_DELAY();
	IIC_SCL_H();
	IIC_DELAY();
	IIC_SCL_L();
}
	
/**************************实现函数********************************************
*函数原型:		void IIC_NAck(void)
*功　　能:	    产生NACK应答
*******************************************************************************/	    
void IIC_NAck(void)
{
	IIC_SCL_L();
	IIC_SDA_DDR_OUT();
	IIC_SDA_H();
	IIC_DELAY();
	IIC_SCL_H();
	IIC_DELAY();
	IIC_SCL_L();
}					 				     

/**************************实现函数********************************************
*函数原型:		void IIC_Send_Byte(u8 txd)
*功　　能:	    IIC发送一个字节
*******************************************************************************/		  
void IIC_Send_Byte(uint8 txd)
{                        
    uint8 t=8;   
    IIC_SDA_DDR_OUT(); 	    
    IIC_SCL_L();
    while(t--)
    {
        if(txd&0x80)
        {
            IIC_SDA_H();
        }
        else
        {
            IIC_SDA_L();
        }
        txd<<=1; 
        IIC_DELAY();   
	IIC_SCL_H();
	IIC_DELAY(); 
	IIC_SCL_L();	
	//IIC_delay();
    }
} 	 
   
/**************************实现函数********************************************
*函数原型:		u8 IIC_Read_Byte(unsigned char ack)
*功　　能:	    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
*******************************************************************************/ 
uint8 IIC_Read_Byte(unsigned char ack1)
{
    unsigned char i,receive=0;
    IIC_SDA_DDR_IN();
    for(i=0;i<8;i++ )
    {
        IIC_SCL_L(); 
        IIC_DELAY();
	IIC_SCL_H();
        receive<<=1;
        if(IIC_SDA_IN())receive++;   
	IIC_DELAY(); 
    }					 
    if (ack1)
        IIC_Ack(); 
    else
        IIC_NAck();
    return receive;
}


/**************************实现函数********************************************
*函数原型:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*功　　能:	    读取指定设备 指定寄存器的一个值
输入	I2C_Addr  目标设备地址
		addr	   寄存器地址
返回   读出来的值
*******************************************************************************/ 
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	IIC_Start();	
	IIC_Send_Byte(I2C_Addr);	   
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  
	IIC_Wait_Ack();	  
	//IIC_Stop();
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++; 	   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
    IIC_Stop();

	return res;
}


/**************************实现函数********************************************
*函数原型:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*功　　能:	    读取指定设备 指定寄存器的 length个值
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要读的字节数
		*data  读出的数据将要存放的指针
返回   读出来的字节数量
*******************************************************************************/
uint8 IICreadBytes(uint8 dev, uint8 reg, uint8 length, uint8 *data){
    uint8 count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev);	   
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   
    IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte(dev+1);  
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=IIC_Read_Byte(1);  
		 	else  data[count]=IIC_Read_Byte(0);	 
	}
    IIC_Stop();
    return count;
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*功　　能:	    将多个字节写入指定设备 指定寄存器
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要写的字节数
		*data  将要写的数据的首地址
返回   返回是否成功
*******************************************************************************/ 
uint8 IICwriteBytes(uint8 dev, uint8 reg, uint8 length, uint8* data){
  
 	uint8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	   
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   
    IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
	 }
	IIC_Stop();

    return 1; //status == 0;
}

/**************************实现函数********************************************
*函数原型:		u8 IICreadByte(u8 dev, u8 reg, u8 *data)
*功　　能:	    读取指定设备 指定寄存器的一个值
输入	dev  目标设备地址
		reg	   寄存器地址
		*data  读出的数据将要存放的地址
返回   1
*******************************************************************************/
uint8 IICreadByte(uint8 dev, uint8 reg, uint8 *data){
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************实现函数********************************************
*函数原型:		unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*功　　能:	    写入指定设备 指定寄存器一个字节
输入	dev  目标设备地址
		reg	   寄存器地址
		data  将要写入的字节
返回   1
*******************************************************************************/
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	dev  目标设备地址
		reg	   寄存器地址
		bitStart  目标字节的起始位
		length   位长度
		data    存放改变目标字节位的值
返回   成功 为1 
 		失败为0
*******************************************************************************/
uint8 IICwriteBits(uint8 dev,uint8 reg,uint8 bitStart,uint8 length,uint8 data)
{

    uint8 b;
    if (IICreadByte(dev, reg, &b) != 0) {
        uint8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	dev  目标设备地址
		reg	   寄存器地址
		bitNum  要修改目标字节的bitNum位
		data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1 
 		失败为0
*******************************************************************************/ 
uint8 IICwriteBit(uint8 dev, uint8 reg, uint8 bitNum, uint8 data){
    uint8 b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}



/*********************************MS5611 IIC**********************************
**MS5611 IIC Read & Write Functions
**(1) MS5611 Write Data
**(2) MS5611 Read 2 Bytes
**(3) MS5611 Read 3 Bytes
*******************************************************************************/
uint8 ms5611_IICwriteData(uint8 dev, uint8 data)
{
    IIC_Start();
    IIC_Send_Byte(dev);
    IIC_Wait_Ack();
    IIC_Send_Byte(data);
    IIC_Wait_Ack();
    IIC_Stop();
    return 1;
}

uint16 ms5611_IICread2Data(uint8 dev)
{
    uint8 temp1, temp2;
    IIC_Start();
    IIC_Send_Byte(dev+1);
    IIC_Wait_Ack();
    temp1 = IIC_Read_Byte(1);
    temp2 = IIC_Read_Byte(0);
    IIC_Stop();
    return (temp1<<8 | temp2);
}

uint32 ms5611_IICread3Data(uint8 dev)
{
    uint8 temp1, temp2, temp3;
    IIC_Start();
    IIC_Send_Byte(dev+1);
    IIC_Wait_Ack();
    temp1 = IIC_Read_Byte(1);
    temp2 = IIC_Read_Byte(1);
    temp3 = IIC_Read_Byte(0);
    IIC_Stop();
    return (temp1<<16 | temp2 <<8 | temp3);
}
//------------------End of File----------------------------
