#ifndef _IO_I2C_H_
#define _IO_I2C_H_

#include "common.h"
#include "include.h"

#define IIC_SCL         PTE24
#define IIC_SDA         PTE25

#define IIC_SCL_H()         PTXn_T(IIC_SCL,OUT) = 1
#define IIC_SCL_L()         PTXn_T(IIC_SCL,OUT) = 0
#define IIC_SCL_DDR_OUT()   PTXn_T(IIC_SCL,DDR) = 1
#define IIC_SCL_DDR_IN()    PTXn_T(IIC_SCL,DDR) = 0
#define IIC_SDA_H()         PTXn_T(IIC_SDA,OUT) = 1
#define IIC_SDA_L()         PTXn_T(IIC_SDA,OUT) = 0
#define IIC_SDA_IN()        PTXn_T(IIC_SDA,IN)
#define IIC_SDA_DDR_OUT()   PTXn_T(IIC_SDA,DDR) = 1
#define IIC_SDA_DDR_IN()    PTXn_T(IIC_SDA,DDR) = 0

#define IIC_DELAY()     IIC_delay(400)


//IIC Functions
void IIC_Init(void);                				 
void IIC_Start(void);				
void IIC_Stop(void);	  			
void IIC_Send_Byte(uint8 txd);			
uint8 IIC_Read_Byte(unsigned char ack1);
uint8 IIC_Wait_Ack(void); 				
void IIC_Ack(void);					
void IIC_NAck(void);				

void IIC_Write_One_Byte(uint8 daddr,uint8 addr,uint8 data);
uint8 IIC_Read_One_Byte(uint8 daddr,uint8 addr);	 
unsigned char I2C_Readkey(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
uint8 IICwriteBytes(uint8 dev, uint8 reg, uint8 length, uint8* data);
uint8 IICwriteBits(uint8 dev,uint8 reg,uint8 bitStart,uint8 length,uint8 data);
uint8 IICwriteBit(uint8 dev,uint8 reg,uint8 bitNum,uint8 data);
uint8 IICreadBytes(uint8 dev, uint8 reg, uint8 length, uint8 *data);
uint8 IICreadByte(uint8 dev, uint8 reg, uint8 *data);


#endif

