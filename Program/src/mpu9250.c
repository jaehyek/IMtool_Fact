/*
The MIT License (MIT)

Copyright (c) 2015-? suhetao

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "mpu9250.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#include "stm32f1_delay.h"

//////////////////////////////////////////////////////////////////////////
//
//static s16 MPU9250_AK8963_ASA[3] = {0, 0, 0};
//////////////////////////////////////////////////////////////////////////
//basic SPI driver for MPU9250

//////////////////////////////////////////////////////////////////////////
//
#define MPU9250_SPIx_SendByte(byte) SPIx_SendByte(pMPU9250, byte);
#define MPU9250_SPIx_SetDivisor(divisor) SPIx_SetDivisor(pMPU9250, divisor);
uint8_t Mmode = 0x06;  
uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
//////////////////////////////////////////////////////////////////////////
//init
void MPU9250_Init(double *destination)
{u8 data = 0, state = 0;
	uint8_t response[3] = {0, 0, 0};
	//Lower level hardware Init

	//////////////////////////////////////////////////////////////////////////
	//MPU9250 Reset
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_1, MPU9250_RESET);
	delay_ms(100);
	//MPU9250 Set Clock Source
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_1,  MPU9250_CLOCK_PLLGYROZ);
	delay_ms(1);
	//MPU9250 Set Interrupt
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_INT_PIN_CFG,  MPU9250_INT_ANYRD_2CLEAR);
	delay_ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_INT_ENABLE, ENABLE);
	delay_ms(1);
	//MPU9250 Set Sensors
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_2, MPU9250_XYZ_GYRO & MPU9250_XYZ_ACCEL);
	delay_ms(1);
	//MPU9250 Set SampleRate
	//SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_SMPLRT_DIV, SMPLRT_DIV);
	delay_ms(1);
	//MPU9250 Set Full Scale Gyro Range
	//Fchoice_b[1:0] = [00] enable DLPF
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_GYRO_CONFIG, (MPU9250_FSR_2000DPS << 3));
	delay_ms(1);
	//MPU9250 Set Full Scale Accel Range PS:2G
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG, (MPU9250_FSR_16G << 3));
	delay_ms(1);
	//MPU9250 Set Accel DLPF
	data = MPU9250_SPIx_Read(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG2);
	data |= MPU9250_ACCEL_DLPF_5HZ;
	delay_ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG2, data);
	delay_ms(1);
	//MPU9250 Set Gyro DLPF
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_CONFIG, MPU9250_GYRO_DLPF_5HZ);
	delay_ms(1);
	//MPU9250 Set SPI Mode
	state = MPU9250_SPIx_Read(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL);
	delay_ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL, state | MPU9250_I2C_IF_DIS);
	delay_ms(1);
	state = MPU9250_SPIx_Read(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL);
	delay_ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL, state | MPU9250_I2C_MST_EN);
	delay_ms(1);
	//////////////////////////////////////////////////////////////////////////
	//AK8963 Setup
	//reset AK8963
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL2, MPU9250_AK8963_CNTL2_SRST);
	delay_ms(2);
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	delay_ms(1);
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_FUSE_ROM_ACCESS);
	delay_ms(1);
    MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MFS_16BITS<<4|0x06);
    delay_ms(1);
	//
	//AK8963 get calibration data
	MPU9250_AK8963_SPIx_Reads(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_ASAX, 3, response);
	//AK8963_SENSITIVITY_SCALE_FACTOR
	//AK8963_ASA[i++] = (s16)((data - 128.0f) / 256.0f + 1.0f) ;
	destination[0] = (s16)((response[0] - 128.0f) / 256.0f + 1.0f) ;
	destination[1] = (s16)((response[1] - 128.0f) / 256.0f + 1.0f) ;
	destination[2] = (s16)((response[2] - 128.0f) / 256.0f + 1.0f) ;
	delay_ms(1);
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	delay_ms(1);
	//
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_CTRL, 0x5D);
	delay_ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | MPU9250_I2C_READ);
	delay_ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_REG, MPU9250_AK8963_ST1);
	delay_ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_CTRL, 0x88);
	delay_ms(1);
	//
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_CONTINUOUS_MEASUREMENT);
	delay_ms(1);

	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 0x09);
	delay_ms(1);
	//
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_DELAY_CTRL, 0x81);
	delay_ms(100);
	
}

void Chip_Select(void)
{
    // Chip Select       
    GPIO_SetBits(GPIOA, GPIO_Pin_4);     
    delay_us(50);
    SPI_Cmd(SPI1, ENABLE);
    GPIO_ResetBits(GPIOA, GPIO_Pin_4); 
}

void Chip_DeSelect(void)
{    
    //  Chip DeSelect    
    delay_us(50);
    SPI_Cmd(SPI1, DISABLE);
    GPIO_SetBits(GPIOA, GPIO_Pin_4); 
}

int MPU9250_SPIx_Write(u8 addr, u8 reg_addr, u8 data){
	Chip_Select();
	//SPI_I2S_SendData(SPI1,reg_addr);
      while((SPI1->SR & SPI_I2S_FLAG_TXE)==RESET);
    SPI1->DR = reg_addr; 
    while((SPI1->SR & SPI_I2S_FLAG_RXNE)==RESET);  
    
	//SPI_I2S_SendData(SPI1,data);
     while((SPI1->SR & SPI_I2S_FLAG_TXE)==RESET);
        SPI1->DR = data; 
    while((SPI1->SR & SPI_I2S_FLAG_RXNE)==RESET);  
    
	Chip_DeSelect();
	return 0;
}

int MPU9250_SPIx_Writes(u8 addr, u8 reg_addr, u8 len, u8* data){
	u32 i = 0;
	Chip_Select();
    
	//SPI_I2S_SendData(SPI1,reg_addr);
     while((SPI1->SR & SPI_I2S_FLAG_TXE)==RESET);
    SPI1->DR = reg_addr; 
    while((SPI1->SR & SPI_I2S_FLAG_RXNE)==RESET);  
  
    
	while(i < len){
        
		//SPI_I2S_SendData(SPI1,data[i++]);
         while((SPI1->SR & SPI_I2S_FLAG_TXE)==RESET);
        SPI1->DR = data[i++]; 
    while((SPI1->SR & SPI_I2S_FLAG_RXNE)==RESET);  
        
	}
	Chip_DeSelect();
	return 0;
}

u8 MPU9250_SPIx_Read(u8 addr, u8 reg_addr)
{
	u8 data = 0;

	Chip_Select();
	//SPI_I2S_SendData(SPI1,0x80 | reg_addr);
      while((SPI1->SR & SPI_I2S_FLAG_TXE)==RESET);
    SPI1->DR = MPU9250_I2C_READ | reg_addr; 
    while((SPI1->SR & SPI_I2S_FLAG_RXNE)==RESET);  
  //  SPI1->DR;
    
	//data = SPI_I2S_ReceiveData(SPI1);
     while((SPI1->SR & SPI_I2S_FLAG_TXE)==RESET);
        SPI1->DR = 0x00; 
        while((SPI1->SR & SPI_I2S_FLAG_RXNE)==RESET);  
        data = SPI1->DR;   
    
    
	Chip_DeSelect();
	return data;
}

int MPU9250_SPIx_Reads(u8 addr, u8 reg_addr, u8 len, u8* data){
	u32 i = 0;
	Chip_Select();
    
	//SPI_I2S_SendData(SPI1,MPU9250_I2C_READ | reg_addr);    
    while((SPI1->SR & SPI_I2S_FLAG_TXE)==RESET);
    SPI1->DR = MPU9250_I2C_READ | reg_addr; 
    while((SPI1->SR & SPI_I2S_FLAG_RXNE)==RESET);  
    SPI1->DR;
    
	while(i < len){
        //data[i++] = SPI_I2S_ReceiveData(SPI1);
        while((SPI1->SR & SPI_I2S_FLAG_TXE)==RESET);
        SPI1->DR = 0x00; 
        while((SPI1->SR & SPI_I2S_FLAG_RXNE)==RESET);  
        data[i++] = SPI1->DR;                
	}
	Chip_DeSelect();
	return 0;
}


int MPU9250_AK8963_SPIx_Read(u8 akm_addr, u8 reg_addr, u8* data) {
	u8 status = 0;
	u32 timeout = 0;

	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &reg_addr);
	delay_ms(1);
	reg_addr = akm_addr | MPU9250_I2C_READ;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &reg_addr);
	delay_ms(1);
	reg_addr = MPU9250_I2C_SLV4_EN;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &reg_addr);
	delay_ms(1);

	do {
		if (timeout++ > 50){
			return -2;
		}
		MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
		delay_ms(1);
	} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DI, 1, data);
	return 0;
}

int MPU9250_AK8963_SPIx_Reads(u8 akm_addr, u8 reg_addr, u8 len, u8* data){
	u8 index = 0;
	u8 status = 0;
	u32 timeout = 0;
	u8 tmp = 0;

	tmp = akm_addr | MPU9250_I2C_READ;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	delay_ms(1);
	while(index < len){
		tmp = reg_addr + index;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
		delay_ms(1);
		tmp = MPU9250_I2C_SLV4_EN;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		delay_ms(1);

		do {
			if (timeout++ > 50){
				return -2;
			}
			MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
			delay_ms(2);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DI, 1, data + index);
		delay_ms(1);
		index++;
	}
	return 0;
}

int MPU9250_AK8963_SPIx_Write(u8 akm_addr, u8 reg_addr, u8 data)
{
	u32 timeout = 0;
	uint8_t status = 0;
	u8 tmp = 0;

	tmp = akm_addr;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	delay_ms(1);
	tmp = reg_addr;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
	delay_ms(1);
	tmp = data;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DO, 1, &tmp);
	delay_ms(1);
	tmp = MPU9250_I2C_SLV4_EN;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
	delay_ms(1);

	do {
		if (timeout++ > 50)
			return -2;

		MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
		delay_ms(1);
	} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
	if (status & MPU9250_I2C_SLV4_NACK)
		return -3;
	return 0;
}

int MPU9250_AK8963_SPIx_Writes(u8 akm_addr, u8 reg_addr, u8 len, u8* data)
{
	u32 timeout = 0;
	uint8_t status = 0;
	u8 tmp = 0;
	u8 index = 0;

	tmp = akm_addr;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	delay_ms(1);

	while(index < len){
		tmp = reg_addr + index;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
		delay_ms(1);
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DO, 1, data + index);
		delay_ms(1);
		tmp = MPU9250_I2C_SLV4_EN;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		delay_ms(1);

		do {
			if (timeout++ > 50)
				return -2;
			MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
			delay_ms(1);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		if (status & MPU9250_I2C_SLV4_NACK)
			return -3;
		index++;
	}
	return 0;
}
//////////////////////////////////////////////////////////////////////////
//
  void initAK8963(double *destination)
{
    uint8_t rawData[3];  // x/y/z gyro calibration data stored here
    MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	delay_ms(1);
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_FUSE_ROM_ACCESS);
	delay_ms(1);
	//
	//AK8963 get calibration data
	MPU9250_AK8963_SPIx_Reads(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_ASAX, 3, rawData);
	//AK8963_SENSITIVITY_SCALE_FACTOR
	//AK8963_ASA[i++] = (s16)((data - 128.0f) / 256.0f + 1.0f) ;
     destination[0] =  (s16)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (s16)(rawData[1] - 128)/256.0f + 1.0f;  
  destination[2] =  (s16)(rawData[2] - 128)/256.0f + 1.0f; 
	delay_ms(1);
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	delay_ms(1);
    
     MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, 0x16);
	delay_ms(1);
}


void MPU9250_Get9AxisRawData(short *accel, short * gyro, short * mag)
{
	u8 data[23];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_ACCEL_XOUT_H, 22, data);
	
	accel[0] = (data[1] << 8) | data[2];
	accel[1] = (data[3] << 8) | data[4];
	accel[2] = (data[5] << 8) | data[6];
	
	gyro[0] = (data[9] << 8) | data[10];
	gyro[1] = (data[11] << 8) | data[12];
	gyro[2] = (data[13] << 8) | data[14];

	

/*
	if (!(data[15] & MPU9250_AK8963_DATA_READY) || (data[15] & MPU9250_AK8963_DATA_OVERRUN)){
		return;
	}
	if (data[22] & MPU9250_AK8963_OVERFLOW){
		return;
	}*/
	mag[0] = (data[17] << 8) | data[16];    
	mag[1] = (data[19] << 8) | data[18];    
	mag[2] = (data[21] << 8) | data[20];

	//ned x,y,z
	//mag[0] = (mag[0] * MPU9250_AK8963_ASA[0]) >> 8;
	//mag[1] = (mag[1] * MPU9250_AK8963_ASA[1]) >> 8;
//	mag[2] = (mag[2] * MPU9250_AK8963_ASA[2]) >> 8;
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get6AxisRawData(short *accel, short * gyro)
{
	u8 data[14];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_ACCEL_XOUT_H, 14, data);
	
	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];

	gyro[0] = (data[8] << 8) | data[9];
	gyro[1] = (data[10] << 8) | data[11];
	gyro[2] = (data[12] << 8) | data[13];
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get3AxisAccelRawData(short * accel)
{
	u8 data[7];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_ACCEL_XOUT_H, 6, data);

	accel[0] = (data[1] << 8) | data[2];
	accel[1] = (data[3] << 8) | data[4];
	accel[2] = (data[5] << 8) | data[6];
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get3AxisGyroRawData(short * gyro)
{
	u8 data[6];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_GYRO_XOUT_H, 6, data);

	gyro[0] = (data[0] << 8) | data[1];
	gyro[1] = (data[2] << 8) | data[3];
	gyro[2] = (data[4] << 8) | data[5];
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get3AxisMagnetRawData(short *mag)
{
u8 data[7];
    //u8 c;
    //u8 state;

   // MPU9250_AK8963_SPIx_Read(MPU9250_SPIx_ADDR, MPU9250_AK8963_ST1, &state); 
 //   if(state&0x01)
   // {
        MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_AK8963_XOUT_L, 7, data);
    //    c = data[6];
     //   if(!(c& data[6]))
     //   {          
            mag[0] = (data[1] << 8) | data[0];
            mag[1] = (data[3] << 8) | data[2];
            mag[2] = (data[5] << 8) | data[4];
       // }
 //   }
/*	mag[0] = ((long)mag[0] * MPU9250_AK8963_ASA[0]) >> 8;
	mag[1] = ((long)mag[1] * MPU9250_AK8963_ASA[1]) >> 8;
	mag[2] = ((long)mag[2] * MPU9250_AK8963_ASA[2]) >> 8;*/
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_GetTemperatureRawData(short *temperature)
{
	u8 data[3];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_TEMP_OUT_H, 2, data);
	temperature[0] = (((s16)data[1]) << 8) | data[2];
}

static vu8 MPU9250_IsNewData = 0;

int MPU9250_IsDataReady(void)
{
	int isNewData = MPU9250_IsNewData;
	MPU9250_IsNewData = 0;
	return isNewData;
}

//////////////////////////////////////////////////////////////////////////
//
void EXTI9_5_IRQHandler(void) 
{
	if(EXTI_GetITStatus(EXTI_Line8) != RESET){
    EXTI_ClearITPendingBit(EXTI_Line8);
		MPU9250_IsNewData = 1;
  }
}

