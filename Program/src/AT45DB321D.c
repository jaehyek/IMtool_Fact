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

#include "AT45DB321D.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#include "stm32f1_delay.h"

//////////////////////////////////////////////////////////////////////////
//

//////////////////////////////////////////////////////////////////////////
//basic SPI driver for Flash

//////////////////////////////////////////////////////////////////////////



void Flash_Chip_Select(void)
{
    // Chip Select       
    GPIO_SetBits(GPIOB, GPIO_Pin_0);     
    delay_us(50);
    SPI_Cmd(SPI1, ENABLE);
    GPIO_ResetBits(GPIOB, GPIO_Pin_0); 
}

void Flash_Chip_DeSelect(void)
{    
    //  Chip DeSelect    
    delay_us(50);
    SPI_Cmd(SPI1, DISABLE);
    GPIO_SetBits(GPIOB, GPIO_Pin_0); 
}

uint8_t SPI_SendByte(uint8_t byte)
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);    
    SPI1->DR = byte; 
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);    
    
    SPI_I2S_ReceiveData(SPI1);
}

int Flash_SPIx_Write(u8 addr, u8 reg_addr, u8 data){
	Flash_Chip_Select();
	//SPI_I2S_SendData(SPI1,reg_addr);
      while((SPI1->SR & SPI_I2S_FLAG_TXE)==RESET);
    SPI1->DR = reg_addr; 
    while((SPI1->SR & SPI_I2S_FLAG_RXNE)==RESET);  
    
	//SPI_I2S_SendData(SPI1,data);
     while((SPI1->SR & SPI_I2S_FLAG_TXE)==RESET);
        SPI1->DR = data; 
    while((SPI1->SR & SPI_I2S_FLAG_RXNE)==RESET);  
    
	Flash_Chip_DeSelect();
	return 0;
}

int Flash_SPIx_Writes(u8 addr, u8 reg_addr, u8 len, u8* data){
	u32 i = 0;
	Flash_Chip_Select();
    
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
	Flash_Chip_DeSelect();
	return 0;
}

u8 Flash_SPIx_Read(u8 addr, u8 reg_addr)
{
	u8 data = 0;

	Flash_Chip_Select();
	//SPI_I2S_SendData(SPI1,0x80 | reg_addr);
      while((SPI1->SR & SPI_I2S_FLAG_TXE)==RESET);
    SPI1->DR = reg_addr; 
    while((SPI1->SR & SPI_I2S_FLAG_RXNE)==RESET);  
  //  SPI1->DR;
    
	//data = SPI_I2S_ReceiveData(SPI1);
     while((SPI1->SR & SPI_I2S_FLAG_TXE)==RESET);
        SPI1->DR = 0x00; 
        while((SPI1->SR & SPI_I2S_FLAG_RXNE)==RESET);  
        data = SPI1->DR;   
    
    
	Flash_Chip_DeSelect();
	return data;
}

int Flash_SPIx_Reads(u8 addr, u8 reg_addr, u8 len, u8* data){
	u32 i = 0;
	Flash_Chip_Select();
    
	//SPI_I2S_SendData(SPI1,Flash_I2C_READ | reg_addr);    
    while((SPI1->SR & SPI_I2S_FLAG_TXE)==RESET);
    SPI1->DR = reg_addr; 
    while((SPI1->SR & SPI_I2S_FLAG_RXNE)==RESET);  
    SPI1->DR;
    
	while(i < len){
        //data[i++] = SPI_I2S_ReceiveData(SPI1);
        while((SPI1->SR & SPI_I2S_FLAG_TXE)==RESET);
        SPI1->DR = 0x00; 
        while((SPI1->SR & SPI_I2S_FLAG_RXNE)==RESET);  
        data[i++] = SPI1->DR;                
	}
	Flash_Chip_DeSelect();
	return 0;
}

uint8_t check_busy(void)
{
    u32 i = 0;
	Flash_Chip_Select();
    	
    SPI_SendByte(0xd7);    
    i=SPI_SendByte(0x00); 
       
	return !(i&0x80);
}

void Flash_Write(short page,short *Flash)
{
   int num=0;
    uint32_t adr;

    adr = (uint32_t)page<<10;
    Flash_Chip_Select();
  
    SPI_SendByte(0x82);
    SPI_SendByte(adr>>16);
    SPI_SendByte(adr>>8);
    SPI_SendByte(adr);    
    
    for(num=0;num<512;num++)
    {
        SPI_SendByte( Flash[num+1]);    
    }
    Flash_Chip_DeSelect();
   // check_busy();
}
void Flash_Read(short page,short *Flash)
{
   int num=0;
   uint32_t adr;

    adr = (uint32_t)page<<10;
    Flash_Chip_Select();
    SPI_SendByte(0xd2);   
    SPI_SendByte(adr>>16);
    SPI_SendByte(adr>>8);
    SPI_SendByte(adr);     
      
    SPI_SendByte(0);   
    SPI_SendByte(0);   
    SPI_SendByte(0);   
    SPI_SendByte(0);   
    
    for(num=0;num<512;num++)
    {        
        Flash[num] = SPI_SendByte(0);
    }
Flash_Chip_DeSelect();
    
}
//////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////
//


