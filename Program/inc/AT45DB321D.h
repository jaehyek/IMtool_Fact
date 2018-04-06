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

#ifndef _AT45DB321D_H
#define _AT45DB321D_H

#include "stm32f10x.h"
#include "usb_type.h"
//////////////////////////////////////////////////////////////////////////
void Flash_Chip_Select(void);
void Flash_Chip_DeSelect(void);
int Flash_SPIx_Write(u8 addr, u8 reg_addr, u8 data);
int Flash_SPIx_Writes(u8 addr, u8 reg_addr, u8 len, u8* data);
u8 Flash_SPIx_Read(u8 addr, u8 reg_addr);
void Flash_Write(short page,short *Flash);
void Flash_Read(short page,short *Flash);
uint8_t SPI_SendByte(uint8_t byte);


#endif
