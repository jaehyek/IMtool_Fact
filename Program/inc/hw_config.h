/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : hw_config.h
* Author             : MCD Application Team
* Version            : V3.0.0
* Date               : 04/06/2009
* Description        : Hardware Configuration & Setup
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "usb_type.h"
#include "stm32f10x.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define MASS_MEMORY_START     0x04002000
#define BULK_MAX_PACKET_SIZE  0x00000040
#define LED_ON                0xF0
#define LED_OFF               0xFF

#define  RGB_R      GPIO_Pin_5
#define  RGB_B      GPIO_Pin_6
#define  RGB_G      GPIO_Pin_7
#define  BT_tx         GPIO_Pin_9
#define  BT_rx        GPIO_Pin_10
#define  BT_IO_1    GPIO_Pin_8
#define  BT_IO_2    GPIO_Pin_2
//#define  BT_IO_3    GPIO_Pin_14
#define IR_LED      GPIO_Pin_1

#define SHDN   GPIO_Pin_0 // PORTA
#define Charge_Check   GPIO_Pin_2 // PORTA
#define Charge_Iset   GPIO_Pin_3 // PORTA
#define ADC      GPIO_Pin_1
#define P_SW      GPIO_Pin_8

#define  SPI_NSS1    GPIO_Pin_4 // PORTA

#define  SPI_SCK      GPIO_Pin_5 // PORTA
#define  SPI_MISO    GPIO_Pin_6 // PORTA
#define  SPI_MOSI    GPIO_Pin_7 // PORTA

#define  SPI_NSS2    GPIO_Pin_0 // PORTB

#define STX				   0x80
#define PREAMBLE_SEQ	0
#define SIZE_SEQ		    1
#define CMD_SEQ			2
#define DATA_SEQ		    4
#define CHECKSUM_SEQ	5

/* Exported functions ------------------------------------------------------- */
unsigned char PaketDecoder(void);
void Set_System(void);
uint32_t CDC_Receive_DATA(void);
void Set_USBClock(void);
void USART1_Set(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void USB_Cable_Config (FunctionalState NewState);
 void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
 void TIM_Configuration(void);
void Send_Int(uint8_t Data);
void Set_SPI(void);
uint16_t GetSensor(void);
uint32_t Send_UART(uint8_t Data,uint8_t Send_length);
void Get_SerialNum(void);
void Send_BT(uint8_t data);
uint32_t CDC_Send_DATA (uint8_t *ptrBuffer, uint8_t Send_length);
/* External variables --------------------------------------------------------*/

#endif  /*__HW_CONFIG_H*/
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
