/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : hw_config.c
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "platform_config.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "usb_pwr.h"
#include "stm32f1_delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

#define ADC1_DR_Address    ((uint32_t)0x4001244C)
__IO uint16_t ADCConvertedValue;
/* Extern variables ----------------------------------------------------------*/
uint8_t buffer_in[VIRTUAL_COM_PORT_DATA_SIZE];
extern uint32_t count_in;
extern LINE_CODING linecoding;
extern int ID;

NVIC_InitTypeDef NVIC_InitStructure;   
/*
#define STX				0x10
#define PREAMBLE_SEQ	0
#define STX_SEQ			1
#define SIZE_SEQ		2
#define CMD_SEQ			3
#define DATA_SEQ		4
#define CHECKSUM_SEQ	5
#define True	1
#define False	0

volatile unsigned int ReceiveMode, PreambleCnt, SCommand=0, CheckSum, DataCnt, S_Size;
volatile unsigned char Rxbuf=0;
volatile unsigned char s_DataBuf[10];  			// Packet내의 정보를 저장하는 Buffer
volatile unsigned char EndFrame=0;
    */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef  ADC_InitStruct;
    DMA_InitTypeDef DMA_InitStructure;
    SystemInit();       
  
    /* Enable GPIOE, GPIOF and GPIOG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);   
          
    /* Enable Timer1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    /*  Configure GPIO as input floating */
    GPIO_InitStructure.GPIO_Pin = RGB_R|RGB_G|RGB_B|IR_LED;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
      GPIO_SetBits(GPIOB,RGB_R|RGB_G|RGB_B);      
      
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin =Charge_Iset|Charge_Check;      
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
           
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin =SHDN|BT_IO_1|BT_IO_2;      
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
        
    GPIO_InitStructure.GPIO_Pin = BT_tx;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
     GPIO_InitStructure.GPIO_Pin = BT_rx|P_SW;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
      
     GPIO_InitStructure.GPIO_Pin = ADC;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
      
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
     /* DMA1 channel1 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 1;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    /* Enable DMA1 channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);
    
    ADC_InitStruct.ADC_Mode				= ADC_Mode_Independent;
	ADC_InitStruct.ADC_ScanConvMode		= ENABLE;;
	ADC_InitStruct.ADC_ContinuousConvMode	= ENABLE;;
	ADC_InitStruct.ADC_ExternalTrigConv		= ADC_ExternalTrigConv_None;
	ADC_InitStruct.ADC_DataAlign			= ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfChannel		= 1;
	ADC_Init(ADC1, &ADC_InitStruct);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);
    
    /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);       
}

void Set_SPI(void)
{
     SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    
        /* Configure SPI1 pins: SCK, MISO and MOSI ---------------------------------*/
    /* Confugure SCK and MOSI pins as Alternate Function Push Pull */
     GPIO_InitStructure.GPIO_Pin = SPI_SCK |SPI_MOSI|SPI_MISO;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* Confugure MISO pin as Input Floating  */
 //   GPIO_InitStructure.GPIO_Pin = ;
  //  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  //  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	GPIO_InitStructure.GPIO_Pin = SPI_NSS1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
  
    GPIO_InitStructure.GPIO_Pin = SPI_NSS2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
  
         /* SPI1 configuration ------------------------------------------------------*/
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);
    
    
}
/*******************************************************************************
* Function Name  : Set_USART1
* Description    : 
* Input          : None.
* Return         : None.
*******************************************************************************/
void USART1_Set(void)
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_init; 
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);     
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);          
    
    USART_ITConfig(USART1, USART_IT_RXNE,ENABLE);
    
    USART_Cmd(USART1, ENABLE);  //usart의 장치 활성화
    NVIC_init.NVIC_IRQChannel = USART1_IRQn;               //USART1 IRQ 설정 
    NVIC_init.NVIC_IRQChannelPreemptionPriority = 0x00;       // IRQ 우선 페리티비트 0으로 설정     
    NVIC_init.NVIC_IRQChannelSubPriority = 0x00;       // Sub 패리티  0으로 설정     
    NVIC_init.NVIC_IRQChannelCmd = ENABLE;                // IRQ Channel command enable     
    NVIC_Init(&NVIC_init);   
}



/*******************************************************************************
* Function Name  : Get_Battery
* Description    : Bettary currents (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
uint16_t GetSensor(void)
{
	uint16_t uResult;
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	uResult = ADC_GetConversionValue(ADC1);

	return uResult;
}

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
      /* USBCLK = PLLCLK / 1.5 */
      RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
      /* Enable USB clock */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
      /* Set the device state to suspend */
      bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
      DEVICE_INFO *pInfo = &Device_Info;

      /* Set the device state to the correct state */
      if (pInfo->Current_Configuration != 0)
      {
            /* Device configured */
            bDeviceState = CONFIGURED;
      }
      else
      {
            bDeviceState = ATTACHED;
      }
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
     // NVIC_InitTypeDef NVIC_InitStructure;

 
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_Init(&NVIC_InitStructure);     
}

/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
      if (NewState != DISABLE)
      {
            GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
      }
      else
      {
            GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
      }
}

/*******************************************************************************
* Function Name  : TIM_Configuration.
* Description    : Timer Configuration
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
extern int Power_SW_Cnt,Power_SW,Send_cnt;
extern uint8_t POWER_ON;
extern uint32_t sec,min,t1;

void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET)
    {
         Power_SW = GPIO_ReadInputDataBit(GPIOA,P_SW);
        Send_cnt++;
        t1++;
        if(t1>15000){
            t1=0;
            sec++;
        }   
        if(sec>60){
            sec=0;
            min++;
        }
         if(Power_SW==1)
         {
             Power_SW_Cnt++;
             if(POWER_ON==1)
             {
                 if(Power_SW_Cnt>40000)
                 {
                    Send_BT(ID);Send_BT(0x26);Send_BT(0x00);Send_BT(0x26);delay_ms(10);
                     Send_BT(ID);Send_BT(0x26);Send_BT(0x00);Send_BT(0x26);delay_ms(10);
                     Send_BT(ID);Send_BT(0x26);Send_BT(0x00);Send_BT(0x26);delay_ms(10);
                     Send_BT(ID);Send_BT(0x26);Send_BT(0x00);Send_BT(0x26);delay_ms(10);
                     
                     GPIO_SetBits(GPIOB,RGB_R|RGB_G|RGB_B);                 
                     GPIO_ResetBits(GPIOA,SHDN);
                 }
             }   
                    
         }else 
             Power_SW_Cnt=0;
        
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // Clear the interrupt flag
        
    }
}
//46
void TIM_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler = 72;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 20;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
    TIM_Cmd(TIM1,ENABLE);  

    // Clear updatae interrupt bit
    TIM_ClearITPendingBit(TIM1,TIM_FLAG_Update);
    // Enaable update interrupt  
    //  TIM_ITConfig(TIM1,TIM_FLAG_Update,ENABLE);                              	   
                   
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
    
    /* TIM2 Clock Enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
     
    /* Enable TIM2 Global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
     
    /* TIM2 Initialize */   
    TIM_TimeBaseStructure.TIM_Period=100-1; // 1kHz
    TIM_TimeBaseStructure.TIM_Prescaler=24-1; // 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision=0;
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
     
    /* TIM2 Enale */
    TIM_Cmd(TIM2,ENABLE);
    TIM_ITConfig(TIM2,TIM_IT_Update, ENABLE); // interrupt enable
}






/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
 void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
      uint8_t idx = 0;
  
      for( idx = 0 ; idx < len ; idx ++)
      {
            if( ((value >> 28)) < 0xA )
            {
                  pbuf[ 2* idx] = (value >> 28) + '0';
            }
            else
            {
                  pbuf[2* idx] = (value >> 28) + 'A' - 10; 
            }
    
            value = value << 4;
    
            pbuf[ 2* idx + 1] = 0;
      }
}



uint32_t Send_UART(uint8_t Data,uint8_t Send_length)
{
      /*if max buffer is Not reached*/
      if(Send_length < VIRTUAL_COM_PORT_DATA_SIZE)     
      {
            /*Sent flag*/
            count_in = 0;
            /* send  packet to PMA*/
            buffer_in[0] = Data;
            UserToPMABufferCopy(buffer_in, ENDP1_TXADDR, Send_length);
            SetEPTxCount(ENDP1, Send_length);
            SetEPTxValid(ENDP1);
      }
      else
      {
            return 0;
      } 

      return 1;

}

uint32_t CDC_Receive_DATA(void)
{
  /*Receive flag*/
  count_in = 0;
  SetEPRxValid(ENDP3);
  return 1 ;
}

/*******************************************************************************
* Function Name  :Send_Integer
* Description    : send inetager to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void Send_Int(uint8_t Data)
{         
      Send_UART(Data,1);  
    delay_us(1500);      
}

 void Send_BT(uint8_t data)
 {
    USART_SendData(USART1, data);   
    /* Loop until USARTy DR register is empty */ 
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}
 }  
 
/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
      uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

      Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
      Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
      Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);

      if (Device_Serial0 != 0)
      {
            Virtual_Com_Port_StringSerial[2] = (uint8_t)(Device_Serial0 & 0x000000FF);
            Virtual_Com_Port_StringSerial[4] = (uint8_t)((Device_Serial0 & 0x0000FF00) >> 8);
            Virtual_Com_Port_StringSerial[6] = (uint8_t)((Device_Serial0 & 0x00FF0000) >> 16);
            Virtual_Com_Port_StringSerial[8] = (uint8_t)((Device_Serial0 & 0xFF000000) >> 24);

            Virtual_Com_Port_StringSerial[10] = (uint8_t)(Device_Serial1 & 0x000000FF);
            Virtual_Com_Port_StringSerial[12] = (uint8_t)((Device_Serial1 & 0x0000FF00) >> 8);
            Virtual_Com_Port_StringSerial[14] = (uint8_t)((Device_Serial1 & 0x00FF0000) >> 16);
            Virtual_Com_Port_StringSerial[16] = (uint8_t)((Device_Serial1 & 0xFF000000) >> 24);

            Virtual_Com_Port_StringSerial[18] = (uint8_t)(Device_Serial2 & 0x000000FF);
            Virtual_Com_Port_StringSerial[20] = (uint8_t)((Device_Serial2 & 0x0000FF00) >> 8);
            Virtual_Com_Port_StringSerial[22] = (uint8_t)((Device_Serial2 & 0x00FF0000) >> 16);
            Virtual_Com_Port_StringSerial[24] = (uint8_t)((Device_Serial2 & 0xFF000000) >> 24);
  }
}

/*******************************************************************************
* Function Name  : Send DATA .
* Description    : send the data received from the STM32 to the PC through USB  
* Input          : None.read
* Output         : None.
* Return         : None.
*******************************************************************************/
uint32_t CDC_Send_DATA (uint8_t *ptrBuffer, uint8_t Send_length)
{
      /*if max buffer is Not reached*/
      if(Send_length < VIRTUAL_COM_PORT_DATA_SIZE)     
      {
            /*Sent flag*/
            count_in = 0;
            /* send  packet to PMA*/
            UserToPMABufferCopy((uint8_t*)ptrBuffer, ENDP1_TXADDR, Send_length);
            SetEPTxCount(ENDP1, Send_length);
            SetEPTxValid(ENDP1);
      }
      else
      {
            return 0;
      } 
      return 1;
}

/*******************************************************************************
* Function Name  : delay_us/ delay_ms .
* Description    : time delay
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
extern unsigned char PaketDecoder(void);
extern unsigned char ReceiveMode, PreambleCnt, SCommand, CheckSum, DataCnt, S_Size,EndFrame,feed;
extern unsigned char s_DataBuf[20]; 
extern short Rxbuf;
extern unsigned char PaketDecoder(void)
{	
    switch(ReceiveMode)
    {
        case PREAMBLE_SEQ   : if(Rxbuf > STX)
                              ReceiveMode = CMD_SEQ;                                                    
                              break;
            
        case CMD_SEQ        : SCommand = Rxbuf;
                              ReceiveMode = SIZE_SEQ;
                              CheckSum += Rxbuf;                                              
                              break;
            
        case SIZE_SEQ       : DataCnt =0;
                              S_Size = Rxbuf;
                              ReceiveMode = DATA_SEQ;
                              CheckSum += Rxbuf;
                              break;                 
            
        case DATA_SEQ       : if(S_Size)
                              {
                                   s_DataBuf[DataCnt++] = Rxbuf;
                                   S_Size--;
                                   CheckSum += Rxbuf;
                              }
                              else
                              {
                                  ReceiveMode = PREAMBLE_SEQ;
                                  if(Rxbuf == CheckSum){
                                      CheckSum = 0;
                                      EndFrame = 1;
                                  }
                                  else{
                                      CheckSum = 0;
                                      EndFrame = 0;
                                  }
                              }break;
    }
    
	return EndFrame;
}
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
