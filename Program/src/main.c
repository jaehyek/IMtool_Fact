/******************** (C) RBIOTECK************************* ********************
* File Name        : main.c
* Author             : RbioTeck GAIT Team
* Version           : V0.9.0
* Date                : 10/13/2015
* Description     : IMU Sensor
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "AT45DB321D.h"
#include "stm32f10x.h"
#include "usb_lib.h"
#include "usb_desc.h" 
#include "hw_config.h"
#include "usb_pwr.h"
#include "misc.h"
#include "stdlib.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_spi.h"
#include <math.h>
#include <mpu9250.h>
#include <MadgwickAHRS.h>
#include <stdlib.h>
#include <FastMath.h>
#include "stm32f1_delay.h"
#include "stm32f10x_usart.h"
#include "EKF.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_pwr.h"
#include <stdlib.h>
#include "EKF.h"
#include "calibration.h"

SPI_InitTypeDef  SPI_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

#define USE_SRCKF
#define RADTODEG(x) ((x) * 57.295779513082320876798154814105f)
#define DEGTORAD(x) ((x) * 0.01745329251994329576923690768489f)

#define ID_No       01
#define SERIAL_No   02
#define PRODUCT_No   06
#define VIRSION     0x09
#define GYRO_BIAS   0x0a
#define GYRO_SCALE  0x10
#define ACC_BIAS    0x1c
#define ACC_SCALE   0x22
#define MAG_BIAS    0x28
#define MAG_SCALE   0x2e


int test_gyro=0;
int TEST,TEST_Temp,TEST_Gyro[3];
int Charger,Charger_cnt=0;
uint32_t sec=0,min=0,t1=0;
/**************************************************************/
/* IMU State                                           */
/**************************************************************/
uint8_t Boot_type=0;
uint8_t Serial_No=0,Product_No,Virsion=10;
uint8_t RGB_cnt=0;

/**************************************************************/
/* USB Communication Variable                                             */
/**************************************************************/
extern __IO uint32_t count_out;
extern __IO uint8_t Receive_Buffer[64];
extern __IO  uint32_t Receive_length ;
extern uint8_t buffer_out[VIRTUAL_COM_PORT_DATA_SIZE];
uint8_t a= 0x10;

__IO uint8_t Receive_Buffer[64];
extern __IO  uint32_t Receive_length ;
extern __IO  uint32_t length ;
int Send_cnt=0, charge_sw=0;

uint8_t START=0,POWER_ON=0;
/**************************************************************/
/* USART Communication Variable                                          */
/**************************************************************/
uint8_t Send_Buffer[64],test_c;
//uint32_t count_in=1;
uint32_t packet_receive=1;
uint32_t count_in=1;
uint32_t  Timer=0,tt=0;
uint8_t Sens_State=0,Check_Sum=0,temp,tc,temp1,result;

int   num=0,mem_state=0,ii,Power_SW=0,Power_SW_Cnt=0;
double tmp, norm;
int send_ID=0;;
short Rxbuf,BT_State,Temperature[1];;

/**************************************************************/
/* Calibration Variable                                                          */
/**************************************************************/
double gyro_cal=0,cal_mode_ON=0;;   
double test_roll,test_pitch,test_yaw,test_cnt=0,
          final_roll[50],final_pitch[50],final_yaw[50],rotation_z[50],rotation=0,test_roll_min=10,test_roll_max=-10,cal_cnt=0;
short Sensor_Param[528];
int STOP=0,Test_mode=0;;
int ft_cnt=0;
int ID = 0x00;
int mode=0;
double gyro_scale_value=0;
double gyro_scale[2];

/****************************************************************/
/* AHRS Variable                                                */
/****************************************************************/
short Gyro_Raw[3],Acc_Raw[3],Mag_Raw[3];
double x[7]={1,0,0,0,};
double q[4]={1,0,0,0,};
double p[4]={1,0,0,0,};
double gx,gy,gz,ax,ay,az,mx,my,mz;   
double HDR_gyro[3],HDR_acc[3],HDR_mag[3],Cal_gyro[3];
double accl_bias[3],accl_cal[3];
double _HDR_I[3];
double R1,P1,Y1;
double beta = 0.3f;								// 2 * proportional gain (Kp)
double sampleFreq=50;
double pre_gyro[3],pre_acc[4],pre_mag[3];
int AHRS_NUM=0;
double unit=0,test=0;
/****************************************************************/
/* AHRS Angle Variable                                          */
/****************************************************************/
double pitch=0,roll=0,yaw=0,pre_roll,pre_pitch,pre_yaw;
double pitch_Ang=0,roll_Ang=0,yaw_Ang=0;
double pitch1,pitch2,pitch3;
double roll1,roll2,roll3;
double yaw1,yaw2,yaw3;
double Acc_A1,Acc_A2,Acc_A3;
double Acc_B1,Acc_B2,Acc_B3;
double Acc_C1,Acc_C2,Acc_C3;
double yaw_rotation,roll_rotation;

// Quaternion
int  quat_0,quat_1,quat_2,quat_3;
int gyro_cnt=0;
double Mag_bias[3],Mag_cal[3],max_mag[3]={-100,-100,-100},min_mag[3]={100,100,100};;
double Acc_bias[3]={0,0,0},Acc_Cal[3]={1.0,1.0,1.0};
double max_acc[3]={-100,-100,-100},min_acc[3]={100,100,100};;
double Gyro_bias[3]={0,0,0},Mag_bias[3]={0,0,0},Gyro_scale= 0;
short RealGyro[3]={0},RealAcc[3]={0},RealMag[3]={0};

/****************************************************************/
/* Packer Define                                                */
/****************************************************************/
unsigned char PaketDecoder(void);
unsigned char ReceiveMode=0, PreambleCnt, SCommand=0, CheckSum, DataCnt, S_Size,EndFrame,feed;
unsigned char s_DataBuf[20]; 
#define STX             0x80
#define PREAMBLE_SEQ	0
#define SIZE_SEQ        1
#define CMD_SEQ			2
#define DATA_SEQ        4
#define CHECKSUM_SEQ	5

void RGB(uint16_t LED)
{
     GPIO_ResetBits(GPIOB,LED);
     GPIO_SetBits(GPIOB,0x00e0-LED);
}

int main(void)
{              
    Set_System(); 
          
    GPIO_SetBits(GPIOA,SHDN);

    TIM_Configuration();
    //RCC_PCLK2Config(RCC_HCLK_Div2);
           
    /* Enable peripheral clocks --------------------------------------------------*/
    /* GPIOA, GPIOB and SPI1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | 
                         RCC_APB2Periph_SPI1|RCC_APB2Periph_AFIO, ENABLE);

    USART1_Set();
    Set_SPI();

    //delay_ms(2000);
  /*  while(1)
    {
      
        if(Power_SW_Cnt>5000)
        {
            GPIO_ResetBits(GPIOB,RGB_G|RGB_B|RGB_R);
            Power_SW_Cnt=0;                    
            charge_sw = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2);
                        
            while(Power_SW!=0)
            {
                Power_SW = GPIO_ReadInputDataBit(GPIOA,P_SW);
            }                       
            break;
        }
        else if(Power_SW_Cnt==0)
         {
             GPIO_SetBits(GPIOB,RGB_R|RGB_G|RGB_B);
             GPIO_ResetBits(GPIOA,SHDN);
         }           
    }    */
    while (1)
    {
        uint8_t bt_cnt=0;
        POWER_ON=1;
      

        if(mode==0)
        {       
            RGB(RGB_R);
            
            for(bt_cnt=0;bt_cnt<5;bt_cnt++)
            {
                Send_BT('A');
                Send_BT('T');
                Send_BT('+');
                
                Send_BT('B');
                Send_BT('T');
                Send_BT('U');
                Send_BT('A');
                Send_BT('R');
                          
                Send_BT('T');
                Send_BT('=');
                Send_BT('1');
                Send_BT('1');         
                Send_BT('5');        
                Send_BT('2');
                Send_BT('0');        
                Send_BT('0');        
                Send_BT(0x0d);
                delay_ms(300);
            }        
            RGB(RGB_B);
            Sensor_Param[ID_No] = 0x83;
            Sensor_Param[SERIAL_No+0]=0;
            Sensor_Param[SERIAL_No+1]=0;
            Sensor_Param[SERIAL_No+2]=0;
            Sensor_Param[SERIAL_No+3]=0;
            
            Sensor_Param[PRODUCT_No]=1;
            Sensor_Param[PRODUCT_No+1]=1;
            Sensor_Param[PRODUCT_No+2]=1;
            
            Sensor_Param[VIRSION]=21;
            
            Sensor_Param[GYRO_BIAS+0] = 0;
            Sensor_Param[GYRO_BIAS+1] = 0;
            Sensor_Param[GYRO_BIAS+2] = 0;
            Sensor_Param[GYRO_BIAS+3] = 0;
            Sensor_Param[GYRO_BIAS+4] = 0;
            Sensor_Param[GYRO_BIAS+5] = 0;
            
            Sensor_Param[GYRO_SCALE+0] = 10000/0x100;
            Sensor_Param[GYRO_SCALE+1] = 10000%0x100;
            Sensor_Param[GYRO_SCALE+2] = 10000/0x100;;
            Sensor_Param[GYRO_SCALE+3] = 10000%0x100;;
            Sensor_Param[GYRO_SCALE+4] = 10000/0x100;;
            Sensor_Param[GYRO_SCALE+5] = 10000%0x100;;
						
						Sensor_Param[GYRO_SCALE+6] = 10000/0x100;;
            Sensor_Param[GYRO_SCALE+7] = 10000%0x100;;
            Sensor_Param[GYRO_SCALE+8] = 10000/0x100;;
            Sensor_Param[GYRO_SCALE+9] = 10000%0x100;;
            Sensor_Param[GYRO_SCALE+10] = 10000/0x100;;
            Sensor_Param[GYRO_SCALE+11] = 1000%0x100;;
            
            Sensor_Param[ACC_BIAS+0] = 0;
            Sensor_Param[ACC_BIAS+1] = 0;
            Sensor_Param[ACC_BIAS+2] = 0;
            Sensor_Param[ACC_BIAS+3] = 0;
            Sensor_Param[ACC_BIAS+4] = 0;
            Sensor_Param[ACC_BIAS+5] = 0;
            
            Sensor_Param[ACC_SCALE+0] = 0;
            Sensor_Param[ACC_SCALE+1] = 100;
            Sensor_Param[ACC_SCALE+2] = 0;
            Sensor_Param[ACC_SCALE+3] = 100;
            Sensor_Param[ACC_SCALE+4] = 0;
            Sensor_Param[ACC_SCALE+5] = 100;
            
            Flash_Write(1,Sensor_Param);
            mode=1;
        }
        else if(mode ==1)
        {
            RGB(RGB_B);
            delay_ms(1000);
            mode=2;
        }
         else if(mode ==2)
         {
             RGB(RGB_G);
         }
    }
        
}





#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
    
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/



