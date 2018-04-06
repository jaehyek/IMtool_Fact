/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : Calibration.c
* Author             : GAITS Application Team
* Version            : V3.0.0
* Date               : 04/07/2016
* Description        : 
********************************************************************************/
#include <mpu9250.h>
#include "Calibration.h"
#include "hw_config.h"
#include <FastMath.h>
#include "AT45DB321D.h"

#define ID_No       01
#define Serial_No   02
#define Product_1   06
#define VIRSION     09
#define GYRO_BIAS   10
#define GYRO_SCALE  16
#define ACC_BIAS    22
#define ACC_SCALE   28
#define MAG_BIAS    34
#define MAG_SCALE   40

extern short Gyro_Raw[3],Acc_Raw[3],Mag_Raw[3];
extern double Gyro_bias[3],Acc_bias[3],Mag_bias[3],Acc_Cal[3];
extern char SCommand, EndFrame;
extern short Sensor_Param[528];
double Temp_gyro[3]={0,0,0},Temp_acc[3]={0,0,0},Temp_mag[3]={0,0,0};
extern double max_acc[3],min_acc[3];
extern double Mag_bias[3],Mag_cal[3];
extern double max_mag[3],min_mag[3];
extern double HDR_gyro[3],HDR_acc[3],HDR_mag[3],Cal_gyro[3];
int Cal_t=0,t4=0;


double lpf1(double x, double a, double prev)
{
    // a : cut off frequency
    // x : row data  
    return (1-a)*prev + a*x;
}

void Wirte_ID(char No)
{
    Sensor_Param[ID_No] = (short)No;
    
     
}
void Gyro_Cali_bias()
{
    Temp_gyro[0] = 0; 
    Gyro_bias[0] = 0;
    Temp_gyro[1] = 0; 
    Gyro_bias[1] = 0;
    Temp_gyro[2] = 0;
    Gyro_bias[2] = 0; 
    while(1)
    {
        MPU9250_Get9AxisRawData(Acc_Raw,Gyro_Raw,Mag_Raw);
        Temp_gyro[0] = -(double)Gyro_Raw[1];            
        Temp_gyro[1] = -(double)Gyro_Raw[0];
        Temp_gyro[2] = -(double)Gyro_Raw[2];    

        Temp_gyro[0] += Gyro_bias[0];            
        Temp_gyro[1] += Gyro_bias[1];
        Temp_gyro[2] += Gyro_bias[2]; 
        
        // gyro bias
        if((Temp_gyro[0]>-5)&&(Temp_gyro[0]<5))Temp_gyro[0]=0;
        if((Temp_gyro[1]>-5)&&(Temp_gyro[1]<5))Temp_gyro[1]=0;
        if((Temp_gyro[2]>-5)&&(Temp_gyro[2]<5))Temp_gyro[2]=0;         

        if(Temp_gyro[0]>0)
            Gyro_bias[0]--;
        else if(Temp_gyro[0]<0)
            Gyro_bias[0]++;
        
        if(Temp_gyro[1]>0)
            Gyro_bias[1]--;
        else if(Temp_gyro[1]<0)
            Gyro_bias[1]++;
        
        if(Temp_gyro[2]>0)
            Gyro_bias[2]--;
        else if(Temp_gyro[2]<0)
            Gyro_bias[2]++;        
        
        if((EndFrame==1)&&(SCommand==0x30) )                   
        {        
            for(Cal_t=0;Cal_t<3;Cal_t++)
            {
                 if(Gyro_bias[Cal_t]<0)
                {
                    Sensor_Param[GYRO_BIAS+Cal_t*2] = ((uint16_t)Gyro_bias[Cal_t]/0xff)|0x80;
                    Sensor_Param[GYRO_BIAS+Cal_t*2+1] = (uint16_t)Gyro_bias[Cal_t]%0xff;
                }            
                else 
                {
                    Sensor_Param[GYRO_BIAS+Cal_t*2] = ((uint16_t)Gyro_bias[Cal_t]/0xff);
                    Sensor_Param[GYRO_BIAS+Cal_t*2+1] = (uint16_t)Gyro_bias[Cal_t]%0xff;
                }
            }     

            Flash_Write(1,Sensor_Param);  
            EndFrame =0;
            break;
        }
    }
}

void Acc_Calibration()
{
    max_acc[0]=-100;max_acc[1]=-100;max_acc[2]=-100;
    min_acc[0]= 100;min_acc[1]= 100;min_acc[2]= 100;
    Acc_bias[0] = 0;Acc_bias[1] = 0;Acc_bias[2] = 0;
    Acc_Cal[0] =0;Acc_Cal[1] =0;Acc_Cal[2] =0;
    while(1)
    {
        MPU9250_Get9AxisRawData(Acc_Raw,Gyro_Raw,Mag_Raw);
        
        
        HDR_acc[0] = lpf1(Acc_Raw[1],0.4,HDR_acc[0]);
        HDR_acc[1] = lpf1(Acc_Raw[0],0.4,HDR_acc[1]);
        HDR_acc[2] = lpf1(Acc_Raw[2],0.4,HDR_acc[2]);
        
        Temp_acc[0] = HDR_acc[0];
        Temp_acc[1] = HDR_acc[1];
        Temp_acc[2] = HDR_acc[2];
         
        
        if((EndFrame==1)&&(SCommand==0x37))
        {
            
            Temp_acc[0] = 0;
            Temp_acc[1] = 0;
            Temp_acc[2] = 0;
            for(t4=0;t4<10;t4++)
            {
                MPU9250_Get9AxisRawData(Acc_Raw,Gyro_Raw,Mag_Raw);
                
                HDR_acc[0] = lpf1(Acc_Raw[1],0.9,HDR_acc[0]);
                HDR_acc[1] = lpf1(Acc_Raw[0],0.9,HDR_acc[1]);
                HDR_acc[2] = lpf1(Acc_Raw[2],0.9,HDR_acc[2]);
                
                Temp_acc[0] += HDR_acc[0];
                Temp_acc[1] += HDR_acc[1];
                Temp_acc[2] += HDR_acc[2];                               
            }
            Temp_acc[0] /= 10;
            Temp_acc[1] /= 10;
            Temp_acc[2] /= 10; 
            
            if(Temp_acc[0]>max_acc[0])max_acc[0] = Temp_acc[0];
            if(Temp_acc[0]<min_acc[0])min_acc[0] = Temp_acc[0];
            if(Temp_acc[1]>max_acc[1])max_acc[1] = Temp_acc[1];
            if(Temp_acc[1]<min_acc[1])min_acc[1] = Temp_acc[1];
            if(Temp_acc[2]>max_acc[2])max_acc[2] = Temp_acc[2];
            if(Temp_acc[2]<min_acc[2])min_acc[2] = Temp_acc[2];
                
           // Send_BT(0x01);
             EndFrame =0;
        }
        
        if((EndFrame==1)&&(SCommand==0x30))
        {           
            for(Cal_t=0;Cal_t<3;Cal_t++)                                 
                Acc_bias[Cal_t] = ((max_acc[Cal_t]-min_acc[Cal_t])/2-max_acc[Cal_t])/2;
                                           
            for(Cal_t=0;Cal_t<3;Cal_t++)            {        
                if(FastAbs(max_acc[Cal_t])<FastAbs(min_acc[Cal_t])){                 
                    Sensor_Param[ACC_BIAS+Cal_t*2] = ((u8)Acc_bias[Cal_t]/0xff)|0x80;
                    Sensor_Param[ACC_BIAS+Cal_t*2+1] = ((u8)Acc_bias[Cal_t]%0xff);
                }
                else{
                    Sensor_Param[ACC_BIAS+Cal_t*2] = ((u8)Acc_bias[Cal_t]/0xff);
                    Sensor_Param[ACC_BIAS+Cal_t*2+1] = ((u8)Acc_bias[Cal_t]%0xff);
                }
            }
            
            for(Cal_t=0;Cal_t<3;Cal_t++){               
                Acc_Cal[Cal_t] = (100/((max_acc[Cal_t]+Acc_bias[Cal_t])*16/32768));
                Sensor_Param[ACC_SCALE+Cal_t*2] = ((u8)Acc_Cal[Cal_t]/0xff);
                Sensor_Param[ACC_SCALE+Cal_t*2+1] = ((u8)Acc_Cal[Cal_t]%0xff);
            }       
            Flash_Write(1,Sensor_Param); 
            EndFrame =0;
            break;
        }
    }
}
void Mag_Calibration()
{
    // Compass Calibration
    while(1)
    {
        MPU9250_Get9AxisRawData(Acc_Raw,Gyro_Raw,Mag_Raw);
        Temp_mag[0] = Mag_Raw[1];
        Temp_mag[1] = Mag_Raw[0];
        Temp_mag[2] = Mag_Raw[2];

        if(Temp_mag[0]>max_mag[0])max_mag[0]=Temp_mag[0];
        if(Temp_mag[0]<min_mag[0])min_mag[0]=Temp_mag[0];
        if(Temp_mag[1]>max_mag[1])max_mag[1]=Temp_mag[1];
        if(Temp_mag[1]<min_mag[1])min_mag[1]=Temp_mag[1];
        if(Temp_mag[2]>max_mag[2])max_mag[2]=Temp_mag[2];
        if(Temp_mag[2]<min_mag[2])min_mag[2]=Temp_mag[2];
                   
        if((EndFrame==1)&&(SCommand==0x30))
        {   
            for(Cal_t=0;Cal_t<3;Cal_t++)
                Mag_bias[Cal_t] = (max_mag[Cal_t]-min_mag[Cal_t])/2-max_mag[Cal_t];
                                           
            for(Cal_t=0;Cal_t<3;Cal_t++)
            {
                if(FastAbs(max_mag[Cal_t])<FastAbs(min_mag[Cal_t]))
                {
                    Sensor_Param[MAG_BIAS+Cal_t*2] = ((u8)Mag_bias[Cal_t]/255)|0x80;
                    Sensor_Param[MAG_BIAS+Cal_t*2+1] = ((u8)Mag_bias[Cal_t]%255);
                }
                else
                {
                    Sensor_Param[MAG_BIAS+Cal_t*2] = ((u8)Mag_bias[Cal_t]/255);
                    Sensor_Param[MAG_BIAS+Cal_t*2+1] = ((u8)Mag_bias[Cal_t]%255);
                }
            }
            
            for(Cal_t=0;Cal_t<3;Cal_t++)
            {
                Mag_cal[Cal_t] = 25000/(max_mag[Cal_t]+Mag_bias[Cal_t]);
                
                Sensor_Param[MAG_SCALE+Cal_t*2] = ((u8)Mag_cal[Cal_t]/255);
                Sensor_Param[MAG_SCALE+Cal_t*2+1] = ((u8)Mag_cal[Cal_t]%255);
            }                           
            Flash_Write(1,Sensor_Param); 
            EndFrame =0;
            break;
          //        
        } 
    }         
}
