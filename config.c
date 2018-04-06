//
// Created by jaehyek.choi on 2017-05-19.
//



#define  JAEHYEK    1

/*

xtal : 12.0
IROM1:0x8000000, size:0x10000
IRAM1:0x20000000, size:0x5000
**define : 
USE_STDPERIPH_DRIVER, STM32F10X_MD, USE_STM3210E_EVAL,ARM_MATH_CM3

**include :
..\Libraries\CMSIS\Core\CM3;..\Libraries\STM32_USB-FS-Device_Driver\inc;..\Libraries\STM32F10x_StdPeriph_Driver\inc;.\inc;.\Math\inc;.\Matrix\inc;.\Algorithm\inc

** compiler control flag
-c --cpu Cortex-M3 -D__MICROLIB -g -O0 --apcs=interwork --split_sections -I ..\Libraries\CMSIS\Core\CM3 -I ..\Libraries\STM32_USB-FS-Device_Driver\inc -I ..\Libraries\STM32F10x_StdPeriph_Driver\inc -I .\inc -I .\Math\inc -I .\Matrix\inc -I .\Algorithm\inc 
-I "D:\GATool_v2\IMU_src\IMU_Program_V3.0\IMU_Program(Bootloader Ver)\Program\RTE" 
-I C:\Keil_v5\ARM\PACK\Keil\STM32F1xx_DFP\2.2.0\Device\Include 
-I C:\Keil_v5\ARM\CMSIS\Include 
-D__UVISION_VERSION="520" -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD -DUSE_STM3210E_EVAL -DARM_MATH_CM3 -o ".\Obj\*.o" --omf_browse ".\Obj\*.crf" --depend ".\Obj\*.d" 

** Assembler control string
--cpu Cortex-M3 -g --apcs=interwork --pd "__MICROLIB SETA 1" 
-I "D:\GATool_v2\IMU_src\IMU_Program_V3.0\IMU_Program(Bootloader Ver)\Program\RTE" 
-I C:\Keil_v5\ARM\PACK\Keil\STM32F1xx_DFP\2.2.0\Device\Include 
-I C:\Keil_v5\ARM\CMSIS\Include 
--pd "__UVISION_VERSION SETA 520" --pd "STM32F10X_MD SETA 1" --list ".\List\*.lst" --xref -o "*.o" --depend "*.d" 


**Linker control string 
--cpu Cortex-M3 *.o 
--library_type=microlib --strict --scatter ".\Obj\IMU_Program(Bootloader).sct" 
--summary_stderr --info summarysizes --map --xref --callgraph --symbols 
--info sizes --info totals --info unused --info veneers 
 --list ".\List\IMU_Program(Bootloader).map" 
-o .\Obj\IMU_Program(Bootloader).axf 

**Debug
Driver DLL:SARMCM3.DLL
Diallog DLL: TARMSTM.DLL -parameter : -pSTM32F103T8







*/



