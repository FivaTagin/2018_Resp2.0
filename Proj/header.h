#ifndef HEADER_H // include guard
#define HEADER_H

//
// header files of libraries
//
#include <stdio.h>
// #include <time.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>

#include <wiringPi.h> // Raspberry Pi GPIO Library
#include <wiringPiI2C.h> // Raspberry Pi I2C Library

#include "CodeBase.h" // Personal Defination for variables


//
// Variables
//

#define VAR_TRUE                             1
#define VAR_FALUSE                           0
#define VAR_ITG3200                          0
#define VAR_MPU0506                          1

#define VAR_TIMESTAMP_SET                    1000000


//
// countrol Flags
//

#define FLAG_DEBUG_MSG                       VAR_TRUE
#define FLAG_SWITCH_GYRO_CHIP                VAR_MPU0506

//
//  I2C address of the device
//
#define ITG3200_DEFAULT_ADDRESS				 0x68
#define MPU0506_DEFAULT_ADDRESS				 0x68

//
//  MPU0506 Register Map
//
#define MPU0506_ACCL_X_H               0x3B
#define MPU0506_ACCL_X_L               0x3C
#define MPU0506_ACCL_Y_H               0x3D
#define MPU0506_ACCL_Y_L               0x3E
#define MPU0506_ACCL_Z_H               0x3F
#define MPU0506_ACCL_Z_L               0x40

#define MPU0506_GYTO_X_H               0x43
#define MPU0506_GYTO_X_L               0x44
#define MPU0506_GYTO_Y_H               0x45
#define MPU0506_GYTO_Y_L               0x46
#define MPU0506_GYTO_Z_H               0x47
#define MPU0506_GYTO_Z_L               0x48
// #define MPU0506

//
// Enum 
//
enum enumAxis {
    X,
    Y,
    Z
};

//
// Stuct 
//
struct datasetMPU0605 {
  double accl_scaled_x; 
  double accl_scaled_y; 
  double accl_scaled_z;
  double gyro_scaled_x; 
  double gyro_scaled_y; 
  double gyro_scaled_z;
   /* declare as many members as desired, but the entire structure size must be known to the compiler. */
};


//
//  MPU0506 Variables
//
#define MPU0506_VAR_ACCL               16384.0
#define MPU0506_VAR_GYRO               131.0
// #define MPU0506


//
//  ITG3200 Register Map
//
#define ITG3200_WHO_AM_I					   0x00 //# Who Am I Register
#define ITG3200_SMPLRT_DIV					 0x15 //# Sample Rate Divider
#define ITG3200_DLPF_PS						   0x16 //# Digital Low Pass Filter Register
#define ITG3200_INT_CFG						   0x17 //# Interrupt Configuration
#define ITG3200_INT_STATUS					 0x1A //# Interrupt Status
#define ITG3200_TEMP_OUT_H					 0x1B //# Temperature High Byte
#define ITG3200_TEMP_OUT_L					 0x1C //# Temperature Low Byte
#define ITG3200_GYRO_XOUT_H					 0x1D //# X-Axis High Byte
#define ITG3200_GYRO_XOUT_L					 0x1E //# X-Axis Low Byte
#define ITG3200_GYRO_YOUT_H					 0x1F //# Y-Axis High Byte
#define ITG3200_GYRO_YOUT_L					 0x20 //# Y-Axis Low Byte
#define ITG3200_GYRO_ZOUT_H					 0x21 //# Z-Axis High Byte
#define ITG3200_GYRO_ZOUT_L					 0x22 //# Z-Axis Low Byte
#define ITG3200_PWR_MGM						   0x3E //# Power Management

//
//  ITG3200 Digital Low Pass Filter Register
//
#define ITG3200_FULLSCALE_2000			 0x18 //# Gyro Full-Scale Range = +/-2000 per sec
#define ITG3200_DLPF_BW_256					 0x00 //# Bandwidth = 256Hz
#define ITG3200_DLPF_BW_188					 0x01 //# Bandwidth = 188Hz
#define ITG3200_DLPF_BW_98					 0x02 //# Bandwidth = 98Hz
#define ITG3200_DLPF_BW_42					 0x03 //# Bandwidth = 42Hz
#define ITG3200_DLPF_BW_20					 0x04 //# Bandwidth = 20Hz
#define ITG3200_DLPF_BW_10					 0x05 //# Bandwidth = 10Hz
#define ITG3200_DLPF_BW_5					   0x06 //# Bandwidth = 5Hz

//
//  ITG3200 Power Management Register
//
#define ITG3200_PWR_H_RESET					   0x80 //# Reset device and internal registers to the power-up-default settings
#define ITG3200_PWR_SLEEP					     0x40 //# Enable low power sleep mode
#define ITG3200_PWR_NRML_X_Y_Z				 0x00 //# Put all gyro axis in normal mode
#define ITG3200_PWR_STBY_XG					   0x20 //# Put gyro X in standby mode
#define ITG3200_PWR_STBY_YG					   0x10 //# Put gyro Y in standby mode
#define ITG3200_PWR_STBY_ZG					   0x08 //# Put gyro Z in standby mode
#define ITG3200_CLOCK_INTERNAL				 0x00 //# Internal oscillator
#define ITG3200_CLOCK_PLL_XGYRO				 0x01 //# PLL with X Gyro reference
#define ITG3200_CLOCK_PLL_YGYRO				 0x02 //# PLL with Y Gyro reference
#define ITG3200_CLOCK_PLL_ZGYRO				 0x03 //# PLL with Z Gyro reference
#define ITG3200_CLOCK_PLL_EXT32K			 0x04 // PLL with external 32.768kHz reference
#define ITG3200_CLOCK_PLL_EXT19M			 0x05 // PLL with external 19.2MHz reference

//
// ITG3200_SETTING
//

#define ITG3200_POWER_CONFIG           (ITG3200_CLOCK_PLL_XGYRO | ITG3200_PWR_NRML_X_Y_Z)
#define ITG3200_FULLSCALE_CONFIG       (ITG3200_FULLSCALE_2000 | ITG3200_DLPF_BW_256)
#define ITG3200_WHOAMI_MASK            0x01111110
#define ITG3200_WHO_AM_I_ID            0x01101000
#define ITG3200_GYRO_FLOAT             32767
#define ITG3200_GYRO_NEGATVE           65536
#define ITG3200_MASK_RAW_DATA_RDY      0x01
//
// Raspberry GPIO ADDRESS 
//

#define RPI_GPIO_0                     0
#define RPI_GPIO_1                     1
#define RPI_GPIO_2                     2
#define RPI_GPIO_3                     3
#define RPI_GPIO_4                     4
#define RPI_GPIO_5                     5
#define RPI_GPIO_6                     6
#define RPI_GPIO_7                     7
#define RPI_GPIO_21                    21
#define RPI_GPIO_22                    22 
#define RPI_GPIO_23                    23 
#define RPI_GPIO_24                    24 
#define RPI_GPIO_25                    25 
#define RPI_GPIO_26                    26 
#define RPI_GPIO_27                    27 
#define RPI_GPIO_28                    28 
#define RPI_GPIO_29                    29 


//
// Functions Defination 
//


 /**
   Get current angles from GYRO sensor.
 
   @param[in]      VOID

   @retval STATUS_SUCCESS   Description of what EFI_SUCCESS means.
   @retval !STATUS_SUCCESS  Failure.
 
 **/
STATUS_CODE funcUpdateAngles (
  struct datasetMPU0605 *dataset
);


 /**
   Platform initialise Process
 
   @param[in]      VOID

   @retval STATUS_SUCCESS   Description of what EFI_SUCCESS means.
   @retval !STATUS_SUCCESS  Failure.
 
 **/
STATUS_CODE platInitialise (
    VOID
    );
    
/**
   Caclulating the timestamp
 
   @param[in]      VOID

   @retval TIMESTAMP   the value of TIMESTAMP

 
 **/
TIMESTAMP  funcGetTimestamp(
    VOID
);    

/**
   Caclulating the distance between two points
 
   @param[in]    double  a   point a
   @param[in]    double  b   point b

   @retval double   the value of distance

 
 **/
double funcDist (
    double a, 
    double b
    );

#if FLAG_SWITCH_GYRO_CHIP

/**
   Reach the Data from MPU0605
 
   @param[inout]   datasetMPU0605 *dataset


   @retval STATUS_CODE   

 
 **/
STATUS_CODE funcReachGYROwithACCLData (
  struct datasetMPU0605 *dataset
);

/**
   Caclulating the Rotation
 
   @param[in]    enum enumAxis flag : the selection of axis from sensor
   @param[in]    double x
   @param[in]    double y
   @param[in]    double z

   @retval double   the value of distance

 
 **/
double funcGetRotation
    (
    enum enumAxis flag,
    double x, 
    double y, 
    double z
    );

/**
   Gyro MPU0506 Initialise Progress
 
   @param[in]      VOID

   @retval STATUS_SUCCESS   Description of what EFI_SUCCESS means.
   @retval !STATUS_SUCCESS  Failure.
 
 **/
STATUS_CODE mpu0506Initialise (
    VOID
    ); 

#else 
/**
   Gyro ITG3200 Initialise Progress
 
   @param[in]      VOID

   @retval STATUS_SUCCESS   Description of what EFI_SUCCESS means.
   @retval !STATUS_SUCCESS  Failure.
 
 **/
STATUS_CODE itg3200Initialise (
    VOID
    ); 


/**
   Gyro ITG3200 Initialise Progress
 
   @param[inout]      x  : reach X raw data 
   @param[inout]      y  : reach Y raw data 
   @param[inout]      z  : reach Z raw data 

   @retval STATUS_SUCCESS   Description of what EFI_SUCCESS means.
   @retval !STATUS_SUCCESS  Failure.
 
 **/
STATUS_CODE funcGetGyroData (
    float *x,
    float *y,
    float *z
    ); 

/**
   Gyro ITG3200 Waiting for Ready
 
   @param[VOID]      


   @retval TRUE  
   @retval FALURE
 **/
VOID funcIsRawDataReady (
    VOID
    ); 

#endif

#endif /* HEADER_H */