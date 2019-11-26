#include "header.h"

//
// Global Values
//
double gx,gy,gz;
double gAngleX = 0.0;
double gAngleY = 0.0;
double gAngleZ = 0.0;
double gDeltaT = 0.0;
double gyro_total_x, gyro_total_y, gyro_total_z;
double gLastX, gLastY, gLastZ;
double gGyroOffsetX, gGyroOffsetY, gGyroOffsetZ;

struct datasetMPU0605 gDataset;

// Complimentary Filter parameters
double gK0 = (double) 0.98;
double gK1 = (double) 0.02;

INT gFD = 0;
TIMESTAMP gCurrentTime;
TIMESTAMP gBeforeTime;



INT main() {

#if FLAG_DEBUG_MSG
    printf ("Program Start\n");
#endif
    platInitialise ();

    while (1) {
        funcUpdateAngles (&gDataset);
        delay(100);
    }
    return 0;
}


 /**
   Platform initialise Process
 
   @param[in]      VOID

   @retval EFI_SUCCESS   Description of what EFI_SUCCESS means.
   @retval !EFI_SUCCESS  Failure.
 
 **/
STATUS_CODE platInitialise (
    VOID
    ) {


        #if FLAG_DEBUG_MSG
            printf ("platInitialise\n");
        #endif
        wiringPiSetup();      // RPI GPIO INIT

        gCurrentTime = funcGetTimestamp(); // update current Time stamp
        gBeforeTime = gCurrentTime; // Set new Time stamp
        gDeltaT = (double) (funcGetTimestamp() - gCurrentTime)/1000000.0;

#if FLAG_SWITCH_GYRO_CHIP
        mpu0506Initialise (); // MPU 0506 Initial
        funcReachGYROwithACCLData (&gDataset);

        gLastX = funcGetRotation(X ,gDataset.accl_scaled_x, gDataset.accl_scaled_y, gDataset.accl_scaled_z);
        gLastY = funcGetRotation(Y ,gDataset.accl_scaled_x, gDataset.accl_scaled_y, gDataset.accl_scaled_z);
        gLastZ = funcGetRotation(Z ,gDataset.accl_scaled_x, gDataset.accl_scaled_y, gDataset.accl_scaled_z);



        gGyroOffsetX = gDataset.gyro_scaled_x;
        gGyroOffsetY = gDataset.gyro_scaled_y;
        gGyroOffsetZ = gDataset.gyro_scaled_z;

        gyro_total_x = gLastX - gGyroOffsetX;
        gyro_total_y = gLastY - gGyroOffsetY;
        gyro_total_z = gLastZ - gGyroOffsetZ;

#else 
        itg3200Initialise (); // ITG3200 INIT
#endif

        
        
        return STATUS_SUCCESS;
    }

/**
   Caclulating the timestamp
 
   @param[in]      VOID

   @retval TIMESTAMP   the value of TIMESTAMP

 
 **/
TIMESTAMP  funcGetTimestamp(
    VOID
)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (TIMESTAMP) tv.tv_sec * VAR_TIMESTAMP_SET + tv.tv_usec;
}


/**
   Caclulating the distance between two points
 
   @param[in]    double  a   point a
   @param[in]    double  b   point b

   @retval double   the value of distance

 
 **/
double funcDist (
    double a, 
    double b
    )
{
  return sqrt((a*a) + (b*b));
}

#if FLAG_SWITCH_GYRO_CHIP

 /**
   Get current angles from GYRO sensor.
 
   @param[in]      VOID

   @retval STATUS_SUCCESS   Description of what EFI_SUCCESS means.
   @retval !STATUS_SUCCESS  Failure.
 
 **/
STATUS_CODE funcUpdateAngles (
  struct datasetMPU0605 *dataset
  
) {
    STATUS_CODE status;
    double gyroDeltaX, gyroDeltaY, gyroDeltaZ;

    status = STATUS_SUCCESS;

    //
    // Get time delta value
    //
    gCurrentTime = funcGetTimestamp();
    gDeltaT = (double) (gCurrentTime - gBeforeTime)/1000000.0;
    gBeforeTime = gCurrentTime;

    funcReachGYROwithACCLData (&gDataset);

    (*dataset).gyro_scaled_x -= gGyroOffsetX;
    (*dataset).gyro_scaled_y -= gGyroOffsetY;
    (*dataset).gyro_scaled_z -= gGyroOffsetZ;

    gyroDeltaX = ((*dataset).gyro_scaled_x * gDeltaT);
    gyroDeltaY = ((*dataset).gyro_scaled_y * gDeltaT);
    gyroDeltaZ = ((*dataset).gyro_scaled_z * gDeltaT);


    gyro_total_x += gyroDeltaX;
    gyro_total_y += gyroDeltaY;
    gyro_total_z += gyroDeltaZ;


    gAngleX = funcGetRotation(X ,(*dataset).accl_scaled_x, (*dataset).accl_scaled_y, (*dataset).accl_scaled_z);
    gAngleY = funcGetRotation(Y ,(*dataset).accl_scaled_x, (*dataset).accl_scaled_y, (*dataset).accl_scaled_z);
    gAngleZ = funcGetRotation(Z ,(*dataset).accl_scaled_x, (*dataset).accl_scaled_y, (*dataset).accl_scaled_z);

//    printf("[BEFORE] gyro_scaled_y=%f, deltaT=%lf, rotation_y=%f, last_y= %f\n", (double)gyro_scaled_y, (double)deltaT, (double)rotation_y, (double) last_y);

//    printf("[1st part] = %f\n", (double) K0*(last_y + gyro_y_delta));
//    printf("[2nd part] = %f\n", (double) K1*rotation_y);
    gLastX = gK0 * (gLastX + gyroDeltaX) + (gK1 * gAngleX);
    gLastY = gK0 * (gLastY + gyroDeltaY) + (gK1 * gAngleY);
    gLastZ = gK0 * (gLastZ + gyroDeltaZ) + (gK1 * gAngleZ);

    printf("[AFTER] gyro_scaled_y=%f, deltaT=%lf, rotation_y=%f, last_y=%f\n", (double)(*dataset).gyro_scaled_y, (double)gDeltaT, (double)gAngleY, (double) gLastY);
    return status;

}

int read_word_2c(int addr)
{
  int val;
  val = wiringPiI2CReadReg8(gFD, addr);
  val = val << 8;
  val += wiringPiI2CReadReg8(gFD, addr+1);
  if (val >= 0x8000)
    val = -(65536 - val);

  return val;
}

/**
   Reach the Data from MPU0605
 
   @param[inout]   datasetMPU0605 *dataset


   @retval STATUS_CODE   

 
 **/
STATUS_CODE funcReachGYROwithACCLData (
  struct datasetMPU0605 *dataset
) {
    STATUS_CODE status;
    status = STATUS_SUCCESS;

    int acclX, acclY, acclZ;
    int gyroX, gyroY, gyroZ;

    acclX = read_word_2c(MPU0506_ACCL_X_H);
    acclY = read_word_2c(MPU0506_ACCL_Y_H);
    acclZ = read_word_2c(MPU0506_ACCL_Z_H);

    (*dataset).accl_scaled_x = acclX / MPU0506_VAR_ACCL;
    (*dataset).accl_scaled_y = acclY / MPU0506_VAR_ACCL;
    (*dataset).accl_scaled_z = acclZ / MPU0506_VAR_ACCL;

    gyroX = read_word_2c(MPU0506_GYTO_X_H);
    gyroY = read_word_2c(MPU0506_GYTO_Y_H);
    gyroZ = read_word_2c(MPU0506_GYTO_Z_H);

    (*dataset).gyro_scaled_x = gyroX / MPU0506_VAR_GYRO;
    (*dataset).gyro_scaled_y = gyroY / MPU0506_VAR_GYRO;
    (*dataset).gyro_scaled_z = gyroZ / MPU0506_VAR_GYRO;

    return status;

}
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
    )
{
  
  double radians;

  switch (flag) {
      case X : 
      radians = atan2(y, funcDist(x, z));
      
      break;
      case Y :

      radians = atan2(x, funcDist(y, z));
      break;
      case Z :

      radians = atan2(z, funcDist(y, x));
      break;
      default :
      break;


  }
  
  return -(radians * (180.0 / M_PI));
}

/**
   Gyro MPU0506 Initialise Progress
 
   @param[in]      VOID

   @retval STATUS_SUCCESS   Description of what EFI_SUCCESS means.
   @retval !STATUS_SUCCESS  Failure.
 
 **/
STATUS_CODE mpu0506Initialise (
    VOID
    ) {
    //
    // Initialise
    //
    STATUS_CODE Status = STATUS_SUCCESS;
    
    gFD = wiringPiI2CSetup (MPU0506_DEFAULT_ADDRESS);
    wiringPiI2CWriteReg8 (gFD,MPU0506_DEFAULT_ADDRESS,0x00);//disable sleep mode 

#if FLAG_DEBUG_MSG
    printf ("MPU 0605 Initialise\n");
#endif
    
  return Status;
}

#else 
/**
   Gyro ITG3200 Initialise Progress
 
   @param[in]      VOID

   @retval EFI_SUCCESS   Description of what EFI_SUCCESS means.
   @retval !EFI_SUCCESS  Failure.
 
 **/
STATUS_CODE itg3200Initialise (
    VOID
    ) {
    STATUS_CODE Status;
    INT         data;

    //
    // Initialise
    //
    Status = STATUS_SUCCESS;
    data = 0;
    gFD = wiringPiI2CSetup (ITG3200_DEFAULT_ADDRESS);

#if FLAG_DEBUG_MSG
    printf ("itg3200Initialise\n");
#endif
    
    data = wiringPiI2CReadReg8 (gFD, ITG3200_WHO_AM_I); // error handle, check itg3200 is there
    if (((ITG3200_WHOAMI_MASK & data)| ITG3200_WHO_AM_I_ID) != ITG3200_WHO_AM_I_ID) {
#if FLAG_DEBUG_MSG
      printf (" Who Am I ID Failure : %x\n", data);
#endif              

    
      return STATUS_ERROR;
    }

#if FLAG_DEBUG_MSG
      printf ("Who Am I ID: %x\n", data);
#endif      

    wiringPiI2CWriteReg8 (gFD, ITG3200_PWR_MGM, ITG3200_POWER_CONFIG);
    wiringPiI2CWriteReg8 (gFD, ITG3200_DLPF_PS, ITG3200_FULLSCALE_CONFIG);
    return Status;
}    


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
    ) 
{
    STATUS_CODE status;
    float tempX;
    float tempY;
    float tempZ;
    status = STATUS_SUCCESS;
    

    tempX = (float)wiringPiI2CReadReg16(gFD, ITG3200_GYRO_XOUT_H);
    if (tempX > ITG3200_GYRO_FLOAT) {
        tempX -= ITG3200_GYRO_NEGATVE;
    }
    tempX /= 131.0;
    tempY = (float)wiringPiI2CReadReg16(gFD, ITG3200_GYRO_YOUT_H);
     if (tempY > ITG3200_GYRO_FLOAT) {
        tempY -= ITG3200_GYRO_NEGATVE;
    }   
    tempZ = (float)wiringPiI2CReadReg16(gFD, ITG3200_GYRO_ZOUT_H);
    if (tempZ > ITG3200_GYRO_FLOAT) {
        tempZ -= ITG3200_GYRO_NEGATVE;
    }


    printf("%04.04f  %f  %f\n", tempX, tempY, tempZ);
    delay(50);

    return status;
}

/**
   Gyro ITG3200 Waiting for Ready
 
   @param[VOID]      


   @retval TRUE  
   @retval FALURE
 **/
VOID funcIsRawDataReady (
    VOID
    ) 
{
  INT data;
  data = wiringPiI2CReadReg16(gFD, ITG3200_INT_STATUS);
  while (((data & ITG3200_MASK_RAW_DATA_RDY) !=  ITG3200_MASK_RAW_DATA_RDY));


}
#endif