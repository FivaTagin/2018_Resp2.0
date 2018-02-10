// Symbol
//
#include "SymbleGlyo.h"

//
// Base Reference Sensor Data
//
//alt_u64 mTimeCnt;
float mRefTemperature, mRefGX, mRefGY, mRefGZ;
float  mDataGX, mDataGY, mDataGZ, mFeqRate;

//
// Interrupt Veriable
//

alt_u16 mIntTmr0feq;

int main(void)
{

  alt_u8        BackData = 0xff;
  alt_u16       Templure, GyroData;

  ShowLEDReady ();
  //
  // Init the Reference Data from the Sensor.
  //

  InitGYROSetting ();
  InitInterrupt ();
  ShowLEDReady ();
  GetITG3205RefData ();

  SetMotor ();

  if (I2C_Read(I2C_SCL_BASE, I2C_SDA_BASE, ITG05_DEVICE_ID, ITG05_WHO_AM_I_REG, &BackData)) { // TODO: Check the I2C and ITG3200 is there
    //
    // Main while to get angle data
    //
    while (1) {
    //ShowLEDReady ();
    GetITG3205TempData (&Templure);
    printf ("Templure = %f  ,", (mRefTemperature - Templure));
    printf ("GyroX : %f, GyroY : %f, GyroZ : %f\n, \n",mDataGX ,mDataGY, mDataGZ);
    }
  }
  return 0;


}


bool
GetITG3205RefData
(
)
/*
 * Get the Reference Data of ITG3205 when the System Reboot
 */
{
  alt_u8        BackData = 0xff;
  short         Templure;
  bool          Status = FALSE;
  int           Cnt;

  //
  //  Get Base temperature
  //
  I2C_Read (I2C_SCL_BASE, I2C_SDA_BASE, ITG05_DEVICE_ID, ITG05_TEMP_H, &BackData);
  Templure = (BackData << 8);
  I2C_Read (I2C_SCL_BASE, I2C_SDA_BASE, ITG05_DEVICE_ID, ITG05_TEMP_L, &BackData);
  Templure = Templure | BackData;
  mRefTemperature = ((Templure + 13200)) / 280 + 35;

  //
  //  Get Base GYRO X
  //
  for (Cnt = 0; Cnt < 10; Cnt++) {
    I2C_Read(I2C_SCL_BASE, I2C_SDA_BASE, ITG05_DEVICE_ID, ITG05_GYRO_X_H, &BackData);
    Templure = (BackData << 8);
    I2C_Read(I2C_SCL_BASE, I2C_SDA_BASE, ITG05_DEVICE_ID, ITG05_GYRO_X_L, &BackData);
    mRefGX += ((Templure | BackData) * 0.1);
  }
  //
  //  Get Base GYRO Y
  //
  for (Cnt = 0; Cnt < 10; Cnt++) {
    I2C_Read(I2C_SCL_BASE, I2C_SDA_BASE, ITG05_DEVICE_ID, ITG05_GYRO_Y_H, &BackData);
    Templure = (BackData << 8);
    I2C_Read(I2C_SCL_BASE, I2C_SDA_BASE, ITG05_DEVICE_ID, ITG05_GYRO_Y_L, &BackData);
    mRefGY += ((Templure | BackData) * 0.1);
  }
  //
  //  Get Base GYRO Z
  //
  for (Cnt = 0; Cnt < 10; Cnt++) {
    I2C_Read(I2C_SCL_BASE, I2C_SDA_BASE, ITG05_DEVICE_ID, ITG05_GYRO_Z_H, &BackData);
    Templure = (BackData << 8);
    I2C_Read(I2C_SCL_BASE, I2C_SDA_BASE, ITG05_DEVICE_ID, ITG05_GYRO_Z_L, &BackData);
    mRefGZ += ((Templure | BackData) * 0.1);
  }
  return Status;
}

bool
GetITG3205TempData
/*
 * Get the temperature Data of ITG3205 when the System Reboot
 */
(
  alt_u16       *Templure
)
{
  bool          Status = FALSE;
  alt_u8        BackData = 0xff;
  alt_u16       Data;
  Status = I2C_Read(I2C_SCL_BASE, I2C_SDA_BASE, ITG05_DEVICE_ID, ITG05_TEMP_H, &BackData);
  if (!Status) { return Status;}

  Data = (BackData << 8);
  Status = I2C_Read(I2C_SCL_BASE, I2C_SDA_BASE, ITG05_DEVICE_ID, ITG05_TEMP_L, &BackData);
  if (!Status) { return Status;}

  Data = Data | BackData;
  *Templure = ((Data + 13200)) / 280 + 35; // temperature
  return Status;
}

bool
GetITG3205GyroData
/*
 * Get the Gyro Data of ITG3205 when the System Reboot
 */
(
  alt_u16       *GyroDeg,
  alt_u8        Type         // Input the GYRO TYPE REGISTER
)
{
  bool          Status = FALSE;
  alt_u8        BackData = 0xff;
  alt_u16       Data;
  Status = I2C_Read(I2C_SCL_BASE, I2C_SDA_BASE, ITG05_DEVICE_ID, Type, &BackData);
  if (!Status) { return Status;}

  Data = (BackData << 8);
  Status = I2C_Read(I2C_SCL_BASE, I2C_SDA_BASE, ITG05_DEVICE_ID, (Type + 1), &BackData);
  if (!Status) { return Status;}

  Data = Data | BackData;
  *GyroDeg = Data;
  return Status;
}

void
ShowLEDReady (
)
{
    IOWR_ALTERA_AVALON_PIO_DATA (LED_BASE, 0xf0);
    usleep (300000);
    IOWR_ALTERA_AVALON_PIO_DATA (LED_BASE, 0x0f);
    usleep (300000);
    IOWR_ALTERA_AVALON_PIO_DATA (LED_BASE, 0xff);
    usleep (300000);
    IOWR_ALTERA_AVALON_PIO_DATA (LED_BASE, 0xA5);
    usleep (300000);
}

bool
InitGYROSetting (
)
{
  alt_u8        BackData;
  bool          Status;

  //
  // Init the Rate Feq
  //
  mIntTmr0feq = INT_4K_FEQ;
  mFeqRate = (GYRO_FEQ_4K * GYRO_DATA_RATE);
  //
  // Init the Gyro Data;
  //

  mRefTemperature = 0.0;
  mRefGX = 0.0;
  mRefGY = 0.0;
  mRefGZ = 0.0;
  mDataGX = 0.0;
  mDataGY = 0.0;
  mDataGZ = 0.0;

  /*
   * 將低通濾波器開到最大 (DLPF_CFG = 0x06)
   *
   */
  BackData = FS_SEL | DLPF_CFG;
  Status = I2C_Write(I2C_SCL_BASE, I2C_SDA_BASE, ITG05_DEVICE_ID, ITG05_DLPF_FS, BackData);
  if (!Status) { return Status;}

  /*
   *  Init the PWM
   */
  InitMotor ();
}

void
InitMotor (
)
{
  IOWR_ALTERA_AVALON_PIO_DATA (MOTOR_1_1_BASE, 0x64);
  IOWR_ALTERA_AVALON_PIO_DATA (MOTOR_1_2_BASE, 0x64);
  IOWR_ALTERA_AVALON_PIO_DATA (MOTOR_0_2_BASE, 0x64);
  IOWR_ALTERA_AVALON_PIO_DATA (MOTOR_0_1_BASE, 0x64);
  usleep (3000000);
  IOWR_ALTERA_AVALON_PIO_DATA (MOTOR_1_1_BASE, 0x00);
  IOWR_ALTERA_AVALON_PIO_DATA (MOTOR_1_2_BASE, 0x00);
  IOWR_ALTERA_AVALON_PIO_DATA (MOTOR_0_2_BASE, 0x00);
  IOWR_ALTERA_AVALON_PIO_DATA (MOTOR_0_1_BASE, 0x00);

}

void
SetMotor (
)
{
  IOWR_ALTERA_AVALON_PIO_DATA (MOTOR_1_1_BASE, 0x64);
  IOWR_ALTERA_AVALON_PIO_DATA (MOTOR_1_2_BASE, 0x64);
  IOWR_ALTERA_AVALON_PIO_DATA (MOTOR_0_2_BASE, 0x64);
  IOWR_ALTERA_AVALON_PIO_DATA (MOTOR_0_1_BASE, 0x64);

}

void
InitInterrupt(
)
{

  //timer0
  alt_irq_register(TIMER_0_IRQ,NULL,Timer_getdegree);//100Hz:7a120 200Hz:3d090 1kHz:C350 4KHz:30D4
  IOWR_ALTERA_AVALON_TIMER_PERIODH(TIMER_0_BASE, (alt_u8)(mIntTmr0feq >> 8));//4kHz
  IOWR_ALTERA_AVALON_TIMER_PERIODL(TIMER_0_BASE, (alt_u8)(mIntTmr0feq & 0xFF));//4kHz
  IOWR_ALTERA_AVALON_TIMER_CONTROL(TIMER_0_BASE, 7);

//    //timer1
//    alt_irq_register(TIMER_1_IRQ,NULL,GET_TIME);//50Hz:f4240 80Hz:98968 250Hz:30d40
//    IOWR_ALTERA_AVALON_TIMER_PERIODH(TIMER_1_BASE, 0x0003);//250Hz
//    IOWR_ALTERA_AVALON_TIMER_PERIODL(TIMER_1_BASE, 0x0D40);//250Hz
//    IOWR_ALTERA_AVALON_TIMER_CONTROL(TIMER_1_BASE, 7);
//
//    //timer2
//    alt_irq_register(TIMER_2_IRQ,NULL,chose_footprint);//50Hz:F4240 100Hz:7a120 200Hz:3d090
//   IOWR_ALTERA_AVALON_TIMER_PERIODH(TIMER_2_BASE, 0x0000);//50Hz
//    IOWR_ALTERA_AVALON_TIMER_PERIODL(TIMER_2_BASE, 0x61a8);//50Hz
//    IOWR_ALTERA_AVALON_TIMER_CONTROL(TIMER_2_BASE, 7);

}

void
Timer_getdegree (
)
{
  bool Status;
  float GyroDeg;

  GyroDeg = 0.0;
  //
  // Get all angle data form Gyro.
  //
  Status = GetITG3205GyroData (&GyroDeg, ITG05_GYRO_Z_H);
  if (!Status) {
    Status = GetITG3205GyroData (&GyroDeg, ITG05_GYRO_Z_H);
  }
  mDataGZ = mDataGZ + (GyroDeg / mFeqRate);

  GyroDeg = 0.0;
  Status = GetITG3205GyroData (&GyroDeg, ITG05_GYRO_Y_H);
  if (!Status) {
    Status = GetITG3205GyroData (&GyroDeg, ITG05_GYRO_Y_H);
  }
  mDataGY = mDataGY + (GyroDeg / mFeqRate);

  GyroDeg = 0.0;
  Status = GetITG3205GyroData (&GyroDeg, ITG05_GYRO_X_H);
  if (!Status) {
    Status = GetITG3205GyroData (&GyroDeg, ITG05_GYRO_X_H);
  }
  mDataGX = mDataGX + (GyroDeg / mFeqRate);


  //
  // Timer Reset
  //
  IOWR_ALTERA_AVALON_TIMER_STATUS(TIMER_0_BASE, 0);
}
