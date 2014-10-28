/*
 * ITG3200_inc.h
 *
 *  Created on: Jan 12, 2014
 *      Author: simon
 */

#ifndef ITG3200_INC_H_
#define ITG3200_INC_H_
// ---------------------------------------------------------------------[ BoF ]

/* ---- bit maps ---- */
#define DLPFFS_FS_SEL             0x18  // 00011000
#define DLPFFS_DLPF_CFG           0x07  // 00000111
#define INTCFG_ACTL               0x80  // 10000000
#define INTCFG_OPEN               0x40  // 01000000
#define INTCFG_LATCH_INT_EN       0x20  // 00100000
#define INTCFG_INT_ANYRD_2CLEAR   0x10  // 00010000
#define INTCFG_ITG_RDY_EN         0x04  // 00000100
#define INTCFG_RAW_RDY_EN         0x01  // 00000001
#define INTSTATUS_ITG_RDY         0x04  // 00000100
#define INTSTATUS_RAW_DATA_RDY    0x01  // 00000001
#define PWRMGM_HRESET             0x80  // 10000000
#define PWRMGM_SLEEP              0x40  // 01000000
#define PWRMGM_STBY_XG            0x20  // 00100000
#define PWRMGM_STBY_YG            0x10  // 00010000
#define PWRMGM_STBY_ZG            0x08  // 00001000
#define PWRMGM_CLK_SEL            0x07  // 00000111


/************************************/
/*    REGISTERS PARAMETERS    */
/************************************/
// Sample Rate Divider
#define NOSRDIVIDER         0 // default    FsampleHz=SampleRateHz/(divider+1)
// Gyro Full Scale Range
#define RANGE2000           3   // default
// Digital Low Pass Filter BandWidth and SampleRate
#define BW256_SR8           0   // default    256Khz BW and 8Khz SR
#define BW188_SR1           1
#define BW098_SR1           2
#define BW042_SR1           3
#define BW020_SR1           4
#define BW010_SR1           5
#define BW005_SR1           6
// Interrupt Active logic lvl
#define ACTIVE_ONHIGH       0 // default
#define ACTIVE_ONLOW        1
// Interrupt drive type
#define PUSH_PULL           0 // default
#define OPEN_DRAIN          1
// Interrupt Latch mode
#define PULSE_50US          0 // default
#define UNTIL_INT_CLEARED   1
// Interrupt Latch clear method
#define READ_STATUSREG      0 // default
#define READ_ANYREG         1
// Power management
#define NORMAL              0 // default
#define STANDBY             1
// Clock Source - user parameters
#define INTERNALOSC         0   // default
#define PLL_XGYRO_REF       1
#define PLL_YGYRO_REF       2
#define PLL_ZGYRO_REF       3
#define PLL_EXTERNAL32      4   // 32.768 kHz
#define PLL_EXTERNAL19      5   // 19.2 Mhz


typedef enum ITG320_Enum_tag
{
	// --- number of axis
	ITG3200_nAxis		= 3,

	// AD0=1 0x69 I2C address when AD0 is connected to HIGH (VCC) - default for sparkfun breakout
	ITG3200_ADDR_AD0_HIGH  = 0x69,

	//AD0=0 0x68 I2C address when AD0 is connected to LOW (GND)
	ITG3200_ADDR_AD0_LOW   = 0x68,

	// "The LSB bit of the 7 bit address is determined by the logic level on pin 9.
	// This allows two ITG-3200 devices to be connected to the same I2C bus.
	// One device should have pin9 (or bit0) LOW and the other should be HIGH." source: ITG3200 datasheet
	// Note that pin9 (AD0 - I2C Slave Address LSB) may not be available on some breakout boards so check
	// the schematics of your breakout board for the correct address to use.

	// 50ms from gyro startup + 20ms register r/w startup
	ITG3200_GYROSTART_UP_DELAY  = 70,


	// ---- Registers ----
	ITG3200_REG_ADDR           = 0x00,  // RW   SETUP: I2C address
	ITG3200_REG_SMPLRT_DIV     = 0x15,  // RW   SETUP: Sample Rate Divider
	ITG3200_REG_DLPF_FS        = 0x16,  // RW   SETUP: Digital Low Pass Filter/ Full Scale range
	ITG3200_REG_INT_CFG        = 0x17,  // RW   Interrupt: Configuration
	ITG3200_REG_INT_STATUS     = 0x1A,  // R	Interrupt: Status
	ITG3200_REG_TEMP_OUT       = 0x1B,  // R	SENSOR: Temperature 2bytes
	ITG3200_REG_GYRO_XOUT      = 0x1D,  // R	SENSOR: Gyro X 2bytes
	ITG3200_REG_GYRO_YOUT      = 0x1F,  // R	SENSOR: Gyro Y 2bytes
	ITG3200_REG_GYRO_ZOUT      = 0x21,  // R	SENSOR: Gyro Z 2bytes
	ITG3200_REG_PWR_MGM        = 0x3E,  // RW	Power Management


} itg32_enum_t;

typedef struct	ITG320_Gyro_Calibration_Param_tag
{
	  float 	gains[		ITG3200_nAxis];
	  float 	polarities[ ITG3200_nAxis];
	  float 	factor    [ ITG3200_nAxis];
	  int 		offsets[	ITG3200_nAxis];

} itg320_GyroCalibrationParam_t;

typedef struct	ITG320_Gyro_BaseData_tag
{
	  int x;
	  int y;
	  int z;

} itg320_GyroBaseData_t;

typedef struct	ITG320_Gyro_RealData_tag
{
	  float x;
	  float y;
	  float z;

} itg320_GyroRealData_t;

typedef union ITG320_Gyro_Raw_Data_tag
{
	itg320_GyroBaseData_t	d;
	int						data[ITG3200_nAxis];

} itg320_GyroRawData_t;

typedef union ITG320_Gyro_Cal_Data_tag
{
	itg320_GyroBaseData_t	d;
	int						data[ITG3200_nAxis];

} itg320_GyroCalData_t;

typedef union ITG320_Gyro_EngineeringData_tag
{
	itg320_GyroRealData_t	d;
	float					data[ITG3200_nAxis];

} itg320_GyroEngData_t;

typedef struct ITG320_GyroData_tag
{
	float					tmp;	// temperature
	//	---------------------------------------------------
	itg320_GyroEngData_t	eng;	// engineering units
	itg320_GyroCalData_t	cal;	// gyro calibrated data
	itg320_GyroRawData_t	raw;	// gyro raw data

} itg320_GyroData_t;


// ---------------------------------------------------------------------[ EoF ]
#endif /* ITG3200_INC_H_ */
