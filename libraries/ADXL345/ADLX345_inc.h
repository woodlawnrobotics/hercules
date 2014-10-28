/*
 * ADLX345_inc.h
 *
 *  Created on: Jan 11, 2014
 *      Author: simon
 */

#ifndef ADLX345_INC_H_
#define ADLX345_INC_H_
// ---------------------------------------------------------------------[ BoF ]


typedef enum ADLX345_Enum_tag
{
	// -- ADXL345 addresses --

	// ADXL345 address when ALT is connected to HIGH
	ADXL345_ADDR_ALT_HIGH  	= 0x1D,

	// ADXL345 address when ALT is connected to LOW
	ADXL345_ADDR_ALT_LOW   	= 0x53,

	// ------- Register names -------
	ADXL345_REG_DEVID  			= 0x00,
	ADXL345_REG_RESERVED1  		= 0x01,
	ADXL345_REG_THRESH_TAP  	= 0x1d,
	ADXL345_REG_OFSX  			= 0x1e,
	ADXL345_REG_OFSY  			= 0x1f,
	ADXL345_REG_OFSZ  			= 0x20,
	ADXL345_REG_DUR  			= 0x21,
	ADXL345_REG_LATENT  		= 0x22,
	ADXL345_REG_WINDOW  		= 0x23,
	ADXL345_REG_THRESH_ACT  	= 0x24,
	ADXL345_REG_THRESH_INACT  	= 0x25,
	ADXL345_REG_TIME_INACT  	= 0x26,
	ADXL345_REG_ACT_INACT_CTL  	= 0x27,
	ADXL345_REG_THRESH_FF  		= 0x28,
	ADXL345_REG_TIME_FF  		= 0x29,
	ADXL345_REG_TAP_AXES  		= 0x2a,
	ADXL345_REG_ACT_TAP_STATUS  = 0x2b,
	ADXL345_REG_BW_RATE  		= 0x2c,
	ADXL345_REG_POWER_CTL  		= 0x2d,
	ADXL345_REG_INT_ENABLE  	= 0x2e,
	ADXL345_REG_INT_MAP  		= 0x2f,
	ADXL345_REG_INT_SOURCE  	= 0x30,
	ADXL345_REG_DATA_FORMAT  	= 0x31,
	ADXL345_REG_DATAX0  		= 0x32,
	ADXL345_REG_DATAX1  		= 0x33,
	ADXL345_REG_DATAY0  		= 0x34,
	ADXL345_REG_DATAY1  		= 0x35,
	ADXL345_REG_DATAZ0  		= 0x36,
	ADXL345_REG_DATAZ1  		= 0x37,
	ADXL345_REG_FIFO_CTL  		= 0x38,
	ADXL345_REG_FIFO_STATUS  	= 0x39,

	// --- errors
	ADXL345_ERROR_OK    		= 1, // no error
	ADXL345_ERROR				= 0, // indicates error is present

	ADXL345_ERROR_NONE			= 0, // initial state
	ADXL345_ERROR_READ 			= 1, // problem reading accel
	ADXL345_ERROR_BAD_ARG    	= 2, // bad method argument

	ADXL345_BW_1600    			= 0xF, // 1111
	ADXL345_BW_800     			= 0xE, // 1110
	ADXL345_BW_400     			= 0xD, // 1101
	ADXL345_BW_200     			= 0xC, // 1100
	ADXL345_BW_100     			= 0xB, // 1011
	ADXL345_BW_50      			= 0xA, // 1010
	ADXL345_BW_25      			= 0x9, // 1001
	ADXL345_BW_12      			= 0x8, // 1000
	ADXL345_BW_6       			= 0x7, // 0111
	ADXL345_BW_3       			= 0x6, // 0110


	// --- default bytes to read:
	// 	   num of bytes we are going to read each time: two bytes for each axis
	ADXL345_BytesToRead			= 6,

}	adlx345_enum_t;

typedef struct ADXL345_ACC_DATA_tag
{
	float	x;
	float	y;
	float 	z;

} adxl345_accData_t;

typedef struct ADXL345_Data_tag
{
	adxl345_accData_t	rawDD;	// raw data
	adxl345_accData_t	dd;		// scaled data

} adxl345_data_t;


// ---------------------------------------------------------------------[ EoF ]
#endif /* ADLX345_INC_H_ */
