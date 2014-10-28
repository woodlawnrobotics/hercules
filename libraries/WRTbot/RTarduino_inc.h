// ---------------------------------------------------------------------------
// *
// * RTarduino_inc.h
// *
// *  Created on: December, 2013
// *      Author: simon
// *
// ---------------------------------------------------------------------------
// M. Simon,  December 2013
// ---------------------------------------------------------------------[ BoF ]
#ifndef RTarduino_inc_H_
#define RTarduino_inc_H_

#include <inttypes.h>

// --- robot drive definitions
#include "WRTbot_inc.h"

// Note timing value frequency hz
#define DO_L   0xE2 // 262
#define DOA_L  0xE4 // 277
#define RE_L   0xE5 // 294
#define REA_L  0xE7 // 311
#define MI_L   0xE8 // 330
#define FA_L   0xEA // 349
#define FAA_L  0xEB // 370
#define SO_L   0xEC // 392
#define SOA_L  0xED // 415
#define LA_L   0xEE // 440
#define LAA_L  0xEF // 466
#define TI_L   0xF0 // 494
#define DO     0xF1 // 523
#define DOA    0xF2 // 554
#define RE     0xF3 // 587
#define REA    0xF3 // 622
#define MI     0xF4 // 659
#define FA     0xF5 // 698
#define FAA    0xF5 // 740
#define SO     0xF6 // 784
#define SOA    0xF7 // 831
#define LA     0xF7 // 880
#define LAA    0xF8 // 932
#define TI     0xF8 // 988
#define DO_H   0xF9 // 1046
#define DOA_H  0xF9 // 1109
#define RE_H   0xF9 // 1175
#define REA_H  0xFA // 1245
#define MI_H   0xFA // 1318
#define FA_H   0xFA // 1397
#define FAA_H  0xFB // 1480
#define SO_H   0xFB // 1568
#define SOA_H  0xFB // 1661
#define LA_H   0xFC // 1760
#define LAA_H  0xFC // 1865
#define TI_H   0xFC // 1976
#define ZERO   0 // pause

typedef enum RT_ARDUINO_ENUM_tag
{
	wrt_io_serialComBaudRate		= 57600,	// baud
	wrt_due_io_serialComBaudRate	= 57600,	// baud
	wrt_due_io_serialComTimeOut		= 10,		// msec

	wrt_due_io_bufferSize			= 0x80,


	// --- bluetooth serial communication
	// ----------------------------------------------------------------------
	wrt_due_io_slave				= 0,
	wrt_due_io_master				= 0xABBA,
	// ---
	wrt_due_io_com_synch			= 0xCAFE,


	// --- I2C communication
	// ----------------------------------------------------------------------
	wrt_io_i2c_synch			= 0xC0FE,
	wrt_io_i2c_master			= 0xC4BA,
	wrt_io_i2c_robot			= 0xBE75,


	// --- I2C commands
	// ----------------------------------------------------------------------
	wrt_io_i2c_cmd_none			= 0,
	wrt_io_i2c_cmd_playTone		= 0x0001,
	wrt_io_i2c_cmd_robotDrive	= 0x0002,
	wrt_io_i2c_cmd_robotControl	= 0x0004,
	wrt_io_i2c_cmd_irRange		= 0x0008,



	// --- commands
	wrt_due_io_com_command_OK		= 0,

	// --- acknowledge
	wrt_due_io_com_ack_OK			= 0,

	wrt_due_digitalPin_id_master	= 35, // set if bluetooth is master
	wrt_due_digitalPin_id_4			= 34,
	wrt_due_digitalPin_id_3			= 33,
	wrt_due_digitalPin_id_2			= 32,
	wrt_due_digitalPin_id_1			= 31,
	wrt_due_digitalPin_id_0			= 30,

	wrt_due_digitalPin_audioAnalyzer_strobe	= 53, // strobe pin ->(yellow)
	wrt_due_digitalPin_audioAnalyzer_rst	= 52, // RST pin 	->(brown)

	wrt_due_analgoPin_audioAnalyzer_data	=  0, // audio analyzer data (blue)
	wrt_due_analgoPin_xsound_back			=  1, // ultrasound back
	wrt_due_analgoPin_xsound_front			=  2, // ultrasound front

	// --- number of ultraSound sensors (not all sensors active at any time)
	wrt_due_nUltraSoundSensors				= 12,
	wrt_due_UltraSoundBufferSize			=  3,	// number of buffer elements

} rt_arduino_enum_t;

typedef struct RT_EncoderData_tag
{
	uint16_t	pw0;	// nominal PWM for motor control
	uint16_t	pwL;	// Left  PWM
	uint16_t	pwR;	// Right PWM

    uint16_t 	absDelta;
    short    	signDelta;

	short		delta;

	// --- wheel encoders
	uint8_t	left;		// digital pin 2, external interrupt 0
	uint8_t	right;		// digital pin 3, external interrupt 1
	// ---
	uint8_t	leftLast;		// digital pin 2, external interrupt 0
	uint8_t	rightLast;		// digital pin 3, external interrupt 1

} rt_encodderData_t;

typedef struct RT_IO_Param_tag
{
	bool		firstTime;
	bool		isMaster;
	// ----
	int			sizeRxBT;	// --- number of bytes available from bluetooth device
	int			nbytesRxBT;	// --- number of bytes read from bluetooth device
	// ---
	int			dVal;		// --- temp digital value
	int			idx;		// --- some pointer
	// ------------------------------------------
	int 		nbytesTx;
	int 		nbytesRx;
	int 		sizeRx;
	// ------------------------------------------
	unsigned short	ticks;

	uint16_t	bluetoothMode;	// master/slave, if slave, which slave number
	uint16_t	mode;
	uint16_t	id;
	uint16_t	irKey;			// button pressed on the IR receiver

	uint16_t	counter;

	// --- holds values for: enum WRT_IR_ENUM_tag
	// ---------------------------------------------
	uint16_t	board;			// --- board flavor
	uint16_t	arch;			// --- arch flavor
	// ---------------------------------------------

}	rt_ioParam_t;


typedef struct RT_ULTRASOUND_SETTINGS_tag
{
	double	suplyVols;		// 3.3 - 5 volts
	double	factor;			// scale factor

	int		nUxDevices;		// number of ultrasound devices
	int		pin[ wrt_due_nUltraSoundSensors ];

} rt_uxSoundSettings_t;

typedef struct RT_ULTRASOUND_FILTER_tag
{
	// --- digital low pass filter
	// ----------------------------
	double  A;
	double  B;
	double  C;
	double  D;
	// ----------------------------

} rt_uxSoundFilter_t;

typedef struct RT_ULTRASOUND_RANGE_DATA_tag
{
	double		in;	// range in inches
	double 		cm;	// range in cm

} rt_uxRangeData_t;

typedef struct RT_ULTRASOUND_RANGE_DATA_SENSORS_tag
{
	int					nUxDevices;
	rt_uxRangeData_t	data[wrt_due_nUltraSoundSensors];

} rt_uxRangeDataSensors_t;

typedef struct RT_ULTRASOUND_UNIT_tag
{
	// --- digital low pass filter
	// ----------------------------
	double	x;		// state
	double	u;		// input
	double	y;		// output

	// --- engineering values
	// ----------------------------
	rt_uxRangeData_t	range;

	// --- filter active flag
	// ----------------------------
	bool 	isActive;

	// --- sensor data
	// ----------------------------
	int		pin;		// which analog pin to use
	int		data;		// data read from the analog sensor

} rt_uxSoundUnit_t;

typedef struct RT_ULTRASOUND_tag
{
	rt_uxSoundSettings_t		settings;
	rt_uxSoundFilter_t		lpf;		// digital low pass filter

	rt_uxSoundUnit_t			ux[wrt_due_nUltraSoundSensors];

} rt_uxSound_t;

typedef struct RT_IR_RangeData_tag
{
	float	ux1Range;
	float	ux2Range;

} rt_irRangeData_t;

typedef struct RT_RobotStatus_Data_tag
{
	uint8_t		status;
	uint8_t		flag;

	uint8_t		usrKey;
	uint8_t		flag1;

} rt_robotStatusData_t;

typedef struct RT_DriveCommand_Data_tag
{
	uint16_t	mode;
	uint16_t	not_used;

} rt_driveCommandData_t;

typedef struct RT_SoundCommand_Data_tag
{
	uint8_t		note;	// --- do, re, mi, fa, sol, la, si
	uint8_t		delay;	// --- x 200

} rt_soundCommandData_t;

typedef union RT_IO_Data_tag
{
	rt_soundCommandData_t	tone;
	rt_irRangeData_t		irRange;
	rt_driveCommandData_t	drive;
	wrt_motor_t				motor;
	rt_robotStatusData_t	botStatus;
	uint8_t					data[ sizeof(struct RT_DriveCommand_Data_tag) ];

} rt_ioData_t;

typedef struct RT_IO_Header_tag
{
	uint16_t	synch;		 // --- universay blutooth synchronizaton word
	uint16_t	id;			 // --- unique unit id
	uint16_t	size;		 // --- size of body data
	uint16_t	cmnd;		 // --- command (from master), ack (from slave)
	uint16_t	counter;	 // --- message counter
	uint16_t	flag;		 // --- context flag (depends on command or status)
	uint16_t	chkSumPLoad; // --- data payload checksum
	uint16_t	chkSum;		 // --- header checksum

} rt_ioHeader_t;

typedef struct RT_IO_SerialData_tag
{
	rt_ioHeader_t	hdr;	// header with command/id
	rt_ioData_t		pLoad;	// data payload

} rt_ioSerialData_t;

typedef struct RT_IO_i2cData_tag
{
	rt_ioHeader_t	hdr;	// header with command/id
	rt_ioData_t		pLoad;	// data payload

} rt_ioI2CData_t;

// --- note: size of buffer set to 128 bytes (0x80)

// ---------------------------------------------------------------------[ EoF ]
#endif /* RTarduino_inc_H_ */
