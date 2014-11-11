/*
 * WRTbot_inc.h
 *
 *  Created on: Jul 21, 2013
 *      Author: simon
 */
// ---------------------------------------------------------------------[ BoF ]
#ifndef WRTBOT_INC_H_
#define WRTBOT_INC_H_

#if defined(ARDUINO) && (ARDUINO >= 100)
#include "Arduino.h"
#endif


#include <inttypes.h>

// ---------------------------------------------------------------------------
// Digital Pin  0:Rx
// Digital Pin  1:Tx
// Digital Pin  2:Encoder
// Digital Pin  3:Encoder
// Digital Pin  4:Left Motor Direction Pin
// Digital Pin  5:Left Motor Enable Pin
// Digital Pin  6:Right Motor Enable Pin
// Digital Pin  7:Right Motor Direction Pin
// Digital Pin  8:Infrared Receiver
// Digital Pin  9:Left Infared Transmitter
// Digital Pin 10:Right Infrared Transmitter
// Digital Pin 11:Buzzer
// Digital Pin 12:Red LED
// Digital Pin 13:Green Led

// Analog Pin 0:Line tracking sensor
// Analog Pin 1:Line tracking sensor
// Analog Pin 2:Line tracking sensor
// Analog Pin 3:Line tracking sensor
// Analog Pin 4:Ambient Light sensor
// Analog Pin 5:Three Key buttons
// Analog Pin 6:Reset button
// Analog Pin 7:Line tracking sensor
// ---------------------------------------------------------------------------

#define IR_BIT_LENGTH 32    // number of bits sent by IR remote
#define  FirstLastBit 15	// divide 32 bits into two 15 bit chunks for integer variables. Ignore center two bits. they are all the same.
#define 		BIT_1 1500	// Binary 1 threshold (Microseconds)
#define 		BIT_0 450	// Binary 0 threshold (Microseconds)
#define 	BIT_START 4000	// Start bit threshold (Microseconds)

#define 	  LED_RED 12	// LED red
#define 	LED_GREEN 13	// LED green

typedef enum WRT_IR_ENUM_tag
{
	// --- known boards:
	// --------------------------------------------------------------------
	wrt_board_hercules			= 0xE3C2,	// --- hercules 		(Duemilanove)
	wrt_board_romeo_v11			= 0xEA11,	// --- romeo v.1.1 		(Uno)
	wrt_board_romeo_v21			= 0xEB21,	// --- romeo v.2.1		(Leonardo)
	wrt_board_min4w3D			= 0x743D,	// --- min4w3D robot	(Uno)
	wrt_board_uno				= 0xD140,	// --- plain old Arduino Uno
	wrt_board_due				= 0xDD4E,	// --- plain old Arduino Due
	// --------------------------------------------------------------------
	// note: hercules board: Atmege328 controller
	//		 Tools->Board->Arduino Duemilanove/ATmega328.
	// --------------------------------------------------------------------

	// --- known architecture:
	// --------------------------------------------------------------------
	wrt_board_romeo_sam			= 0x05A4,	// --- (due)
	wrt_board_romeo_avr			= 0x1A25,	// --- (uno)

	wrt_bot_obstacleAvoidance_nPulses	= 20,

	wrt_bot_minQ4wd_pin_buzzer	= 11,

	wrt_bot_ir_nkeys			= 12,

	wrt_bot_ecoder_nticks		= 12,	// twelve tiks per revolution
	wrt_bot_wheelDiamater		= 42,	// [mm] wheel diamater (milimeters)


	// --- analog reference volts
	// ------------------------------------------------------
	wrt_analog_referenceVolts		= 5,

	// --- analog distance infrared sensors in  Romeo and Hercules
	// --------------------------------------------------------------------
	wrt_pin_analog_dIR1			= 1,
	wrt_pin_analog_dIR2			= 2,
	wrt_pin_analog_dIR3			= 3,
	// ---
	wrt_irSwitchThreshold		= 400,


	//	Line Tracking sensor (4WD MinQ robot)
	// ------------------------------------------------------
	wrt_pin_lineTrack_0			= 0,	// Analog Pin 0
	wrt_pin_lineTrack_1			= 1,	// Analog Pin 1
	wrt_pin_lineTrack_2			= 2,	// Analog Pin 2
	wrt_pin_lineTrack_3			= 3,	// Analog Pin 3
	wrt_pin_lineTrack_4			= 7,	// Analog Pin 7
	// ---
	wrt_pin_nLineTrackSensors   = 5,

	//	Vex Claw (Romeo)
	// ------------------------------------------------------
	wrt_pin_romeo_vexClaw		= 9,	// digital pin 9 (PWM)

	//	Ambient Light sensor (4WD MinQ robot)
	// ------------------------------------------------------
	wrt_pin_ambientLight_sensor		= 4,	// Analog Pin 4


	// --- motor direction pins: (4WD MinQ and Romeo robot)
	// ------------------------------------------------------
	wrt_pin_motor_direction_left	= 4,	// Digital Pin 4
	wrt_pin_motor_direction_right	= 7,	// Digital Pin 7

	// --- motor enable pins:
	// ------------------------------------------------------
	wrt_pin_motor_speed_left		= 5,	// Digital Pin 5
	wrt_pin_motor_speed_right		= 6,	// Digital Pin 6

	// --- motor enable pins:
	// ------------------------------------------------------
	wrt_pin_ir_sensorHercules		=  2,	// Digital Pin  8
	wrt_pin_ir_sensorRomeo			=  8,	// Digital Pin  8
	wrt_pin_ir_sensor				=  8,	// Digital Pin  8
	wrt_pin_ir_transmitter_left		=  9,	// Digital Pin  9
	wrt_pin_ir_transmitter_right	= 10,	// Digital Pin 10

	// note: non the Arduino Duemilanove
	// -----------------------------------------------------------------
	//		External Interrupts: 2 and 3.
	//		These pins can be configured to trigger an interrupt on a
	//		low value, a rising or falling edge, or a change in value.
	//		See the attachInterrupt() function for details.
	// -----------------------------------------------------------------


	// --- miniQ4wd robot: analog pin 5: key buttons
	// ------------------------------------------------------
	wrt_pin_analogKey_mQ4wd			= 5,
	wrt_button_key_number_mQ4wd		= 3,
	// ---
	wrt_button_key_none				= 0,
	wrt_button_key_mQ4wd_1			= 1,
	wrt_button_key_mQ4wd_2			= 2,
	wrt_button_key_mQ4wd_3			= 3,

	// --- romeo v1.1 robot: analog pin 7: key buttons
	// ------------------------------------------------------
	wrt_pin_analogKey_romeo			= 7,
	wrt_button_key_number_romeo		= 5,
	// ---
	wrt_button_key_romeo_none		= 0,
	wrt_button_key_romeo_1			= 1,
	wrt_button_key_romeo_2			= 2,
	wrt_button_key_romeo_3			= 3,
	wrt_button_key_romeo_4			= 4,
	wrt_button_key_romeo_5			= 5,

	// --- motor direction codes:
	// ---------------------------------------------------------------
	wrt_code_motor_speed_hercules_smallIncrement = 5,	// step increment for PWM
	wrt_code_motor_speed_hercules_increment  	 = 7,	// step increment for PWM
	wrt_code_motor_speed_hercules_rotate		 = 15,	// default rotate PWM
	wrt_code_motor_speed_hercules_nominal		 = 45,	// nominal for PWM
	wrt_code_motor_speed_hercules_medium		 = 50,	// medium  for PWM
	wrt_code_motor_speed_hercules_low			 = 20,	// low for PWM
	wrt_code_motor_speed_hercules_min  			 = 10,	//     max for PWM
	wrt_code_motor_speed_hercules_max  			 = 50,	//     max for PWM
	wrt_code_motor_speed_hercules_dMax  		 = 100,	//     max for PWM

	// --- motor direction codes:
	// ---------------------------------------------------------------
	wrt_code_motor_stop  = 0,				// stop
	wrt_code_motor_speed_smallIncrement = 6,	// step increment for PWM
	wrt_code_motor_speed_increment  	= 25,	// step increment for PWM
	wrt_code_motor_speed_rotate			= 95,	// default rotate PWM
	wrt_code_motor_speed_nominal		= 100,	// nominal for PWM
	wrt_code_motor_speed_medium			= 150,	// medium  for PWM
	wrt_code_motor_speed_low			= 85,	// low for PWM
	wrt_code_motor_speed_min  			= 70,	//     max for PWM
	wrt_code_motor_speed_max  			= 200,	//     max for PWM

	wrt_code_motor_drive_forward		= 1,	// wheel moves forward
	wrt_code_motor_drive_reverse		= 0,	// wheel moves in reverse

	wrt_code_motor_stop_time_msec		= 50,	// msec, nominal delay while stop


	wrt_ir_state_idle					= 0,
	wrt_ir_state_irPulseLeft			= 0x7e37,
	wrt_ir_state_irPulseRight			= 0x317E,
	wrt_ir_state_irPulseForward			= 0xE71A,
	wrt_ir_state_irPulseLeftAction		= 0xABBA,
	wrt_ir_state_irPulseRightAction 	= 0xC0FE,
	wrt_ir_state_irPulseLeftActionTurn  = 0x1532,
	wrt_ir_state_irPulseRightActionTurn = 0x3981,

	wrt_ir_key_none		= 0,
	// ---------------------------------------------------[ infrared receiver ]
	wrt_ir_key_power	= 0x7F80,	// = 32640:  // turns on UUT power
	wrt_ir_key_funcStop	= 0x7E81,	// = 32385:  // FUNC/STOP turns off UUT power
	wrt_ir_key_fbkp		= 0x7D82,	// = 32130:  // |<< ReTest failed Test
	wrt_ir_key_fwrdEnd	= 0x7D02,	// = 32002:  // >|| Test
	wrt_ir_key_ffwrd	= 0x7C83,	// = 31875:  // >>| perform selected test number

	wrt_ir_key_volUp	= 0x7F00,	// = 32512:  // VOL+ turns on individual test beeper
	wrt_ir_key_volDown	= 0x7B04,	// = 31492:  // VOL- turns off individual test beeper
	wrt_ir_key_down		= 0x7B84,	// = 31620:  // v scroll down tests
	wrt_ir_key_up		= 0x7A85,	// = 31365:  // ^ scroll up tests
	wrt_ir_key_eq		= 0x7906,	// = 30982:  // EQ negative tests internal setup
	wrt_ir_key_srt		= 0x7887,	// = 30855:  // ST/REPT Positive tests Select Test and Repeat Test

	wrt_ir_key_0		= 0x7986,	// = 31110:  // 0
	wrt_ir_key_1		= 0x7788,	// = 30600:  // 1
	wrt_ir_key_2		= 0x7708,	// = 30472:  // 2
	wrt_ir_key_3		= 0x7689,	// = 30345:  // 3
	wrt_ir_key_4		= 0x758A,	// = 30090:  // 4

	wrt_ir_key_5		= 0x750A,	// = 29962:  // 5
	wrt_ir_key_6		= 0x748B,	// = 29835:  // 6
	wrt_ir_key_7		= 0x738C,	// = 29580:  // 7
	wrt_ir_key_8		= 0x730C,	// = 29452:  // 8
	wrt_ir_key_9		= 0x728D,	// = 29325:  // 9

	// ---------------------------------------------------[ infrared receiver ]
	wrt_ir_key38KHz_power	= 0x00FF,	// turns on UUT power
	wrt_ir_key38KHz_funcStop = 0x40BF,	// FUNC/STOP turns off UUT power
	wrt_ir_key38KHz_fbkp	= 0x20DF,	// |<< ReTest failed Test
	wrt_ir_key38KHz_fwrdEnd	= 0xA05F,	// >|| Test
	wrt_ir_key38KHz_ffwrd	= 0x609F,	// >>| perform selected test number

	wrt_ir_key38KHz_volUp	= 0x807F,	// VOL+ turns on individual test beeper
	wrt_ir_key38KHz_volDown	= 0x906F,	// VOL- turns off individual test beeper
	wrt_ir_key38KHz_down	= 0x10EF,	// v scroll down tests
	wrt_ir_key38KHz_up		= 0x50AF,	// ^ scroll up tests
	wrt_ir_key38KHz_eq		= 0xB04F,	// EQ negative tests internal setup
	wrt_ir_key38KHz_srt		= 0x708F,	// ST/REPT Positive tests Select Test and Repeat Test

	wrt_ir_key38KHz_0		= 0x30CF,	// 0
	wrt_ir_key38KHz_1		= 0x08F7,	// 1
	wrt_ir_key38KHz_2		= 0x8877,	// 2
	wrt_ir_key38KHz_3		= 0x48B7,	// 3
	wrt_ir_key38KHz_4		= 0x28D7,	// 4

	wrt_ir_key38KHz_5		= 0xA857,	// 5
	wrt_ir_key38KHz_6		= 0x6897,	// 6
	wrt_ir_key38KHz_7		= 0x18E7,	// 7
	wrt_ir_key38KHz_8		= 0x9867,	// 8
	wrt_ir_key38KHz_9		= 0x58A7,	// 9

	// --- remote control (rc) commands
	// --------------------------------------------------[ simple rc commands ]
	wrt_rc_cmd_reset			= 0,
	wrt_rc_cmd_toggle_power		= 0x80000000,	// start/stop	(key: power )
	// ---
	wrt_rc_cmd_forward			= 0x00000001,	// forward 		(key: ^ 	)
	wrt_rc_cmd_reverse			= 0x00000002,	// reverse 		(key: v 	)
	wrt_rc_cmd_left				= 0x00000004,	// to the left	(key: |<< 	)
	wrt_rc_cmd_right			= 0x00000008,	// to the right	(key: >>| 	)
	wrt_rc_cmd_spin_left		= 0x00000010,	// rotate right	(key: vol- 	)
	wrt_rc_cmd_spin_right		= 0x00000020,	// rotate right	(key: vol+ 	)

	wrt_rc_cmd_claw_openFull	= 0x00000040,	// rotate right	(key: 1 	)
	wrt_rc_cmd_claw_clooseFull	= 0x00000080,	// rotate right	(key: 3 	)
	wrt_rc_cmd_claw_nominal		= 0x00000100,	// rotate right	(key: 0 	)
	wrt_rc_cmd_claw_linear		= 0x00000200,	// rotate right	(key: 0 	)
	wrt_rc_cmd_claw_linear2		= 0x00000400,	// rotate right	(key: 0 	)
	wrt_rc_cmd_claw_linear4		= 0x00000800,	// rotate right	(key: 0 	)


	// --- mask infrared rc commands
	wrt_rc_cmd_mask_set			= 0x0000003F,	// mask rc cmds
	wrt_rc_cmd_mask_clear		= 0xFFFFFFC0,	// mask rc cmds

	// --- mask subset of linear rc commands
	wrt_rc_cmd_mask_fvset		= 0x00000003,	// mask  direction cmds
	wrt_rc_cmd_mask_fvclear		= 0xFFFFFFFC,	// mask  direction cmds

	// --- mask subset of spin rc commands
	wrt_rc_cmd_mask_svSet		= 0x00000030,	// mask  spin cmds
	wrt_rc_cmd_mask_svClear		= 0xFFFFFFCF,	// mask  spin cmds


	// -----------------------------------------------------------[ behaviors ]
	// --- behavior: infrared remote control (irc)
	// 	   key combination: (key: >|| )
	wrt_rc_cmd_behavior_irc 	 = 0x00010000,

	// --- pray behavior (avoid all obstacles)
	// 	   key combination: (key: func/stop  )
	wrt_rc_cmd_behavior_pray	 = 0x00020000,	// default behavior

	// --- follow the leader behavior or predator behavior (follow obstacles)
	//	   note: how to avoid fixed obstacles?
	wrt_rc_cmd_behavior_predator = 0x00040000,

	// --- follow the (ambient sensor) light
	// 	   key combination: (key: EQ )
	//	   note: follow the light, else stop
	wrt_rc_cmd_behavior_lightHunter = 0x00080000,

	// --- activate/operate the claw
	wrt_rc_cmd_behavior_clawAction = 0x00100000,

	// --- obstacle avoidance
	wrt_rc_cmd_behavior_obstacleAvoidance = 0x00200000,

	// --- mask behaviors
	wrt_rc_cmd_behavior_clear	 = 0xF000FFFF,	// note: update as needed
	wrt_rc_cmd_behavior_set		 = 0x0FFF0000,	// note: update as needed

	//	vex claw state
	// ------------------------------------------------------
	wrt_vexClaw_set_max			= 2000,			// max pulse width in usec
	wrt_vexClaw_set_min			=  900,			// min pulse width in usec
	wrt_vexClaw_set_nominal		= 1500,			// nominal pulse width in usec
	wrt_vexClaw_set_delta		=   50,			// increment
	wrt_vexClaw_set_delta2		=  100,			// increment
	wrt_vexClaw_set_delta4		=  200,			// increment


	//	Line Tracking state machine
	// ------------------------------------------------------
	wrt_lineTrack_state_forwardDrive				= 0x0100,
	wrt_lineTrack_state_differentialForwardDrive 	= 0x0200,
	wrt_lineTrack_state_differentialRotation		= 0x0400,
	// ---
	wrt_lineTrack_sensorActive_ir0					= 0x0001,
	wrt_lineTrack_sensorActive_ir1					= 0x0002,
	wrt_lineTrack_sensorActive_ir2					= 0x0004,
	wrt_lineTrack_sensorActive_ir3					= 0x0008,
	wrt_lineTrack_sensorActive_ir4					= 0x0010,
	// ---
	wrt_lineTrack_sensorActiveX_ir01				= (wrt_lineTrack_sensorActive_ir0 | wrt_lineTrack_sensorActive_ir1),
	wrt_lineTrack_sensorActiveX_ir12				= (wrt_lineTrack_sensorActive_ir1 | wrt_lineTrack_sensorActive_ir2),
	wrt_lineTrack_sensorActiveX_ir23				= (wrt_lineTrack_sensorActive_ir2 | wrt_lineTrack_sensorActive_ir3),
	wrt_lineTrack_sensorActiveX_ir34				= (wrt_lineTrack_sensorActive_ir3 | wrt_lineTrack_sensorActive_ir4),
	wrt_lineTrack_sensorActiveX_ir123				= (wrt_lineTrack_sensorActive_ir1 | wrt_lineTrack_sensorActive_ir2 | wrt_lineTrack_sensorActive_ir3 ),

	// ----------------------------------------------[ direct drive commands ]
	wrt_stop			 		= 0,
	wrt_forward		 	 		= 0x0001,
	wrt_forward_slow 	 		= 0x0002,
	wrt_forward_fast	 		= 0x0004,
	wrt_reverse		 	 		= 0x0008,
	wrt_reverse_fast	 		= 0x0010,
	wrt_turn_left		 		= 0x0020,
	wrt_turn_left_fast 	 		= 0x0040,
	wrt_drive_left	 	 		= 0x0080,
	wrt_turn_right	 	 		= 0x0100,
	wrt_turn_right_fast 		= 0x0200,
	wrt_drive_right 			= 0x0400,
	// ---
	wrt_turnRightDiff			= 0x0800,
	wrt_turnRightDiff_fast		= 0x1000,
	wrt_turnLeftDiff			= 0x2000,
	wrt_turnLeftDiff_fast		= 0x4000,


	// (struct WRT_ROBOT_MOTOR_CONTROL_tag)->control
	// -------------------------------------------------[ robot control flags ]
	wrt_motorControl_power_bit	= 0x0001,	// bit set = power on,
	wrt_motorControl_power_ON	= 0x0001,	// bit set = power on,
	wrt_motorControl_power_OFF	= 0xFFFE,	// bit set = power on,

	// bit set = forward, off = backwards
	wrt_motorControl_direction_bit		= 0x0002,
	wrt_motorControl_direction_Forward	= 0x0002,
	wrt_motorControl_direction_Reverse	= 0xFFFD,

} wrt_ir_enum_t;

// --------------------------------------------------[ robot platform sensors ]
typedef struct WRT_ROBOT_ROMEO_ANALOG_DISTANCE_IR_tag
{
	int		left;
	int		forward;
	int		right;
	// ---
	int 	action;

} wrt_robotRomeoAnalog_irDistanceSensor_t;

// --------------------------------------------------[ robot platform sensors ]
typedef struct WRT_ROBOT_SENSORS_AMBIENT_LIGHT_tag
{
	float 		data;				// analog pin 4, ambient light sensor

	float		limitMin;			// minimum limit
	float		limitLowerBound;	// highest lower bound
	float		limitUpperBound;	// lowest  upper bound
	float		limitMax;			// maximum limit

	// --- conversion factor
	float		factor;

} wrt_robotSensors_ambientLight_t;

// --------------------------------------------------[ robot platform sensors ]
typedef struct WRT_ROBOT_SENSORS_tag
{
	// analog pin 4, ambient light sensor
	wrt_robotSensors_ambientLight_t  light;

} wrt_robotSensors_t;

// -----------------------------------------------------[ robot motor control ]
typedef struct WRT_ROBOT_MOTOR_CONTROL_tag
{
	// 0x0001,	// bit set = power on,	off = power off
	// 0x0002,	// bit set = forward, 	off = backwards
	// ------------------------------------------------
	uint16_t	control;

	uint16_t	speed;

} wrt_robotMotorControl_t;

// -----------------------------------------------[ infrared receiver control ]
typedef struct WRT_IR_RC_tag
{
	bool	isNewBehavior;
	bool	isGoodKey;				// current key is good == true,
	// ---------------------------------------------------------------
	int 	idx;					// array index of the current command
	int 	ncmd;					// number of commands stored in the array
	int 	nGoodKeys;				// number of good consecutive keys
									// note: resets to zero
									// when there is an failure
	// ---------------------------------------------------------------
	uint32_t commandDirection;	// preserve last direction command
	uint32_t commandLast;		// compound command record
	uint32_t command;			// compound command record
	uint32_t behavior;			// avoid obstacles by default
	// ---------------------------------------------------------------
	uint32_t auxCommand;			// compound command record
	uint32_t auxBehavior;			// avoid obstacles by default

	// --- array of commands used to determine action for multi-keyed
	//	   compound commands.
	// ---------------------------------------------------------------
	unsigned int cmd[wrt_bot_ir_nkeys];
	// ---------------------------------------------------------------

	uint16_t key;					// last good key
	uint16_t notUsedYet;


} wrt_ir_rc_t;

typedef struct WRT_IR_tag
{
	wrt_ir_rc_t				rc;

} wrt_ir_t;

typedef struct WRT_MOTOR_tag
{
	wrt_robotMotorControl_t	left;	// left  motor control
	wrt_robotMotorControl_t	right;	// right motor control

	// --- when shifting directions abruptly, we stop the robot momentarily
	//		to avoid current spikes.
	// ----------------------------------------------------------------------
	bool	isShiftDirection;
	bool	isResetDirection;

} wrt_motor_t;


// ---------------------------------------------------------------------[ EoF ]
#endif /* WRTBOT_INC_H_ */


