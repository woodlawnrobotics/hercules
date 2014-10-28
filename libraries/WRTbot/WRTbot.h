// *
// * WRTbot.cpp
// *
// *  Created on: Jul 21, 2013
// *      Author: simon
// *
// ---------------------------------------------------------------------[ BoF ]
#ifndef WRTBOT_H_
#define WRTBOT_H_


#include "RTsys_inc.h"
// -------------------------------------------------
#include "RTarduino_inc.h"
#include "WRTbot_inc.h"
#include "WRTirReceiver.h"
#include "WRTbotDrive.h"
#include "WRTirObstacleAvoidance.h"
#include <WRTLineTrack.h>

namespace WRT_ARDUINO_ROBOT
// -------------------------------------------------------[ WRT_ARDUINO_ROBOT ]
{

class WRTbot
{
public:
	WRTbot();
	~WRTbot();

	void init( const uint8_t  );

	bool stepBehavior( void );
	bool readIRC( rt_ioParam_t & );
	bool readIRC( void );
	bool stepIRC( void );
	bool stepVexClaw( void );


	void trackLine( bool );
	void stepObjectTrack(       const uint16_t );
	void stepObstacleAvoidance( const uint16_t );
	bool readKeyButton( void );
	bool readKeyButtonRomeo( void );

	inline void setReceiverPinIR( const uint8_t usrPinForReceiverIR )
	{
		WRTbot::irRx->setReceiverPinIR( usrPinForReceiverIR );
	}

	inline void drive( const uint16_t isHow )
	{
		motor.left.control  |= wrt_motorControl_power_ON;
		motor.right.control |= wrt_motorControl_power_ON;

		WRTbot::drv->drive( isHow, motor );
	}

	inline void drive( const rt_encodderData_t &edr )
	{
		motor.left.control  |= wrt_motorControl_power_ON;
		motor.right.control |= wrt_motorControl_power_ON;
		// ---
		motor.left.control  |= wrt_motorControl_direction_Forward;
		motor.right.control |= wrt_motorControl_direction_Forward;
		// ---
		motor.left.speed  = edr.pwL;
		motor.right.speed = edr.pwR;
		// ---
		motorEncoderLeft  = edr.leftLast;
		motorEncoderRight = edr.rightLast;
		// ------------------------------------
		WRTbot::drv->drive( motor );
		// ------------------------------------
	}

	inline void drive( const wrt_motor_t &motor )
	{
		WRTbot::drv->drive( motor );
	}


	inline uint16_t getKeyButtonPress( void )
	{
		isButtonPressed = false;
		return( dKeyButtonIs );
	}

	inline int getKey( void )
	{
		return( WRTbot::ir.rc.key );
	}

	inline int setVerbose( const int verboseInput )
	{
		verbose = verboseInput;
		irRx->setVerbose( verboseInput );

		return( verbose );
	}

	inline void setMotorEncoder( const uint16_t eLeft, const uint16_t eRight )
	{
		motorEncoderLeft  = eLeft;
		motorEncoderRight = eRight;
	}

	inline void irSendPulse( const uint16_t usrState )
	{
		WRTbot::irObAv->irSend38KHzPulse( usrState );
	}

	inline void setLongitudinalGains( const float usrLonGain[wrt_pin_nLineTrackSensors] )
	{
		lineTrk->setLongitudinalGains( usrLonGain );

	}

	inline void setLateralGains( const float usrLatGain[wrt_pin_nLineTrackSensors] )
	{
		lineTrk->setLateralGains( usrLatGain );

	}


private:

	bool driveIRC( void );
	bool actionIRC( void );

	WRT_ARDUINO_IR_RECEIVER::WRTirReceiver	*irRx;
	WRT_ARDUINO_BOT_DRIVE::WRTbotDrive		*drv;
	WRT_ARDUINO_IR_AVOIDANCE::WRTirObstacleAvoidance	*irObAv;
	WRT_LINE_TRACK::WRTLineTrack	*lineTrk;

	wrt_motor_t motor;
	wrt_ir_t	ir;	// infrared database
	// --- other
	bool		isFirstPass;
	bool 		isGoodKeyIR;
	int  		verbose;


	int 		dataFromKeyButton;
	bool		isButtonPressed;
	uint16_t	dKeyButtonIs;
	uint16_t	irState;			// for obstacle avoidance maneuver
	uint16_t	vexClawPWM;			// dutty cycle for the vex claw servo
	bool		vexClawDirection;	// true/false,

	// left,  digital pin 2, external interrupt 0
	// right, digital pin 3, external interrupt 1
	uint16_t	motorEncoderLeft;
	uint16_t	motorEncoderRight;

#if defined( WRT_ROBOT_USE_SERVO_LIB )

	// --- Romeo robot Vex Calw
	// -------------------------
	Servo vexClaw;

#endif

};

// -------------------------------------------------------[ WRT_ARDUINO_ROBOT ]
} /* namespace WRT_ARDUINO_ROBOT */
#endif /* WRTBOT_H_ */
// ---------------------------------------------------------------------[ EoF ]
