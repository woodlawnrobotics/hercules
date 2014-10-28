/*
 * RTHerculesDrive.cpp
 *
 *  Created on: Oct 21, 2014
 *      Author: simon
 */

#include "RTHerculesDrive.h"


#if !defined(ARDUINO_ARCH_AVR)
#error "hercules robot runs on AVR hardware only"
#endif	// not ARDUINO_ARCH_AVR

#if !defined( WRT_ROBOT_USE_IR_REMOTE_LIB )
#error "hercules robot uses the IR Remote library"
#endif	// not WRT_ROBOT_USE_IR_REMOTE_LIB


namespace RT_HERCULES_DRIVE
{

RTHerculesDrive::RTHerculesDrive()
{
	RTHerculesDrive::isFirstPass = true;
	// ---------------------------------
	useNominal = false;
	// ---------------------------------
	verbose = 0;
	ir.rc.isGoodKey  = false;
	// ---
	ir.rc.key = -1;
	ir.rc.nGoodKeys = 0;
	ir.rc.idx = 0;
	// ---
	ir.rc.command  = wrt_rc_cmd_reset;
	ir.rc.behavior = wrt_rc_cmd_behavior_irc;

	RTHerculesDrive::irRx = new RT_HERCULES_IR_RECEIVER::RTHerculesCommandIR();

}	// done consutructor

RTHerculesDrive::~RTHerculesDrive()
{
	useNominal = true;

	delete [] RTHerculesDrive::irRx;

}	// done desctructor

void RTHerculesDrive::init( const uint8_t usrPinForReceiverIR )
{
	// --- Hercules robot driving know-how
    // ----------------------------------------------------------------
	herc.init();
    // ----------------------------------------------------------------

	// --- IR controller
    // ----------------------------------------------------------------
	RTHerculesDrive::irRx->init( usrPinForReceiverIR );
    // ----------------------------------------------------------------

	// --- behavior init
    // ----------------------------------------------------------------
	RTHerculesDrive::ceo.init();
    // ----------------------------------------------------------------


    // --- electrical motor operation
    // ----------------------------------------------------------------
    herc.setReverseMotorLeft(  false ); // left  motor is normal
    herc.setReverseMotorRight( true  ); // right motor is reversed (why?)


	// --- enable control of the motor
	motor.left.control  |= wrt_motorControl_power_ON;
	motor.right.control |= wrt_motorControl_power_ON;

	// --- forward
	motor.left.control  |= wrt_motorControl_direction_bit;
	motor.right.control |= wrt_motorControl_direction_bit;

	motor.left.speed  = wrt_code_motor_speed_hercules_nominal;
	motor.right.speed = wrt_code_motor_speed_hercules_nominal;

}	// done init()

// -----------------------------------------------------------------[ stepIRC ]
bool RTHerculesDrive::stepIRC( const bool isGoDrive )
{
	bool isWorking = true;

	// ------------------------------------------------------------[ preamble ]
	if( true == RTHerculesDrive::isFirstPass )
	{
		RTHerculesDrive::isFirstPass = false;

		// --- implement default behavior
		ir.rc.isNewBehavior = true;

		Serial.println("stepIRC: first time");
	}

	// ----------------------------------------------------[ mostly debugging ]
	if( true == ir.rc.isNewBehavior )
	{
		Serial.println("stepIRC: isNewBehavior");

	}

	// -------------------------------------------------------[ power control ]
	if( 0 == (ir.rc.command & wrt_rc_cmd_toggle_power) )
	{
		motor.left.control  &= wrt_motorControl_power_OFF;
		motor.right.control &= wrt_motorControl_power_OFF;

		motor.left.speed  = wrt_code_motor_stop;
		motor.right.speed = wrt_code_motor_stop;

		// --- stop the robot now
		// ------------------------------------
		RTHerculesDrive::stop();
		// ------------------------------------
		isWorking = false;
	}
	else
	{	isWorking = true;
		// ----------------------------------------------[  get drive commands]
		RTHerculesDrive::ceo.ircGetDriveBehavior( ir.rc, motor );
		// --------------------------------------------------[ do the driving ]
		if( true == isGoDrive )
		{
			RTHerculesDrive::drive();
		}
	}

	// --- reset behavior
	ir.rc.isNewBehavior = false;

	return( isWorking );

}	// done stepIRC()

// -----------------------------------------------------------------[ readIRC ]
bool RTHerculesDrive::readIRC( rt_ioParam_t &iPad )
{

	ir.rc.key 	 	= wrt_ir_key_none;			// infrared remote control key
	ir.rc.isGoodKey =  false;
	isGoodKeyIR     =  false;

	iPad.ticks = irRx->getTicks();

	ir.rc.key = irRx->getKey();
	// -----------------------------
	if( ir.rc.key > 0 )
	{
		isGoodKeyIR = irRx->irResponse( ir.rc );
	}


	ir.rc.isGoodKey = isGoodKeyIR;
	// ---------------------------
	return( isGoodKeyIR );

}	 // done readIRC()



// ---------------------------------------------------[ stepObstacleAvoidance ]
void RTHerculesDrive::stepObstacleAvoidance( const uint16_t usrState )
{
	switch( usrState )
	{	default:
		case wrt_ir_state_idle:
		{
			useNominal = true;
			driveForward();

		}	break;
		case wrt_ir_state_irPulseLeft:
		{	// --- response to positive left IR pulse
			// -------------------------------------------------------------
			drive( wrt_reverse );
			// -------------------------------------------------------------
			delay(500);//Delay for 0.5 second

			// -------------------------------------------------------------
			drive( wrt_turn_right );
			// -------------------------------------------------------------
			delay(500);//Delay for 0.5 second

		}	break;
		case wrt_ir_state_irPulseForward:
		{
			// --- right IR pulse
			// -------------------------------------------------------------
			drive( wrt_reverse );
			// -------------------------------------------------------------
			delay(700);//delay for 0.7 second

			// -------------------------------------------------------------
			drive( wrt_turn_left );
			// -------------------------------------------------------------
			delay(200);//delay for 0.5 second

			// -------------------------------------------------------------
			drive( wrt_reverse );
			// -------------------------------------------------------------
			delay(300);//delay for 0.7 second

			// -------------------------------------------------------------
			drive( wrt_turn_left );
			// -------------------------------------------------------------
			delay(200);//delay for 0.5 second

		}	break;

		case wrt_ir_state_irPulseRight:
		{
			// --- right IR pulse
			// -------------------------------------------------------------
			drive( wrt_reverse );
			// -------------------------------------------------------------
			delay(500);//delay for 0.5 second

			// -------------------------------------------------------------
			drive( wrt_turn_left );
			// -------------------------------------------------------------
			delay(500);//delay for 0.5 second

		}	break;
	}	// done switch

}	// done stepObstacleAvoidance()

//  drive the robot
// -------------------------------------------------------------------[ drive ]
void RTHerculesDrive::drive( const uint16_t driveThisWay )
{
	// --- motor ON by default
	// -----------------------
	motor.left.control  |= wrt_motorControl_power_ON;
	motor.right.control |= wrt_motorControl_power_ON;


	switch( driveThisWay )
	{	default:
		// ------------------------------------------------------------[ stop ]
		case wrt_stop:
		{
			motor.left.control  &= wrt_motorControl_power_OFF;
			motor.right.control &= wrt_motorControl_power_OFF;
			// ---
			motor.left.speed   = 0;
			motor.right.speed  = 0;
			// ---
			drive();

		}	break;
		// ---------------------------------------------------------[ forward ]
		case wrt_forward:
		{
			this->useNominal = true;
			// ---
			driveForward();

		}	break;
		// ---------------------------------------------------------[ forward ]
		case wrt_forward_slow:
		{
			this->useNominal = false;
			motor.right.speed = wrt_code_motor_speed_hercules_min;
			motor.left.speed  = wrt_code_motor_speed_hercules_min;
			// ---
			driveForward();

		}	break;
		// ----------------------------------------------------[ fast forward ]
		case wrt_forward_fast:
		{
			this->useNominal = false;
			motor.right.speed = wrt_code_motor_speed_hercules_max;
			motor.left.speed  = wrt_code_motor_speed_hercules_max;
			// ---
			driveForward();

		}	break;
		// ---------------------------------------------------------[ reverse ]
		case wrt_reverse:
		{
			this->useNominal = true;
			// ---
			driveReverse();

		}	break;
		// ----------------------------------------------------[ fast reverse ]
		case wrt_reverse_fast:
		{
			this->useNominal = false;
			motor.right.speed = wrt_code_motor_speed_hercules_max;
			motor.left.speed  = wrt_code_motor_speed_hercules_max;
			// ---
			driveReverse();

		}	break;
		// -------------------------------------------------------[ left turn ]
		case wrt_drive_left:
		{
			driveLeft();

		}	break;
		// --------------------------------------------------[ fast left turn ]
		case wrt_turn_left_fast:
		{
			turnLeftFast();

		}	break;
		// --------------------------------------------------[ slow left turn ]
		case wrt_turn_left:
		{
			this->useNominal = true;

			turnLeft();

		}	break;
		// ------------------------------------------------------[ right turn ]
		case wrt_drive_right:
		{
			driveRight();

		}	break;
		// -------------------------------------------------[ fast right turn ]
		case wrt_turn_right_fast:
		{
			turnRightFast();

		}	break;
		// -------------------------------------------------[ slow right turn ]
		case wrt_turn_right:
		{
			this->useNominal = true;

			turnRight();

		}	break;
		// -----------------------------------------[ differential right turn ]
		case wrt_turnRightDiff:
		case wrt_turnRightDiff_fast:
		{
			turnRightDiferential( driveThisWay );

		}	break;
		// -----------------------------------------[ differential left turn ]
		case wrt_turnLeftDiff:
		case wrt_turnLeftDiff_fast:
		{
			turnLeftDiferential( driveThisWay );

		}	break;
	}


}	// done drive()


// ------------------------------------------------------------[ driveForward ]
void RTHerculesDrive::driveForward( void )
{
	if( true == useNominal )
	{
		motor.right.speed = wrt_code_motor_speed_hercules_medium;
		motor.left.speed  = wrt_code_motor_speed_hercules_medium;
	}
	// ---
	useNominal = false;	// context sensitive, reset to false after use

	motor.left.control  |= wrt_motorControl_direction_Forward;
	motor.right.control |= wrt_motorControl_direction_Forward;

	// --- drive with current settings
	RTHerculesDrive::drive();

}	// done driveForward()

// ------------------------------------------------------------[ driveReverse ]
void RTHerculesDrive::driveReverse( void )
{
	if( true == useNominal )
	{
		motor.right.speed = wrt_code_motor_speed_hercules_medium;
		motor.left.speed  = wrt_code_motor_speed_hercules_medium;
	}
	// ---
	useNominal = false;	// context sensitive, reset to false after use

	motor.left.control  &= wrt_motorControl_direction_Reverse;
	motor.right.control &= wrt_motorControl_direction_Reverse;

	// --- drive with current settings
	RTHerculesDrive::drive();

}	// done driveReverse()

// -----------------------------------------------------[ turnLeftDiferential ]
void RTHerculesDrive::turnLeftDiferential( uint16_t turnHow )
{
	switch( turnHow )
	{	default:
		case wrt_turnLeftDiff:
		{
			motor.right.speed = wrt_code_motor_speed_hercules_medium;
			motor.left.speed  = wrt_code_motor_speed_hercules_nominal;

		}	break;
		case wrt_turnLeftDiff_fast:
		{
			motor.right.speed = wrt_code_motor_speed_hercules_max;
			motor.left.speed  = wrt_code_motor_speed_hercules_low;

		}	break;
	}
	// ---
	useNominal = false;	// context sensitive, reset to false after use


	motor.left.control  |= wrt_motorControl_direction_Forward;
	motor.right.control |= wrt_motorControl_direction_Forward;

	// --- drive with current settings
	RTHerculesDrive::drive();

}	// done turnLeftDiferential()

// ----------------------------------------------------[ turnRightDiferential ]
void RTHerculesDrive::turnRightDiferential( uint16_t turnHow )
{
	switch( turnHow )
	{	default:
		case wrt_turnRightDiff:
		{
			motor.left.speed  = wrt_code_motor_speed_hercules_medium;
			motor.right.speed = wrt_code_motor_speed_hercules_nominal;

		}	break;
		case wrt_turnRightDiff_fast:
		{
			motor.left.speed  = wrt_code_motor_speed_hercules_max;
			motor.right.speed = wrt_code_motor_speed_hercules_low;

		}	break;
	}
	// ---
	useNominal = false;	// context sensitive, reset to false after use


	motor.left.control  |= wrt_motorControl_direction_Forward;
	motor.right.control |= wrt_motorControl_direction_Forward;

	// --- drive with current settings
	RTHerculesDrive::drive();

}	// done turnRightDiferential()

// -----------------------------------------------------------[ turnRightFast ]
void RTHerculesDrive::turnRightFast( void  )
{
	// --- fast == max speed
	this->useNominal = false;
	// ---
	motor.right.speed = wrt_code_motor_speed_hercules_max;
	motor.left.speed  = wrt_code_motor_speed_hercules_max;

	// ---
	RTHerculesDrive::turnRight();

}	// done turnRightFast()

// --------------------------------------------------------------[ turnRight ]
void RTHerculesDrive::turnRight( void  )
{
	if( true == useNominal )
	{
		motor.right.speed = wrt_code_motor_speed_hercules_medium;
		motor.left.speed  = wrt_code_motor_speed_hercules_medium;
	}
	// ---
	useNominal = false;	// context sensitive, reset to false after use

	// --- motor has power by default
	motor.left.control  |= wrt_motorControl_power_ON;
	motor.right.control |= wrt_motorControl_power_ON;

	motor.left.control  |= wrt_motorControl_direction_Forward;
	motor.right.control &= wrt_motorControl_direction_Reverse;

	// --- drive with current settings
	RTHerculesDrive::drive();

}	// done turnRight()

// -----------------------------------------------------------[ turnLeftFast ]
void RTHerculesDrive::turnLeftFast( void )
{
	// --- fast == max speed
	this->useNominal = false;
	// ---
	motor.right.speed = wrt_code_motor_speed_hercules_max;
	motor.left.speed  = wrt_code_motor_speed_hercules_max;

	RTHerculesDrive::turnLeft();

}	// done turnLeftFast()

// ----------------------------------------------------------------[ turnLeft ]
void RTHerculesDrive::turnLeft( void )
{
	if( true == useNominal )
	{
		motor.right.speed = wrt_code_motor_speed_hercules_medium;
		motor.left.speed  = wrt_code_motor_speed_hercules_medium;
	}
	// ---
	useNominal = false;	// context sensitive, reset to false after use

	// --- motor has power by default
	motor.left.control  |= wrt_motorControl_power_ON;
	motor.right.control |= wrt_motorControl_power_ON;

	motor.left.control  &= wrt_motorControl_direction_Reverse;
	motor.right.control |= wrt_motorControl_direction_Forward;


	// --- drive with current settings
	RTHerculesDrive::drive();

}	// done turnLeft()

// --------------------------------------------------------------[ driveRight ]
void RTHerculesDrive::driveRight( void  )
{
	motor.right.speed  = wrt_code_motor_speed_hercules_nominal - wrt_code_motor_speed_hercules_smallIncrement;
	motor.left.speed   = wrt_code_motor_speed_hercules_nominal + wrt_code_motor_speed_hercules_smallIncrement;
	// ---
	useNominal = false;	// context sensitive, reset to false after use

	motor.left.control  |= wrt_motorControl_direction_Forward;
	motor.right.control |= wrt_motorControl_direction_Forward;

	// --- drive with current settings
	RTHerculesDrive::drive();

}	// done driveRight()

// ---------------------------------------------------------------[ driveLeft ]
void RTHerculesDrive::driveLeft( void  )
{
	motor.right.speed = wrt_code_motor_speed_hercules_nominal + wrt_code_motor_speed_hercules_smallIncrement;
	motor.left.speed  = wrt_code_motor_speed_hercules_nominal - wrt_code_motor_speed_hercules_smallIncrement;
	// ---
	useNominal = false;	// context sensitive, reset to false after use

	motor.left.control  |= wrt_motorControl_direction_Forward;
	motor.right.control |= wrt_motorControl_direction_Forward;

	// --- drive with current settings
	RTHerculesDrive::drive();

}	// done driveLeft()

void RTHerculesDrive::drive( void )
{
	uint8_t speedLeft;
	uint8_t speedRight;

	uint8_t dirLeft;
	uint8_t dirRight;

	if( true == motor.isShiftDirection )
	{
		// --- abrupt change in direction detected; reset
		motor.isShiftDirection = false;

		// --- stop first before going on
		RTHerculesDrive::herc.stop();

		// --- should we delay here?

	}


	// --- check/limit speed:
	// -------------------------------------------------------------------
	speedLeft  = (motor.left.speed  < 0 ) ? 0  : motor.left.speed;
	// ---
	speedRight = (motor.right.speed < 0 ) ? 0  : motor.right.speed;
	// -------------------------------------------------------------------
	speedLeft  = (speedLeft  < wrt_code_motor_speed_hercules_max ) ?
				  speedLeft  : wrt_code_motor_speed_hercules_max;
	// ---
	speedRight = (speedRight < wrt_code_motor_speed_hercules_max ) ?
				  speedRight : wrt_code_motor_speed_hercules_max;
	// -------------------------------------------------------------------

	// --- check for power to the motors
	// -------------------------------------------------------------------
	if( 0 == (motor.left.control & wrt_motorControl_power_bit) )
	{	speedLeft  = wrt_code_motor_stop;
	}
	if( 0 == (motor.right.control & wrt_motorControl_power_bit) )
	{	speedRight = wrt_code_motor_stop;
	}

	// --- left motor direction: bit set = forward, off = backwards
	dirLeft = (0 == (motor.left.control & wrt_motorControl_direction_bit)) ?
							rt_robot_hercules_direction_reverse :
							rt_robot_hercules_direction_forward ;

	// --- right motor direction
	dirRight = (0 == (motor.right.control & wrt_motorControl_direction_bit)) ?
							rt_robot_hercules_direction_reverse :
							rt_robot_hercules_direction_forward ;


	// --- left motor
	if( speedLeft <= 0 )
	{
		herc.setStopLeft();
	}
	else
	{	// -------------------------------------------------------------------
		speedLeft  = (speedLeft  > wrt_code_motor_speed_hercules_min ) ?
					  speedLeft  : wrt_code_motor_speed_hercules_min;
		// -------------------------------------------------------------------
		herc.setSpeedDirLeft(speedLeft, dirLeft);
	}

	// --- right motor
	if( speedRight <= 0 )
	{
		herc.setStopRight();
	}
	else
	{	// -------------------------------------------------------------------
		speedRight = (speedRight > wrt_code_motor_speed_hercules_min ) ?
					  speedRight : wrt_code_motor_speed_hercules_min;
		// -------------------------------------------------------------------
		herc.setSpeedDirRight(speedRight, dirRight);
	}


}	// done drive()

// ----------------------------------------------------------------------------
//  stop all motors
// --------------------------------------------------------------------[ stop ]
void RTHerculesDrive::stop( void )
{
	herc.stop();

}	// done stop()

} /* namespace RT_HERCULES_DRIVE */
