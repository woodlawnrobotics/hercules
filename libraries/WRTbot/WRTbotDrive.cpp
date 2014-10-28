/*
 * WRTbotDrive.cpp
 *
 *  Created on: Jul 23, 2013
 *      Author: simon
 */
// ---------------------------------------------------------------------[ BoF ]
#include "Arduino.h"

#include "WRTbotDrive.h"

namespace WRT_ARDUINO_BOT_DRIVE
{

// -------------------------------------------------------------[ consturctor ]
WRTbotDrive::WRTbotDrive()
{
	useNominal = false;	// context sensitive, reset to false after use

	// --- clear memory
	memset( &sensor, 0, sizeof( struct WRT_ROBOT_SENSORS_tag) );

	// --- default operational widow
	sensor.light.limitMin			= 0.5;	// minimum limit
	sensor.light.limitLowerBound	= 2.0;	// highest lower bound
	sensor.light.limitUpperBound	= 4.0;	// lowest  upper bound
	sensor.light.limitMax			= 5.0;	// maximum limit
	// ---
	sensor.light.factor	= static_cast<float>(wrt_analog_referenceVolts) / 1024.0F;

	driveByLightFactor = 0;

}	// done consturctor()

// --------------------------------------------------------------[ destructor ]
WRTbotDrive::~WRTbotDrive()
{
}	// done destructor

// --------------------------------------------------------------------[ init ]
void WRTbotDrive::init( void )
{
	//	Digital Pin 4:Left Motor Direction Pin
	pinMode (wrt_pin_motor_direction_left, OUTPUT);

	//	Digital Pin 5:Left Motor Enable Pin
	pinMode (wrt_pin_motor_speed_left, OUTPUT);

	//	Digital Pin 6:Right Motor Enable Pin
	pinMode (wrt_pin_motor_speed_right, OUTPUT);

	//	Digital Pin 7:Right Motor Direction Pin
	pinMode (wrt_pin_motor_direction_right, OUTPUT);

}	// done init()

// ----------------------------------------------------------------------------
//  set new behavior and perform default actions
// ---------------------------------------------------------------------[ set ]
void WRTbotDrive::set( wrt_ir_rc_t &rc, wrt_motor_t &motor )
{

	switch( rc.behavior & wrt_rc_cmd_behavior_set )
	{	default:
		// -------------------------------[ default behavior: ir hand control ]
		case wrt_rc_cmd_behavior_irc:
		{	// ----------------------------------------------------------------
			// --- do: respond to hand held infrared remote control motion keys
			// --- do: forward (key: ^  ), reverse (key: v   )
			// --- do: left	   (key: |<<), right   (key: >>| )
			// ----------------------------------------------------------------
			setBehavior( rc, motor );

		}	break;
		// ------------------------------------[ behavior: hunt ambient light ]
		case wrt_rc_cmd_behavior_lightHunter:
		{	// ----------------------------------------------------------------
			// --- do: hunt for ambient light sensor
			// --- do: stop if no light is sensensed
			// --- do: avoid obstacles?
			// ----------------------------------------------------------------
			setBehavior( rc, motor );

		}	break;
		// --------------------------------------------------[ behavior: pray ]
		case wrt_rc_cmd_behavior_pray:
		{	// ----------------------------------------------------------------
			// --- do: avoid obstacles, turn away from obstacles
			// --- do: stop if no way forward
			// --- do: backtrack and turn if no way forward after N msec
			// ----------------------------------------------------------------


		}	break;
	}	// switch

}	// done set()

// ----------------------------------------------------------------------------
//  reset commands, implement behavior
// -------------------------------------------------------------[ setBehavior ]
void WRTbotDrive::setBehavior( wrt_ir_rc_t &rc, wrt_motor_t &motor )
{
	switch( rc.behavior & wrt_rc_cmd_behavior_set )
	{	default:
		// ---------------------------------------------------------[ forward ]
		case wrt_rc_cmd_behavior_irc:		// key combination: (key: >|| )
		{
			// --- clear old preserved command history
			rc.commandDirection  = 0;
			rc.commandDirection |= wrt_rc_cmd_forward;
			// ---
			//motor.left.direction  = static_cast<uint32_t>(wrt_code_motor_drive_forward);
			//motor.right.direction = static_cast<uint32_t>(wrt_code_motor_drive_forward);
			motor.left.control  |= wrt_motorControl_direction_Forward;
			motor.right.control |= wrt_motorControl_direction_Forward;

			// --- set nominal forward speed
			motor.right.speed = wrt_code_motor_speed_nominal;
			motor.left.speed  = wrt_code_motor_speed_nominal;

		}	break;
		// ---------------------------------------[ hunt ambient light sensor ]
		case wrt_rc_cmd_behavior_lightHunter:	// key combination: (key: EQ )
		{
			// --- clear old preserved command history
			rc.commandDirection  = 0;
			rc.commandDirection |= wrt_rc_cmd_forward;
			// ---
			motor.left.control  |= wrt_motorControl_direction_Forward;
			motor.right.control |= wrt_motorControl_direction_Forward;

			// --- stop the robot momentarily
			motor.right.speed = wrt_code_motor_stop;
			motor.left.speed  = wrt_code_motor_stop;

		}	break;
	}	// switch
	// -------------------------------------------------------------------
	motor.left.control  |= wrt_motorControl_power_ON;
	motor.right.control |= wrt_motorControl_power_ON;


	// --- verify power button setting
	// -----------------------------------------------------------------------
	if( 0 == (rc.command & wrt_rc_cmd_toggle_power) )
	{
		motor.left.control  &= wrt_motorControl_power_OFF;
		motor.right.control &= wrt_motorControl_power_OFF;

		motor.left.speed  = wrt_code_motor_stop;
		motor.right.speed = wrt_code_motor_stop;
	}
	// -----------------------------------------------------------------------

	// --- drive now
	// ----------------------
	WRTbotDrive::drive( motor );

}	// done setBehavior()

// ----------------------------------------------------------------------------
//  infrared remote control command drive the robot
// ----------------------------------------------------------------[ ircDrive ]
void WRTbotDrive::ircDrive( wrt_ir_rc_t &rc, wrt_motor_t &motor )
{
	// --- check/limit speed:
	// -------------------------------------------------------------------
	motor.left.speed  = (motor.left.speed  < 0 ) ?
				    0  : motor.left.speed;
	// ---
	motor.right.speed = (motor.right.speed < 0 ) ?
					0  : motor.right.speed;
	// -------------------------------------------------------------------
	motor.left.speed  = (motor.left.speed  < wrt_code_motor_speed_max ) ?
						 motor.left.speed  : wrt_code_motor_speed_max;
	// ---
	motor.right.speed = (motor.right.speed < wrt_code_motor_speed_max ) ?
						 motor.right.speed : wrt_code_motor_speed_max;
	// -------------------------------------------------------------------

	// --- move forward by default
	// -------------------------------------------------------------------
	motor.left.control  |= wrt_motorControl_direction_Forward;
	motor.right.control |= wrt_motorControl_direction_Forward;

	// --- from preserved command history
	if( (0 == (rc.commandDirection & wrt_rc_cmd_mask_fvset)) &&
		(0 == (rc.commandDirection & wrt_rc_cmd_mask_svSet)) )
	{
		// --- set nominal forward speed
		motor.right.speed = wrt_code_motor_speed_nominal;
		motor.left.speed  = wrt_code_motor_speed_nominal;
	}

	switch( rc.command & wrt_rc_cmd_mask_set )
	{	// ---------------------------------------------------------[ forward ]
		case wrt_rc_cmd_forward:
		{
			// --- update speed:
			motor.left.speed  += wrt_code_motor_speed_increment;
			motor.right.speed += wrt_code_motor_speed_increment;

			// --- from preserved command history
			if( 0 == (rc.commandDirection & static_cast<uint32_t>(wrt_rc_cmd_forward) ) )
			{
				WRTbotDrive::stop();

				// --- set nominal forward speed
				motor.right.speed = wrt_code_motor_speed_nominal;
				motor.left.speed  = wrt_code_motor_speed_nominal;
			}

			// --- reset direction command history
			rc.commandDirection = static_cast<uint32_t>(wrt_rc_cmd_forward);

		}	break;
		// ---------------------------------------------------------[ forward ]
		case wrt_rc_cmd_reverse:
		{
			// --- update speed:
			motor.left.speed  += wrt_code_motor_speed_increment;
			motor.right.speed += wrt_code_motor_speed_increment;

			// --- from preserved command history
			if( 0 == (rc.commandDirection & static_cast<uint32_t>(wrt_rc_cmd_reverse) ) )
			{
				WRTbotDrive::stop();

				// --- set nominal forward speed
				motor.right.speed = wrt_code_motor_speed_nominal;
				motor.left.speed  = wrt_code_motor_speed_nominal;
			}

			// --- reset direction command history
			rc.commandDirection = static_cast<uint32_t>(wrt_rc_cmd_reverse);
			// ---
			//motor.left.direction  = static_cast<uint32_t>(wrt_code_motor_drive_reverse);
			//motor.right.direction = static_cast<uint32_t>(wrt_code_motor_drive_reverse);
			motor.left.control  &= wrt_motorControl_direction_Reverse;
			motor.right.control &= wrt_motorControl_direction_Reverse;

		}	break;
		// -------------------------------------------------------[ turn left ]
		case wrt_rc_cmd_left:
		{
			motor.right.speed +=  wrt_code_motor_speed_increment;
			// --------------------------------------------------------
			if( motor.right.speed < static_cast<int>(wrt_code_motor_speed_min) )
			{
				motor.right.speed =  wrt_code_motor_speed_min + wrt_code_motor_speed_increment;
			}

			motor.left.speed  -= (wrt_code_motor_speed_increment >> 2);
			// --------------------------------------------------------
			if( motor.left.speed < static_cast<int>(wrt_code_motor_speed_min) )
			{
				motor.left.speed =  wrt_code_motor_speed_min;
			}
		}	break;
		// ------------------------------------------------------[ turn right ]
		case wrt_rc_cmd_right:
		{
			motor.left.speed  +=  wrt_code_motor_speed_increment;
			// --------------------------------------------------------
			if( motor.left.speed < static_cast<int>(wrt_code_motor_speed_min) )
			{
				motor.left.speed =  wrt_code_motor_speed_min + wrt_code_motor_speed_increment;
			}

			motor.right.speed -= (wrt_code_motor_speed_increment >> 2);
			// --------------------------------------------------------
			if( motor.right.speed < static_cast<int>(wrt_code_motor_speed_min) )
			{
				motor.right.speed =  wrt_code_motor_speed_min;
			}
		}	break;
		// -------------------------------------------------------[ spin left ]
		case wrt_rc_cmd_spin_left:
		{
			motor.right.speed += wrt_code_motor_speed_rotate;
			motor.left.speed  += wrt_code_motor_speed_rotate;

			// --- from preserved command history
			if( 0 == (rc.commandDirection & static_cast<uint32_t>(wrt_rc_cmd_spin_left) ) )
			{
				WRTbotDrive::stop();

				// --- set nominal forward speed
				motor.right.speed =  wrt_code_motor_speed_rotate;
				motor.left.speed  =  wrt_code_motor_speed_rotate;
			}

			// --- reset direction command history
			rc.commandDirection = static_cast<uint32_t>(wrt_rc_cmd_spin_left);
			// ---
			motor.left.control  &= wrt_motorControl_direction_Reverse;
			motor.right.control |= wrt_motorControl_direction_Forward;


		}	break;
		// ------------------------------------------------------[ spin right ]
		case wrt_rc_cmd_spin_right:
		{
			motor.right.speed += wrt_code_motor_speed_rotate;
			motor.left.speed  += wrt_code_motor_speed_rotate;

			// --- from preserved command history
			if( 0 == (rc.commandDirection & static_cast<uint32_t>(wrt_rc_cmd_spin_right) ) )
			{
				WRTbotDrive::stop();

				// --- set nominal forward speed
				motor.right.speed = wrt_code_motor_speed_rotate;
				motor.left.speed  = wrt_code_motor_speed_rotate;
			}

			// --- reset direction command history
			rc.commandDirection = static_cast<uint32_t>(wrt_rc_cmd_spin_right);
			// ---
			motor.left.control  |= wrt_motorControl_direction_Forward;
			motor.right.control &= wrt_motorControl_direction_Reverse;


		}	break;
		// ---------------------------------------------------[ default: stop ]
		default:	// stop, do nothing
		{
			// --- update speed:
			motor.left.speed  = wrt_code_motor_stop;
			motor.right.speed = wrt_code_motor_stop;

		}	break;

	}	// switch

	motor.left.control  |= wrt_motorControl_power_ON;
	motor.right.control |= wrt_motorControl_power_ON;
	// -------------------------------------------------------------------
	if( 0 == (rc.command & static_cast<uint32_t>(wrt_rc_cmd_toggle_power) ) )
	{
		motor.left.control  &= wrt_motorControl_power_OFF;
		motor.right.control &= wrt_motorControl_power_OFF;

		motor.left.speed  = wrt_code_motor_stop;
		motor.right.speed = wrt_code_motor_stop;
	}

	// --- drive
	// -------------------------------------------------------------------
	WRTbotDrive::drive( motor );

}	// done ircDrive()

// ----------------------------------------------------------------------------
//  drive the robot forward
// -----------------------------------------------------------[ driveBehavior ]
bool WRTbotDrive::driveBehavior( const wrt_ir_rc_t &rc,
								       wrt_motor_t &motor )
{
	bool isBehaviorDoingWell = true;

	switch( rc.behavior )
	{	case wrt_rc_cmd_behavior_lightHunter:	// key combination: ( EQ )
		{
			driveByLight( motor );

		}	break;
		// ---------------------------------------------------------[ default ]
		default:
		case wrt_rc_cmd_behavior_pray:			// key combination: ( func/stop )
		case wrt_rc_cmd_behavior_predator:
		case wrt_rc_cmd_behavior_irc:			// key combination: (  >||  )
		{
			// --- drive with current settings
			WRTbotDrive::drive( motor );

		}	break;
	}	// switch

	return( isBehaviorDoingWell );

}	// done driveBehavior()

// ----------------------------------------------------------------------------
//  drive using light ambient sensors for directional command
// ------------------------------------------------------------[ driveByLight ]
void WRTbotDrive::driveByLight( wrt_motor_t &motor )
{
	// -------------------------------------------------------------------
	if( ( 0 == (motor.left.control  & wrt_motorControl_power_bit) ) ||
		( 0 == (motor.right.control & wrt_motorControl_power_bit) ) )
	{
		WRTbotDrive::stop();

		return;
	}

	// --- read ambient light sensor
	// -------------------------------------------------------------------
	sensor.light.data = analogRead( wrt_pin_ambientLight_sensor );

	Serial.println(" ");
	Serial.print( sensor.light.data );	Serial.print( " , " );

    // --- convert to ambient light units
	// -------------------------------------------------------------------
	sensor.light.data *= sensor.light.factor;

	Serial.print( sensor.light.data );	Serial.println( " , " );


	useNominal = true;
	// -------------------------------------------------------------------
	if( (sensor.light.data > sensor.light.limitMin) && 			// minimum limit
		(sensor.light.data < sensor.light.limitLowerBound) )	// highest lower bound
	{

		Serial.print( "driveLeft" );	Serial.println( "   " );


		driveLeft( motor );
	}
	else if( (sensor.light.data > sensor.light.limitUpperBound) &&	// lowest  upper bound
			 (sensor.light.data < sensor.light.limitMax) )			// maximum limit
	{

		Serial.print( "driveRight" );	Serial.println( "   " );

		driveRight( motor );
	}
	else
	{
		Serial.print( "driveForward" );	Serial.println( "  now " );

		driveForward( motor );
	}

}	// done driveByLight()

// --------------------------------------------------------------[ driveRight ]
void WRTbotDrive::driveRight( wrt_motor_t &motor  )
{
	switch( driveByLightFactor )
	{	case 1:
		case 2:
		case 3:
		{
			driveByLightFactor += ( driveByLightFactor < 3 ) ? 1 : 0;
			// ------------------------------------------------------
			motor.right.speed -= static_cast<int>(wrt_code_motor_speed_smallIncrement);
			motor.left.speed  += static_cast<int>(wrt_code_motor_speed_smallIncrement);

		}	break;
		default:
		{
			driveByLightFactor = 1;
			// ----------------------
			motor.right.speed  = wrt_code_motor_speed_nominal - wrt_code_motor_speed_smallIncrement;
			motor.left.speed   = wrt_code_motor_speed_nominal + wrt_code_motor_speed_smallIncrement;

		}	break;
	}
	// ---
	useNominal = false;	// context sensitive, reset to false after use

	motor.left.control  |= wrt_motorControl_direction_Forward;
	motor.right.control |= wrt_motorControl_direction_Forward;

	// --- drive with current settings
	WRTbotDrive::drive( motor );

}	// done driveRight()

// ---------------------------------------------------------------[ driveLeft ]
void WRTbotDrive::driveLeft( wrt_motor_t &motor  )
{
	switch( driveByLightFactor )
	{	case 1:
		case 2:
		case 3:
		{
			driveByLightFactor += ( driveByLightFactor < 3 ) ? 1 : 0;
			// ------------------------------------------------------
			motor.right.speed += static_cast<int>(wrt_code_motor_speed_smallIncrement);
			motor.left.speed  -= static_cast<int>(wrt_code_motor_speed_smallIncrement);

		}	break;
		default:
		{
			driveByLightFactor = 1;
			// ----------------------
			motor.right.speed = wrt_code_motor_speed_nominal + wrt_code_motor_speed_smallIncrement;
			motor.left.speed  = wrt_code_motor_speed_nominal - wrt_code_motor_speed_smallIncrement;

		}	break;
	}
	// ---
	useNominal = false;	// context sensitive, reset to false after use

	motor.left.control  |= wrt_motorControl_direction_Forward;
	motor.right.control |= wrt_motorControl_direction_Forward;

	// --- drive with current settings
	WRTbotDrive::drive( motor );

}	// done driveLeft()

// -----------------------------------------------------------[ turnRightFast ]
void WRTbotDrive::turnRightFast( wrt_motor_t &motor  )
{
	// --- fast == max speed
	this->useNominal = false;
	// ---
	motor.right.speed = wrt_code_motor_speed_max;
	motor.left.speed  = wrt_code_motor_speed_max;

	// ---
	turnRight( motor );

}	// done turnRightFast()

// --------------------------------------------------------------[ turnRight ]
void WRTbotDrive::turnRight( wrt_motor_t &motor  )
{
	if( true == useNominal )
	{
		motor.right.speed = wrt_code_motor_speed_medium;
		motor.left.speed  = wrt_code_motor_speed_medium;
	}
	// ---
	useNominal = false;	// context sensitive, reset to false after use

	// --- motor has power by default
	motor.left.control  |= wrt_motorControl_power_ON;
	motor.right.control |= wrt_motorControl_power_ON;

	motor.left.control  |= wrt_motorControl_direction_Forward;
	motor.right.control &= wrt_motorControl_direction_Reverse;

	// --- drive with current settings
	WRTbotDrive::drive( motor );

}	// done turnRight()

// -----------------------------------------------------------[ turnLeftFast ]
void WRTbotDrive::turnLeftFast( wrt_motor_t &motor  )
{
	// --- fast == max speed
	this->useNominal = false;
	// ---
	motor.right.speed = wrt_code_motor_speed_max;
	motor.left.speed  = wrt_code_motor_speed_max;

	turnLeft( motor );

}	// done turnLeftFast()

// ----------------------------------------------------------------[ turnLeft ]
void WRTbotDrive::turnLeft( wrt_motor_t &motor  )
{
	if( true == useNominal )
	{
		motor.right.speed = wrt_code_motor_speed_medium;
		motor.left.speed  = wrt_code_motor_speed_medium;
	}
	// ---
	useNominal = false;	// context sensitive, reset to false after use

	// --- motor has power by default
	motor.left.control  |= wrt_motorControl_power_ON;
	motor.right.control |= wrt_motorControl_power_ON;

	motor.left.control  &= wrt_motorControl_direction_Reverse;
	motor.right.control |= wrt_motorControl_direction_Forward;


	// --- drive with current settings
	WRTbotDrive::drive( motor );

}	// done turnLeft()

// -----------------------------------------------------[ turnLeftDiferential ]
void WRTbotDrive::turnLeftDiferential( uint16_t turnHow, wrt_motor_t &motor  )
{
	switch( turnHow )
	{	default:
		case wrt_turnLeftDiff:
		{
			motor.right.speed = wrt_code_motor_speed_medium;
			motor.left.speed  = wrt_code_motor_speed_nominal;

		}	break;
		case wrt_turnLeftDiff_fast:
		{
			motor.right.speed = wrt_code_motor_speed_max;
			motor.left.speed  = wrt_code_motor_speed_low;

		}	break;
	}
	// ---
	useNominal = false;	// context sensitive, reset to false after use


	motor.left.control  |= wrt_motorControl_direction_Forward;
	motor.right.control |= wrt_motorControl_direction_Forward;

	// --- drive with current settings
	WRTbotDrive::drive( motor );

}	// done turnLeftDiferential()

// ----------------------------------------------------[ turnRightDiferential ]
void WRTbotDrive::turnRightDiferential( uint16_t turnHow, wrt_motor_t &motor  )
{
	switch( turnHow )
	{	default:
		case wrt_turnRightDiff:
		{
			motor.left.speed  = wrt_code_motor_speed_medium;
			motor.right.speed = wrt_code_motor_speed_nominal;

		}	break;
		case wrt_turnRightDiff_fast:
		{
			motor.left.speed  = wrt_code_motor_speed_max;
			motor.right.speed = wrt_code_motor_speed_low;

		}	break;
	}
	// ---
	useNominal = false;	// context sensitive, reset to false after use


	motor.left.control  |= wrt_motorControl_direction_Forward;
	motor.right.control |= wrt_motorControl_direction_Forward;

	// --- drive with current settings
	WRTbotDrive::drive( motor );

}	// done turnRightDiferential()


// ------------------------------------------------------------[ driveForward ]
void WRTbotDrive::driveForward( wrt_motor_t &motor  )
{
	if( true == useNominal )
	{
		motor.right.speed = wrt_code_motor_speed_medium;
		motor.left.speed  = wrt_code_motor_speed_medium;
	}
	// ---
	useNominal = false;	// context sensitive, reset to false after use
	driveByLightFactor = 0;

	motor.left.control  |= wrt_motorControl_direction_Forward;
	motor.right.control |= wrt_motorControl_direction_Forward;

	// --- drive with current settings
	WRTbotDrive::drive( motor );

}	// done driveForward()

// ------------------------------------------------------------[ driveReverse ]
void WRTbotDrive::driveReverse( wrt_motor_t &motor  )
{
	if( true == useNominal )
	{
		motor.right.speed = wrt_code_motor_speed_medium;
		motor.left.speed  = wrt_code_motor_speed_medium;
	}
	// ---
	useNominal = false;	// context sensitive, reset to false after use
	driveByLightFactor = 0;

	motor.left.control  &= wrt_motorControl_direction_Reverse;
	motor.right.control &= wrt_motorControl_direction_Reverse;

	// --- drive with current settings
	WRTbotDrive::drive( motor );

}	// done driveReverse()

//  drive the robot
// -------------------------------------------------------------------[ drive ]
void WRTbotDrive::drive( const uint16_t driveThisWay, wrt_motor_t &motor )
{
	// --- motor ON by default
	// -----------------------
	motor.left.control  |= wrt_motorControl_power_ON;
	motor.right.control |= wrt_motorControl_power_ON;

	// --- no drive by light
	// --------------------------
	this->driveByLightFactor = 0;

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
			drive( motor );

		}	break;
		// ---------------------------------------------------------[ forward ]
		case wrt_forward:
		{
			this->useNominal = true;
			// ---
			driveForward( motor );

		}	break;
		// ---------------------------------------------------------[ forward ]
		case wrt_forward_slow:
		{
			this->useNominal = false;
			motor.right.speed = wrt_code_motor_speed_min;
			motor.left.speed  = wrt_code_motor_speed_min;
			// ---
			driveForward( motor );

		}	break;
		// ----------------------------------------------------[ fast forward ]
		case wrt_forward_fast:
		{
			this->useNominal = false;
			motor.right.speed = wrt_code_motor_speed_max;
			motor.left.speed  = wrt_code_motor_speed_max;
			// ---
			driveForward( motor );

		}	break;
		// ---------------------------------------------------------[ reverse ]
		case wrt_reverse:
		{
			this->useNominal = true;
			// ---
			driveReverse( motor );

		}	break;
		// ----------------------------------------------------[ fast reverse ]
		case wrt_reverse_fast:
		{
			this->useNominal = false;
			motor.right.speed = wrt_code_motor_speed_max;
			motor.left.speed  = wrt_code_motor_speed_max;
			// ---
			driveReverse( motor );

		}	break;
		// -------------------------------------------------------[ left turn ]
		case wrt_drive_left:
		{
			driveLeft( motor );

		}	break;
		// --------------------------------------------------[ fast left turn ]
		case wrt_turn_left_fast:
		{
			turnLeftFast( motor );

		}	break;
		// --------------------------------------------------[ slow left turn ]
		case wrt_turn_left:
		{
			this->useNominal = true;

			turnLeft( motor );

		}	break;
		// ------------------------------------------------------[ right turn ]
		case wrt_drive_right:
		{
			driveRight( motor );

		}	break;
		// -------------------------------------------------[ fast right turn ]
		case wrt_turn_right_fast:
		{
			turnRightFast( motor );

		}	break;
		// -------------------------------------------------[ slow right turn ]
		case wrt_turn_right:
		{
			this->useNominal = true;

			turnRight( motor );

		}	break;
		// -----------------------------------------[ differential right turn ]
		case wrt_turnRightDiff:
		case wrt_turnRightDiff_fast:
		{
			turnRightDiferential( driveThisWay, motor );

		}	break;
		// -----------------------------------------[ differential left turn ]
		case wrt_turnLeftDiff:
		case wrt_turnLeftDiff_fast:
		{
			turnLeftDiferential( driveThisWay, motor );

		}	break;
	}


}	// done drive()


//  drive the robot
// -------------------------------------------------------------------[ drive ]
void WRTbotDrive::drive( const wrt_motor_t &motor )
{
	uint32_t speedLeft;
	uint32_t speedRight;


	// --- check/limit speed:
	// -------------------------------------------------------------------
	speedLeft  = (motor.left.speed  < 0 ) ? 0  : motor.left.speed;
	// ---
	speedRight = (motor.right.speed < 0 ) ? 0  : motor.right.speed;
	// -------------------------------------------------------------------
	speedLeft  = (speedLeft  < wrt_code_motor_speed_max ) ?
				  speedLeft  : wrt_code_motor_speed_max;
	// ---
	speedRight = (speedRight < wrt_code_motor_speed_max ) ?
				  speedRight : wrt_code_motor_speed_max;
	// -------------------------------------------------------------------

	// --- check for power to the motors
	// -------------------------------------------------------------------
	if( 0 == (motor.left.control & wrt_motorControl_power_bit) )
	{	speedLeft  = wrt_code_motor_stop;
	}
	if( 0 == (motor.right.control & wrt_motorControl_power_bit) )
	{	speedRight = wrt_code_motor_stop;
	}

	// --- left motor direction
	digitalWrite (wrt_pin_motor_direction_left,
			(0 == (motor.left.control & wrt_motorControl_direction_bit)) ? LOW : HIGH );

	// --- right motor direction
	digitalWrite (wrt_pin_motor_direction_right,
		(0 == (motor.right.control & wrt_motorControl_direction_bit)) ? LOW : HIGH );

	// --- left motor speed
	if( speedLeft <= 0 )
	{
		analogWrite (wrt_pin_motor_speed_left, LOW );
	}
	else
	{	// -------------------------------------------------------------------
		speedLeft  = (speedLeft  > wrt_code_motor_speed_min ) ?
					  speedLeft  : wrt_code_motor_speed_min;
		// -------------------------------------------------------------------
		analogWrite (wrt_pin_motor_speed_left, speedLeft );
	}

	// --- right motor speed
	if( speedRight <= 0 )
	{
		analogWrite (wrt_pin_motor_speed_right, LOW );
	}
	else
	{	// -------------------------------------------------------------------
		speedRight = (speedRight > wrt_code_motor_speed_min ) ?
					  speedRight : wrt_code_motor_speed_min;
		// -------------------------------------------------------------------
		analogWrite (wrt_pin_motor_speed_right, speedRight );
	}


}	// done drive()

// ----------------------------------------------------------------------------
//  stop all motors
// --------------------------------------------------------------------[ stop ]
void WRTbotDrive::stop( void )
{
	// --- to reverse direction: stop first
	analogWrite (wrt_pin_motor_speed_left,  LOW );
	analogWrite (wrt_pin_motor_speed_right, LOW );

	// --- delay for nominal msec
	delay( wrt_code_motor_stop_time_msec );

}	// done stop()


// ---------------------------------------------------------------------[ EoF ]
} /* namespace WRT_ARDUINO_BOT_DRIVE */
