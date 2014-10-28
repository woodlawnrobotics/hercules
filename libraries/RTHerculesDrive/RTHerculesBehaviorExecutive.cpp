/*
 * RTHerculesBehaviorExecutive.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: simon
 */

#include "RTHerculesBehaviorExecutive.h"

namespace RT_HERCULES_BEHAVIOR_EXECUTIVE
{

RTHerculesBehaviorExecutive::RTHerculesBehaviorExecutive()
{
	// TODO Auto-generated constructor stub

}

RTHerculesBehaviorExecutive::~RTHerculesBehaviorExecutive()
{
	// TODO Auto-generated destructor stub
}


void RTHerculesBehaviorExecutive::init( void )
{



}	// done init()

// ----------------------------------------------------------------------------
//  infrared remote control command drive the robot
// ----------------------------------------------------------------[ ircDrive ]
void RTHerculesBehaviorExecutive::ircGetDriveBehavior( wrt_ir_rc_t &rc, wrt_motor_t &motor )
{
	// --- check/limit speed:
	// -------------------------------------------------------------------
	motor.left.speed  = (motor.left.speed  < 0 ) ?
				    0  : motor.left.speed;
	// ---
	motor.right.speed = (motor.right.speed < 0 ) ?
					0  : motor.right.speed;
	// -------------------------------------------------------------------
	motor.left.speed  = (motor.left.speed  < wrt_code_motor_speed_hercules_max ) ?
						 motor.left.speed  : wrt_code_motor_speed_hercules_max;
	// ---
	motor.right.speed = (motor.right.speed < wrt_code_motor_speed_hercules_max ) ?
						 motor.right.speed : wrt_code_motor_speed_hercules_max;
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
		motor.right.speed = wrt_code_motor_speed_hercules_nominal;
		motor.left.speed  = wrt_code_motor_speed_hercules_nominal;
	}

	switch( rc.command & wrt_rc_cmd_mask_set )
	{	// ---------------------------------------------------[ default: stop ]
		default:	// stop, do nothing
		{
			// --- update speed:
			motor.left.speed  = wrt_code_motor_stop;
			motor.right.speed = wrt_code_motor_stop;

		}	break;
		// ---------------------------------------------------------[ forward ]
		case wrt_rc_cmd_forward:
		{
			// --- update speed:
			if( false == motor.isResetDirection )
			{	motor.left.speed  += wrt_code_motor_speed_hercules_increment;
				motor.right.speed += wrt_code_motor_speed_hercules_increment;
			}
			// ---
			motor.isResetDirection = false;

			// --- from preserved command history
			if( 0 == (rc.commandDirection & static_cast<uint32_t>(wrt_rc_cmd_forward) ) )
			{
				motor.isShiftDirection = true;		// stop first
				motor.isResetDirection = true;

				// --- set nominal forward speed
				motor.right.speed = wrt_code_motor_speed_hercules_nominal;
				motor.left.speed  = wrt_code_motor_speed_hercules_nominal;
			}

			// --- reset direction command history
			rc.commandDirection = static_cast<uint32_t>(wrt_rc_cmd_forward);

		}	break;
		// ---------------------------------------------------------[ reverse ]
		case wrt_rc_cmd_reverse:
		{
			// --- update speed:
			if( false == motor.isResetDirection )
			{	motor.left.speed  += wrt_code_motor_speed_hercules_increment;
				motor.right.speed += wrt_code_motor_speed_hercules_increment;
			}
			// ---
			motor.isResetDirection = false;

			// --- from preserved command history
			if( 0 == (rc.commandDirection & static_cast<uint32_t>(wrt_rc_cmd_reverse) ) )
			{
				motor.isShiftDirection = true;		// stop first
				motor.isResetDirection = true;

				// --- set nominal forward speed
				motor.right.speed = wrt_code_motor_speed_hercules_nominal;
				motor.left.speed  = wrt_code_motor_speed_hercules_nominal;
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
			motor.right.speed +=  wrt_code_motor_speed_hercules_increment;
			// --------------------------------------------------------
			if( motor.right.speed < static_cast<int>(wrt_code_motor_speed_hercules_min) )
			{
				motor.right.speed =  wrt_code_motor_speed_hercules_min + wrt_code_motor_speed_hercules_increment;
			}

			motor.left.speed  -= (wrt_code_motor_speed_hercules_increment >> 2);
			// --------------------------------------------------------
			if( motor.left.speed < static_cast<int>(wrt_code_motor_speed_hercules_min) )
			{
				motor.left.speed =  wrt_code_motor_speed_hercules_min;
			}
		}	break;
		// ------------------------------------------------------[ turn right ]
		case wrt_rc_cmd_right:
		{
			motor.left.speed  +=  wrt_code_motor_speed_hercules_increment;
			// --------------------------------------------------------
			if( motor.left.speed < static_cast<int>(wrt_code_motor_speed_hercules_min) )
			{
				motor.left.speed =  wrt_code_motor_speed_hercules_min + wrt_code_motor_speed_hercules_increment;
			}

			motor.right.speed -= (wrt_code_motor_speed_hercules_increment >> 2);
			// --------------------------------------------------------
			if( motor.right.speed < static_cast<int>(wrt_code_motor_speed_hercules_min) )
			{
				motor.right.speed =  wrt_code_motor_speed_hercules_min;
			}
		}	break;
		// -------------------------------------------------------[ spin left ]
		case wrt_rc_cmd_spin_left:
		{
			if( false == motor.isResetDirection )
			{	motor.right.speed += wrt_code_motor_speed_hercules_rotate;
				motor.left.speed  += wrt_code_motor_speed_hercules_rotate;
			}
			// ---
			motor.isResetDirection = false;

			// --- from preserved command history
			if( 0 == (rc.commandDirection & static_cast<uint32_t>(wrt_rc_cmd_spin_left) ) )
			{
				motor.isShiftDirection = true;		// stop first
				motor.isResetDirection = true;

				// --- set nominal forward speed
				motor.right.speed =  wrt_code_motor_speed_hercules_rotate;
				motor.left.speed  =  wrt_code_motor_speed_hercules_rotate;
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
			if( false == motor.isResetDirection )
			{
				motor.right.speed += wrt_code_motor_speed_hercules_rotate;
				motor.left.speed  += wrt_code_motor_speed_hercules_rotate;
			}
			// ---
			motor.isResetDirection = false;

			// --- from preserved command history
			if( 0 == (rc.commandDirection & static_cast<uint32_t>(wrt_rc_cmd_spin_right) ) )
			{
				motor.isShiftDirection = true;		// stop first
				motor.isResetDirection = true;

				// --- set nominal forward speed
				motor.right.speed = wrt_code_motor_speed_hercules_rotate;
				motor.left.speed  = wrt_code_motor_speed_hercules_rotate;
			}

			// --- reset direction command history
			rc.commandDirection = static_cast<uint32_t>(wrt_rc_cmd_spin_right);
			// ---
			motor.left.control  |= wrt_motorControl_direction_Forward;
			motor.right.control &= wrt_motorControl_direction_Reverse;


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

}	// done ircDrive()



} /* namespace RT_HERCULES_BEHAVIOR_EXECUTIVE */
