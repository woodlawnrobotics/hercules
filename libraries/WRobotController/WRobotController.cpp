/**
 * File: WRobotController.cpp
 * Description:
 * WRobotController library is an easy to use robot controller for Arduino.
 *
 * Author: Miguel Simon
 * Contact: simon  at  ou punto edu
 * Copyright: Woodlawn School, Moresville, NC
 * Copying permission statement:
    This file is part of WRobotController.

    WRobotController is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
/*
 * WRobotController.cpp
 *
 *  Created on: May 5, 2013
 *      Author: simon
 */

#include "Arduino.h"
#include "WRobotController.h"

const short WRobotController::infraRedSensorThreshold = 200;
// --------------------------------------------------------------------------
WRobotController::WRobotController( void )
{
	memset( (char *) &iPad, 0, sizeof( struct WR_Pad_tag) );
	// ---
	taskState = wr_taskState_iddle;

	iPad.irState 		= wr_ir_stop;
	iPad.dRobotCommand  = wr_command_stop;
	iPad.dRobotMode 	= wr_mode_none;

	WRobotController::verbose = 0;

}	// done constructor

WRobotController::~WRobotController( void )
{
	memset( (char *) &iPad, 0, sizeof( struct WR_Pad_tag) );

}	// done destructor

// --------------------------------------------------------------------[ init ]
int WRobotController::init( const wr_pad_t &somePad )
{

	WRobotController::taskState = wr_taskState_step1;
	// ----------------------------------------------
	WRobotController::iPad.pinMotorLeft  = somePad.pinMotorLeft;
	WRobotController::iPad.pinMotorRight = somePad.pinMotorRight;
	// ---
	WRobotController::iPad.irLeftChannel  = somePad.irLeftChannel;
	WRobotController::iPad.irRightChannel = somePad.irRightChannel;

	// initialize the digital pin as an output.
	// ----------------------------------------------
//	pinMode( iPad.pinMotorLeft,   OUTPUT);
//	pinMode( iPad.pinMotorRight,  OUTPUT);

	return( 1 );

}	// done init()
// ----------------------------------------------------------------[ getState ]
int WRobotController::getState( wr_pad_t &userPad )
{
	readInfraRedSensors();
	getRobotCommand();
	// --------------------------------------------
	userPad.irState 	  	= iPad.irState;
	userPad.irLeftVal  		= iPad.irLeftVal;
	userPad.irRightVal 		= iPad.irRightVal;
	// --------------------------------------------
	userPad.dRobotCommand 	= iPad.dRobotCommand;
	userPad.dRobotMode 	  	= iPad.dRobotMode;
	// --------------------------------------------

	return( 1 );

}	// done getState()

// -------------------------------------------------------------[ stepBehavior ]
int WRobotController::stepBehavior( const wr_mode_t dRobotMode )
{
	int rVal = 1;

	// --- note: command here is based on IR sensor value

	iPad.dRobotMode = dRobotMode;
	// ----------------------------------------------------------------------
	if( wr_command_foward == iPad.dRobotCommand )
	{
		rVal = moveForward( true );

		if( WRobotController::verbose > 0 )
		{
			Serial.println("stepBehavior: ir command: forward");
		}
	}
	else if ( wr_command_stop == iPad.dRobotCommand )
	{
		rVal = moveForward( false );	// --- stop

		if( WRobotController::verbose > 0 )
		{
			Serial.println("stepBehavior: ir command: stop");
		}
	}
	else if( wr_command_left == iPad.dRobotCommand )
	{
	    switch( dRobotMode )
	    {  case wr_mode_predator:
	       {
	    	   rVal  = moveToLeft();	// --- hunt the pray
	    	   rVal += 10;

	       }  break;
	       case wr_mode_pray:
	       {
	    	   rVal  = moveToRight();	// --- avoid the predator
	    	   rVal += 20;

				if( WRobotController::verbose > 0 )
				{
					Serial.println("stepBehavior: ir command: to right, mode: pray");
				}
	       }  break;
	       default:
	       {
	       }  break;
	    }  // switch
	}
	else if( wr_command_right == iPad.dRobotCommand )
	{
	    switch( dRobotMode )
	    {  case wr_mode_predator:
	       {
	    	   rVal  = moveToRight();	// --- hunt the pray
	    	   rVal += 30;

	       }  break;
	       case wr_mode_pray:
	       {
	    	   rVal  = moveToLeft();	// --- avoid the predator
	    	   rVal += 40;

				if( WRobotController::verbose > 0 )
				{
					Serial.println("stepBehavior: ir command: to left, mode: pray");
				}
	       }  break;
	       default:
	       {
	       }  break;
	    }  // switch
	}
	else
	{
		rVal = -1;
	}

	return( rVal );

}	// done stepBehavior()

// -------------------------------------------------------------[ moveForward ]
int WRobotController::moveForward( bool isForward )
{
	int rVal = 0;

	if( true == isForward )
	{
		switch( taskState )
		{	case wr_taskState_step1:
			{
				digitalWrite( iPad.pinMotorLeft,  HIGH);
				digitalWrite( iPad.pinMotorRight, HIGH);
				// ---
				taskState = wr_taskState_step2;
			}	break;
			case wr_taskState_step2:
			{
				digitalWrite( iPad.pinMotorLeft,  HIGH);
				digitalWrite( iPad.pinMotorRight, HIGH);
				// ---
				taskState = wr_taskState_step3;
			}	break;
			case wr_taskState_step3:
			{
				digitalWrite( iPad.pinMotorLeft,  HIGH);
				digitalWrite( iPad.pinMotorRight, HIGH);
				// ---
				taskState = wr_taskState_step4;
			}	break;
			case wr_taskState_step4:
			{
				digitalWrite( iPad.pinMotorLeft,  LOW);
				digitalWrite( iPad.pinMotorRight, LOW);
				// ---
				taskState = wr_taskState_step5;
			}	break;
			case wr_taskState_step5:
			{
				digitalWrite( iPad.pinMotorLeft,  LOW);
				digitalWrite( iPad.pinMotorRight, LOW);
				// ---
				taskState = wr_taskState_step1;
			}	break;
			// ---------------------------------------------------------
			default:
			case wr_taskState_iddle:
			{
				digitalWrite( iPad.pinMotorLeft,  LOW);
				digitalWrite( iPad.pinMotorRight, LOW);
				// ---
				taskState = wr_taskState_step1;
			}	break;
		}	// switch
		// ---
		rVal = 1;
	}
	else
	{	taskState = wr_taskState_step1;

		// --- stop
		digitalWrite( iPad.pinMotorLeft,  LOW);
		digitalWrite( iPad.pinMotorRight, LOW);
	}

	return( rVal );

}	// done moveForward()

// -------------------------------------------------------------[ moveToRight ]
int WRobotController::moveToRight( void )
{
	//    digitalWrite( iPad.pinMotorLeft,  HIGH);
	//    digitalWrite( iPad.pinMotorRight, LOW);

	switch( taskState )
	{	case wr_taskState_step1:
		{
			digitalWrite( iPad.pinMotorLeft,  HIGH);
			digitalWrite( iPad.pinMotorRight, LOW);
			// ---
			taskState = wr_taskState_step2;
		}	break;
		case wr_taskState_step2:
		{
			digitalWrite( iPad.pinMotorLeft,  HIGH);
			digitalWrite( iPad.pinMotorRight, HIGH);
			// ---
			taskState = wr_taskState_step3;
		}	break;
		case wr_taskState_step3:
		{
			digitalWrite( iPad.pinMotorLeft,  HIGH);
			digitalWrite( iPad.pinMotorRight, LOW);
			// ---
			taskState = wr_taskState_step4;
		}	break;
		case wr_taskState_step4:
		{
			digitalWrite( iPad.pinMotorLeft,  HIGH);
			digitalWrite( iPad.pinMotorRight, LOW);
			// ---
			taskState = wr_taskState_step5;
		}	break;
		case wr_taskState_step5:
		{
			digitalWrite( iPad.pinMotorLeft,  HIGH);
			digitalWrite( iPad.pinMotorRight, HIGH);
			// ---
			taskState = wr_taskState_step1;
		}	break;
		// ---------------------------------------------------------
		default:
		case wr_taskState_iddle:
		{
			digitalWrite( iPad.pinMotorLeft,  LOW);
			digitalWrite( iPad.pinMotorRight, LOW);
			// ---
			taskState = wr_taskState_step1;
		}	break;
	}	// switch

	return( 3 );

}	// done moveRight()

// --------------------------------------------------------------[ moveToLeft ]
int WRobotController::moveToLeft( void )
{
	//    digitalWrite( iPad.pinMotorLeft,  LOW);
	//    digitalWrite( iPad.pinMotorRight, HIGH);

	switch( taskState )
	{	case wr_taskState_step1:
		{
			digitalWrite( iPad.pinMotorLeft,  LOW);
			digitalWrite( iPad.pinMotorRight, HIGH);
			// ---
			taskState = wr_taskState_step2;
		}	break;
		case wr_taskState_step2:
		{
			digitalWrite( iPad.pinMotorLeft,  HIGH);
			digitalWrite( iPad.pinMotorRight, HIGH);
			// ---
			taskState = wr_taskState_step3;
		}	break;
		case wr_taskState_step3:
		{
			digitalWrite( iPad.pinMotorLeft,  HIGH);
			digitalWrite( iPad.pinMotorRight, HIGH);
			// ---
			taskState = wr_taskState_step4;
		}	break;
		case wr_taskState_step4:
		{
			digitalWrite( iPad.pinMotorLeft,  LOW);
			digitalWrite( iPad.pinMotorRight, HIGH);
			// ---
			taskState = wr_taskState_step5;
		}	break;
		case wr_taskState_step5:
		{
			digitalWrite( iPad.pinMotorLeft,  HIGH);
			digitalWrite( iPad.pinMotorRight, HIGH);
			// ---
			taskState = wr_taskState_step1;
		}	break;
		// ---------------------------------------------------------
		default:
		case wr_taskState_iddle:
		{
			digitalWrite( iPad.pinMotorLeft,  LOW);
			digitalWrite( iPad.pinMotorRight, LOW);
			// ---
			taskState = wr_taskState_step1;
		}	break;
	}	// switch

	return( 9 );

}	// done moveToLeft()

// ---------------------------------------------------------[ getRobotCommand ]
int WRobotController::getRobotCommand( void )
{
	switch( iPad.irState )
	{   default:
		case wr_ir_foward:
		{
			iPad.dRobotCommand = wr_command_foward;
		}  	break;
		case wr_ir_left:
		{
			iPad.dRobotCommand = wr_command_left;
		}  	break;
		case wr_ir_right:
		{
			iPad.dRobotCommand = wr_command_right;
		}  	break;
		case wr_ir_stop:
		{
			iPad.dRobotCommand = wr_command_stop;
		}  	break;
	}

	return( 1 );

}	// done getRobotCommand()

// -----------------------------------------------------[ readInfraRedSensors ]
int WRobotController::readInfraRedSensors( void )
{
	// reads the value of the sharp sensor
	// --------------------------------------------------
	iPad.irLeftVal  = analogRead(WRobotController::iPad.irLeftChannel  );
	iPad.irRightVal = analogRead(WRobotController::iPad.irRightChannel );

	// --------------------------------------------------
	if( (WRobotController::infraRedSensorThreshold < iPad.irLeftVal ) &&
	    (WRobotController::infraRedSensorThreshold < iPad.irRightVal)  )
	{
		iPad.irState = wr_ir_stop;
	}
	else if( WRobotController::infraRedSensorThreshold < iPad.irLeftVal )
	{
		iPad.irState = wr_ir_left;
	}
	else if( WRobotController::infraRedSensorThreshold < iPad.irRightVal )
	{
		iPad.irState = wr_ir_right;
	}
	else
	{
		iPad.irState = wr_ir_foward;
	}


	return( iPad.irState );

}	// done readInfraRedSensors()
