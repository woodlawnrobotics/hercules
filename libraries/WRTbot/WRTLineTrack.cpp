/*
 * WRTLineTrack.cpp
 *
 *  Created on: Mar 17, 2014
 *      Author: simon
 */
// ---------------------------------------------------------------------[ BoF ]

#include <WRTLineTrack.h>

namespace WRT_LINE_TRACK
{

const float  WRTLineTrack::dOffSet = static_cast<float>( wrt_analog_referenceVolts);
const double WRTLineTrack::dFactor = static_cast<double>(wrt_analog_referenceVolts)/1024.0;
// -------------------------------------------------------------[ constructor ]
WRTLineTrack::WRTLineTrack()
{
	init();

}

// --------------------------------------------------------------[ destructor ]
WRTLineTrack::~WRTLineTrack()
{
	driveState = static_cast<uint16_t>(wrt_lineTrack_state_forwardDrive);

}

// --------------------------------------------------------------------[ init ]
void WRTLineTrack::init( void )
{
	driveCommand = 0;
	driveState = static_cast<uint16_t>(wrt_lineTrack_state_forwardDrive);

	for( int ii = 0; ii < static_cast<int>(wrt_pin_nLineTrackSensors); ii++ )
	{
		data[ ii] = 0.0;
	}

	delta[ 0 ] = 0.0;
	delta[ 1 ] = 0.0;
	delta[ 2 ] = 0.0;
	delta[ 3 ] = 0.0;
	delta[ 4 ] = 0.0;

	gainLon[0] =  0.0;	 	gainLat[0] = 0.0;
	gainLon[1] =  5.0;	 	gainLat[1] = 0.0;
	gainLon[2] = 10.0;	 	gainLat[2] = 0.0;
	gainLon[3] =  5.0;	 	gainLat[3] = 0.0;
	gainLon[4] =  0.0;	 	gainLat[4] = 0.0;


}	// done init()

// ----------------------------------------------------[ setLongitudinalGains ]
void WRTLineTrack::setLongitudinalGains( const float usrLonGain[wrt_pin_nLineTrackSensors] )
{
	gainLon[0] = usrLonGain[0];
	gainLon[1] = usrLonGain[1];
	gainLon[2] = usrLonGain[2];
	gainLon[3] = usrLonGain[3];
	gainLon[4] = usrLonGain[4];

}	// done setLongitudinalGains()

// ------------------------------------------------------------[ setSideGains ]
void WRTLineTrack::setLateralGains( const float usrLatGain[wrt_pin_nLineTrackSensors] )
{
	gainLat[0] = usrLatGain[0];
	gainLat[1] = usrLatGain[1];
	gainLat[2] = usrLatGain[2];
	gainLat[3] = usrLatGain[3];
	gainLat[4] = usrLatGain[4];

}	// done setSideGains()

// ------------------------------------------------[ differentialForwardDrive ]
void WRTLineTrack::forwardDrive( wrt_motor_t &motor )
{
	motor.left.control  |= wrt_motorControl_direction_Forward;
	motor.right.control |= wrt_motorControl_direction_Forward;
	// ---
	motor.right.speed = wrt_code_motor_speed_low;
	motor.left.speed  = wrt_code_motor_speed_low;

	if( (cmdLat > 10) || (cmdLat < 10) )
	{
		motor.right.speed -= (cmdLat >> 1);
		motor.left.speed  += (cmdLat >> 1);
	}

}	// done differentialForwardDrive()

// ------------------------------------------------[ differentialForwardDrive ]
void WRTLineTrack::differentialForwardDrive( wrt_motor_t &motor )
{
	motor.left.control  |= wrt_motorControl_direction_Forward;
	motor.right.control |= wrt_motorControl_direction_Forward;
	// ---
	motor.right.speed = wrt_code_motor_speed_rotate;
	motor.left.speed  = wrt_code_motor_speed_rotate;
	// ---
	if( cmdLat < 0 )
	{
		motor.right.control |= wrt_motorControl_direction_Forward;
		motor.left.control  &= wrt_motorControl_direction_Reverse;

		motor.right.speed += ( (cmdLon << 1) ) - cmdLat;
		motor.left.speed  += (-(cmdLon << 1) ) + cmdLat;
	}
	else
	{
		motor.right.control &= wrt_motorControl_direction_Reverse;
		motor.left.control  |= wrt_motorControl_direction_Forward;

		motor.right.speed += (-(cmdLon << 1) ) - cmdLat;
		motor.left.speed  += ( (cmdLon << 1) ) + cmdLat;
	}

//	motor.right.speed += (cmdLon >> 1) - cmdLat;
//	motor.left.speed  += (cmdLon >> 1) + cmdLat;

}	// done differentialForwardDrive()

// -----------------------------------------------------[ differentialRotation ]
void WRTLineTrack::differentialRotation( wrt_motor_t &motor )
{
	if( cmdLat < 0 )
	{
		motor.left.control  &= wrt_motorControl_direction_Reverse;
		motor.right.control |= wrt_motorControl_direction_Forward;
	}
	else
	{	motor.left.control  |= wrt_motorControl_direction_Forward;
		motor.right.control &= wrt_motorControl_direction_Reverse;
	}
	// ---
	motor.right.speed = wrt_code_motor_speed_rotate;
	motor.left.speed  = wrt_code_motor_speed_rotate;

	motor.right.speed -= cmdLat;
	motor.left.speed  += cmdLat;

}	// done differentialRotation()

// -----------------------------------------------------[ differentialRotation ]
void WRTLineTrack::differentialRotationFast( wrt_motor_t &motor )
{
	if( cmdLat < 0 )
	{
		motor.left.control  &= wrt_motorControl_direction_Reverse;
		motor.right.control |= wrt_motorControl_direction_Forward;
	}
	else
	{	motor.left.control  |= wrt_motorControl_direction_Forward;
		motor.right.control &= wrt_motorControl_direction_Reverse;
	}
	// ---
	motor.right.speed = wrt_code_motor_speed_nominal;
	motor.left.speed  = wrt_code_motor_speed_nominal;

}	// done differentialRotation()

// ---------------------------------------------------------------[ nextStep ]
void WRTLineTrack::nextStep( wrt_motor_t &motor )
{
	// --- motor ON by default
	// -----------------------
	motor.left.control  |= wrt_motorControl_power_ON;
	motor.right.control |= wrt_motorControl_power_ON;

	// --- read sensors
	// ------------------------------------------
	readSensorValues();

	switch( driveState )
	{	default:
		case wrt_lineTrack_state_forwardDrive:
		{	if( cmdLon < 0 )
			{
				driveState = static_cast<uint16_t>(wrt_lineTrack_state_differentialRotation);
				differentialRotation( motor );
			}
			else if( cmdLon < 150 )
			{
				driveState = static_cast<uint16_t>(wrt_lineTrack_state_differentialForwardDrive);
				differentialForwardDrive( motor );
			}
			else
			{
				forwardDrive( motor );
			}
		}	break;
		case wrt_lineTrack_state_differentialForwardDrive:
		{	if( cmdLon < 10 )
			{
				driveState = static_cast<uint16_t>(wrt_lineTrack_state_differentialRotation);
				differentialRotation( motor );
			}
			else if( cmdLon > 170 )
			{
				driveState = static_cast<uint16_t>(wrt_lineTrack_state_forwardDrive);
				forwardDrive( motor );
			}
			else
			{
				differentialForwardDrive( motor );
			}
		}	break;
		case wrt_lineTrack_state_differentialRotation:
		{
			if( cmdLon > 40 )
			{
				driveState = static_cast<uint16_t>(wrt_lineTrack_state_differentialForwardDrive);
				differentialForwardDrive( motor );
			}
			else if( cmdLon < -60 )
			{
				differentialRotationFast( motor );
			}
			else
			{
				differentialRotation( motor );
			}
		}	break;
	}	// switch


}	// done nextStep()

// ---------------------------------------------------------------[ nextStepX ]
void WRTLineTrack::nextStepX( wrt_motor_t &motor )
{
	// --- motor ON by default
	// -----------------------
	motor.left.control  |= wrt_motorControl_power_ON;
	motor.right.control |= wrt_motorControl_power_ON;

	// --- read sensors
	// ------------------------------------------
	readSensorValues();

	switch( driveCommand )
	{	default:
		case wrt_lineTrack_state_forwardDrive:
		case wrt_lineTrack_sensorActive_ir2:
		{	if( cmdLon < 70 )
			{
				driveCommand = static_cast<uint16_t>(wrt_lineTrack_state_differentialRotation);
				differentialRotation( motor );
			}
			else
			{
				forwardDrive( motor );
			}
		}	break;
		case wrt_lineTrack_state_differentialForwardDrive:
		case wrt_lineTrack_sensorActive_ir1:
		case wrt_lineTrack_sensorActive_ir3:
		case wrt_lineTrack_sensorActiveX_ir12:
		case wrt_lineTrack_sensorActiveX_ir23:
		{	if( cmdLon < 0 )
			{
				driveCommand = static_cast<uint16_t>(wrt_lineTrack_state_differentialRotation);
				differentialRotation( motor );
			}
			else if( cmdLon > 100 )
			{
				driveCommand = static_cast<uint16_t>(wrt_lineTrack_state_forwardDrive);
				forwardDrive( motor );
			}
			else
			{
				differentialForwardDrive( motor );
			}
		}	break;
		case wrt_lineTrack_state_differentialRotation:
		case wrt_lineTrack_sensorActive_ir0:
		case wrt_lineTrack_sensorActive_ir4:
		case wrt_lineTrack_sensorActiveX_ir01:
		case wrt_lineTrack_sensorActiveX_ir34:
		case wrt_lineTrack_sensorActiveX_ir123:
		{
			if( cmdLon > 80 )
			{
				driveCommand = static_cast<uint16_t>(wrt_lineTrack_state_differentialForwardDrive);
				forwardDrive( motor );
			}
			else if( cmdLon < -60 )
			{
				differentialRotationFast( motor );
			}
			else
			{
				differentialRotation( motor );
			}
		}	break;
	}	// switch


}	// done nextStepX()

// --------------------------------------------------------[ readSensorValues ]
void WRTLineTrack::readSensorValues( void )
{
	int analogValue = 0;

	driveCommand = 0;

	// --- read analog port voltage for line tracking sensors
	// ----------------------------------------------------------------
	analogValue = analogRead( wrt_pin_lineTrack_0 );
	data[0]   = WRTLineTrack::dOffSet - (static_cast<float>(analogValue) * WRTLineTrack::dFactor);
	// ---
	if( (delta[0] < data[0]) && (1.0 < data[0]) )
	{
		delta[0] = data[0];
		// ---
		gainLon[0] =  -50.0/delta[0];
		gainLat[0] = -100.0/delta[0];
	}
	// ---
	if( data[0] > 1.0 )
	{
		driveCommand |= wrt_lineTrack_sensorActive_ir0;
	}
	// ---
	cmdLon 	  = static_cast<int>(data[0] * gainLon[0]);
	cmdLat    = static_cast<int>(data[0] * gainLat[0]);

	analogValue = analogRead( wrt_pin_lineTrack_1 );
	data[1] = WRTLineTrack::dOffSet - (static_cast<float>(analogValue) * WRTLineTrack::dFactor);
	// ---
	if( (delta[1] < data[1]) && (1.0 < data[1]) )
	{
		delta[1] = data[1];
		// ---
		gainLon[1] =   25.0/delta[1];
		gainLat[1] =  -50.0/delta[1];
	}
	// ---
	if( data[1] > 1.0 )
	{
		driveCommand |= wrt_lineTrack_sensorActive_ir1;
	}
	// ---
	cmdLon    += static_cast<int>(data[1] * gainLon[1]);
	cmdLat    += static_cast<int>(data[1] * gainLat[1]);

	analogValue = analogRead( wrt_pin_lineTrack_2 );
	data[2] = WRTLineTrack::dOffSet - (static_cast<float>(analogValue) * WRTLineTrack::dFactor);
	// ---
	if( (delta[2] < data[2]) && (1.0 < data[2]) )
	{
		delta[2] = data[2];
		// ---
		gainLon[2] = 170.0/delta[2];
		gainLat[2] =   0;
	}
	// ---
	if( data[2] > 1.0 )
	{
		driveCommand |= wrt_lineTrack_sensorActive_ir2;
	}
	// ---
	cmdLon 	 += static_cast<int>(data[2] * gainLon[2]);

	analogValue = analogRead( wrt_pin_lineTrack_3 );
	data[3]    = WRTLineTrack::dOffSet - (static_cast<float>(analogValue) * WRTLineTrack::dFactor);
	// ---
	if( (delta[3] < data[3]) && (1.0 < data[3]) )
	{
		delta[3] = data[3];
		// ---
		gainLon[3] =   25.0/delta[3];
		gainLat[3] =   50.0/delta[3];
	}
	// ---
	if( data[3] > 1.0 )
	{
		driveCommand |= wrt_lineTrack_sensorActive_ir3;
	}
	// ---
	cmdLon 	  += static_cast<int>(data[3] * gainLon[3]);
	cmdLat    += static_cast<int>(data[3] * gainLat[3]);

	analogValue = analogRead( wrt_pin_lineTrack_4 );
	data[4]    = WRTLineTrack::dOffSet - (static_cast<float>(analogValue) * WRTLineTrack::dFactor);
	// ---
	if( (delta[4] < data[4]) && (1.0 < data[4]) )
	{
		delta[4] = data[4];
		// ---
		gainLon[4] =  -50.0/delta[4];
		gainLat[4] =  100.0/delta[4];
	}
	// ---
	if( data[3] > 1.0 )
	{
		driveCommand |= wrt_lineTrack_sensorActive_ir4;
	}
	// ---
	cmdLon 	  += static_cast<int>(data[4] * gainLon[4]);
	cmdLat    += static_cast<int>(data[4] * gainLat[4]);

}	// done readSensorValues()

// -------------------------------------------------------[ printSensorValues ]
void WRTLineTrack::printSensorValues( void )
{
	//Serial.print( "data/value: ");

	for( int ii = 0; ii < static_cast<int>(wrt_pin_nLineTrackSensors); ii++ )
	{
		Serial.print( data[ii] );
		Serial.print( " ");
		Serial.print( gainLon[ii] );
		Serial.print( " ");
		Serial.print( gainLat[ii] );
		Serial.print( " ");
	}

	Serial.print( cmdLon );
	Serial.print( " ");
	Serial.print( cmdLat );

	Serial.println( " ");

}	// done printSensorValues()


// ---------------------------------------------------------------------[ EoF ]
} /* namespace WRT_LINE_TRACK */
