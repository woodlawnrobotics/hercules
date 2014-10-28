/*
 * WRTirObstacleAvoidance.cpp
 *
 *  Created on: Nov 19, 2013
 *      Author: simon
 */


#include "Arduino.h"

#include <inttypes.h>

#include "WRTirObstacleAvoidance.h"

namespace WRT_ARDUINO_IR_AVOIDANCE
{

WRTirObstacleAvoidance::WRTirObstacleAvoidance()
{
	init();
}

WRTirObstacleAvoidance::~WRTirObstacleAvoidance()
{
	// TODO Auto-generated destructor stub
}



void WRTirObstacleAvoidance::init( void )
{
	this->state = static_cast<uint16_t>(wrt_ir_state_idle);

}	// done init()

// -------------------------------------------------------[ irSend38KHzPulse ]
void WRTirObstacleAvoidance::irSend38KHzPulse( const uint16_t usrState )
{
	const uint16_t currentState = this->state;

	this->state = usrState;

	WRTirObstacleAvoidance::irSend38KHzPulse();

	this->state = currentState;

}	// done irSend38KHzPulse()

// -------------------------------------------------------[ irSend38KHzPulse ]
void WRTirObstacleAvoidance::irSend38KHzPulse( void )
{
	nPulses = static_cast<int>(wrt_bot_obstacleAvoidance_nPulses);

	switch( state )
	{	default:
		case wrt_ir_state_idle:
		{
			;
		}	break;
		case wrt_ir_state_irPulseLeft:
		{
			// --- left IR pulse
			// -------------------------------------------------------------
			for( int ii = 0; ii < nPulses; ii++)
			{
				// set the infrared to HIGH level
				digitalWrite( wrt_pin_ir_transmitter_left, HIGH);
				delayMicroseconds(10);		// delay 10 microsecond

				// set the infrared to LOW level
				digitalWrite( wrt_pin_ir_transmitter_left, LOW);
				delayMicroseconds(10);		// delay 10 microsecond
			}
		}	break;
		case wrt_ir_state_irPulseRight:
		{
			// --- right IR pulse
			// -------------------------------------------------------------
			for( int ii = 0; ii < nPulses; ii++)
			{
				// set the infrared to HIGH level
				digitalWrite( wrt_pin_ir_transmitter_right, HIGH);
				delayMicroseconds(10);		// delay 10 microsecond

				// set the infrared to LOW level
				digitalWrite( wrt_pin_ir_transmitter_right, LOW);
				delayMicroseconds(10);		// delay 10 microsecond
			}
		}	break;
	}

}	// done irSend38KHzPulse()

} /* namespace WRT_ARDUINO_IR_AVOIDANCE */
