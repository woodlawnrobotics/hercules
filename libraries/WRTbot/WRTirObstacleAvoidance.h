/*
 * WRTirObstacleAvoidance.h
 *
 *  Created on: Nov 19, 2013
 *      Author: simon
 */

#ifndef WRTIROBSTACLEAVOIDANCE_H_
#define WRTIROBSTACLEAVOIDANCE_H_

#include "WRTbot_inc.h"

namespace WRT_ARDUINO_IR_AVOIDANCE
{

class WRTirObstacleAvoidance
{
public:
	WRTirObstacleAvoidance();
	~WRTirObstacleAvoidance();

	void init( void );
	void irSend38KHzPulse( const uint16_t );

private:

	void irSend38KHzPulse( void );

	// --- delay these many msec
	int 		nPulses;

	uint16_t	state;

};

} /* namespace WRT_ARDUINO_IR_AVOIDANCE */

#endif /* WRT_ARDUINO_IR_AVOIDANCE */
