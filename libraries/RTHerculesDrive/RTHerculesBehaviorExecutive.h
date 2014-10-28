/*
 * RTHerculesBehaviorExecutive.h
 *
 *  Created on: Oct 22, 2014
 *      Author: simon
 */

#ifndef RTHERCULESBEHAVIOREXECUTIVE_H_
#define RTHERCULESBEHAVIOREXECUTIVE_H_

#if defined(ARDUINO) && (ARDUINO >= 100)
#include "Arduino.h"
#endif

#include "WRTbot_inc.h"


namespace RT_HERCULES_BEHAVIOR_EXECUTIVE
{

class RTHerculesBehaviorExecutive
{
public:
	RTHerculesBehaviorExecutive();
	~RTHerculesBehaviorExecutive();


	void init( void );

	void ircGetDriveBehavior( wrt_ir_rc_t &, wrt_motor_t & );


private:

	// --- other
	bool useNominal;	// context sensitive, reset to false after use

};

} /* namespace RT_HERCULES_BEHAVIOR_EXECUTIVE */

#endif /* RTHERCULESBEHAVIOREXECUTIVE_H_ */
