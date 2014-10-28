/*
 * WRTbotDrive.h
 *
 *  Created on: Jul 23, 2013
 *      Author: simon
 */
// ---------------------------------------------------------------------[ BoF ]
#ifndef WRTBOTDRIVE_H_
#define WRTBOTDRIVE_H_

#if defined(ARDUINO) && (ARDUINO >= 100)
#include "Arduino.h"
#endif


#include "WRTbot_inc.h"

namespace WRT_ARDUINO_BOT_DRIVE
{

class WRTbotDrive
{
public:
	WRTbotDrive();
	~WRTbotDrive();

	void init( void );

	void set(      wrt_ir_rc_t &, wrt_motor_t & );
	void ircDrive( wrt_ir_rc_t &, wrt_motor_t & );
	bool driveBehavior( const wrt_ir_rc_t &, wrt_motor_t  & );
	void stop( void );

	void drive( const uint16_t , wrt_motor_t & );
	void drive( const   wrt_motor_t &  );

	inline void resetNominalSpeed( void )
	{
		useNominal = false;
	}

private:

	void driveReverse(   wrt_motor_t &  );
	void driveForward(   wrt_motor_t &  );
	void turnRightFast(  wrt_motor_t &  );
	void turnRight( 	 wrt_motor_t &  );
	void driveRight(     wrt_motor_t &  );
	void driveLeft( 	 wrt_motor_t &  );
	void turnLeftFast(   wrt_motor_t &  );
	void turnLeft(  	 wrt_motor_t &  );
	// ---
	void turnLeftDiferential(  uint16_t, wrt_motor_t &  );
	void turnRightDiferential( uint16_t, wrt_motor_t &  );

	void setBehavior(  wrt_ir_rc_t &, wrt_motor_t & );
	void driveByLight( wrt_motor_t & );

	wrt_robotSensors_t		sensor;

	// --- other
	bool useNominal;	// context sensitive, reset to false after use

	// ---
	int  driveByLightFactor;
};

} /* namespace WRT_ARDUINO_BOT_DRIVE */
// ---------------------------------------------------------------------[ EoF ]
#endif /* WRTBOTDRIVE_H_ */
