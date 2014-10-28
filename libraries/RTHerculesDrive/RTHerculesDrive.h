/*
 * RTHerculesDrive.h
 *
 *  Created on: Oct 21, 2014
 *      Author: simon
 */

#ifndef RTHERCULESDRIVE_H_
#define RTHERCULESDRIVE_H_

#include <RTarduino_inc.h>
#include <WRTbot_inc.h>
#include "WRTirReceiver.h"
#include "RTHerculesCommandIR.h"
#include "RTHerculesBehaviorExecutive.h"
#include <RTHerculesRobot.h>


namespace RT_HERCULES_DRIVE
{

class RTHerculesDrive
{
public:
	RTHerculesDrive();
	~RTHerculesDrive();

	void init( const uint8_t  );

	bool stepIRC( const bool );
	bool readIRC( rt_ioParam_t & );


	void stepObstacleAvoidance( const uint16_t );
	void drive( const uint16_t );

	void driveForward( void );
	void driveReverse( void );

	void turnLeftDiferential(  uint16_t );
	void turnRightDiferential( uint16_t );

	void turnRightFast( void  );
	void turnRight( void  );

	void turnLeftFast( void );
	void turnLeft( void );

	void driveLeft(  void  );
	void driveRight( void  );

	void drive( void );
	void stop(  void );

	inline uint32_t getCurrentBehavior( void )
	{
		return( ir.rc.behavior );
	}

    // --- electrical motor operation
    // ----------------------------------------------------------------
    inline void setReverseMotorRight( const bool isReversed )
    {
    	herc.setReverseMotorRight( isReversed );
    }
    // ----------------------------------------------------------------
    inline void setReverseMotorLeft( const bool isReversed )
    {
    	herc.setReverseMotorLeft( isReversed );
    }
    // ----------------------------------------------------------------

private:

	// --- hercules robot
	// -----------------------------------------------------------------------
    RT_HERCULES_ROBOT::RTHerculesRobot herc;


	// --- generic motor
	// -----------------------------------------------------------------------
    wrt_motor_t motor;

	// --- infrared database
	// -----------------------------------------------------------------------
    wrt_ir_t	ir;

	// --- IR receiver
	// -----------------------------------------------------------------------
    RT_HERCULES_IR_RECEIVER::RTHerculesCommandIR	*irRx;

	// --- behavior manager
	// -----------------------------------------------------------------------
   RT_HERCULES_BEHAVIOR_EXECUTIVE::RTHerculesBehaviorExecutive ceo;

	// --- other
	bool isFirstPass;
	bool isGoodKeyIR;
	int  verbose;
	bool useNominal;	// context sensitive, reset to false after use

};

} /* namespace RT_HERCULES_DRIVE */

#endif /* RTHERCULESDRIVE_H_ */
