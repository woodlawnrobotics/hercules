// *
// * RTirSenRcv.h
// *
// *  Created on: May 25, 2014
// *      Author: simon
// *
// ---------------------------------------------------------------------------
// --- original work:
// ---------------------------------------------------------------------------
// * IRremote
// * Version 0.1 July, 2009
// * Copyright 2009 Ken Shirriff
// * For details,
// 		see http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.htm
//			http://arcfn.com
// * Edited by Mitra to add new controller SANYO
// *
// * Interrupt code based on NECIRrcv by Joe Knapp
// * 	http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1210243556
// * Also influenced by
//		http://zovirl.com/2008/11/12/building-a-universal-remote-with-an-arduino/
// *
// * JVC and Panasonic protocol added by Kristian Lauszus
// * (Thanks to zenwheel and other people at the original blog post)
// *
// ---------------------------------------------------------------------------

#ifndef RTIRSENRCV_H_
#define RTIRSENRCV_H_
// ---------------------------------------------------------------------[ BoF ]
#include "RTsys_def.h"
// ------------------------
#include "IRremote_inc.h"


namespace RT_INFRARED_SENSOR
{

class RTirSenRcv
{
public:
	RTirSenRcv();
	~RTirSenRcv();

	void init( int  );

	void service( const uint8_t );
	int  decode(  rt_irSensor_decodeResults_t &);

	void enableIRIn( void );
	void resume( 	 void );

	void enableBlink13( bool );


	inline bool match( const int measured, const int desired )
	{	return( (measured >= ticsLow( desired))  &&
				(measured <= ticsHigh(desired))  );
	}

	inline int matchMark(int measured_ticks, int desired_us)
	{	return match(measured_ticks, (desired_us + irCDat.markExcess));
	}

	inline int matchSpace(int measured_ticks, int desired_us)
	{	return match(measured_ticks, (desired_us - irCDat.markExcess));
	}


private:

	// These are called by decode
	int getRClevel( rt_irSensor_decodeResults_t &, int *, int *, const int );
	long decodeNEC( rt_irSensor_decodeResults_t &);


	long decodeHash( rt_irSensor_decodeResults_t &);
	int compare(unsigned int, unsigned int );

	inline int ticsLow( const int meaMicroSec )
	{
		// TICKS_LOW(us) (int) (((us)*LTOL/rt_irSensor_usecPerTick))
		// --------------------------------------------------------------------
		irParams.ticksLow = static_cast<int>( meaMicroSec * irCDat.tolMin );

		return( irParams.ticksLow );
	}

	inline int ticsHigh( const int meaMicroSec )
	{
		// TICKS_HIGH(us) (int) (((us)*UTOL/rt_irSensor_usecPerTick + 1))
		// --------------------------------------------------------------------
		irParams.ticksHigh = static_cast<int>( 1 + meaMicroSec * irCDat.tolMax);

		return( irParams.ticksHigh );
	}

	volatile rt_irSensor_param_t irParams;

	rt_irSensor_ConstData_t irCDat;

};



} /* namespace RT_INFRARED_SENSOR */

// ---------------------------------------------------------------------[ EoF ]
#endif /* RTIRSENRCV_H_ */
