/*
 * RTHerculesCommandIR.h
 *
 *  Created on: Oct 24, 2014
 *      Author: simon
 */

#ifndef RTHERCULESCOMMANDIR_H_
#define RTHERCULESCOMMANDIR_H_

#include "RTsys_def.h"
#include "RTsys_inc.h"
// -----------------------

#include "WRTbot_inc.h"

#if !defined(ARDUINO_ARCH_AVR)
#error "class RTHerculesCommandIR is not compatible with this architecture"
#endif
// ---
#if !defined( WRT_ROBOT_USE_IR_REMOTE_LIB )
#error "class RTHerculesCommandIR requires IRremote library"
#endif


namespace RT_HERCULES_IR_RECEIVER
{

class RTHerculesCommandIR
{
public:
	RTHerculesCommandIR();
	~RTHerculesCommandIR();

	void init( const uint8_t );

	bool  irResponse(  wrt_ir_rc_t & );
	uint16_t  getKey( void );


	inline unsigned short  getTicks( void )
	{
		return( RTHerculesCommandIR::nTicks );
	}

	inline void setReceiverPinIR( const uint8_t usrPinForReceiverIR )
	{
		RTHerculesCommandIR::irPin = static_cast<uint8_t>(usrPinForReceiverIR);
	}

	inline int setVerbose( const int verboseInput )
	{
		verbose = verboseInput;
		return( verbose );
	}

private:

	void printKey( const uint16_t );


	// --- infrared receiver operating at 38 KHz
	IRrecv  *irRcv38KHz; // irrecv( wrt_pin_ir_sensorRomeo );

	decode_results irResults;

	uint8_t	irPin;

	// flag as true to output raw IR pulse data stream length in microseconds
	bool debug;

	// flag as true to print decoded verification integers.
	// same number for all buttons
	bool output_verify;

	// flag as true to print decoded key integers
	bool output_key;

	// verifies first bits are 11111100000000 different remotes
	// may have different start codes
	int remote_verify;

	bool isGoodKey;
	bool isInit;

	int verbose;
	unsigned short nTicks;


	// --- other
	int irPulse[IR_BIT_LENGTH];
	int irBbits[IR_BIT_LENGTH];

	// --- Arduino function 'pulseIn()' timout in microseconds
	static const unsigned long irPulseTimeOutInMicroSec;

};

} /* namespace RT_HERCULES_IR_RECEIVER */

#endif /* RTHERCULESCOMMANDIR_H_ */
