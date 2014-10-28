/*
 * WRTirReceiver.h
 *
 *  Created on: Jul 21, 2013
 *      Author: simon
 */
// ---------------------------------------------------------------------[ BoF ]

#ifndef WRTIRRECEIVER_H_
#define WRTIRRECEIVER_H_

#include "RTsys_def.h"
#include "RTsys_inc.h"
// -----------------------

#include "WRTbot_inc.h"

namespace WRT_ARDUINO_IR_RECEIVER
{

class WRTirReceiver
{
public:
	WRTirReceiver();
	~WRTirReceiver();

	void init( const uint8_t );

	uint16_t  getKey(    void );
	bool  irResponse(  wrt_ir_rc_t & );
	// ---
	uint16_t getKey38KHz( void );
	inline unsigned short  getTicks( void )
	{
		return( WRTirReceiver::nTicks );
	}

	inline void setReceiverPinIR( const uint8_t usrPinForReceiverIR )
	{
		WRTirReceiver::irPin = static_cast<uint8_t>(usrPinForReceiverIR);
	}

	inline int setVerbose( const int verboseInput )
	{
		verbose = verboseInput;
		return( verbose );
	}

private:

	void readPulse(    void );
	void remoteVerify( void );
	void pulse2bits(   void );
	int  bits2int(     void );
	// ---
	void badBuzzer( void );

	#if defined(ARDUINO_ARCH_AVR) && defined( WRT_ROBOT_USE_IR_REMOTE_LIB )

	decode_results irResults;

	// --- infrared receiver operating at 38 KHz
	IRrecv  *irRcv38KHz; // irrecv( wrt_pin_ir_sensorRomeo );

	#endif

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

// ---------------------------------------------------------------------[ EoF ]
} /* namespace WRT_ARDUINO_IR_RECEIVER */
#endif /* WRTIRRECEIVER_H_ */
