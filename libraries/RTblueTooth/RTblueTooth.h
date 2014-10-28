// *
// * RTblueTooth.h
// *
// *  Created on: Feb 9, 2014
// *      Author: simon
// *
// ---------------------------------------------------------------------------
// M. Simon,  December 2013
// ---------------------------------------------------------------------[ BoF ]
#ifndef RTBLUETOOTH_H_
#define RTBLUETOOTH_H_

#include <inttypes.h>

#include <DueTimer.h>
#include <Thread.h>
#include <ThreadController.h>

#include "RTarduino_inc.h"

namespace RT_BLUETOOTH
{

class RTblueTooth : public Thread
{
public:
	RTblueTooth();
	~RTblueTooth();

	void init( void );
	void initMode( void );

	int readSerial(  void );
	int writeSerial( void );

	// we MUST overload this method here. using anything other
	// than "run" will not work properly...
	void run()
	{
		readSerial();

		runned();
	}

private:

	static const int SerialBufferSize; // 	= 0x1000;	// one page
	uint8_t	*btBuffer;

	rt_ioParam_t			pad;
	rt_ioSerialData_t	txData; 	// blueToothCom message buffer

};

} /* namespace RT_BLUETOOTH */

// ---------------------------------------------------------------------[ EoF ]
#endif /* RTBLUETOOTH_H_ */
