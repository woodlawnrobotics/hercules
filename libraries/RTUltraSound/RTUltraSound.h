// ---------------------------------------------------------------------------
// *
// * RTUltraSound.h
// *
// *  Created on: December, 2013
// *      Author: simon
// *
// ---------------------------------------------------------------------------
// M. Simon,  December 2013
// ---------------------------------------------------------------------[ BoF ]
#ifndef RTULTRASOUND_H_
#define RTULTRASOUND_H_

#include <DueTimer.h>
#include <Thread.h>
#include <ThreadController.h>

#include "RTarduino_inc.h"


namespace WRT_ARDUINO_XSOUND
{

class RTUltraSound : public Thread
{
public:
	RTUltraSound();
	~RTUltraSound();

	void init( const rt_uxSoundSettings_t & );

	void readSensor( void );
	void getRange( rt_uxRangeDataSensors_t & );


	// we MUST overload this method here. using anything other
	// than "run" will not work properly...
	void run()
	{
		readSensor();

		runned();
	}

private:

	void lowPassFilter( void );

	rt_uxSound_t		    uxSound;	// --- ultrasound device interface
	rt_uxRangeDataSensors_t uxRange[wrt_due_UltraSoundBufferSize];


	bool isNewDataAvailable;

	// --- other
	uint8_t	idx;	// range data buffer index
	uint8_t	rdx; 	// read index

};

// ---------------------------------------------------------------------[ EoF ]
} /* namespace WRT_ARDUINO_XSOUND */

#endif /* RTULTRASOUND_H_ */
