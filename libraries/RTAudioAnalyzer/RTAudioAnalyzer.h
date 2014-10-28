// ---------------------------------------------------------------------------
// *
// * RTAudioAnalyzer.h
// *
// *  Created on: Oct 28, 2013
// *      Author: simon
// *
// ---------------------------------------------------------------------------
//	AudioAnalyzer.h - Library for audio spectrum analyzer.
//	Created by Lauren Pan,November 16, 2010.
//	Version 1.2
//	Add optional analog pin
// ---------------------------------------------------------------------------
//
//  - port to Arduino Due
//  - add timer interrupt
//  - add thread support
//
// M. Simon,  October 2013
// ---------------------------------------------------------------------[ BoF ]
#ifndef RTAudioAnalyzer_hh
#define RTAudioAnalyzer_hh

#include <DueTimer.h>
#include <Thread.h>
#include <ThreadController.h>

#include "RTarduino_inc.h"
#include "RTAudioAnalyzer_inc.h"

class RTAudioAnalyzer : public Thread
{
public:
	RTAudioAnalyzer();
	RTAudioAnalyzer( const int, const int, const int);

	void init( void );

	inline double getFrequency(rt_audioAnalyzerFrequencyVec_t & freqVec )
	{
		// frequency(Hz): 63  160  400  1K  2.5K  6.25K  16K
		freqVec.vec[0] = aa.freq.vec[0];
		freqVec.vec[1] = aa.freq.vec[1];
		freqVec.vec[2] = aa.freq.vec[2];
		freqVec.vec[3] = aa.freq.vec[3];
		freqVec.vec[4] = aa.freq.vec[4];
		freqVec.vec[5] = aa.freq.vec[5];
		freqVec.vec[6] = aa.freq.vec[6];

		return( aa.psd );

	}	// done ReadFreq()

	// we MUST overload this method here. using anything other
	// than "run" will not work properly...
	void run()
	{
		// Reads the analog pin, and saves it locally
		readFrequency();
		runned();
	}

private:

	void resetModule( void );
	void readFrequency( rt_audioAnalyzerFrequencyVec_t & );
	void readFrequency( void  );



	rt_audioAnalyzer_t	aa;

};

// ---------------------------------------------------------------------[ EoF ]
#endif	// RTAudioAnalyzer_hh
