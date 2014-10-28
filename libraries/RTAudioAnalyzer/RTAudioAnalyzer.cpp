// ---------------------------------------------------------------------------
// *
// * RTAudioAnalyzer.cpp
// *
// *  Created on: Oct 28, 2013
// *      Author: simon
// *
// ---------------------------------------------------------------------------
//	AudioAnalyzer.cpp - Library for audio spectrum analyzer.
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
#include "RTarduino_inc.h"
#include "RTAudioAnalyzer_inc.h"

#include "RTAudioAnalyzer.h"


namespace RT_AA
{
	bool isReset = false;

	double psdFactor = 1.0 / (2.0 * rt_aa_freqSpectrumSize);

}

// Init Analyzer connecter Pin
// -------------------------------------------------------------[ constructor ]
RTAudioAnalyzer::RTAudioAnalyzer(void)
{
	memset( (char *) &aa, 0, sizeof( struct RT_AudioAnalyzer_tag ) );

	aa.pin.strobe 	= wrt_due_digitalPin_audioAnalyzer_strobe;
	aa.pin.rst 		= wrt_due_digitalPin_audioAnalyzer_rst;
	aa.pin.dc 		= wrt_due_analgoPin_audioAnalyzer_data;

}	// end constructor

// -------------------------------------------------------------[ constructor ]
RTAudioAnalyzer::RTAudioAnalyzer( const int strobePin,
								  const int rstPin,
								  const int znalogPin)
{
	memset( (char *) &aa, 0, sizeof( struct RT_AudioAnalyzer_tag ) );

	aa.pin.strobe 	= strobePin;
	aa.pin.rst 		= rstPin;
	aa.pin.dc 		= znalogPin;

}	// end constructor

// --------------------------------------------------------------------[ init ]
void RTAudioAnalyzer::init()
{
	// --- Due has 12 bit resolution
	analogReadResolution(12);

	pinMode(aa.pin.strobe,	OUTPUT);
	pinMode(aa.pin.rst,		OUTPUT);

	RT_AA::isReset = false;
	// ---------------------
	resetModule();

} // done init()


// Reset analyzer module
// -------------------------------------------------------------[ resetModule ]
void RTAudioAnalyzer::resetModule( void )
{
	digitalWrite(aa.pin.strobe,	LOW);
	digitalWrite(aa.pin.rst,	HIGH);
	digitalWrite(aa.pin.strobe, HIGH);
	digitalWrite(aa.pin.strobe, LOW);
	digitalWrite(aa.pin.rst,	LOW);

	delayMicroseconds(72);  

} // done resetModule()

// Read DC out value
// ----------------------------------------------------------------[ ReadFreq ]
void RTAudioAnalyzer::readFrequency(rt_audioAnalyzerFrequencyVec_t &freqVec)
{
	readFrequency();

	getFrequency(freqVec);

}	// done ReadFreq()

// Read DC out value
// ----------------------------------------------------------------[ ReadFreq ]
void RTAudioAnalyzer::readFrequency( void )
{
	int    iVal = 0;
	double dVal = 0.0;

	if( false == RT_AA::isReset)
	{
		aa.timePoint.start = millis();
		RT_AA::isReset = true;
	}
	else
	{
		aa.timePoint.now = millis();
		if(aa.timePoint.now - aa.timePoint.start > 3000)
		{
			resetModule();
			RT_AA::isReset = false;
			//Serial.println("Rst");
		}
	}

	aa.psd = 0;
	// -------------------------------------------------------
	for( byte band = 0; band < rt_aa_freqSpectrumSize; band++)
	{
		delayMicroseconds(10);
		iVal = analogRead( aa.pin.dc );
		aa.freq.vec[band] = map( iVal, 0, 4095, 0, 100 );
		// ---
		aa.psd += static_cast<double>( iVal );

		delayMicroseconds(50);
		digitalWrite(aa.pin.strobe,HIGH);

		delayMicroseconds(18);
		digitalWrite(aa.pin.strobe,LOW);
	}  
	// ---
	aa.psd *= RT_AA::psdFactor;


}	// done ReadFreq()
