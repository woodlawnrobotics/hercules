// ---------------------------------------------------------------------------
// *
// * RTUltraSound.cpp
// *
// *  Created on: December, 2013
// *      Author: simon
// *
// ---------------------------------------------------------------------------
// M. Simon,  December 2013
// ---------------------------------------------------------------------[ BoF ]


#include "RTUltraSound.h"

namespace WDUE
{
	int kt = 0;
}

namespace WRT_ARDUINO_XSOUND
{



RTUltraSound::RTUltraSound()
{
	// --- scale factor is (Vcc/512) per inch. A 5V supply yields ~9.8mV/in
	// ---------------------------------------------------------------------
	uxSound.settings.suplyVols	= 5.0; 	// --- volts
	uxSound.settings.factor		= uxSound.settings.suplyVols / 512.0;

	// --- low pass filter for sampling rate 100Hz, cutoff requency 1 Hz
	// --- [0.1137    0.7025    0.6323    0.4988]
	// ---------------------------------------------------------------------
	uxSound.lpf.A =  0.1137;
	uxSound.lpf.B =  0.7025;
	uxSound.lpf.C =  0.6323;
	uxSound.lpf.D =  0.4988;
	// ---------------------------------------------------------------------

	// ---------------------------------------------------------------------
	for( int ii = 0; ii < wrt_due_nUltraSoundSensors; ii++ )
	{
		uxSound.ux[ii].x = 0.0;
		uxSound.ux[ii].u = 0.0;
		uxSound.ux[ii].y = 0.0;

		uxSound.ux[ii].pin = 0;
		uxSound.ux[ii].isActive = false;
	}
	// ---------------------------------------------------------------------
	this->isNewDataAvailable = false;
	this->idx = 0;

}

RTUltraSound::~RTUltraSound()
{

}

// --------------------------------------------------------------------[ init ]
void RTUltraSound::init( const rt_uxSoundSettings_t & uxSettings )
{
	const int nSensors = ( uxSettings.nUxDevices < wrt_due_nUltraSoundSensors ) ?
						   uxSettings.nUxDevices : wrt_due_nUltraSoundSensors;

	uxSound.settings.nUxDevices = nSensors;
	uxSound.settings.suplyVols  = uxSettings.suplyVols;
	uxSound.settings.factor	    = uxSound.settings.suplyVols / 512.0;
	// ---------------------------------------------------------------------
	for( int ii = 0; ii < nSensors; ii++ )
	{
		uxSound.ux[ii].x = 0.0;
		uxSound.ux[ii].u = 0.0;
		uxSound.ux[ii].y = 0.0;

		uxSound.ux[ii].pin = uxSettings.pin[ii];

		uxSound.ux[ii].isActive = true;
	}
	// ---------------------------------------------------------------------
	this->isNewDataAvailable = false;
	this->idx = 0;

	// change the resolution to 12 bits and read A0
#ifdef __arm__
	// --- Arduino Due
	analogReadResolution(12);
#endif


} // done init()

// ----------------------------------------------------------------[ getRange ]
void RTUltraSound::getRange( rt_uxRangeDataSensors_t &range  )
{
	const int nSensors = ( range.nUxDevices < uxSound.settings.nUxDevices ) ?
						   range.nUxDevices : uxSound.settings.nUxDevices;
	// ---------------------------------------------------------------------
	for( int ii = 0; ii < nSensors; ii++ )
	{
		if( true == uxSound.ux[ii].isActive )
		{
			range.data[ii].in = uxRange[this->rdx].data[ii].in;
			range.data[ii].cm = uxRange[this->rdx].data[ii].cm;
		}
	}
	// ---------------------------------------------------------------------
	this->isNewDataAvailable = false;

}	// done getRange()

// --------------------------------------------------------------[ readSensor ]
void RTUltraSound::readSensor( void )
{
	double xx = 0.0;
	double yy = 0.0;


	// read in the analog voltage output from the MaxSonar device.
	// analog pin goes from 0 to 1024; the read value has to be
	// divided by 2 to get the actual inches
	// ---------------------------------------------------------------------
	// ---------------------------------------------------------------------
	for( int ii = 0; ii < wrt_due_nUltraSoundSensors; ii++ )
	{
		if( true == uxSound.ux[ii].isActive )
		{
		    // used to read in the analog voltage output that is being sent by
			// the MaxSonar device.
		    // scale factor is (Vcc/512) per inch. A 5V supply yields ~9.8mV/in
		    // Arduino analog pin goes from 0 to 1024, so the value has to
			// be divided by 2 to get the actual inches
			// ---------------------------------------------------
			#ifdef __arm__
				  // --- Arduino Due
				uxSound.ux[ii].data = (analogRead(uxSound.ux[ii].pin) >> 3);
			#else
				// --- Arduino Uno
				uxSound.ux[ii].data = (analogRead(uxSound.ux[ii].pin) >> 1);
			#endif
			// ---------------------------------------------------

			uxSound.ux[ii].data = analogRead(uxSound.ux[ii].pin);
			uxSound.ux[ii].u    = static_cast<double>(uxSound.ux[ii].data  >> 3);

			xx  = (uxSound.lpf.A * uxSound.ux[ii].x);
			xx += (uxSound.lpf.B * uxSound.ux[ii].u);

			yy  = (uxSound.lpf.C * uxSound.ux[ii].x);
			yy += (uxSound.lpf.D * uxSound.ux[ii].u);

			uxSound.ux[ii].x = xx;
			uxSound.ux[ii].y = yy;

			uxSound.ux[ii].range.in = yy;		// inches
			uxSound.ux[ii].range.cm = 2.54 * yy;	// cm


			uxRange[this->idx].data[ii].in = uxSound.ux[ii].range.in;

#if defined( RT_DEBUG )
			if( 0 == WDUE::kt)
			{
				Serial.print( "range2: [" );
				Serial.print( this->idx );
				Serial.print( "], u = " );
				Serial.print( uxSound.ux[ii].u );
				Serial.print( ", " );
				Serial.println( uxSound.ux[ii].range.in ); //uxRange[this->idx].data[ii].in );
			}

			WDUE::kt += ( WDUE::kt < 100 ) ? 1 : (-WDUE::kt);
#endif

		}
	}
	// ---------------------------------------------------------------------
	this->rdx  =  this->idx;
	this->idx += (this->idx < (wrt_due_UltraSoundBufferSize - 1) ) ?
			1 : (-this->idx);


	// --- filter the sensor data
	// ---------------------------------------------------------------------
	//lowPassFilter();



	// --- signal data availability
	this->isNewDataAvailable = true;

} // done readSensor()

// -----------------------------------------------------------[ lowPassFilter ]
void RTUltraSound::lowPassFilter( void )
{
	double xx = 0.0;
	double yy = 0.0;

	// ---------------------------------------------------------------------
	for( int ii = 0; ii < wrt_due_nUltraSoundSensors; ii++ )
	{
		if( true == uxSound.ux[ii].isActive )
		{
			xx  = uxSound.lpf.A * uxSound.ux[ii].x;
			xx += uxSound.lpf.B * uxSound.ux[ii].u;

			yy  = uxSound.lpf.C * uxSound.ux[ii].x;
			yy += uxSound.lpf.D * uxSound.ux[ii].u;

			uxSound.ux[ii].x = xx;
			uxSound.ux[ii].y = yy;

			uxSound.ux[ii].range.in = yy;		// inches
			uxSound.ux[ii].range.cm = 2.54 * yy;	// cm


			uxRange[this->idx].data[ii].in = uxSound.ux[ii].range.in;

			Serial.print( "range2: [" );
			Serial.print( this->idx );
			Serial.print( "], u = " );
			Serial.print( uxSound.ux[ii].x );
			Serial.print( ", " );
			Serial.println( uxRange[this->idx].data[ii].in );
		}
	}
	// ---------------------------------------------------------------------
	this->rdx  =  this->idx;
	this->idx += (this->idx < (wrt_due_UltraSoundBufferSize - 1) ) ?
			1 : (-this->idx);

} // done lowPassFilter()


// ---------------------------------------------------------------------[ EoF ]
} /* namespace WRT_ARDUINO_XSOUND */
