/*
 * RTirSenRcv.cpp
 *
 *  Created on: May 25, 2014
 *      Author: simon
 */

#include <Arduino.h>

#include <RTirSenRcv.h>

namespace RT_INFRARED_SENSOR
{

RTirSenRcv::RTirSenRcv()
{
	irParams.recvpin = 8;
	irParams.isBlink = false;

}

RTirSenRcv::~RTirSenRcv()
{
	irParams.isBlink = false;
}

// --------------------------------------------------------------------[ init ]
void RTirSenRcv::init( int useThisPin )
{
	irParams.recvpin = useThisPin;
	irParams.isBlink = false;

}	// done init()

// TIMER2 interrupt code to collect raw data.
// Widths of alternating SPACE, MARK are recorded in rawbuf.
// Recorded in ticks of 50 microseconds.
// rawlen counts the number of entries recorded so far.
// First entry is the SPACE between transmissions.
// As soon as a SPACE gets long, ready is set, state switches to IDLE, timing of SPACE continues.
// As soon as first MARK arrives, gap width is recorded, ready is cleared, and new logging starts
// ---------------------------------------------------------------------[ ISR ]
void RTirSenRcv::service( const uint8_t irdata )
{
	//const uint8_t irdata = (uint8_t) digitalRead( irParams.recvpin );

	irParams.timer++; // One more 50us tick

	if (irParams.rawlen >= rt_irSensor_rawBufferLen)
	{
		// Buffer overflow
		irParams.rcvstate = static_cast<uint8_t>(rt_irSensor_state_stop);;
	}

	switch( irParams.rcvstate )
	{	// -----------------------------------------[  in the middle of a gap ]
		case rt_irSensor_state_idle:
		{	if (irdata == irCDat.mark )
			{
				if (irParams.timer < irCDat.gapTicks)
				{
					// Not big enough to be a gap.
					irParams.timer = 0;
				}
				else
				{
					// gap just ended, record duration and start recording transmission
					irParams.rawlen = 0;
					irParams.rawbuf[irParams.rawlen++] = irParams.timer;
					irParams.timer = 0;
					irParams.rcvstate = static_cast<uint8_t>(rt_irSensor_state_mark);
				}
			}
		} break;
		// -----------------------------------------------------[ timing MARK ]
		case rt_irSensor_state_mark:
		{
			if (irdata == rt_irSensor_state_space )
			{   	// MARK ended, record time
				  irParams.rawbuf[irParams.rawlen++] = irParams.timer;
				  irParams.timer = 0;
				  irParams.rcvstate = static_cast<uint8_t>(rt_irSensor_state_space);
			}
		} break;
		// ----------------------------------------------------[ timing SPACE ]
		case rt_irSensor_state_space:
		{	if (irdata == irCDat.mark)
			{ 	// SPACE just ended, record it
				irParams.rawbuf[irParams.rawlen++] = irParams.timer;
				irParams.timer = 0;
				irParams.rcvstate = static_cast<uint8_t>(rt_irSensor_state_mark);
			}
			else
			{ 	// SPACE
				if (irParams.timer > irCDat.gapTicks )
				{
					// big SPACE, indicates gap between codes
					// Mark current code as ready for processing
					// Switch to STOP
					// Don't reset timer; keep counting space width
					irParams.rcvstate = static_cast<uint8_t>(rt_irSensor_state_stop);
				}
			}
		} break;
		case rt_irSensor_state_stop: // reset gap timer
		{
			if (irdata == irCDat.mark)
			{
				irParams.timer = 0;
			}

		} break;
	}	// switch

	if( true == irParams.isBlink)
	{
		if (irdata == irCDat.mark)
		{	// turn pin 13 LED on

			#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
				PORTB |= B10000000;
			#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
				PORTD |= B00000001;
			#else
				digitalWrite( rt_irSensor_pinGreenLED, HIGH);
			#endif
		}
		else
		{	// turn pin 13 LED off

			#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
				PORTB &= B01111111;
			#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
				PORTD &= B11111110;
			#else
				digitalWrite(rt_irSensor_pinGreenLED, LOW);
			#endif

		}
	}	// IF

} // done ISR


// Decodes the received IR message
// Returns 0 if no data ready, 1 if data ready.
// Results of decoding are stored in results
// ------------------------------------------------------------------[ decode ]
int RTirSenRcv::decode( rt_irSensor_decodeResults_t &results)
{
	results.rawbuf = irParams.rawbuf;
	results.rawlen = irParams.rawlen;

	if (irParams.rcvstate != static_cast<long>(rt_irSensor_state_stop) )
	{
		return( static_cast<long>(rt_irSensor_isError) );
	}

	if( decodeNEC(results))
	{
		return( static_cast<long>(rt_irSensor_isDecoded) );
	}

	// decodeHash returns a hash on any input.
	// Thus, it needs to be last in the list.
	// If you add any decodes, add them before this.
	if (decodeHash(results))
	{
		return( static_cast<long>(rt_irSensor_isDecoded) );
	}

	// Throw away and start over
	resume();

	return( static_cast<long>(rt_irSensor_isError) );

}	// done decode()

// NECs have a repeat only 4 items long
long RTirSenRcv::decodeNEC( rt_irSensor_decodeResults_t &results)
{

	return( static_cast<long>(rt_irSensor_isError) );

//  long data = 0;
//  int offset = 1; // Skip first space
//  // Initial mark
//  if (!MATCH_MARK(results.rawbuf[offset], NEC_HDR_MARK)) {
//    return ERR;
//  }
//  offset++;
//  // Check for repeat
//  if (irParams.rawlen == 4 &&
//    MATCH_SPACE(results.rawbuf[offset], NEC_RPT_SPACE) &&
//    MATCH_MARK(results.rawbuf[offset+1], NEC_BIT_MARK)) {
//    results.bits = 0;
//    results.value = REPEAT;
//    results.decode_type = NEC;
//    return DECODED;
//  }
//  if (irParams.rawlen < 2 * NEC_BITS + 4) {
//    return ERR;
//  }
//  // Initial space
//  if (!MATCH_SPACE(results.rawbuf[offset], NEC_HDR_SPACE)) {
//    return ERR;
//  }
//  offset++;
//  for (int i = 0; i < NEC_BITS; i++) {
//    if (!MATCH_MARK(results.rawbuf[offset], NEC_BIT_MARK)) {
//      return ERR;
//    }
//    offset++;
//    if (MATCH_SPACE(results.rawbuf[offset], NEC_ONE_SPACE)) {
//      data = (data << 1) | 1;
//    }
//    else if (MATCH_SPACE(results.rawbuf[offset], NEC_ZERO_SPACE)) {
//      data <<= 1;
//    }
//    else {
//      return ERR;
//    }
//    offset++;
//  }
//  // Success
//  results.bits = NEC_BITS;
//  results.value = data;
//  results.decode_type = NEC;
//  return DECODED;

}

// Gets one undecoded level at a time from the raw buffer.
// The RC5/6 decoding is easier if the data is broken into time intervals.
// E.g. if the buffer has MARK for 2 time intervals and SPACE for 1,
// successive calls to getRClevel will return MARK, MARK, SPACE.
// offset and used are updated to keep track of the current position.
// t1 is the time interval for a single bit in microseconds.
// Returns -1 for error (measured time interval is not a multiple of t1).
// --------------------------------------------------------------[ getRClevel ]
int RTirSenRcv::getRClevel( rt_irSensor_decodeResults_t &results,
							int *offset,
							int *used,
							const int t1)
{
	int avail = 0;
	int width = results.rawbuf[ offset[0] ];

	const int val = ((offset[0]) % 2) ? rt_irSensor_state_mark : rt_irSensor_state_space;
	const int correction = (rt_irSensor_state_mark == val) ? irCDat.markExcess : - irCDat.markExcess;


	if ( offset[0] >= results.rawlen)
	{
		// After end of recorded buffer, assume SPACE.
		return( rt_irSensor_state_space );
	}


	if( match(width, t1 + correction) )
	{
		avail = 1;
	}
	else if( match(width, 2*t1 + correction) )
	{
		avail = 2;
	}
	else if( match(width, 3*t1 + correction) )
	{
		avail = 3;
	}
	else
	{
		return -1;
	}

	used[0] += 1;
	// -------------------
	if (used[0] >= avail)
	{
		used[0]    = 0;
		offset[0] += 1;
	}

	return( val );
}


/* -----------------------------------------------------------------------
 * hashdecode - decode an arbitrary IR code.
 * Instead of decoding using a standard encoding scheme
 * (e.g. Sony, NEC, RC5), the code is hashed to a 32-bit value.
 *
 * The algorithm: look at the sequence of MARK signals, and see if each one
 * is shorter (0), the same length (1), or longer (2) than the previous.
 * Do the same with the SPACE signals.  Hszh the resulting sequence of 0's,
 * 1's, and 2's to a 32-bit value.  This will give a unique value for each
 * different code (probably), for most code systems.
 *
 * http://arcfn.com/2010/01/using-arbitrary-remotes-with-arduino.html
 */

// Compare two tick values, returning 0 if newval is shorter,
// 1 if newval is equal, and 2 if newval is longer
// Use a tolerance of 20%
// -----------------------------------------------------------------[ compare ]
int RTirSenRcv::compare( unsigned int oldval, unsigned int newval )
{
	int rVal = 0;
	const float dTreshHold = 0.8;

	if( static_cast<float>(newval) < ( static_cast<float>(oldval) * 0.8 ) )
	{
		rVal = 0;
	}
	else if( static_cast<float>(oldval) < ( static_cast<float>(newval) * 0.8 ) )
	{
		rVal = 2;
	}
	else
	{
		rVal = 1;
	}

	return( rVal );

}	// done compare()



// * Converts the raw code values into a 32-bit hash code.
// * Hopefully this code is unique for each button.
// * This isn't a "real" decoding, just an arbitrary value.
// --------------------------------------------------------------[ decodeHash ]
long RTirSenRcv::decodeHash( rt_irSensor_decodeResults_t &results)
{
	long rVal = static_cast<long>(rt_irSensor_isError);
	long hash = FNV_BASIS_32;
	int value = 0;

	// Require at least 6 samples to prevent triggering on noise
	// ------------------------------------------------------------
	if ( 6 < results.rawlen   )
	{

		for (int ii = 1; ii+2 < results.rawlen; ii++)
		{
			value =  compare(results.rawbuf[ii], results.rawbuf[ii+2]);

			// Add value into the hash
			hash = (hash * FNV_PRIME_32) ^ value;
		}

		results.value 		= hash;
		results.bits 		= 32;
		results.decode_type = UNKNOWN;

		rVal = static_cast<long>(rt_irSensor_isDecoded);

	}	// if

	return ( rVal );

}	// done decodeHash()



// enable/disable blinking of pin 13 on IR processing
// -----------------------------------------------------------[ enableBlink13 ]
void RTirSenRcv::enableBlink13( bool isBlinkflag )
{
	irParams.isBlink = isBlinkflag;
	// ------------------------------
	if ( true == isBlinkflag )
	{
		pinMode( rt_irSensor_pinGreenLED, OUTPUT);
	}
	// ------------------------------

}	// done enableBlink13()

// --------------------------------------------------------------[ enableIRIn ]
void RTirSenRcv::enableIRIn( void )
{
	// initialize state machine variables
	irParams.rcvstate = rt_irSensor_state_idle;
	irParams.rawlen = 0;

	// set pin modes
	pinMode( irParams.recvpin, INPUT);

}	// done enableIRIn()

// ------------------------------------------------------------------[ resume ]
void RTirSenRcv::resume( void )
{
	irParams.rcvstate = rt_irSensor_state_idle;
	irParams.rawlen   = 0;

}	// dopne resume()




} /* namespace RT_INFRARED_SENSOR */
