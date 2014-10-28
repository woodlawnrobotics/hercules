/*
 * IRremote_inc.h
 *
 *  Created on: May 25, 2014
 *      Author: simon
 */

#ifndef IRREMOTE_INC_H_
#define IRREMOTE_INC_H_
// ---------------------------------------------------------------------[ BoF ]

#include <inttypes.h>

#define NEC_BITS 			32
#define SONY_BITS 			12
#define SANYO_BITS 			12
#define MITSUBISHI_BITS 	16
#define MIN_RC5_SAMPLES 	11
#define MIN_RC6_SAMPLES 	 1
#define PANASONIC_BITS 		48
#define JVC_BITS 			16

// Values for decode_type
#define NEC 				1
#define SONY 				2
#define RC5 				3
#define RC6 				4
#define DISH 				5
#define SHARP 				6
#define PANASONIC 			7
#define JVC 				8
#define SANYO 				9
#define MITSUBISHI 			10
#define UNKNOWN 			-1

// Decoded value for NEC when a repeat code is received
#define REPEAT 			0xffffffff



// Use FNV hash algorithm: http://isthe.com/chongo/tech/comp/fnv/#FNV-param
#define FNV_PRIME_32 	16777619
#define FNV_BASIS_32 	2166136261

typedef enum RT_InfraRedSensor_Enum_tag
{
	rt_irSensor_isError			=   0,
	rt_irSensor_isDecoded		=   1,

	rt_irSensor_usecPerTick		=  50, // microseconds per clock interrupt tick
	rt_irSensor_rawBufferLen	= 100, // length of raw duration buffer

	// receiver states
	// -------------------------------------
	rt_irSensor_state_idle		= 2,
	rt_irSensor_state_mark		= 3,
	rt_irSensor_state_space		= 4,
	rt_irSensor_state_stop		= 5,


	rt_irSensor_pinGreenLED		= 13,


} rt_irSensor_enum_t;

typedef struct RT_IR_Sensor_ConstData
{
	static const int tolerance = (int) 25; // percent tolerance in measurements
	const float		 tolMin;
	const float 	 tolMax;

	static const int usecPerTick = static_cast<int>(rt_irSensor_usecPerTick);
	static const int gap = (int) 25; // minimum map between transmissions
	const float 	 gapTicks;

	// Marks tend to be 100us too long, and spaces 100us too short
	// when received due to sensor lag.
	static const int markExcess = 100;

	// IR detector output is active low
	static const int mark  = 0;
	static const int space = 1;

    // ----------------------------------------------------------------------
	RT_IR_Sensor_ConstData
    (
    	// ------------------------------------ [ arguments declaration ].start
    	float arg_tol = (static_cast<float>(tolerance) / 100.0),
    	float arg_gap = (static_cast<float>(gap)/static_cast<float>(usecPerTick))

        // -------------------------------------- [ arguments declaration ].end
        ) :
        // ---------------------------------------- [ default assigment ].start
        tolMin( (1.0 - arg_tol)/static_cast<float>(usecPerTick) ),
        tolMax( (1.0 + arg_tol)/static_cast<float>(usecPerTick) ),

        // note: ticksLow  = microsec * tolMin
        //		 ticksHigh = microsec * tolMax + 1

        gapTicks(arg_gap)

        // ------------------------------------------ [ default assigment ].end
        {}

} rt_irSensor_ConstData_t;

// information for the interrupt handler
typedef struct RT_InfraRedSensor_Parameters_tag
{
  unsigned int  timer;     			// state timer, counts 50uS ticks.
  unsigned int  rawbuf[rt_irSensor_rawBufferLen]; // raw data

  int ticksLow;
  int ticksHigh;

  // TRUE to enable blinking of pin 13 on IR processing
  bool 			isBlink;

  uint8_t 		rawlen;         	// counter of entries in rawbuf
  uint8_t 		recvpin;           // pin for IR data from detector
  uint8_t 		rcvstate;          // state machine

  uint8_t		notUsedYet;

} rt_irSensor_param_t;

// Results returned from the decoder
typedef struct RT_InfraRedSensor_DecodeResults_tag
{
	// This is only used for decoding Panasonic data
	unsigned int panasonicAddress;
	unsigned long value; 				// decoded value

	int decode_type; 					// NEC, SONY, RC5, UNKNOWN
	int bits; 							// number of bits in decoded value
	int rawlen; 						// number of records in rawbuf.

	volatile unsigned int *rawbuf; 		// raw intervals in .5 us ticks

} rt_irSensor_decodeResults_t;


// ---------------------------------------------------------------------[ EoF ]
#endif /* IRREMOTE_INC_H_ */
