// ---------------------------------------------------------------------------
// *
// * RTAudioAnalyzer_inc.h
// *
// *  Created on: Oct 28, 2013
// *      Author: simon
// *
// ---------------------------------------------------------------------------
//
//  - port to Arduino Due
//
// M. Simon,  October 2013
// ---------------------------------------------------------------------[ BoF ]
#ifndef RTAUDIOANALYZER_INC_H_
#define RTAUDIOANALYZER_INC_H_

#include <inttypes.h>

typedef enum RT_AudioAnalyzerDefs_tag
{
	rt_aa_freqSpectrumSize	= 7,

} rt_audioAnalyzerDefs_t;

typedef struct RT_AudioAnalyzerFrequency_tag
{
	//Frequency(Hz): 63  160  400  1K  2.5K  6.25K  16K
	int	hz63;
	int hz160;
	int hz400;
	int hz1k;
	int hz2k5;
	int hz6k25;
	int hz16k;

} rt_audioAnalyzerFrequency_t;

typedef union RT_AudioAnalyzerFrequencyVector_tag
{
	rt_audioAnalyzerFrequency_t	spec;	// frequency spectrum
	int	vec[rt_aa_freqSpectrumSize];

} rt_audioAnalyzerFrequencyVec_t;

typedef struct RT_AudioAnalyzerPin_tag
{
	// --- pins
	int dc;			// DC output
	int strobe;		// strobe
	int rst;		// reset  to strobe

} rt_audioAnalyzerPin_t;

typedef struct RT_AudioAnalyzerTimePoint_tag
{
	unsigned long start;
	unsigned long now;

} rt_audioAnalyzerTimePoint_t;

typedef struct RT_AudioAnalyzer_tag
{
	double  psd;	// power spectral density

	rt_audioAnalyzerFrequencyVec_t	freq;	// frequency spectrum vector

	rt_audioAnalyzerTimePoint_t 	timePoint;

	// --- pins
	rt_audioAnalyzerPin_t			pin;


} rt_audioAnalyzer_t;


// ---------------------------------------------------------------------[ EoF ]
#endif /* RTAUDIOANALYZER_INC_H_ */
