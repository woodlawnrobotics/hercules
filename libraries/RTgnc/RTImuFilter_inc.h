/*
 * RTImuFilter_inc.h
 *
 *  Created on: Jan 26, 2014
 *      Author: simon
 */

#ifndef RTIMUFILTER_INC_H_
#define RTIMUFILTER_INC_H_
// ---------------------------------------------------------------------[ BoF ]
#include <inttypes.h>


typedef struct RT_GNC_IPAD_tag
{
	// ---
	float deg2rad;	// degrees to radians
	float rad2deg;	// inverse

	// half the sample period expressed in seconds
	float sampleFreq;

	// sample period expressed in milliseconds
	unsigned long lastUpdate;
	unsigned long now;
	unsigned long nowDiff;

} rt_gnc_iPad_t;

typedef struct RT_ImuFilter_Parameters_tag
{
	// = 15.0f; // sample frequency in Hz
	float sampleFreq;

	// 2 * proportional gain (Kp)
	float beta;

	// --- temporary filter quantities
	// -------------------------------------------------------------
	float recipNorm;

	float s0, s1, s2, s3;

	float qDot1, qDot2, qDot3, qDot4;

	float hx, hy;
	// -------------------------------------------------------------

} rt_imuFilterParameters_t;

typedef struct RT_ImuFilter_Quaternions_tag
{
	float q0;
	float q1;
	float q2;
	float q3;

} rt_imuFilterQuaternions_t;

typedef struct RT_ImuFilter_Input_tag
{
	// --- gyro inputs in rad/sec
	float gx;
	float gy;
	float gz;

	// --- accelerometer inputs in m/sec/sec
	float ax;
	float ay;
	float az;

	// --- magnetometers inputs
	float mx;
	float my;
	float mz;

} rt_imuFilterInput_t;

typedef struct RT_ImuFilter_EulerAngles_tag
{
	float phi;		// pitch
	float theta;	// roll
	float psi;		// yaw

} rt_imuFilterEulerAngles_t;



// ---------------------------------------------------------------------[ EoF ]
#endif /* RTIMUFILTER_INC_H_ */
