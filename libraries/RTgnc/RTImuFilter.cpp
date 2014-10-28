/*
 * RTImuFilter.cpp
 *
 *  Created on: Jan 26, 2014
 *      Author: simon
 */
// ---------------------------------------------------------------------[ BoF ]
#include <Arduino.h>
#include <math.h>

#include "RTImuFilter.h"


RTImuFilter::RTImuFilter()
{
	prm.sampleFreq = 15.0f; 	// sample frequency in Hz
	prm.beta 	   =  0.1f;		// 2 * proportional gain
	// -----------------------------------------------
	qtn.q0 = 1.0f;
	qtn.q1 = 0.0f;
	qtn.q2 = 0.0f;
	qtn.q3 = 0.0f;
	// -----------------------------------------------

	pad.rad2deg = 180.0F/M_PI;
	pad.deg2rad = M_PI/180.0F;

}	// end constructor

RTImuFilter::~RTImuFilter()
{
	// TODO Auto-generated destructor stub

}	// end destructor

// ---------------------------------------------------------[ setFilterInputs ]
void RTImuFilter::setFilterInputs( rt_imuFilterInput_t &inP )
{
	// --- gyro inputs in rad/sec
	fin.gx = inP.gx * pad.deg2rad;		// TODO: verify units on gyro data
	fin.gy = inP.gy * pad.deg2rad;
	fin.gz = inP.gz * pad.deg2rad;

	// --- accelerometer inputs in m/sec/sec
	fin.ax = inP.ax;
	fin.ay = inP.ay;
	fin.az = inP.az;

	// --- magnetometers inputs
	fin.mx = inP.mx;
	fin.my = inP.my;
	fin.mz = inP.mz;

}	// done setFilterInputs()

// ----------------------------------------------------------[ getQuaternions ]
void RTImuFilter::getQuaternions( void )
{
	// --- update timing
	pad.now 		= micros();
	pad.nowDiff 	= pad.now - pad.lastUpdate;
	prm.sampleFreq  = 1.0 /static_cast<float>(pad.nowDiff / 1000000.0);	// sec
	pad.lastUpdate  = pad.now;

	// gyro values are expressed in deg/sec, the * M_PI/180 will convert it to radians/sec
	//AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[6], val[7], val[8]);
	// use the call below when using a 6DOF IMU
	ahrSupdate();

}	// done getQuaternions()

// ----------------------------------------------------------[ getQuaternions ]
void RTImuFilter::getQuaternions( rt_imuFilterInput_t &inP )
{
	setFilterInputs( inP );

	getQuaternions();

}	// done getQuaternions()

// ----------------------------------------------------------------[ getEuler ]
void RTImuFilter::getEuler( rt_imuFilterEulerAngles_t	&usrAng )
{
	usrAng.phi 	 = angDeg.phi;
	usrAng.theta = angDeg.theta;
	usrAng.psi 	 = angDeg.psi;

}	// done getEuler()

// Returns the Euler angles in radians defined with the Aerospace sequence.
// See Sebastian O.H. Madwick report
// "An efficient orientation filter for inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
// ----------------------------------------------------------------[ getEuler ]
void RTImuFilter::getEuler( rt_imuFilterInput_t 		&inP,
						   rt_imuFilterEulerAngles_t	&usrAng )
{
	setFilterInputs( inP );
	getQuaternions();

	// --- angles in radians
	eulerAng.psi   =  atan2(2 * qtn.q1 * qtn.q2 - 2 * qtn.q0 * qtn.q3, 2 * qtn.q0*qtn.q0 + 2 * qtn.q1 * qtn.q1 - 1); // rad
	eulerAng.theta = -asin( 2 * qtn.q1 * qtn.q3 + 2 * qtn.q0 * qtn.q2); // rad
	eulerAng.phi   =  atan2(2 * qtn.q2 * qtn.q3 - 2 * qtn.q0 * qtn.q1, 2 * qtn.q0 * qtn.q0 + 2 * qtn.q3 * qtn.q3 - 1); // rad
	// ---
	angDeg.phi   = eulerAng.phi   * pad.rad2deg; // deg
	angDeg.theta = eulerAng.theta * pad.rad2deg; // deg
	angDeg.psi   = eulerAng.psi   * pad.rad2deg; // deg
	// ---
	angDeg.phi 	 += ( angDeg.phi   < 0.0 ) ? 360.0 : 0.0;
	angDeg.theta += ( angDeg.theta < 0.0 ) ? 360.0 : 0.0;
	angDeg.psi 	 += ( angDeg.psi   < 0.0 ) ? 360.0 : 0.0;
	// ---
	usrAng.phi 	 = angDeg.phi;
	usrAng.theta = angDeg.theta;
	usrAng.psi 	 = angDeg.psi;

}	// done getEuler()

// --------------------------------------------------------------[ ahrSupdate ]
void RTImuFilter::ahrSupdate( void )
{
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz;
	float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((fin.mx == 0.0f) && (fin.my == 0.0f) && (fin.mz == 0.0f))
    {
		//adgwickAHRSupdateIMU(fin.gx, fin.gy, fin.gz, fin.ax, fin.ay, fin.az);
		ahrSupdateIMU();
		return;
	}

	// Rate of change of quaternion from gyroscope
	prm.qDot1 = 0.5f * (-qtn.q1 * fin.gx - qtn.q2 * fin.gy - qtn.q3 * fin.gz);
	prm.qDot2 = 0.5f * ( qtn.q0 * fin.gx + qtn.q2 * fin.gz - qtn.q3 * fin.gy);
	prm.qDot3 = 0.5f * ( qtn.q0 * fin.gy - qtn.q1 * fin.gz + qtn.q3 * fin.gx);
	prm.qDot4 = 0.5f * ( qtn.q0 * fin.gz + qtn.q1 * fin.gy - qtn.q2 * fin.gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((fin.ax == 0.0f) && (fin.ay == 0.0f) && (fin.az == 0.0f)))
    {

		// Normalise accelerometer measurement
		prm.recipNorm = invSqrt(fin.ax * fin.ax + fin.ay * fin.ay + fin.az * fin.az);
		fin.ax *= prm.recipNorm;
		fin.ay *= prm.recipNorm;
		fin.az *= prm.recipNorm;

		// Normalise magnetometer measurement
		prm.recipNorm = invSqrt(fin.mx * fin.mx + fin.my * fin.my + fin.mz * fin.mz);
		fin.mx *= prm.recipNorm;
		fin.my *= prm.recipNorm;
		fin.mz *= prm.recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * qtn.q0 * fin.mx;
		_2q0my = 2.0f * qtn.q0 * fin.my;
		_2q0mz = 2.0f * qtn.q0 * fin.mz;
		_2q1mx = 2.0f * qtn.q1 * fin.mx;
		_2q0 = 2.0f * qtn.q0;
		_2q1 = 2.0f * qtn.q1;
		_2q2 = 2.0f * qtn.q2;
		_2q3 = 2.0f * qtn.q3;
		_2q0q2 = 2.0f * qtn.q0 * qtn.q2;
		_2q2q3 = 2.0f * qtn.q2 * qtn.q3;
		q0q0 = qtn.q0 * qtn.q0;
		q0q1 = qtn.q0 * qtn.q1;
		q0q2 = qtn.q0 * qtn.q2;
		q0q3 = qtn.q0 * qtn.q3;
		q1q1 = qtn.q1 * qtn.q1;
		q1q2 = qtn.q1 * qtn.q2;
		q1q3 = qtn.q1 * qtn.q3;
		q2q2 = qtn.q2 * qtn.q2;
		q2q3 = qtn.q2 * qtn.q3;
		q3q3 = qtn.q3 * qtn.q3;

		// Reference direction of Earth's magnetic field
		prm.hx = fin.mx * q0q0 - _2q0my * qtn.q3 + _2q0mz * qtn.q2 + fin.mx * q1q1 + _2q1 * fin.my * qtn.q2 + _2q1 * fin.mz * qtn.q3 - fin.mx * q2q2 - fin.mx * q3q3;
		prm.hy = _2q0mx * qtn.q3 + fin.my * q0q0 - _2q0mz * qtn.q1 + _2q1mx * qtn.q2 - fin.my * q1q1 + fin.my * q2q2 + _2q2 * fin.mz * qtn.q3 - fin.my * q3q3;
		_2bx = sqrt(prm.hx * prm.hx + prm.hy * prm.hy);
		_2bz = -_2q0mx * qtn.q2 + _2q0my * qtn.q1 + fin.mz * q0q0 + _2q1mx * qtn.q3 - fin.mz * q1q1 + _2q2 * fin.my * qtn.q3 - fin.mz * q2q2 + fin.mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		prm.s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - fin.ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - fin.ay) - _2bz * qtn.q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - fin.mx) + (-_2bx * qtn.q3 + _2bz * qtn.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - fin.my) + _2bx * qtn.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - fin.mz);
		prm.s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - fin.ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - fin.ay) - 4.0f * qtn.q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - fin.az) + _2bz * qtn.q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - fin.mx) + (_2bx * qtn.q2 + _2bz * qtn.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - fin.my) + (_2bx * qtn.q3 - _4bz * qtn.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - fin.mz);
		prm.s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - fin.ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - fin.ay) - 4.0f * qtn.q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - fin.az) + (-_4bx * qtn.q2 - _2bz * qtn.q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - fin.mx) + (_2bx * qtn.q1 + _2bz * qtn.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - fin.my) + (_2bx * qtn.q0 - _4bz * qtn.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - fin.mz);
		prm.s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - fin.ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - fin.ay) + (-_4bx * qtn.q3 + _2bz * qtn.q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - fin.mx) + (-_2bx * qtn.q0 + _2bz * qtn.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - fin.my) + _2bx * qtn.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - fin.mz);
		prm.recipNorm = invSqrt(prm.s0 * prm.s0 + prm.s1 * prm.s1 + prm.s2 * prm.s2 + prm.s3 * prm.s3); // normalise step magnitude
		prm.s0 *= prm.recipNorm;
		prm.s1 *= prm.recipNorm;
		prm.s2 *= prm.recipNorm;
		prm.s3 *= prm.recipNorm;

		// Apply feedback step
		prm.qDot1 -= prm.beta * prm.s0;
		prm.qDot2 -= prm.beta * prm.s1;
		prm.qDot3 -= prm.beta * prm.s2;
		prm.qDot4 -= prm.beta * prm.s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	qtn.q0 += prm.qDot1 * (1.0f / prm.sampleFreq);
	qtn.q1 += prm.qDot2 * (1.0f / prm.sampleFreq);
	qtn.q2 += prm.qDot3 * (1.0f / prm.sampleFreq);
	qtn.q3 += prm.qDot4 * (1.0f / prm.sampleFreq);

	// Normalise quaternion
	prm.recipNorm = invSqrt(qtn.q0 * qtn.q0 + qtn.q1 * qtn.q1 + qtn.q2 * qtn.q2 + qtn.q3 * qtn.q3);
	qtn.q0 *= prm.recipNorm;
	qtn.q1 *= prm.recipNorm;
	qtn.q2 *= prm.recipNorm;
	qtn.q3 *= prm.recipNorm;

}	// done ahrSupdate()

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

//void RTImuFilter::ahrSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
//adgwickAHRSupdateIMU(fin.gx, fin.gy, fin.gz, fin.ax, fin.ay, fin.az);
void RTImuFilter::ahrSupdateIMU( void )
{
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	prm.qDot1 = 0.5f * (-qtn.q1 * fin.gx - qtn.q2 * fin.gy - qtn.q3 * fin.gz);
	prm.qDot2 = 0.5f * ( qtn.q0 * fin.gx + qtn.q2 * fin.gz - qtn.q3 * fin.gy);
	prm.qDot3 = 0.5f * ( qtn.q0 * fin.gy - qtn.q1 * fin.gz + qtn.q3 * fin.gx);
	prm.qDot4 = 0.5f * ( qtn.q0 * fin.gz + qtn.q1 * fin.gy - qtn.q2 * fin.gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((fin.ax == 0.0f) && (fin.ay == 0.0f) && (fin.az == 0.0f)))
	{
		// Normalise accelerometer measurement
		prm.recipNorm = invSqrt(fin.ax * fin.ax + fin.ay * fin.ay + fin.az * fin.az);
		fin.ax *= prm.recipNorm;
		fin.ay *= prm.recipNorm;
		fin.az *= prm.recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * qtn.q0;
		_2q1 = 2.0f * qtn.q1;
		_2q2 = 2.0f * qtn.q2;
		_2q3 = 2.0f * qtn.q3;
		_4q0 = 4.0f * qtn.q0;
		_4q1 = 4.0f * qtn.q1;
		_4q2 = 4.0f * qtn.q2;
		_8q1 = 8.0f * qtn.q1;
		_8q2 = 8.0f * qtn.q2;
		q0q0 = qtn.q0 * qtn.q0;
		q1q1 = qtn.q1 * qtn.q1;
		q2q2 = qtn.q2 * qtn.q2;
		q3q3 = qtn.q3 * qtn.q3;

		// Gradient decent algorithm corrective step
		prm.s0 = _4q0 * q2q2 + _2q2 * fin.ax + _4q0 * q1q1 - _2q1 * fin.ay;
		prm.s1 = _4q1 * q3q3 - _2q3 * fin.ax + 4.0f * q0q0 * qtn.q1 - _2q0 * fin.ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * fin.az;
		prm.s2 = 4.0f * q0q0 * qtn.q2 + _2q0 * fin.ax + _4q2 * q3q3 - _2q3 * fin.ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * fin.az;
		prm.s3 = 4.0f * q1q1 * qtn.q3 - _2q1 * fin.ax + 4.0f * q2q2 * qtn.q3 - _2q2 * fin.ay;
		prm.recipNorm = invSqrt(prm.s0 * prm.s0 + prm.s1 * prm.s1 + prm.s2 * prm.s2 + prm.s3 * prm.s3); // normalise step magnitude
		prm.s0 *= prm.recipNorm;
		prm.s1 *= prm.recipNorm;
		prm.s2 *= prm.recipNorm;
		prm.s3 *= prm.recipNorm;

		// Apply feedback step
		prm.qDot1 -= prm.beta * prm.s0;
		prm.qDot2 -= prm.beta * prm.s1;
		prm.qDot3 -= prm.beta * prm.s2;
		prm.qDot4 -= prm.beta * prm.s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	qtn.q0 += prm.qDot1 * (1.0f / prm.sampleFreq);
	qtn.q1 += prm.qDot2 * (1.0f / prm.sampleFreq);
	qtn.q2 += prm.qDot3 * (1.0f / prm.sampleFreq);
	qtn.q3 += prm.qDot4 * (1.0f / prm.sampleFreq);

	// Normalise quaternion
	prm.recipNorm = invSqrt(qtn.q0 * qtn.q0 + qtn.q1 * qtn.q1 + qtn.q2 * qtn.q2 + qtn.q3 * qtn.q3);
	qtn.q0 *= prm.recipNorm;
	qtn.q1 *= prm.recipNorm;
	qtn.q2 *= prm.recipNorm;
	qtn.q3 *= prm.recipNorm;

}	// done ahrSupdateIMU


// ---------------------------------------------------------------[ printData ]
void RTImuFilter::printData( void )
{
	Serial.print("Euler Ang (deg):\t");
	// -----------------------------
	Serial.print( angDeg.phi);
	Serial.print("   ");
	Serial.print( angDeg.theta);
	Serial.print("   ");
	Serial.println( angDeg.psi);

}	// done printData()

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
//------------------------------------------------------------------[ invSqrt ]
float RTImuFilter::invSqrt( const float &xVar )
{
	float halfx = 0.5f * xVar;
	float y = xVar;
	long i = *(long*)&y;

	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));

	return( y );

}	// done invSqrt()

// ---------------------------------------------------------------------[ EoF ]
