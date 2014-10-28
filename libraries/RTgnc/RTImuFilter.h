/*
 * RTImuFilter.h
 *
 *  Created on: Jan 26, 2014
 *      Author: simon
 */

#ifndef RTIMUFILTER_H_
#define RTIMUFILTER_H_
// ---------------------------------------------------------------------[ BoF ]

#include "RTImuFilter_inc.h"

class RTImuFilter
{
public:
	RTImuFilter();
	~RTImuFilter();

	void getQuaternions( void );
	void getQuaternions( rt_imuFilterInput_t & );

	void getEuler( rt_imuFilterEulerAngles_t	& );
	void getEuler( rt_imuFilterInput_t &, rt_imuFilterEulerAngles_t & );

	void setFilterInputs( rt_imuFilterInput_t & );

	void printData( void );

private:

	void ahrSupdate( void );
	void  ahrSupdateIMU( void );
	float invSqrt( const float & );

	// --- filter inputs
	rt_imuFilterInput_t 		fin;

	// --- filter parameters
	rt_imuFilterParameters_t	prm;

	// --- quaternion of sensor frame relative to auxiliary frame
	rt_imuFilterQuaternions_t	qtn;

	// --- Euler angles
	rt_imuFilterEulerAngles_t	eulerAng;
	rt_imuFilterEulerAngles_t	angDeg;

	// --- other
	rt_gnc_iPad_t			pad;


};

// ---------------------------------------------------------------------[ EoF ]
#endif /* RTIMUFILTER_H_ */
