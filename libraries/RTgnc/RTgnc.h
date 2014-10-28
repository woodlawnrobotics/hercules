/*
 * RTgnc.h
 *
 *  Created on: Jan 28, 2014
 *      Author: simon
 */

#ifndef RTGNC_H_
#define RTGNC_H_
// ---------------------------------------------------------------------[ BoF ]
// Reference the I2C Library
#include <Wire.h>


// Reference the HMC5883L Compass Library
#include <HMC5883L.h>	// magenomeer, compass
#include <ADXL345.h>	// accelerometer
#include <ITG3200.h>	// gyro

// ----
#include "RTImuFilter.h"
#include "RTgnc_inc.h"



namespace RT_GNC
{

class RTgnc
{
public:
	RTgnc();
	~RTgnc();


	void init( void );
	void step( void );

	void printData( void );

private:

	// compass
	HMC5883L *compass;

	// accelerometer
	ADXL345 *acc;

	// --- gyro
	ITG3200	*gyro;

	// --- imu sensor fusion
	RTImuFilter *imu;


	// --- sensor magnetometer data
	magnetometerData_t		magData;
	adxl345_data_t			accData;
	itg320_GyroData_t		gyroData;

	// --- input to data fusion
	rt_imuFilterInput_t			senData;

	rt_imuFilterEulerAngles_t	eulerAng;

	// --- miscelaneous
	int	debug;


};

} /* namespace RT_GNC */

// ---------------------------------------------------------------------[ EoF ]
#endif /* RTGNC_H_ */
