/*
 * RTgnc.cpp
 *
 *  Created on: Jan 28, 2014
 *      Author: simon
 */
// ---------------------------------------------------------------------[ BoF ]
#include <Arduino.h>
#include "RTgnc.h"

namespace RT_GNC
{

RTgnc::RTgnc()
{
	debug = 0;

	// compass
	compass = new HMC5883L();

	// accelerometer
	acc = new ADXL345();

	// --- gyro
	gyro = new ITG3200();

	// --- IMU sensor fusion
	imu = new RTImuFilter();

}

RTgnc::~RTgnc()
{
	// compass
	delete compass;

	// accelerometer
	delete acc;

	// --- gyro
	delete gyro;

	// --- imu
	delete imu;
}

// --------------------------------------------------------------------[ init ]
void RTgnc::init( void )
{
	// --- set scale to +/- 1.3 Ga
	compass->init( hmc5883L_gaussScale_130 );
	acc->init(  ADXL345_ADDR_ALT_LOW  );
	gyro->init( ITG3200_ADDR_AD0_LOW );
	// --------------------------------------
	delay(1000);

	// calibrate the ITG3200
	// --------------------------------------
	gyro->zeroCalibrate(128, 5);

	// --- find magnetic dec angle here: http://www.magnetic-declination.com/
	// --- Davidson, NC, USA, magnetic declination: 7 d 38' 33'' W,
	// -----------------------------------------
	compass->setDeclinationAngle( -0.1333 );

}	// done init()

// --------------------------------------------------------------------[ step ]
void RTgnc::step( void )
{
	// --- get sensor values
	acc->getData(     accData  );
	compass->getData( magData  );
	gyro->getData(    gyroData );
	// -------------------------------
	senData.ax =	accData.dd.x;
	senData.ay =	accData.dd.y;
	senData.az =	accData.dd.z;
	// -------------------------------
	senData.gx = 	gyroData.eng.d.x;
	senData.gy = 	gyroData.eng.d.y;
	senData.gz = 	gyroData.eng.d.z;
	// -------------------------------
	senData.mx = 	magData.scaled.XAxis;
	senData.my = 	magData.scaled.YAxis;
	senData.mz = 	magData.scaled.ZAxis;
	// -------------------------------
	imu->getQuaternions( senData );
	imu->getEuler(eulerAng);

	if( debug > 0 )
	{
		acc->printData();
		compass->printData();
		gyro->printData();
		imu->printData();
	}

}	// done step()


// ---------------------------------------------------------------[ printData ]
void RTgnc::printData( void )
{
//	acc->printData();
	compass->printData();
//	gyro->printData();
//	imu->printData();

}	// done printData()

// ---------------------------------------------------------------------[ EoF ]
} /* namespace RT_GNC */
