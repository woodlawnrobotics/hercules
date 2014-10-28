/*
 * WRTLineTrack.h
 *
 *  Created on: Mar 17, 2014
 *      Author: simon
 */
// ---------------------------------------------------------------------[ BoF ]
#ifndef WRTLINETRACK_H_
#define WRTLINETRACK_H_

#include "WRTbot_inc.h"

namespace WRT_LINE_TRACK
{

class WRTLineTrack
{
public:
	WRTLineTrack();
	~WRTLineTrack();

	void init( void );

	void 	nextStep(  wrt_motor_t &  );
	void 	nextStepX( wrt_motor_t &  );
	void 	forwardDrive( wrt_motor_t &  );
	void	differentialForwardDrive( wrt_motor_t &  );
	void 	differentialRotation( wrt_motor_t &  );
	void    differentialRotationFast( wrt_motor_t &  );

	void 	printSensorValues( void  );

	void 	setLongitudinalGains( const float usrSideGain[wrt_pin_nLineTrackSensors] );
	void 	setLateralGains( const float usrSideGain[wrt_pin_nLineTrackSensors] );

private:

	void readSensorValues( void );

	uint16_t	driveState;
	uint16_t	driveCommand;
	// --------------------------------------------------
	int			cmdLon;
	int			cmdLat;
	// --------------------------------------------------
	float 		delta[       wrt_pin_nLineTrackSensors ];
	float 		data[        wrt_pin_nLineTrackSensors ];
	float 		gainLon[ wrt_pin_nLineTrackSensors ];
	float 		gainLat[    wrt_pin_nLineTrackSensors ];
	// --------------------------------------------------
	static const double dFactor;
	static const float  dOffSet;

};

} /* namespace WRT_LINE_TRACK */

// ---------------------------------------------------------------------[ EoF ]
#endif /* WRTLINETRACK_H_ */
