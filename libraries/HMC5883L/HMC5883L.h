/*
HMC5883L.h - Header file for the HMC5883L Triple Axis Magnetometer Arduino Library.
Copyright (C) 2011 Love Electronics (loveelectronics.co.uk)

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

 WARNING: THE HMC5883L IS NOT IDENTICAL TO THE HMC5883!
 Datasheet for HMC5883L:
 http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/HMC5883L_3-Axis_Digital_Compass_IC.pdf
 http://c48754.r54.cf3.rackcdn.com/HMC5883L.pdf

*/

#ifndef HMC5883L_h
#define HMC5883L_h
// ---------------------------------------------------------------------[ BoF ]
#include <inttypes.h>

#include "Arduino.h"

#include "HMC5883L_inc.h"

class HMC5883L
{
public:
	HMC5883L();

	void init( float );
	void getData( magnetometerData_t & );
	void printData( void );
	bool ensureConnected();


	// --- set the local magnetic declination angle:
	inline void setDeclinationAngle( float someDecAngle )
	{
		this->declinationAngle = someDecAngle;
	}

protected:

	void writeReg( const int address, const int byte   );
	void readReg(  const int address, const int length );

private:

	void getMagnetometerData( void );
	void readRawAxis();
	void readScaledAxis();

	int setMeasurementMode(uint8_t mode);
	int setScale( uint16_t );

	void printError(int errorCode);


	float declinationAngle;
	float m_Scale;

	bool isConnected;

	// --- sensor magnetometer data
	magnetometerData_t		data;

	// --- miscelaneous
	int errorCode;

};

// ---------------------------------------------------------------------[ EoF ]
#endif
