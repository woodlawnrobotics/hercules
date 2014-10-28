// ----------------------------------------------------------------------------
//HMC5883L.cpp - Class file for the HMC5883L Triple Axis Magnetometer Arduino Library.
//Copyright (C) 2011 Love Electronics (loveelectronics.co.uk)
//
//This program is free software: you can redistribute it and/or modify
//it under the terms of the version 3 GNU General Public License as
//published by the Free Software Foundation.
//
//This program is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// WARNING: THE HMC5883L IS NOT IDENTICAL TO THE HMC5883!
// Datasheet for HMC5883L:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/HMC5883L_3-Axis_Digital_Compass_IC.pdf
// http://c48754.r54.cf3.rackcdn.com/HMC5883L.pdf
// ----------------------------------------------------------------------------
#include <math.h>
#include "Arduino.h"
#include "Wire.h"

#include "HMC5883L.h"

namespace HMC5883L_private
{
	const float	rad2deg = 180.0/M_PI;
	const float piX2	= 2.0 * M_PI;

	const int bufferSize = 64;
	uint8_t   buffer[bufferSize];

}

// NOTE: HMC5883L runs at 15Hz (default bandwidth for the HMC5883L).

// --- constructor
// -------------------------------------------------------------[ constructor ]
HMC5883L::HMC5883L()
{
	m_Scale = 1;
	errorCode = hmc5883L_err_none;

	// 'Declination Angle',  is the 'Error' of the magnetic field in our location.
	// Find it here: http://www.magnetic-declination.com/
	// example: 2 37' W, which is 2.617 Degrees,
	//		or (which we need) 0.0456752665 radians,
	// ----------------------------------------------------------------------
	declinationAngle = -0.1333;  // Davidson, NC, USA, 7 d 38' 33'' W
	// ----------------------------------------------------------------------
}

// --------------------------------------------------------------[ readSensor ]
void HMC5883L::init(float gauss)
{
	int error = 0;

	// --- set scale to +/- 1.3 Ga
	// -------------------------------------------------------------
	error = setScale( gauss ); // Set the scale of the compass.
	// -------------------------------------------------------------
	if( errorCode != hmc5883L_err_none )
	{	 printError(error);
	}

	// --- set measurement mode to continuous
	// -------------------------------------------------------------
	error = setMeasurementMode( hmc5883L_mea_continuous );
	// -------------------------------------------------------------
	if(error != hmc5883L_err_none)
	{	 printError(error);
	}

}	// done init()


// -------------------------------------------------------------[ readRawAxis ]
void HMC5883L::getData( magnetometerData_t &usrMagData )
{
	getMagnetometerData();
	// -------------------------------------------
	usrMagData.raw.XAxis 	= data.raw.XAxis;
	usrMagData.raw.YAxis 	= data.raw.YAxis;
	usrMagData.raw.ZAxis 	= data.raw.ZAxis;
	// ---
	usrMagData.scaled.XAxis = data.scaled.XAxis;
	usrMagData.scaled.YAxis = data.scaled.YAxis;
	usrMagData.scaled.ZAxis = data.scaled.ZAxis;
	// -------------------------------------------
	usrMagData.heading    = data.heading;
	usrMagData.headingDeg = data.headingDeg;

}	// done getMagnetometerData()

// ---------------------------------------------------- [ getMagnetometerData ]
void HMC5883L::getMagnetometerData( void )
{
	readRawAxis();
	readScaledAxis();
	// ---------------

	// --- calculate heading when the magnetometer is level,
	//	   then correct for signs of axis.
	// ----------------------------------------------------------------------
	data.heading = atan2( data.scaled.YAxis, data.scaled.XAxis );
	// ----------------------------------------------------------------------

	// Once we have the heading, we must then add our 'Declination Angle',
	//	which is the 'Error' of the magnetic field in your location.
	// Find it here: http://www.magnetic-declination.com/
	// If we cannot find your Declination,
	// 	comment out these two lines, your compass will be slightly off.
	// ----------------------------------------------------------------------
	data.heading += this->declinationAngle;
	// ----------------------------------------------------------------------

	// Correct for when signs are reversed.
	if( data.heading < 0 )
	{	data.heading += HMC5883L_private::piX2;
	}

	// Check for wrap due to addition of declination.
	if( data.heading  > HMC5883L_private::piX2 )
	{	data.heading -= HMC5883L_private::piX2;
	}

	// Convert radians to degrees for readability.
	data.headingDeg = data.heading * HMC5883L_private::rad2deg;


}	// done getMagnetometerData()

// -------------------------------------------------------------[ readRawAxis ]
void HMC5883L::readRawAxis()
{
	readReg( hmc5883L_reg_dataBegin, 6 );
	// ------------------------------------
	data.raw.XAxis = (((int16_t)HMC5883L_private::buffer[0]) << 8) | HMC5883L_private::buffer[1];
	data.raw.ZAxis = (((int16_t)HMC5883L_private::buffer[2]) << 8) | HMC5883L_private::buffer[3];
	data.raw.YAxis = (((int16_t)HMC5883L_private::buffer[4]) << 8) | HMC5883L_private::buffer[5];


//	data.raw.XAxis = (~data.raw.XAxis + 1);
//	data.raw.YAxis = (~data.raw.YAxis + 1);
//	data.raw.ZAxis = (~data.raw.ZAxis + 1);

	// Convert three 16-bit 2s compliment hex values to decimal values
	// and assign to X, Z, Y, respectively.

}	// done readRawAxis()

// ----------------------------------------------------------[ readScaledAxis ]
void HMC5883L::readScaledAxis()
{
  data.scaled.XAxis = data.raw.XAxis * m_Scale;
  data.scaled.ZAxis = data.raw.ZAxis * m_Scale;
  data.scaled.YAxis = data.raw.YAxis * m_Scale;

}	// done readScaledAxis()

int HMC5883L::setScale( uint16_t gauss)
{
	uint8_t regValue = 0x01;
	m_Scale = 0.92;
	// ---
	errorCode = hmc5883L_err_none;
	// -----------------------
	switch( gauss )
	{
		case hmc5883L_gaussScale_088:
		{
			regValue = 0x00;
			m_Scale = 0.73;
		}	break;
		case hmc5883L_gaussScale_130:
		{
			regValue = 0x01;
			m_Scale = 0.92;
		}	break;
		case hmc5883L_gaussScale_190:
		{
			regValue = 0x02;
			m_Scale = 1.22;
		}	break;
		case hmc5883L_gaussScale_250:
		{
			regValue = 0x03;
			m_Scale = 1.52;
		}	break;
		case hmc5883L_gaussScale_400:
		{
			regValue = 0x04;
			m_Scale = 2.27;
		}	break;
		case hmc5883L_gaussScale_470:
		{
			regValue = 0x05;
			m_Scale = 2.56;
		}	break;
		case hmc5883L_gaussScale_560:
		{
			regValue = 0x06;
			m_Scale = 3.03;
		}	break;
		case hmc5883L_gaussScale_810:
		{
			regValue = 0x07;
			m_Scale = 4.35;
		}	break;
		default:
		{
			regValue = 0x01;
			m_Scale  = 0.92;

			errorCode = hmc5883L_err_code1;

		}	break;
	}
	
	// Setting is in the top 3 bits of the register.
	regValue = regValue << 5;
	writeReg( hmc5883L_reg_configB, regValue);

	return( 0 );

}	// done setScale()

// ------------------------------------------------------[ setMeasurementMode ]
int HMC5883L::setMeasurementMode(uint8_t mode)
{
	writeReg( hmc5883L_reg_mode, mode);

	return( 0 );

}	// DONE setMeasurementMode()

// ---------------------------------------------------------[ ensureConnected ]
bool HMC5883L::ensureConnected()
{
	readReg( hmc5883L_reg_id, 1);
	// --------------------------------------------------------------------
	this->isConnected = ( hmc5883L_reg_id_value == HMC5883L_private::buffer[0] ) ? true : false;

	return( this->isConnected );

}	// done ensureConnected()

// ----------------------------------------------------------------[ writeReg ]
void HMC5883L::writeReg( const int someAddress, const int someData)
{
	Wire.beginTransmission(hmc5883L_address);
	Wire.write( someAddress  );
	Wire.write( someData );
	Wire.endTransmission();

}	// done writeReg()

// -----------------------------------------------------------------[ readReg ]
void HMC5883L::readReg( const int someAddress, const int length)
{
	const int nbytes = (length < HMC5883L_private::bufferSize) ?
				    	length : HMC5883L_private::bufferSize;

	Wire.beginTransmission(hmc5883L_address);
	Wire.write(someAddress);
	Wire.endTransmission();

	Wire.beginTransmission(hmc5883L_address);
	Wire.requestFrom(hmc5883L_address, length);

	if( Wire.available() == nbytes )
	{
	  for(uint8_t ii = 0; ii < nbytes; ii++)
	  {
		  HMC5883L_private::buffer[ii] = Wire.read();
	  }
	}
	Wire.endTransmission();

}	// done readReg()

// ---------------------------------------------------------------[ printData ]
void HMC5883L::printData( void )
{
	Serial.print("Raw:\t");
	Serial.print( data.raw.XAxis);
	Serial.print("   ");
	Serial.print( data.raw.YAxis);
	Serial.print("   ");
	Serial.print( data.raw.ZAxis);
	Serial.print("   \tScaled:\t");

	Serial.print( data.scaled.XAxis);
	Serial.print("   ");
	Serial.print( data.scaled.YAxis);
	Serial.print("   ");
	Serial.print( data.scaled.ZAxis);

	Serial.print("   \tHeading:\t");
	Serial.print( data.heading);
	Serial.print(" Radians   \t");
	Serial.print( data.headingDeg);
	Serial.println(" Degrees   \t");

}	// done printData()

// --------------------------------------------------------------[ printError ]
void HMC5883L::printError(int errorCode)
{

	if( hmc5883L_err_code1 == errorCode )
	{	Serial.println( "HMC5883L::Entered scale was not valid, valid gauss values are: 0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1" );
	}
	else
	{
		Serial.println( "HMC5883L::error not defined");
	}

}	// done printError()
