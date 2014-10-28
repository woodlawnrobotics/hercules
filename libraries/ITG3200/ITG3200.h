/****************************************************************************
* ITG3200.h - ITG-3200/I2C library v0.5 for Arduino                         *
* Copyright 2010-2011 Filipe Vieira & various contributors                  *
* http://code.google.com/p/itg-3200driver                                   *
* This file is part of ITG-3200 Arduino library.                            *
*                                                                           *
* This library is free software: you can redistribute it and/or modify      *
* it under the terms of the GNU Lesser General Public License as published  *
* by the Free Software Foundation, either version 3 of the License, or      *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU Lesser General Public License for more details.                       *
*                                                                           *
* You should have received a copy of the GNU Lesser General Public License  *
* along with this program.  If not, see <http://www.gnu.org/licenses/>.     *
****************************************************************************/
/****************************************************************************
* Tested on Arduino Mega with ITG-3200 Breakout                             *
* SCL     -> pin 21     (no pull up resistors)                              *
* SDA     -> pin 20     (no pull up resistors)                              *
* CLK & GND -> pin GND                                                    *
* INT       -> not connected  (but can be used)                             *
* VIO & VDD -> pin 3.3V                                                     *
*****************************************************************************/
#ifndef FIMU_ITG3200_h
#define FIMU_ITG3200_h

#include "Arduino.h"
#include <Wire.h>

#include <inttypes.h>

#include "ITG3200_inc.h"


class ITG3200
{

public:

  ITG3200( void );
  
  // Gyro initialization
  void init(unsigned int address);
  void init(unsigned int address, byte _SRateDiv, byte _Range, byte _filterBW, byte _ClockSrc, bool _ITGReady, bool _INTRawDataReady);
  // ---
  void printData( void );
    
  // Who Am I
  byte getDevAddr();
  void setDevAddr(unsigned int _addr);

  // Sample Rate Divider
  byte getSampleRateDiv();          
  void setSampleRateDiv(byte _SampleRate);

  // Digital Low Pass Filter BandWidth and SampleRate 
  byte getFSRange();
  void setFSRange(byte _Range); // RANGE2000
  byte getFilterBW(); 
  void setFilterBW(byte _BW); // see register parameters above

  // Interrupt Configuration
  bool isINTActiveOnLow();
  void setINTLogiclvl(bool _State); //ACTIVE_ONHIGH, ACTIVE_ONLOW

  // Interrupt drive type
  bool isINTOpenDrain();
  void setINTDriveType(bool _State); //OPEN_DRAIN, PUSH_PULL

  // Interrupt Latch mode
  bool isLatchUntilCleared();
  void setLatchMode(bool _State); //UNTIL_INT_CLEARED, PULSE_50US

  // Interrupt Latch clear method
  bool isAnyRegClrMode();
  void setLatchClearMode(bool _State); //READ_ANYREG, READ_STATUSREG

  // INT pin triggers
  bool isITGReadyOn();          
  void setITGReady(bool _State);
  bool isRawDataReadyOn();
  void setRawDataReady(bool _State);

  // Trigger Status
  bool isITGReady();
  bool isRawDataReady();

  // -------------------------------
  void getData( void );
  void getData( itg320_GyroData_t & );
  // -------------------------------

  // --- assuming gyroscope is stationary (updates XYZ offsets for zero)
  // --------------------------------------------------------------------
  void zeroCalibrate( const unsigned int, const unsigned int );


  // Power management
  void reset(); // after reset all registers have default values
  bool isLowPower();
  void setPowerMode(bool _State); // NORMAL, STANDBY
  bool isXgyroStandby();            
  bool isYgyroStandby();
  bool isZgyroStandby();
  void setXgyroStandby(bool _Status); // NORMAL, STANDBY
  void setYgyroStandby(bool _Status);
  void setZgyroStandby(bool _Status);
  byte getClockSource();
  void setClockSource(byte _CLKsource); // see register parameters above
  
  
private:

	// Gyro Sensors
	// --------------------------------------------------------
	void readTemp( void );
	void readTemp( float *_Temp );
	// --------------------------------------------------------
	void readGyro( void ); // includes gain and offset
	void readGyro( float *_GyroXYZ); // includes gain and offset
	void readGyro( float *_GyroX, float *_GyroY, float *_GyroZ); // includes gain and offset
	// --------------------------------------------------------
	void readGyroRaw( void );
	void readGyroRaw( int  *_GyroXYZ );
	void readGyroRaw( int *_GyroX, int *_GyroY, int *_GyroZ );
	// --------------------------------------------------------
	void readGyroCal( void );
	void readGyroCal( int *_GyroX, int *_GyroY, int *_GyroZ );
	void readGyroCal( int *_GyroXYZ );
	// --------------------------------------------------------
	void setOffsets(int _Xoffset, int _Yoffset, int _Zoffset);
	void setRevPolarity(bool _Xpol, bool _Ypol, bool _Zpol);	// true = Reversed  false = default
	void setGains(float _Xgain, float _Ygain, float _Zgain);
	// --------------------------------------------------------
	void writeTo(  const uint8_t, const uint8_t );
	void readFrom( const uint8_t, const uint8_t, uint8_t buff[] );

	itg320_GyroData_t				gyroDat;
	itg320_GyroEngData_t			gyroEng;
	itg320_GyroRawData_t			gyroRaw;
	itg320_GyroCalData_t			gyroCal;
	itg320_GyroCalibrationParam_t	calParam;

	uint8_t devAddr;
	uint8_t dBuffer[6];

};

// ---------------------------------------------------------------------[ EoF ]
#endif
