/****************************************************************************
* ITG3200.cpp - ITG-3200/I2C library v0.5 for Arduino                         *
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
* CLK & GND -> pin GND                                                      *
* INT       -> not connected  (but can be used)                             *
* VIO & VDD -> pin 3.3V                                                     *
*****************************************************************************/
#include "ITG3200.h"


// -------------------------------------------------------------[ constructor ]
ITG3200::ITG3200( void )
{
	memset( &calParam, 0, sizeof(ITG320_Gyro_Calibration_Param_tag) );
	// ---
	memset( &gyroRaw, 0, sizeof(ITG320_Gyro_Raw_Data_tag) );
	memset( &gyroCal, 0, sizeof(ITG320_Gyro_Cal_Data_tag) );
	memset( &gyroEng, 0, sizeof(ITG320_Gyro_EngineeringData_tag) );
	memset( &gyroDat, 0, sizeof(itg320_GyroData_t) );

	// --- set proper gains and initial polarities
	// --------------------------------------------
	calParam.gains[0] = 1.0;
	calParam.gains[1] = 1.0;
	calParam.gains[2] = 1.0;
	// ----------------------------
	calParam.polarities[0] = 1.0;
	calParam.polarities[1] = 1.0;
	calParam.polarities[2] = 1.0;
	// ----------------------------
	calParam.factor[0] = calParam.polarities[0] * calParam.gains[0] / 14.375;
	calParam.factor[1] = calParam.polarities[1] * calParam.gains[1] / 14.375;
	calParam.factor[2] = calParam.polarities[2] * calParam.gains[2] / 14.375;


}	// done constructor()

// --------------------------------------------------------------------[ init ]
void ITG3200::init(unsigned int  address)
{
  // Uncomment or change your default ITG3200 initialization
  
  // fast sample rate - divisor = 0 filter = 0 clocksrc = 0, 1, 2, or 3  (raw values)
  init(address, NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_XGYRO_REF, true, true);
  
  // slow sample rate - divisor = 0  filter = 1,2,3,4,5, or 6  clocksrc = 0, 1, 2, or 3  (raw values)
  //init(NOSRDIVIDER, RANGE2000, BW010_SR1, INTERNALOSC, true, true);
  
  // fast sample rate 32Khz external clock - divisor = 0  filter = 0  clocksrc = 4  (raw values)
  //init(NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_EXTERNAL32, true, true);
  
  // slow sample rate 32Khz external clock - divisor = 0  filter = 1,2,3,4,5, or 6  clocksrc = 4  (raw values)
  //init(NOSRDIVIDER, RANGE2000, BW010_SR1, PLL_EXTERNAL32, true, true);

}	// done init()

// --------------------------------------------------------------------[ init ]
void ITG3200::init(	unsigned int address,
					byte _SRateDiv,
					byte _Range,
					byte _filterBW,
					byte _ClockSrc,
					bool _ITGReady,
					bool _INTRawDataReady)
{
	this->devAddr = address;
	// ------------------------------------
	setSampleRateDiv(_SRateDiv);
	setFSRange(_Range);
	setFilterBW(_filterBW);
	setClockSource(_ClockSrc);
	setITGReady(_ITGReady);
	setRawDataReady(_INTRawDataReady);
	// -----------------------------------
	delay( ITG3200_GYROSTART_UP_DELAY );  // startup

}		// done init()

// --------------------------------------------------------------[ getDevAddr ]
byte ITG3200::getDevAddr()
{
	// readFrom(ITG3200_REG_ADDR, 1, &dBuffer[0]);
	// return dBuffer[0];
	// ----------------------------------
	return this->devAddr;

}	// done getDevAddr()

void ITG3200::setDevAddr( const unsigned int  someAddr)
{
  writeTo(ITG3200_REG_ADDR, someAddr);
  this->devAddr = someAddr;

}

byte ITG3200::getSampleRateDiv() {
  readFrom(ITG3200_REG_SMPLRT_DIV, 1, &dBuffer[0]);
  return dBuffer[0];
}

void ITG3200::setSampleRateDiv(byte _SampleRate) {
  writeTo(ITG3200_REG_SMPLRT_DIV, _SampleRate);
}

byte ITG3200::getFSRange() {
  readFrom(ITG3200_REG_DLPF_FS, 1, &dBuffer[0]);
  return ((dBuffer[0] & DLPFFS_FS_SEL) >> 3);
}

void ITG3200::setFSRange(byte _Range) {
  readFrom(ITG3200_REG_DLPF_FS, 1, &dBuffer[0]);
  writeTo(ITG3200_REG_DLPF_FS, ((dBuffer[0] & ~DLPFFS_FS_SEL) | (_Range << 3)) );
}

byte ITG3200::getFilterBW() {  
  readFrom(ITG3200_REG_DLPF_FS, 1, &dBuffer[0]);
  return (dBuffer[0] & DLPFFS_DLPF_CFG);
}

void ITG3200::setFilterBW(byte _BW) {   
  readFrom(ITG3200_REG_DLPF_FS, 1, &dBuffer[0]);
  writeTo(ITG3200_REG_DLPF_FS, ((dBuffer[0] & ~DLPFFS_DLPF_CFG) | _BW));
}

bool ITG3200::isINTActiveOnLow() {  
  readFrom(ITG3200_REG_INT_CFG, 1, &dBuffer[0]);
  return ((dBuffer[0] & INTCFG_ACTL) >> 7);
}

void ITG3200::setINTLogiclvl(bool _State) {
  readFrom(ITG3200_REG_INT_CFG, 1, &dBuffer[0]);
  writeTo(ITG3200_REG_INT_CFG, ((dBuffer[0] & ~INTCFG_ACTL) | (_State << 7)));
}

bool ITG3200::isINTOpenDrain() {  
  readFrom(ITG3200_REG_INT_CFG, 1, &dBuffer[0]);
  return ((dBuffer[0] & INTCFG_OPEN) >> 6);
}

void ITG3200::setINTDriveType(bool _State) {
  readFrom(ITG3200_REG_INT_CFG, 1, &dBuffer[0]);
  writeTo(ITG3200_REG_INT_CFG, ((dBuffer[0] & ~INTCFG_OPEN) | _State << 6));
}

bool ITG3200::isLatchUntilCleared() {    
  readFrom(ITG3200_REG_INT_CFG, 1, &dBuffer[0]);
  return ((dBuffer[0] & INTCFG_LATCH_INT_EN) >> 5);
}

void ITG3200::setLatchMode(bool _State) {
  readFrom(ITG3200_REG_INT_CFG, 1, &dBuffer[0]);
  writeTo(ITG3200_REG_INT_CFG, ((dBuffer[0] & ~INTCFG_LATCH_INT_EN) | _State << 5));
}

bool ITG3200::isAnyRegClrMode() {    
  readFrom(ITG3200_REG_INT_CFG, 1, &dBuffer[0]);
  return ((dBuffer[0] & INTCFG_INT_ANYRD_2CLEAR) >> 4);
}

void ITG3200::setLatchClearMode(bool _State) {
  readFrom(ITG3200_REG_INT_CFG, 1, &dBuffer[0]);
  writeTo(ITG3200_REG_INT_CFG, ((dBuffer[0] & ~INTCFG_INT_ANYRD_2CLEAR) | _State << 4));
}

bool ITG3200::isITGReadyOn() {   
  readFrom(ITG3200_REG_INT_CFG, 1, &dBuffer[0]);
  return ((dBuffer[0] & INTCFG_ITG_RDY_EN) >> 2);
}

void ITG3200::setITGReady(bool _State) {
  readFrom(ITG3200_REG_INT_CFG, 1, &dBuffer[0]);
  writeTo(ITG3200_REG_INT_CFG, ((dBuffer[0] & ~INTCFG_ITG_RDY_EN) | _State << 2));
}

bool ITG3200::isRawDataReadyOn() {
  readFrom(ITG3200_REG_INT_CFG, 1, &dBuffer[0]);
  return (dBuffer[0] & INTCFG_RAW_RDY_EN);
}

void ITG3200::setRawDataReady(bool _State) {
  readFrom(ITG3200_REG_INT_CFG, 1, &dBuffer[0]);
  writeTo(ITG3200_REG_INT_CFG, ((dBuffer[0] & ~INTCFG_RAW_RDY_EN) | _State));
}

bool ITG3200::isITGReady() {
  readFrom(ITG3200_REG_INT_STATUS, 1, &dBuffer[0]);
  return ((dBuffer[0] & INTSTATUS_ITG_RDY) >> 2);
}

bool ITG3200::isRawDataReady() {
  readFrom(ITG3200_REG_INT_STATUS, 1, &dBuffer[0]);
  return (dBuffer[0] & INTSTATUS_RAW_DATA_RDY);
}

// ----------------------------------------------------------------[ readTemp ]
void ITG3200::readTemp( void )
{
	readFrom(ITG3200_REG_TEMP_OUT, 2, dBuffer);
	// -----------------------------------------
	gyroDat.tmp = 35 + (((dBuffer[0] << 8) | dBuffer[1]) + 13200) / 280.0; // F=C*9/5+32

}	// done readTemp()

// ----------------------------------------------------------------[ readTemp ]
void ITG3200::readTemp(float *_Temp)
{
	readTemp();
	// -----------------------------------------
	_Temp[0] = gyroDat.tmp;

}	// done readTemp()

// -------------------------------------------------------------[ readGyroRaw ]
void ITG3200::readGyroRaw( void )
{
	readFrom(ITG3200_REG_GYRO_XOUT, 6, dBuffer);
	// -----------------------------------------
	gyroRaw.d.x = ((dBuffer[0] << 8) | dBuffer[1]);
	gyroRaw.d.y = ((dBuffer[2] << 8) | dBuffer[3]);
	gyroRaw.d.z = ((dBuffer[4] << 8) | dBuffer[5]);

}	// done readGyroRaw()

// -------------------------------------------------------------[ readGyroRaw ]
void ITG3200::readGyroRaw(int *_GyroX, int *_GyroY, int *_GyroZ)
{
	readGyroRaw();
	// -----------------------------------------
	_GyroX[0] = gyroRaw.d.x;
	_GyroY[0] = gyroRaw.d.y;
	_GyroZ[0] = gyroRaw.d.z;

}	// done readGyroRaw()

// -------------------------------------------------------------[ readGyroRaw ]
void ITG3200::readGyroRaw(int *_GyroXYZ)
{
	readGyroRaw( _GyroXYZ, _GyroXYZ+1, _GyroXYZ+2 );

}	// done readGyroRaw()

// -------------------------------------------------------------[ readGyroCal ]
void ITG3200::readGyroCal( void )
{
	readGyroRaw();
	// -----------------------------------
	gyroCal.d.x = gyroRaw.d.x + calParam.offsets[0];
	gyroCal.d.y = gyroRaw.d.y + calParam.offsets[1];
	gyroCal.d.z = gyroRaw.d.z + calParam.offsets[2];

}	// done readGyroCal()

// -------------------------------------------------------------[ readGyroCal ]
void ITG3200::readGyroCal(int *_GyroX, int *_GyroY, int *_GyroZ)
{
	readGyroCal();
	// -----------------------------------
	_GyroX[0] = gyroCal.d.x;
	_GyroY[0] = gyroCal.d.y;
	_GyroZ[0] = gyroCal.d.z;

}	// done readGyroCal()

// -------------------------------------------------------------[ readGyroCal ]
void ITG3200::readGyroCal( int *_GyroXYZ )
{
	readGyroCal( _GyroXYZ, _GyroXYZ+1, _GyroXYZ+2 );

} // done readGyroCal()

// ----------------------------------------------------------[ setRevPolarity ]
void ITG3200::setRevPolarity(bool _Xpol, bool _Ypol, bool _Zpol)
{
	calParam.polarities[0] = _Xpol ? -1.0 : 1.0;
	calParam.polarities[1] = _Ypol ? -1.0 : 1.0;
	calParam.polarities[2] = _Zpol ? -1.0 : 1.0;
	// -------------------------------------------------------------
	calParam.factor[0] = calParam.polarities[0] * calParam.gains[0] / 14.375;
	calParam.factor[1] = calParam.polarities[1] * calParam.gains[1] / 14.375;
	calParam.factor[2] = calParam.polarities[2] * calParam.gains[2] / 14.375;

}	// done setRevPolarity()

// --------------------------------------------------------------[ setOffsets ]
void ITG3200::setGains(float _Xgain, float _Ygain, float _Zgain)
{
	calParam.gains[0] = _Xgain;
	calParam.gains[1] = _Ygain;
	calParam.gains[2] = _Zgain;

}	// done setGains()

// --------------------------------------------------------------[ setOffsets ]
void ITG3200::setOffsets(int _Xoffset, int _Yoffset, int _Zoffset)
{
	calParam.offsets[0] = _Xoffset;
	calParam.offsets[1] = _Yoffset;
	calParam.offsets[2] = _Zoffset;

}	// done setOffsets()

// -----------------------------------------------------------[ zeroCalibrate ]
void ITG3200::zeroCalibrate( const unsigned int nSamples,
							 const unsigned int sampleDelayMS)
{
	float tmpOffsets[] = {0,0,0};
	const float dFactor = 1.0/static_cast<float>(nSamples);

	// --- assuming gyroscope is stationary (updates XYZ offsets for zero)
	// ---------------------------------------------------------------------
	for (int ii = 0; ii < nSamples; ii++)
	{
		delay(sampleDelayMS);

		readGyroRaw();
		// ---------------------
		tmpOffsets[0] += gyroRaw.d.x;
		tmpOffsets[1] += gyroRaw.d.y;
		tmpOffsets[2] += gyroRaw.d.z;
	}
	// ---------------------------------------------------------------------
	calParam.offsets[0] = -static_cast<int>(tmpOffsets[0] * dFactor);
	calParam.offsets[1] = -static_cast<int>(tmpOffsets[1] * dFactor);
	calParam.offsets[2] = -static_cast<int>(tmpOffsets[2] * dFactor);

}	// zeroCalibrate()

// -----------------------------------------------------------------[ getData ]
void ITG3200::getData( void )
{
	// --------------------------
	readTemp();
	// ---------
	delay(5);
	// ---------
	readGyro();
	// ---------
	delay(5);
	// --------------------------
	gyroDat.eng.d.x =  gyroEng.d.x;
	gyroDat.eng.d.y =  gyroEng.d.y;
	gyroDat.eng.d.z =  gyroEng.d.z;
	// --------------------------
	gyroDat.cal.d.x =  gyroCal.d.x;
	gyroDat.cal.d.y =  gyroCal.d.y;
	gyroDat.cal.d.z =  gyroCal.d.z;
	// --------------------------
	gyroDat.raw.d.x =  gyroRaw.d.x;
	gyroDat.raw.d.y =  gyroRaw.d.y;
	gyroDat.raw.d.z =  gyroRaw.d.z;
	// --------------------------

}	// done getData()

// -----------------------------------------------------------------[ getData ]
void ITG3200::getData( itg320_GyroData_t &gData )
{
	// ------------------------------------------------------------------------
	getData();
	// ------------------------------------------------------------------------
	gData.eng.d.x =  gyroDat.eng.d.x;
	gData.eng.d.y =  gyroDat.eng.d.y;
	gData.eng.d.z =  gyroDat.eng.d.z;
	// --------------------------
	gData.cal.d.x =  gyroDat.cal.d.x;
	gData.cal.d.y =  gyroDat.cal.d.y;
	gData.cal.d.z =  gyroDat.cal.d.z;
	// --------------------------
	gData.raw.d.x =  gyroDat.raw.d.x;
	gData.raw.d.y =  gyroDat.raw.d.y;
	gData.raw.d.z =  gyroDat.raw.d.z;
	// --------------------------
	gData.tmp = gyroDat.tmp;

}	// done getData()

// ----------------------------------------------------------------[ readGyro ]
void ITG3200::readGyro( void )
{
	// calibrated integer values from the sensor
	// ------------------------------------------------------------------------
	readGyroCal();
	// ------------------------------------------------------------------------
	gyroEng.d.x =  calParam.factor[0] * static_cast<float>(gyroCal.d.x);
	gyroEng.d.y =  calParam.factor[1] * static_cast<float>(gyroCal.d.y);
	gyroEng.d.z =  calParam.factor[2] * static_cast<float>(gyroCal.d.z);

}	// done readGyro()

// ----------------------------------------------------------------[ readGyro ]
void ITG3200::readGyro(float *_GyroX, float *_GyroY, float *_GyroZ)
{
	// calibrated integer values from the sensor
	// ------------------------------------------------------------------------
	readGyro();
	// ------------------------------------------------------------------------
	_GyroX[0] =  gyroEng.d.x;
	_GyroY[0] =  gyroEng.d.y;
	_GyroZ[0] =  gyroEng.d.z;

}	// done readGyro()

// ----------------------------------------------------------------[ readGyro ]
void ITG3200::readGyro(float *_GyroXYZ)
{
	readGyro( _GyroXYZ, _GyroXYZ+1, _GyroXYZ+2 );

}	// readGyro()

// -------------------------------------------------------------------[ reset ]
void ITG3200::reset( void )
{
  writeTo( ITG3200_REG_PWR_MGM, PWRMGM_HRESET);

  delay( ITG3200_GYROSTART_UP_DELAY ); //  startup

}	// done reset()

bool ITG3200::isLowPower()
{
  readFrom(ITG3200_REG_PWR_MGM, 1, &dBuffer[0]);
  return (dBuffer[0] & PWRMGM_SLEEP) >> 6;

}
  
void ITG3200::setPowerMode(bool _State) {
  readFrom(ITG3200_REG_PWR_MGM, 1, &dBuffer[0]);
  writeTo(ITG3200_REG_PWR_MGM, ((dBuffer[0] & ~PWRMGM_SLEEP) | _State << 6));
}

bool ITG3200::isXgyroStandby()
{
  readFrom(ITG3200_REG_PWR_MGM, 1, &dBuffer[0]);
  return (dBuffer[0] & PWRMGM_STBY_XG) >> 5;
}

bool ITG3200::isYgyroStandby() {
  readFrom(ITG3200_REG_PWR_MGM, 1, &dBuffer[0]);
  return (dBuffer[0] & PWRMGM_STBY_YG) >> 4;
}

bool ITG3200::isZgyroStandby() {
  readFrom(ITG3200_REG_PWR_MGM, 1, &dBuffer[0]);
  return (dBuffer[0] & PWRMGM_STBY_ZG) >> 3;
}

void ITG3200::setXgyroStandby(bool _Status)
{
  readFrom(ITG3200_REG_PWR_MGM, 1, &dBuffer[0]);
  writeTo(ITG3200_REG_PWR_MGM, ((dBuffer[0] & PWRMGM_STBY_XG) | _Status << 5));
}

void ITG3200::setYgyroStandby(bool _Status)
{
  readFrom(ITG3200_REG_PWR_MGM, 1, &dBuffer[0]);
  writeTo(ITG3200_REG_PWR_MGM, ((dBuffer[0] & PWRMGM_STBY_YG) | _Status << 4));
}

void ITG3200::setZgyroStandby(bool _Status)
{
  readFrom(ITG3200_REG_PWR_MGM, 1, &dBuffer[0]);
  writeTo(ITG3200_REG_PWR_MGM, ((dBuffer[0] & PWRMGM_STBY_ZG) | _Status << 3));
}

byte ITG3200::getClockSource()
{
  readFrom(ITG3200_REG_PWR_MGM, 1, &dBuffer[0]);
  return (dBuffer[0] & PWRMGM_CLK_SEL);
}

void ITG3200::setClockSource(byte _CLKsource)
{
  readFrom(ITG3200_REG_PWR_MGM, 1, &dBuffer[0]);
  writeTo(ITG3200_REG_PWR_MGM, ((dBuffer[0] & ~PWRMGM_CLK_SEL) | _CLKsource));
}

// ----------------------------------------------------------------[ writeTo ]
void ITG3200::writeTo( const uint8_t iAddress, const uint8_t iVal)
{
	// Writes val to address register on device
	// -----------------------------------------------------------------------
	Wire.beginTransmission(this->devAddr); 	// start transmission to device
	Wire.write( iAddress );             	// send register address
	Wire.write( iVal );                 	// send value to write
	Wire.endTransmission();         		// end transmission

}	// done writeTo()

// ----------------------------------------------------------------[ readFrom ]
void ITG3200::readFrom( const uint8_t iAddress,
						const uint8_t num,
						uint8_t someBuffer[] )
{
	// Reads num bytes starting from address register
	// on device in to dBuffer array

	Wire.beginTransmission(this->devAddr);  // start transmission to device
	Wire.write( iAddress );             	// sends address to read from
	Wire.endTransmission();         		// end transmission

	Wire.beginTransmission(this->devAddr);	// start transmission to device
	Wire.requestFrom(this->devAddr, num);	// request 6 bytes from device

	int ki = 0;
	// ------------------------------------------------
	while( Wire.available() )
	{
		// device may send less than requested (abnormal)
		someBuffer[ki] = Wire.read();
		ki++;
	}
	// ------------------------------------------------
	Wire.endTransmission();         // end transmission

}	// done readFrom()

// ---------------------------------------------------------------[ printData ]
void ITG3200::printData( void )
{
	Serial.print("Raw:\t");
	// -----------------------------
	Serial.print( gyroDat.raw.d.x);
	Serial.print("   ");
	Serial.print( gyroDat.raw.d.y);
	Serial.print("   ");
	Serial.print( gyroDat.raw.d.z);

	Serial.print("   \tCalibrated:\t");
	// -----------------------------
	Serial.print( gyroDat.cal.d.x);
	Serial.print("   ");
	Serial.print( gyroDat.cal.d.y);
	Serial.print("   ");
	Serial.print( gyroDat.cal.d.z);

	Serial.print("   \tEng:\t");
	// -----------------------------
	Serial.print( gyroDat.eng.d.x);
	Serial.print("   ");
	Serial.print( gyroDat.eng.d.y);
	Serial.print("   ");
	Serial.print( gyroDat.eng.d.z);

	Serial.print("   \tFactor:\t");
	// -----------------------------
	Serial.print( calParam.factor[0] );
	Serial.print("   ");
	Serial.print( calParam.factor[1]);
	Serial.print("   ");
	Serial.print( calParam.factor[2]);


	Serial.print("   \tTmp:\t");
	// -----------------------------
	Serial.println( gyroDat.tmp);

}	// done printData()


