/**************************************************************************
 *                                                                         *
 * ADXL345 Driver for Arduino                                              *
 *                                                                         *
 ***************************************************************************
 *                                                                         * 
 * This program is free software; you can redistribute it and/or modify    *
 * it under the terms of the GNU License.                                  *
 * This program is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 * GNU License V2 for more details.                                        *
 *                                                                         *
 ***************************************************************************/

#include "ADXL345.h"
#include <Wire.h>



// -------------------------------------------------------------[ constructor ]
ADXL345::ADXL345()
{
	status 	 	= ADXL345_ERROR_OK;
	error_code	= ADXL345_ERROR_NONE;

	gains[0]	= 0.00376390;
	gains[1] 	= 0.00376009;
	gains[2] 	= 0.00349265;
}

// --------------------------------------------------------------------[ init ]
void ADXL345::init( const int iAddress )
{
	this->devAddr = iAddress;
	// ----------------------
	powerOn();
}

// -----------------------------------------------------------------[ powerOn ]
void ADXL345::powerOn( void )
{
	// --- turning on the ADXL345
	//writeTo( ADXL345_REG_POWER_CTL, 0);
	//writeTo( ADXL345_REG_POWER_CTL, 16);
	// ------------------------------------------------
	writeTo( ADXL345_REG_POWER_CTL, 8);
	// ------------------------------------------------

}	// done powerOn()

// Reads the acceleration into three variable x, y and z
// ---------------------------------------------------------------[ readAccel ]
void ADXL345::readAccel( void )
{
	//read the acceleration data from the ADXL345
	// ----------------------------------------------------------------
	readFrom( ADXL345_REG_DATAX0, ADXL345_BytesToRead, this->dBuffer);
	// ----------------------------------------------------------------

	//  each axis reading comes in 10 bit resolution, ie 2 bytes.
	//	NOTE: Least Significat Byte first!!
	//  thus we are converting both bytes in to one int
	// -----------------------------------------------------------------------
	this->accData.rawDD.x = (((int)this->dBuffer[1]) << 8) | this->dBuffer[0];
	this->accData.rawDD.y = (((int)this->dBuffer[3]) << 8) | this->dBuffer[2];
	this->accData.rawDD.z = (((int)this->dBuffer[5]) << 8) | this->dBuffer[4];
	// -----------------------------------------------------------------------
	this->accData.dd.x = this->accData.rawDD.x * gains[0];
	this->accData.dd.y = this->accData.rawDD.y * gains[1];
	this->accData.dd.z = this->accData.rawDD.z * gains[2];
	// -----------------------------------------------------------------------

}	// done readAccel()

// ----------------------------------------------------------------[ getAccel ]
void ADXL345::getData( adxl345_data_t &iAcc )
{
	readAccel();
	// ------------------------------------------------
	iAcc.rawDD.x = this->accData.rawDD.x;
	iAcc.rawDD.y = this->accData.rawDD.y;
	iAcc.rawDD.z = this->accData.rawDD.z;
	// ------------------------------------------------
	iAcc.dd.x = this->accData.dd.x;
	iAcc.dd.y = this->accData.dd.y;
	iAcc.dd.z = this->accData.dd.z;
	// ------------------------------------------------

}	// done getAccel()

// ----------------------------------------------------------------[ writeTo ]
void ADXL345::writeTo( const byte iAddress, const byte iVal)
{
	// Writes val to address register on device
	// -----------------------------------------------------------------------
	Wire.beginTransmission(this->devAddr); 	// start transmission to device
	Wire.write( iAddress );             	// send register address
	Wire.write( iVal );                 	// send value to write
	Wire.endTransmission();         		// end transmission

}	// done writeTo()

// ----------------------------------------------------------------[ readFrom ]
void ADXL345::readFrom( const byte iAddress, const int num, byte someBuffer[] )
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
	if( ki != num)
	{
		status 	   = ADXL345_ERROR;
		error_code = ADXL345_ERROR_READ;
	}
	// ------------------------------------------------
	Wire.endTransmission();         // end transmission

}	// done readFrom()




// gets the state of the SELF_TEST bit
bool ADXL345::getSelfTestBit()
{
  return getRegisterBit(ADXL345_REG_DATA_FORMAT, 7);
}

// Sets the SELF-TEST bit
// if set to 1 it applies a self-test force to the sensor causing a shift in the output data
// if set to 0 it disables the self-test force
void ADXL345::setSelfTestBit(bool selfTestBit)
{
  setRegisterBit(ADXL345_REG_DATA_FORMAT, 7, selfTestBit);
}

// Gets the state of the SPI bit
bool ADXL345::getSpiBit()
{
  return getRegisterBit(ADXL345_REG_DATA_FORMAT, 6);
}

// Sets the SPI bit
// if set to 1 it sets the device to 3-wire mode
// if set to 0 it sets the device to 4-wire SPI mode
void ADXL345::setSpiBit(bool spiBit)
{
  setRegisterBit(ADXL345_REG_DATA_FORMAT, 6, spiBit);
}

// Gets the state of the INT_INVERT bit
bool ADXL345::getInterruptLevelBit()
{
  return getRegisterBit(ADXL345_REG_DATA_FORMAT, 5);
}

// Sets the INT_INVERT bit
// if set to 0 sets the interrupts to active high
// if set to 1 sets the interrupts to active low
void ADXL345::setInterruptLevelBit(bool interruptLevelBit)
{
  setRegisterBit(ADXL345_REG_DATA_FORMAT, 5, interruptLevelBit);
}

// Gets the state of the FULL_RES bit
bool ADXL345::getFullResBit()
{
  return getRegisterBit(ADXL345_REG_DATA_FORMAT, 3);
}

// Sets the FULL_RES bit
// if set to 1, the device is in full resolution mode, where the output resolution increases with the
//   g range set by the range bits to maintain a 4mg/LSB scal factor
// if set to 0, the device is in 10-bit mode, and the range buts determine the maximum g range
//   and scale factor
void ADXL345::setFullResBit(bool fullResBit)
{
  setRegisterBit(ADXL345_REG_DATA_FORMAT, 3, fullResBit);
}

// Gets the state of the justify bit
bool ADXL345::getJustifyBit()
{
  return getRegisterBit(ADXL345_REG_DATA_FORMAT, 2);
}


// -----------------------------------------------------------[ setJustifyBit ]
void ADXL345::setJustifyBit(bool justifyBit)
{
	// Sets the JUSTIFY bit
	// if sets to 1 selects the left justified mode
	// if sets to 0 selects right justified mode with sign extension

	setRegisterBit(ADXL345_REG_DATA_FORMAT, 2, justifyBit);

}	// done setJustifyBit()


// ---------------------------------------------------------[ setTapThreshold ]
void ADXL345::setTapThreshold(int tapThreshold)
{
	// Sets the THRESH_TAP byte value
	// it should be between 0 and 255
	// the scale factor is 62.5 mg/LSB
	// A value of 0 may result in undesirable behavior

	tapThreshold = min(max(tapThreshold,0),255);
	byte someByte = byte (tapThreshold);
	writeTo(ADXL345_REG_THRESH_TAP, someByte);

}	// done setTapThreshold()

// Gets the THRESH_TAP byte value
// return value is comprised between 0 and 255
// the scale factor is 62.5 mg/LSB
int ADXL345::getTapThreshold()
{
  byte someByte;
  readFrom(ADXL345_REG_THRESH_TAP, 1, &someByte);
  return int (someByte);
}

// set/get the gain for each axis in Gs / count
void ADXL345::setAxisGains(float *_gains)
{
  int i;
  for(i = 0; i < 3; i++)
  {
    gains[i] = _gains[i];
  }
}
void ADXL345::getAxisGains(float *_gains)
{
  int i;
  for(i = 0; i < 3; i++)
  {
    _gains[i] = gains[i];
  }
}


// Sets the OFSX, OFSY and OFSZ bytes
// OFSX, OFSY and OFSZ are user offset adjustments in twos complement format with
// a scale factor of 15,6mg/LSB
// OFSX, OFSY and OFSZ should be comprised between
void ADXL345::setAxisOffset(int x, int y, int z)
{
	writeTo(ADXL345_REG_OFSX, byte (x));
	writeTo(ADXL345_REG_OFSY, byte (y));
	writeTo(ADXL345_REG_OFSZ, byte (z));
}

// Gets the OFSX, OFSY and OFSZ bytes
void ADXL345::getAxisOffset(int* x, int* y, int*z)
{
	byte someByte;
	readFrom(ADXL345_REG_OFSX, 1, &someByte);
	*x = int (someByte);
	readFrom(ADXL345_REG_OFSY, 1, &someByte);
	*y = int (someByte);
	readFrom(ADXL345_REG_OFSZ, 1, &someByte);
	*z = int (someByte);
}

// Sets the DUR byte
// The DUR byte contains an unsigned time value representing the maximum time
// that an event must be above THRESH_TAP threshold to qualify as a tap event
// The scale factor is 625microsec/LSB
// A value of 0 disables the tap/float tap funcitons. Max value is 255.
void ADXL345::setTapDuration(int tapDuration)
{
  tapDuration = min(max(tapDuration,0),255);
  byte someByte = byte (tapDuration);
  writeTo(ADXL345_REG_DUR, someByte);
}

// Gets the DUR byte
int ADXL345::getTapDuration()
{
  byte someByte;
  readFrom(ADXL345_REG_DUR, 1, &someByte);
  return int (someByte);
}

// Sets the latency (latent register) which contains an unsigned time value
// representing the wait time from the detection of a tap event to the start
// of the time window, during which a possible second tap can be detected.
// The scale factor is 1.25ms/LSB. A value of 0 disables the float tap function.
// It accepts a maximum value of 255.
void ADXL345::setDoubleTapLatency(int floatTapLatency)
{
  byte someByte = byte (floatTapLatency);
  writeTo(ADXL345_REG_LATENT, someByte);
}

// Gets the Latent value
int ADXL345::getDoubleTapLatency()
{
  byte someByte;
  readFrom(ADXL345_REG_LATENT, 1, &someByte);
  return int (someByte);
}

// Sets the Window register, which contains an unsigned time value representing
// the amount of time after the expiration of the latency time (Latent register)
// during which a second valud tap can begin. The scale factor is 1.25ms/LSB. A
// value of 0 disables the float tap function. The maximum value is 255.
void ADXL345::setDoubleTapWindow(int floatTapWindow)
{
  floatTapWindow = min(max(floatTapWindow,0),255);
  byte someByte = byte (floatTapWindow);
  writeTo(ADXL345_REG_WINDOW, someByte);
}

// Gets the Window register
int ADXL345::getDoubleTapWindow()
{
  byte someByte;
  readFrom(ADXL345_REG_WINDOW, 1, &someByte);
  return int (someByte);
}

// Sets the THRESH_ACT byte which holds the threshold value for detecting activity.
// The data format is unsigned, so the magnitude of the activity event is compared
// with the value is compared with the value in the THRESH_ACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
// activity interrupt is enabled. The maximum value is 255.
void ADXL345::setActivityThreshold(int activityThreshold)
{
  activityThreshold = min(max(activityThreshold,0),255);
  byte someByte = byte (activityThreshold);
  writeTo(ADXL345_REG_THRESH_ACT, someByte);
}

// Gets the THRESH_ACT byte
int ADXL345::getActivityThreshold()
{
  byte someByte;
  readFrom(ADXL345_REG_THRESH_ACT, 1, &someByte);
  return int (someByte);
}

// Sets the THRESH_INACT byte which holds the threshold value for detecting inactivity.
// The data format is unsigned, so the magnitude of the inactivity event is compared
// with the value is compared with the value in the THRESH_INACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
// inactivity interrupt is enabled. The maximum value is 255.
void ADXL345::setInactivityThreshold(int inactivityThreshold)
{
  inactivityThreshold = min(max(inactivityThreshold,0),255);
  byte someByte = byte (inactivityThreshold);
  writeTo(ADXL345_REG_THRESH_INACT, someByte);
}

// Gets the THRESH_INACT byte
int ADXL345::getInactivityThreshold()
{
  byte someByte;
  readFrom(ADXL345_REG_THRESH_INACT, 1, &someByte);
  return int (someByte);
}

// Sets the TIME_INACT register, which contains an unsigned time value representing the
// amount of time that acceleration must be less thant the value in the THRESH_INACT
// register for inactivity to be declared. The scale factor is 1sec/LSB. The value must
// be between 0 and 255.
void ADXL345::setTimeInactivity(int timeInactivity)
{
  timeInactivity = min(max(timeInactivity,0),255);
  byte someByte = byte (timeInactivity);
  writeTo(ADXL345_REG_TIME_INACT, someByte);
}

// Gets the TIME_INACT register
int ADXL345::getTimeInactivity()
{
  byte someByte;
  readFrom(ADXL345_REG_TIME_INACT, 1, &someByte);
  return int (someByte);
}

// Sets the THRESH_FF register which holds the threshold value, in an unsigned format, for
// free-fall detection. The root-sum-square (RSS) value of all axes is calculated and
// compared whith the value in THRESH_FF to determine if a free-fall event occured. The
// scale factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the free-fall
// interrupt is enabled. The maximum value is 255.
void ADXL345::setFreeFallThreshold(int freeFallThreshold)
{
  freeFallThreshold = min(max(freeFallThreshold,0),255);
  byte someByte = byte (freeFallThreshold);
  writeTo(ADXL345_REG_THRESH_FF, someByte);
}

// Gets the THRESH_FF register.
int ADXL345::getFreeFallThreshold()
{
  byte someByte;
  readFrom(ADXL345_REG_THRESH_FF, 1, &someByte);
  return int (someByte);
}

// Sets the TIME_FF register, which holds an unsigned time value representing the minimum
// time that the RSS value of all axes must be less than THRESH_FF to generate a free-fall
// interrupt. The scale factor is 5ms/LSB. A value of 0 may result in undesirable behavior if
// the free-fall interrupt is enabled. The maximum value is 255.
void ADXL345::setFreeFallDuration(int freeFallDuration)
{
  freeFallDuration = min(max(freeFallDuration,0),255);
  byte someByte = byte (freeFallDuration);
  writeTo(ADXL345_REG_TIME_FF, someByte);
}

// Gets the TIME_FF register.
int ADXL345::getFreeFallDuration()
{
  byte someByte;
  readFrom(ADXL345_REG_TIME_FF, 1, &someByte);
  return int (someByte);
}

bool ADXL345::isActivityXEnabled()
{
  return getRegisterBit(ADXL345_REG_ACT_INACT_CTL, 6);
}
bool ADXL345::isActivityYEnabled()
{
  return getRegisterBit(ADXL345_REG_ACT_INACT_CTL, 5);
}
bool ADXL345::isActivityZEnabled()
{
  return getRegisterBit(ADXL345_REG_ACT_INACT_CTL, 4);
}
bool ADXL345::isInactivityXEnabled()
{
  return getRegisterBit(ADXL345_REG_ACT_INACT_CTL, 2);
}
bool ADXL345::isInactivityYEnabled()
{
  return getRegisterBit(ADXL345_REG_ACT_INACT_CTL, 1);
}
bool ADXL345::isInactivityZEnabled()
{
  return getRegisterBit(ADXL345_REG_ACT_INACT_CTL, 0);
}

void ADXL345::setActivityX(bool state)
{
  setRegisterBit(ADXL345_REG_ACT_INACT_CTL, 6, state);
}
void ADXL345::setActivityY(bool state)
{
  setRegisterBit(ADXL345_REG_ACT_INACT_CTL, 5, state);
}
void ADXL345::setActivityZ(bool state)
{
  setRegisterBit(ADXL345_REG_ACT_INACT_CTL, 4, state);
}
void ADXL345::setInactivityX(bool state)
{
  setRegisterBit(ADXL345_REG_ACT_INACT_CTL, 2, state);
}
void ADXL345::setInactivityY(bool state)
{
  setRegisterBit(ADXL345_REG_ACT_INACT_CTL, 1, state);
}
void ADXL345::setInactivityZ(bool state)
{
  setRegisterBit(ADXL345_REG_ACT_INACT_CTL, 0, state);
}

bool ADXL345::isActivityAc()
{
  return getRegisterBit(ADXL345_REG_ACT_INACT_CTL, 7);
}
bool ADXL345::isInactivityAc()
{
  return getRegisterBit(ADXL345_REG_ACT_INACT_CTL, 3);
}

void ADXL345::setActivityAc(bool state)
{
  setRegisterBit(ADXL345_REG_ACT_INACT_CTL, 7, state);
}
void ADXL345::setInactivityAc(bool state)
{
  setRegisterBit(ADXL345_REG_ACT_INACT_CTL, 3, state);
}

bool ADXL345::getSuppressBit()
{
  return getRegisterBit(ADXL345_REG_TAP_AXES, 3);
}
void ADXL345::setSuppressBit(bool state)
{
  setRegisterBit(ADXL345_REG_TAP_AXES, 3, state);
}

bool ADXL345::isTapDetectionOnX()
{
  return getRegisterBit(ADXL345_REG_TAP_AXES, 2);
}
void ADXL345::setTapDetectionOnX(bool state)
{
  setRegisterBit(ADXL345_REG_TAP_AXES, 2, state);
}
bool ADXL345::isTapDetectionOnY()
{
  return getRegisterBit(ADXL345_REG_TAP_AXES, 1);
}
void ADXL345::setTapDetectionOnY(bool state)
{
  setRegisterBit(ADXL345_REG_TAP_AXES, 1, state);
}
bool ADXL345::isTapDetectionOnZ()
{
  return getRegisterBit(ADXL345_REG_TAP_AXES, 0);
}
void ADXL345::setTapDetectionOnZ(bool state)
{
  setRegisterBit(ADXL345_REG_TAP_AXES, 0, state);
}

bool ADXL345::isActivitySourceOnX()
{
  return getRegisterBit(ADXL345_REG_ACT_TAP_STATUS, 6);
}
bool ADXL345::isActivitySourceOnY()
{
  return getRegisterBit(ADXL345_REG_ACT_TAP_STATUS, 5);
}
bool ADXL345::isActivitySourceOnZ()
{
  return getRegisterBit(ADXL345_REG_ACT_TAP_STATUS, 4);
}

bool ADXL345::isTapSourceOnX()
{
  return getRegisterBit(ADXL345_REG_ACT_TAP_STATUS, 2);
}
bool ADXL345::isTapSourceOnY()
{
  return getRegisterBit(ADXL345_REG_ACT_TAP_STATUS, 1);
}
bool ADXL345::isTapSourceOnZ()
{
  return getRegisterBit(ADXL345_REG_ACT_TAP_STATUS, 0);
}

bool ADXL345::isAsleep()
{
  return getRegisterBit(ADXL345_REG_ACT_TAP_STATUS, 3);
}

bool ADXL345::isLowPower()
{
  return getRegisterBit(ADXL345_REG_BW_RATE, 4);
}
void ADXL345::setLowPower(bool state)
{
  setRegisterBit(ADXL345_REG_BW_RATE, 4, state);
}

float ADXL345::getRate()
{
  byte someByte;
  readFrom(ADXL345_REG_BW_RATE, 1, &someByte);
  someByte &= B00001111;
  return (pow(2,((int) someByte)-6)) * 6.25;
}

void ADXL345::setRate(float rate)
{
  byte someByte,_s;
  int v = (int) (rate / 6.25);
  int r = 0;
  while (v >>= 1)
  {
    r++;
  }
  if (r <= 9)
  {
    readFrom(ADXL345_REG_BW_RATE, 1, &someByte);
    _s = (byte) (r + 6) | (someByte & B11110000);
    writeTo(ADXL345_REG_BW_RATE, _s);
  }
}

void ADXL345::set_bw(byte bw_code)
{
  if((bw_code < ADXL345_BW_3) || (bw_code > ADXL345_BW_1600))
  {
    status = false;
    error_code = ADXL345_ERROR_BAD_ARG;
  }
  else
  {
    writeTo(ADXL345_REG_BW_RATE, bw_code);
  }
}

byte ADXL345::get_bw_code()
{
  byte bw_code;
  readFrom(ADXL345_REG_BW_RATE, 1, &bw_code);
  return bw_code;
}

byte ADXL345::getInterruptSource()
{
  byte someByte;
  readFrom(ADXL345_REG_INT_SOURCE, 1, &someByte);
  return someByte;
}

bool ADXL345::getInterruptSource(byte interruptBit)
{
  return getRegisterBit(ADXL345_REG_INT_SOURCE,interruptBit);
}

bool ADXL345::getInterruptMapping(byte interruptBit)
{
  return getRegisterBit(ADXL345_REG_INT_MAP,interruptBit);
}

// Set the mapping of an interrupt to pin1 or pin2
// eg: setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT,ADXL345_INT2_PIN);
void ADXL345::setInterruptMapping(byte interruptBit, bool interruptPin)
{
  setRegisterBit(ADXL345_REG_INT_MAP, interruptBit, interruptPin);
}

bool ADXL345::isInterruptEnabled(byte interruptBit)
{
  return getRegisterBit(ADXL345_REG_INT_ENABLE,interruptBit);
}

void ADXL345::setInterrupt(byte interruptBit, bool state)
{
  setRegisterBit(ADXL345_REG_INT_ENABLE, interruptBit, state);
}

// --------------------------------------------------------[ setRegisterBit ]
void ADXL345::setRegisterBit( byte regAdress, int bitPos, bool state)
{
  byte someByte;

  readFrom(regAdress, 1, &someByte);

  if (state)
  {
    someByte |= (1 << bitPos);  // forces nth bit of someByte to be 1.  all other bits left alone.
  }
  else
  {
    someByte &= ~(1 << bitPos); // forces nth bit of someByte to be 0.  all other bits left alone.
  }

  writeTo(regAdress, someByte);

}	// done setRegisterBit()

// ----------------------------------------------------------[ getRegisterBit ]
bool ADXL345::getRegisterBit( const byte regAdress, const int bitPos)
{
  byte someByte;
  readFrom(regAdress, 1, &someByte);
  return ((someByte >> bitPos) & 1);

}	// done getRegisterBit()

// ----------------------------------------------------------------[ readFrom ]
void ADXL345::getRangeSetting( byte &rangeSetting)
{
	// Gets the range setting and return it into rangeSetting
	// it can be 2, 4, 8 or 16
	byte someByte;
	// ------------------------------------------------
	readFrom( ADXL345_REG_DATA_FORMAT, 1, &someByte);
	// ------------------------------------------------
	rangeSetting = someByte & B00000011;

}	// done getRangeSetting()

// ---------------------------------------------------------[ setRangeSetting ]
void ADXL345::setRangeSetting( const int val)
{
	// Sets the range setting, possible values are: 2, 4, 8, 16

	byte _s;
	byte someByte;
	// ------------------------------------------------
	switch (val)
	{	default:
		case 2:
			_s = B00000000;
			break;
		case 4:
			_s = B00000001;
			break;
		case 8:
			_s = B00000010;
			break;
		case 16:
			_s = B00000011;
			break;
	}
	// ------------------------------------------------
	readFrom( ADXL345_REG_DATA_FORMAT, 1, &someByte);
	// ------------------------------------------------
	_s |= (someByte & B11101100);
	// ------------------------------------------------
	writeTo( ADXL345_REG_DATA_FORMAT, _s);

}	// done setRangeSetting()

// --------------------------------------------------------[ printAllRegister ]
void ADXL345::printAllRegister()
{
	// print all register value to the serial output, which requires the port
	// to be setup. this can be used to manually to check the current
	// configuration of the device

	byte someByte;

	Serial.print("0x00: ");
	readFrom( 0x00, 1, &someByte);
	printByte( someByte );
	Serial.println("");

	for (int ii=29; ii<=57; ii++)
	{
		Serial.print("0x");
		Serial.print(ii, HEX);
		Serial.print(": ");
		readFrom(ii, 1, &someByte);
		printByte(someByte);
		Serial.println("");
	}

}	// done printAllRegister()

// --------------------------------------------------------------[ printByte ]
void ADXL345::printByte(byte val)
{
  int i;
  Serial.print("B");
  for(i=7; i>=0; i--)
  {
    Serial.print(val >> i & 1, BIN);
  }

} // done printByte()

// ---------------------------------------------------------------[ printData ]
void ADXL345::printData( void )
{
	Serial.print("Raw:\t");
	// -----------------------------
	Serial.print( accData.rawDD.x);
	Serial.print("   ");
	Serial.print( accData.rawDD.y);
	Serial.print("   ");
	Serial.print( accData.rawDD.z);
	Serial.print("   \tScaled:\t");

	Serial.print("   \tScaled:\t");
	// -----------------------------
	Serial.print( accData.dd.x);
	Serial.print("   ");
	Serial.print( accData.dd.y);
	Serial.print("   ");
	Serial.println( accData.dd.z);




}	// done printData()


// ---------------------------------------------------------------------[ EoF ]


