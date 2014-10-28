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
// ---------------------------------------------------------------------[ BoF ]
#ifndef FIMU_ADXL345_h
#define FIMU_ADXL345_h

#include "Arduino.h"

#include "ADLX345_inc.h"






/* 
 Interrupt PINs
 INT1: 0
 INT2: 1
 */
#define ADXL345_INT1_PIN 0x00
#define ADXL345_INT2_PIN 0x01

/* 
 Interrupt bit position
 */
#define ADXL345_INT_DATA_READY_BIT 0x07
#define ADXL345_INT_SINGLE_TAP_BIT 0x06
#define ADXL345_INT_DOUBLE_TAP_BIT 0x05
#define ADXL345_INT_ACTIVITY_BIT   0x04
#define ADXL345_INT_INACTIVITY_BIT 0x03
#define ADXL345_INT_FREE_FALL_BIT  0x02
#define ADXL345_INT_WATERMARK_BIT  0x01
#define ADXL345_INT_OVERRUNY_BIT   0x00



class ADXL345
{
public:

	ADXL345();


	void init( const int );
	void printData( void );




	void powerOn();
	void getData( adxl345_data_t & );

	void setTapThreshold(int tapThreshold);
	int getTapThreshold();
	void setAxisGains(float *_gains);
	void getAxisGains(float *_gains);
	void setAxisOffset(int x, int y, int z);
	void getAxisOffset(int* x, int* y, int*z);
	void setTapDuration(int tapDuration);
	int getTapDuration();
	void setDoubleTapLatency(int floatTapLatency);
	int getDoubleTapLatency();
	void setDoubleTapWindow(int floatTapWindow);
	int getDoubleTapWindow();
	void setActivityThreshold(int activityThreshold);
	int getActivityThreshold();
	void setInactivityThreshold(int inactivityThreshold);
	int getInactivityThreshold();
	void setTimeInactivity(int timeInactivity);
	int getTimeInactivity();
	void setFreeFallThreshold(int freeFallthreshold);
	int getFreeFallThreshold();
	void setFreeFallDuration(int freeFallDuration);
	int getFreeFallDuration();

	bool isActivityXEnabled();
	bool isActivityYEnabled();
	bool isActivityZEnabled();
	bool isInactivityXEnabled();
	bool isInactivityYEnabled();
	bool isInactivityZEnabled();
	bool isActivityAc();
	bool isInactivityAc();
	void setActivityAc(bool state);
	void setInactivityAc(bool state);

	bool getSuppressBit();
	void setSuppressBit(bool state);
	bool isTapDetectionOnX();
	void setTapDetectionOnX(bool state);
	bool isTapDetectionOnY();
	void setTapDetectionOnY(bool state);
	bool isTapDetectionOnZ();
	void setTapDetectionOnZ(bool state);

	void setActivityX(bool state);
	void setActivityY(bool state);
	void setActivityZ(bool state);
	void setInactivityX(bool state);
	void setInactivityY(bool state);
	void setInactivityZ(bool state);

	bool isActivitySourceOnX();
	bool isActivitySourceOnY();
	bool isActivitySourceOnZ();
	bool isTapSourceOnX();
	bool isTapSourceOnY();
	bool isTapSourceOnZ();
	bool isAsleep();

	bool isLowPower();
	void setLowPower(bool state);
	float getRate();
	void setRate(float rate);
	void set_bw(byte bw_code);
	byte get_bw_code();

	byte getInterruptSource();
	bool getInterruptSource(byte interruptBit);
	bool getInterruptMapping(byte interruptBit);
	void setInterruptMapping(byte interruptBit, bool interruptPin);
	bool isInterruptEnabled(byte interruptBit);
	void setInterrupt(byte interruptBit, bool state);

	bool getSelfTestBit();
	void setSelfTestBit(bool selfTestBit);
	bool getSpiBit();
	void setSpiBit(bool spiBit);
	bool getInterruptLevelBit();
	void setInterruptLevelBit(bool interruptLevelBit);
	bool getFullResBit();
	void setFullResBit(bool fullResBit);
	bool getJustifyBit();
	void setJustifyBit(bool justifyBit);

	void printAllRegister();

protected:

	bool  status;           // set when error occurs
						 // see error code for details
	byte  error_code;       // Initial state
	float gains[3];        // counts to Gs

private:

	void readAccel( void );

	void writeTo(  const byte, const byte );
	void readFrom( const byte, const int, byte buff[] );

	void getRangeSetting( byte &);
	void setRangeSetting( const int );


	void printByte(byte val);

	void setRegisterBit(byte regAdress, int bitPos, bool state);
	bool getRegisterBit(byte regAdress, int bitPos);

	byte dBuffer[6] ;    //6 bytes buffer for saving data read from the device
	int  devAddr;

	// ----
	adxl345_data_t	accData;

};



#endif
