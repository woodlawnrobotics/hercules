/*
 * RTHerculesRobot.h
 *
 *  Created on: Sep 9, 2014
 *      Author: simon
 */
//
//  moto_4wd.h
//  2012 Copyright (c) Seeed Technology Inc.  All right reserved.
//
//  Author: Loovee
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
// ---------------------------------------------------------------------[ BoF ]
#ifndef RTHERCULESROBOT_H_
#define RTHERCULESROBOT_H_

#include <Arduino.h>


#include "RTHerculesRobot_def.h"

namespace RT_HERCULES_ROBOT
{

class RTHerculesRobot
{
public:
	RTHerculesRobot( void );
	~RTHerculesRobot( void );

	void init( void );

    void setRunLeft(  void );
    void setRunRight( void );

    void setStopLeft( void );
    void setStopRight( void );

    void setSpeedLeft(  uint8_t );      // pwm, 0-100
    void setSpeedRight( uint8_t );      // pwm, 0-100

    void setDirLeft( const uint8_t );
    void setDirRight( const uint8_t );

    void setSpeedDir(      uint8_t, uint8_t );
    void setSpeedDirLeft(  uint8_t, uint8_t );
    void setSpeedDirRight( uint8_t, uint8_t );
    void stop( void );
    // --------------------------------------------------------
    void pwmInit(       void );
    void pwmSetPwm(     char , int  );     // Hz
    bool pwmSetPwmDuty( char , int );        // duty: 0-100 %
    void pwmDisablePwm( char );              // pin = 9 or 10
    void pwmSetFreq(    void );                // Hz


    // --- electrical motor operation
    // ----------------------------------------------------------------
    inline void setReverseMotorRight( const bool isReversed )
    {
    	iHerc.isMotorRightReversed = isReversed;
    }
    // ----------------------------------------------------------------
    inline void setReverseMotorLeft( const bool isReversed )
    {
    	iHerc.isMotorLefttReversed = isReversed;
    }
    // ----------------------------------------------------------------

private:

	rt_robotHercules_t	iHerc;

};

} /* namespace RT_HERCULES_ROBOT */

// ---------------------------------------------------------------------[ EoF ]
#endif /* RTHERCULESROBOT_H_ */
