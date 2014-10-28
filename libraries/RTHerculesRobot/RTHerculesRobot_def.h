/*
 * RTHerculesRobot_def.h
 *
 *  Created on: Sep 9, 2014
 *      Author: simon
 */

//  moto_4wd_dfs.h
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
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#ifndef RTHERCULESROBOT_DEF_H_
#define RTHERCULESROBOT_DEF_H_
// ---------------------------------------------------------------------[ BoF ]

#include <inttypes.h>


// order SEND
// -------------------------------------------------
#define RETM1SPEED      0x87
#define RETM2SPEED      0x88
#define RETM1STATE      0x90
#define RETM2STATE      0x91

//ctrl
// -------------------------------------------------
//#define ALLMOSON        digitalWrite(PINCS, HIGH)
//#define ALLMOSOFF       digitalWrite(PINCS, LOW)

typedef enum RT_ROBOT_HERCULES_ENUM_tag
{
	rt_robot_hercules_pinMotor_left		= 9,
	rt_robot_hercules_pinMotor_right	= 10,

	// --- pin control
	rt_robot_hercules_pin_chipSelect			= 6, 	// all mos cs (PINCS)
	rt_robot_hercules_pinMotorLeft_wForward		= 4, 	// left motor, forward wheel (PINM1F)
	rt_robot_hercules_pinMotorLeft_wRear		= 5, 	// left motor, rear wheel (PINM1R)
	rt_robot_hercules_pinMotorRight_wForward	= 7, 	// (PINM2F)
	rt_robot_hercules_pinMotorRight_wRear		= 8, 	// (PINM2R)



	rt_robot_hercules_frequencyPWM		 = 10000,	// [Hz], = 10khZ (FREQPWM)

	// motor state
	// -------------------------------------------------
	rt_robot_hercules_state_stop		 = 0,
	rt_robot_hercules_state_run			 = 1,

	rt_robot_hercules_direction_forward	 = 0,
	rt_robot_hercules_direction_reverse	 = 1,

	rt_robot_hercules_speed_isZzero		 =  0,
	rt_robot_hercules_speed_max			= 100,
	rt_robot_hercules_speed_eMax		=  99,	// effective maximum speed


} rt_robotHerculesEnum_t;

typedef struct RT_ROBOT_HERCULES_motor_tag
{
	uint8_t speed;      // pwm: 0-100
	uint8_t dir;		// direction
	uint8_t state;
	uint8_t flag;

} rt_robotHerculesMotor_t;

typedef struct RT_ROBOT_HERCULES_tag
{
	uint32_t	pwmPeriod;
	bool		isMotorRightReversed;
	bool		isMotorLefttReversed;
	// ---
	uint8_t		clockSelectBits;
	// ---
	uint8_t		_notUsed1;
	uint8_t		_notUsed2;
	uint8_t		_notUsed3;

	rt_robotHerculesMotor_t  mLeft;
	rt_robotHerculesMotor_t  mRight;

} rt_robotHercules_t;



// ---------------------------------------------------------------------[ EoF ]
#endif /* RTHERCULESROBOT_DEF_H_ */

