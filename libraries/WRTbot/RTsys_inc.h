/*
 * RTsys_inc.h
 *
 *  Created on: May 27, 2014
 *      Author: simon
 */

#ifndef RTSYS_INC_H_
#define RTSYS_INC_H_



#include "RTsys_def.h"
// --------------------------------------------------------------------------
#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#if defined( WRT_ROBOT_USE_SERVO_LIB )
#include <Servo.h>
#endif

#if defined(ARDUINO_ARCH_AVR) && defined( WRT_ROBOT_USE_IR_REMOTE_LIB )
#include <IRremote.h>
#endif



#endif /* RTSYS_INC_H_ */
