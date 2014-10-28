/*
 * RTHerculesRobot.cpp
 *
 *  Created on: Sep 9, 2014
 *      Author: simon
 */

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>


#include "RTHerculesRobot.h"

namespace RT_HERCULES_ROBOT
{

#define TIMER1COUNT 65536    	// Timer1 is 16 bit

// -------------------------------------------------------------[ constructor ]
RTHerculesRobot::RTHerculesRobot( void )
{
	memset( &iHerc, 0, sizeof(struct RT_ROBOT_HERCULES_tag) );
	// -------------------------------------------------------

	iHerc.mLeft.dir  = static_cast<uint8_t>(rt_robot_hercules_direction_forward);
	iHerc.mRight.dir = static_cast<uint8_t>(rt_robot_hercules_direction_forward);

	iHerc.mLeft.speed  = static_cast<uint8_t>(rt_robot_hercules_speed_isZzero);
	iHerc.mRight.speed = static_cast<uint8_t>(rt_robot_hercules_speed_isZzero);

	iHerc.mLeft.state  = static_cast<uint8_t>(rt_robot_hercules_state_stop);
	iHerc.mRight.state = static_cast<uint8_t>(rt_robot_hercules_state_stop);

	iHerc.mLeft.flag  = 0;
	iHerc.mRight.flag = 0;

	iHerc.isMotorLefttReversed = false;
	iHerc.isMotorRightReversed = true;

}

// --------------------------------------------------------------[ destructor ]
RTHerculesRobot::~RTHerculesRobot( void )
{
	// TODO Auto-generated destructor stub
}


// ***************************************************************************
// ** Function name: init
// ** Descriptions:  init all pins
// ***************************************************************************
void RTHerculesRobot::init( void )
{
	iHerc.mLeft.dir  = static_cast<uint8_t>(rt_robot_hercules_direction_forward);
	iHerc.mRight.dir = static_cast<uint8_t>(rt_robot_hercules_direction_forward);

	iHerc.mLeft.speed  = static_cast<uint8_t>(rt_robot_hercules_speed_isZzero);
	iHerc.mRight.speed = static_cast<uint8_t>(rt_robot_hercules_speed_isZzero);

	iHerc.mLeft.state  = static_cast<uint8_t>(rt_robot_hercules_state_stop);
	iHerc.mRight.state = static_cast<uint8_t>(rt_robot_hercules_state_stop);

	iHerc.mLeft.flag  = 0;
	iHerc.mRight.flag = 0;

    // pwm set
    pwmInit();
    pwmSetPwm( rt_robot_hercules_pinMotor_left,  0 );
    pwmSetPwm( rt_robot_hercules_pinMotor_right, 0 );

    pinMode( rt_robot_hercules_pin_chipSelect, OUTPUT);
    pinMode( rt_robot_hercules_pinMotorLeft_wForward, OUTPUT);
    pinMode( rt_robot_hercules_pinMotorLeft_wRear, OUTPUT);
    pinMode( rt_robot_hercules_pinMotorRight_wForward, OUTPUT);
    pinMode( rt_robot_hercules_pinMotorRight_wRear, OUTPUT);

    digitalWrite( rt_robot_hercules_pin_chipSelect, LOW);
    digitalWrite( rt_robot_hercules_pinMotorLeft_wForward, LOW);
    digitalWrite( rt_robot_hercules_pinMotorLeft_wRear, LOW);
    digitalWrite( rt_robot_hercules_pinMotorRight_wForward, LOW);
    digitalWrite( rt_robot_hercules_pinMotorRight_wRear, LOW);

}	// done init()

// ************************************************************************
// ** Function name: setDirLeft
// ** Descriptions:  set motor 1 direction
// ************************************************************************
void RTHerculesRobot::setDirLeft( const uint8_t dir)
{
	iHerc.mLeft.dir = dir;
	// -----------------------------------------
	if( true == iHerc.isMotorLefttReversed )
	{
		switch( dir )
		{	default:
			{
				iHerc.mLeft.dir = dir;
			}	break;
			case rt_robot_hercules_direction_forward:
			{
				iHerc.mLeft.dir = rt_robot_hercules_direction_reverse;
			}	break;
			case rt_robot_hercules_direction_reverse:
			{
				iHerc.mLeft.dir = rt_robot_hercules_direction_forward;
			}	break;
		}	// switch
	}	// if

}	// done setDirLeft()

// ************************************************************************
// ** Function name: setDirRight
// ** Descriptions:  set motor 2 direction
// ************************************************************************
void RTHerculesRobot::setDirRight( const uint8_t dir)
{
	iHerc.mRight.dir = dir;
	// -----------------------------------------
	if( true == iHerc.isMotorRightReversed )
	{
		switch( dir )
		{	default:
			{
				iHerc.mRight.dir = dir;
			}	break;
			case rt_robot_hercules_direction_forward:
			{
				iHerc.mRight.dir = rt_robot_hercules_direction_reverse;
			}	break;
			case rt_robot_hercules_direction_reverse:
			{
				iHerc.mRight.dir = rt_robot_hercules_direction_forward;
			}	break;
		}	// switch
	}	// if

}	// done setDirRight()

// ***************************************************************************
// ** Function name: setRunLeft
// ** Descriptions:  set motor 1 (right) run
// ***************************************************************************
void RTHerculesRobot::setRunLeft( void )
{
	pwmSetPwm( rt_robot_hercules_pinMotor_left, iHerc.mLeft.speed );

	digitalWrite( rt_robot_hercules_pin_chipSelect, HIGH);  // ALLMOSON;
	// ------------------------------------
	switch( iHerc.mLeft.dir )
	{	default:
		case rt_robot_hercules_direction_forward:
		{
	        digitalWrite( rt_robot_hercules_pinMotorLeft_wForward, HIGH);
	        digitalWrite( rt_robot_hercules_pinMotorLeft_wRear, LOW);

		}	break;
		case rt_robot_hercules_direction_reverse:
		{
	        digitalWrite( rt_robot_hercules_pinMotorLeft_wForward, LOW);
	        digitalWrite( rt_robot_hercules_pinMotorLeft_wRear, HIGH);

		}	break;
	}
	// ---
    iHerc.mLeft.state = static_cast<uint8_t>(rt_robot_hercules_state_run);

}	// done setRunLeft()

// ***************************************************************************
// ** Function name: setRunRight
// ** Descriptions:  set motor 2 (left) run
// ***************************************************************************
void RTHerculesRobot::setRunRight( void )
{
	pwmSetPwm( rt_robot_hercules_pinMotor_right, iHerc.mRight.speed );

	digitalWrite( rt_robot_hercules_pin_chipSelect, HIGH); // ALLMOSON;
	// ------------------------------------
	switch( iHerc.mRight.dir )
	{	default:
		case rt_robot_hercules_direction_forward:
		{
	        digitalWrite( rt_robot_hercules_pinMotorRight_wForward, HIGH);
	        digitalWrite( rt_robot_hercules_pinMotorRight_wRear, LOW);

		}	break;
		case rt_robot_hercules_direction_reverse:
		{
	        digitalWrite( rt_robot_hercules_pinMotorRight_wForward, LOW);
	        digitalWrite( rt_robot_hercules_pinMotorRight_wRear, HIGH);

		}	break;
	}
	// ---
    iHerc.mRight.state = static_cast<uint8_t>(rt_robot_hercules_state_run);

}	// done setRunRight()

// ***************************************************************************
// ** Function name: setStopLeft
// ** Descriptions:  set motor 1 stop
// ***************************************************************************
void RTHerculesRobot::setStopLeft( void )
{
	pwmSetPwm( rt_robot_hercules_pinMotor_left, 0 );

    digitalWrite( rt_robot_hercules_pinMotorLeft_wForward, LOW);
    digitalWrite( rt_robot_hercules_pinMotorLeft_wRear, LOW);

    iHerc.mLeft.state = static_cast<uint8_t>(rt_robot_hercules_state_stop);

}	// setStopLeft()

// ***************************************************************************
// ** Function name: setStopRight
// ** Descriptions:  set motor 2 stop
// ***************************************************************************
void RTHerculesRobot::setStopRight( void )
{
	pwmSetPwm( rt_robot_hercules_pinMotor_right, 0 );
    digitalWrite( rt_robot_hercules_pinMotorRight_wForward, LOW);
    digitalWrite( rt_robot_hercules_pinMotorRight_wRear, LOW);

    iHerc.mRight.state = static_cast<uint8_t>(rt_robot_hercules_state_stop);

}	// done setStopRight()

// ***************************************************************************
// ** Function name: stop
// ** Descriptions:  stop both motors
// ***************************************************************************
void RTHerculesRobot::stop( void )
{
	setStopLeft();
	setStopRight();

}	// done stop()


// ***************************************************************************
// ** Function name: setSpeedLeft
// ** Descriptions:  set motor 1 speed
// ***************************************************************************
void RTHerculesRobot::setSpeedLeft( uint8_t ispeed)       // pwm, 0-100
{
    iHerc.mLeft.speed = (ispeed < rt_robot_hercules_speed_max) ?
    					 ispeed : rt_robot_hercules_speed_eMax;
    setRunLeft();
}

// ***************************************************************************
// ** Function name: setSpeedRight
// ** Descriptions:  set motor 1 speed
// ***************************************************************************
void RTHerculesRobot::setSpeedRight( uint8_t ispeed)       // pwm, 0-100
{
    iHerc.mRight.speed = (ispeed < rt_robot_hercules_speed_max) ?
    					  ispeed : rt_robot_hercules_speed_eMax;
    setRunRight();
}

// ***************************************************************************
// ** Function name: setSpeedDir
// ** Descriptions:  set motor 2 direction
// ***************************************************************************
void RTHerculesRobot::setSpeedDir( uint8_t ispeed, uint8_t dir)
{
    setSpeedDirLeft(  ispeed, dir);
    setSpeedDirRight( ispeed, dir);
}

// ***************************************************************************
// ** Function name: setSpeedDirLeft
// ** Descriptions:  set motor1 speed and direction
// ***************************************************************************
void RTHerculesRobot::setSpeedDirLeft( uint8_t ispeed, uint8_t dir)
{
    setSpeedLeft(ispeed);
    setDirLeft(dir);
    setRunLeft();

}

// ***************************************************************************
// ** Function name: setSpeedDirRight
// ** Descriptions:  set motor2 speed and direction
// ***************************************************************************
void RTHerculesRobot::setSpeedDirRight( uint8_t ispeed, uint8_t dir)
{
    setSpeedRight(ispeed);
    setDirRight(dir);
    setRunRight();
}



// -----------------------------------------------------------------[ pwmInit ]
void RTHerculesRobot::pwmInit( void )
{
    TCCR1A = 0;                                 // clear control register A
    TCCR1B = _BV(WGM13);                        // set mode 8: phase and frequency correct pwm, stop the timer

}	// done init()

// --------------------------------------------------------------[ pwmSetFreq ]
void RTHerculesRobot::pwmSetFreq( void )                // AR modified for atomic access
{
	char    tmp = 0;
	const long freq = static_cast<long>(rt_robot_hercules_frequencyPWM);
    long cycles = (F_CPU / 2000000) * (1000000/freq);                               // the counter runs backwards after TOP, interrupt is at BOTTOM so divide microseconds by 2
    // -----------------------------------------------------------------------
    if(cycles < TIMER1COUNT)              iHerc.clockSelectBits = _BV(CS10);              // no prescale, full xtal
    else if((cycles >>= 3) < TIMER1COUNT) iHerc.clockSelectBits = _BV(CS11);              // prescale by /8
    else if((cycles >>= 3) < TIMER1COUNT) iHerc.clockSelectBits = _BV(CS11) | _BV(CS10);  // prescale by /64
    else if((cycles >>= 2) < TIMER1COUNT) iHerc.clockSelectBits = _BV(CS12);              // prescale by /256
    else if((cycles >>= 2) < TIMER1COUNT) iHerc.clockSelectBits = _BV(CS12) | _BV(CS10);  // prescale by /1024
    else        cycles = TIMER1COUNT - 1, iHerc.clockSelectBits = _BV(CS12) | _BV(CS10);  // request was out of bounds, set as maximum
    // -----------------------------------------------------------------------
    iHerc.pwmPeriod = cycles;
    // -----------------------------------------------------------------------
    tmp = SREG;
    cli();                                            // Disable interrupts for 16 bit register access
    ICR1    = iHerc.pwmPeriod;                        // ICR1 is TOP in p & f correct setPwm mode
    SREG    = tmp;
    TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
    TCCR1B |= iHerc.clockSelectBits;                  // reset clock select register, and starts the clock

}	// done pwmSetFreq()


// -----------------------------------------------------------[ pwmSetPwmDuty ]
bool RTHerculesRobot::pwmSetPwmDuty(char pin, int duty)
{
	// duty, 0-100

	bool isOk = true;
	// ----------------------------
	const int  maxDutyCycle = 100;
    unsigned long dutyCycle = 0;
    char tmp = SREG;

    duty = (duty < maxDutyCycle) ? duty : maxDutyCycle;
    duty = (duty < 0) ? 0 : duty;

    dutyCycle   = iHerc.pwmPeriod*10;
    dutyCycle  *= duty;
    dutyCycle >>= 10;

    cli();
    // ---------------------
    switch( pin )
    {	default:
		{
			isOk = false;

		}	break;
		case 1:
		case rt_robot_hercules_pinMotor_left:
		{
			OCR1A = dutyCycle;

		}	break;
		case  2:
		case rt_robot_hercules_pinMotor_right:
		{
			OCR1B = dutyCycle;

		}	break;
    }
    // ---------------------
    SREG = tmp;

    return( isOk );

}	// done setPwmDuty()


// ---------------------------------------------------------------[ pwmSetPwm ]
void RTHerculesRobot::pwmSetPwm(char pin, int duty )
{
	bool  isOk = true;
	const long freq = static_cast<long>(rt_robot_hercules_frequencyPWM);

	// setPwm, pin: 9, 10
	// expects duty cycle to be 10 bit (1024)

    if( ((pin != rt_robot_hercules_pinMotor_left )  &&
    	 (pin != rt_robot_hercules_pinMotor_right)) ||
    	 (freq <= 0) )
	{
    	return ;                // error paramount
	}
    // -----------------------------------------------------------------------
    pwmSetFreq();
    // -----------------------------------------------------------------------
    switch( pin )
    {	default:
		{
			isOk = false;

		}	break;
		case rt_robot_hercules_pinMotor_left:
		{
			// --- sets data direction register for setPwm output pin
			DDRB |= _BV(PORTB1);

			// --- activates the output pin
			TCCR1A |= _BV(COM1A1);

		}	break;
		case rt_robot_hercules_pinMotor_right:
		{
	        DDRB |= _BV(PORTB2);
	        TCCR1A |= _BV(COM1B1);

		}	break;
    }
    // -----------------------------------------------------------------------
    pwmSetPwmDuty( pin, duty );
    // -----------------------------------------------------------------------

}	// done pwmSetPwm()

// -----------------------------------------------------------[ pwmDisablePwm ]
void RTHerculesRobot::pwmDisablePwm(char pin)
{
    switch( pin )
    {	default:
		{
			; // do nothing...
		}	break;
		case 1:
		case rt_robot_hercules_pinMotor_left:
		{
			// clear the bit that enables setPwm on PB1
			TCCR1A &= ~_BV(COM1A1);

		}	break;
		case  2:
		case rt_robot_hercules_pinMotor_right:
		{
			// clear the bit that enables setPwm on PB2
			TCCR1A &= ~_BV(COM1B1);

		}	break;
    }

}	// done disablePwm()

// ---------------------------------------------------------------------[ EoF ]
} /* namespace RT_HERCULES_ROBOT */
