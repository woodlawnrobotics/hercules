//============================================================================
// Name        : hercules.cpp
// Author      : Miguel Simon
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <Arduino.h>

#include <inttypes.h>

#include <Thread.h>
#include <ThreadController.h>

#include <RTsys_def.h>
// -------------------------------------------------------
#if defined(ARDUINO_ARCH_AVR) && defined( WRT_ROBOT_USE_IR_REMOTE_LIB )
#include <IRremote.h>
#endif
// -------------------------------------------------------
#include <RTsys_inc.h>
#include <RTarduino_inc.h>
#include <WRTbot_inc.h>

#include <RTHerculesRobot.h>
#include <RTHerculesDrive.h>
//#include <RTHerculesEncoder.h>

#if defined( RT_TEST_HERCULES )
#undef RT_TEST_HERCULES
#endif
//define RT_TEST_HERCULES 1
// -----------------------------------------------------------[ namespace RTA ]
namespace rta
{

	// --- system configuration scratch pad
	// -----------------------------------------------------------------------
	rt_ioParam_t	cPad;

	// --- robot Romeo Analog distance infrared sensors
	wrt_robotRomeoAnalog_irDistanceSensor_t romIR;
	// -----------------------------------------------------------------------


#if defined(RT_TEST_HERCULES)

	// --- hercules robot
	// -----------------------------------------------------------------------
    RT_HERCULES_ROBOT::RTHerculesRobot herc;

#else

	// --- hercules robot
	// -----------------------------------------------------------------------
    RT_HERCULES_DRIVE::RTHerculesDrive dHerc;

#endif

    short    dKeyButtonPress = 0;
	int              verbose = 1;
	uint32_t currentBehavior = 0;
	// ---------------------------
	bool isLineTrackDrive = false;
	bool      isPlayMusic = false;
	bool      isGoodKeyIR = false;
	bool      isCaptureIR = false;
	bool     isRunVehicle = true;
	// ---
	const short 	deltaStatusTime = 10;	// cycles
	unsigned short      deltaStatus = 0;
	// -------------------------------------------------------

	// ---- instantiate a new ThreadController
	// -----------------------------------------------------------------------
	ThreadController controller = ThreadController();

	// ----------------------------------------------------[ tasks definition ]
	Thread taskHeartBit 	 	 = Thread();
	Thread taskMain     	 	 = Thread();

	Thread taskObstackeAvoidance = Thread();
	Thread taskLineTrack     	 = Thread();

	Thread taskStepBehavior      = Thread();
	Thread taskUpdateBehavior    = Thread();

} // done namespace RTA

// --- On for 200ms off for 100ms, repeat it 2 times,
// --- sleep for 2000 ms and than start again.
// BlinkTask heartbeat(rta::pinGreenLED, 200, 100, 2, 2000);
// --------------------------------------------------------[ stepTaskHeartBit ]
void stepTaskHeartBit( void )
{
//	digitalWrite( rta::pinGreenLED, HIGH);	// red LED off
//	delay( 200 );
//
//	digitalWrite( rta::pinGreenLED, LOW);	// red LED off
//	delay( 100 );
//
//	digitalWrite( rta::pinGreenLED, HIGH);	// red LED off
//	delay( 200 );
//
//	digitalWrite( rta::pinGreenLED, LOW);	// red LED off

}  // done stepTaskHeartBit()



// ----------------------------------------------------[ runObstackeAvoidance ]
void runObstackeAvoidance( void )
{
	uint16_t usrState = wrt_ir_state_idle;

  	rta::romIR.left     = analogRead( wrt_pin_analog_dIR2 );	// look left
	rta::romIR.forward  = analogRead( wrt_pin_analog_dIR3 );	// look forward
	rta::romIR.right    = analogRead( wrt_pin_analog_dIR1 );	// look right
	// -----------------------------------------------------------------------
	rta::romIR.action = static_cast<int>(wrt_ir_state_idle);

	usrState  = static_cast<uint16_t>(wrt_ir_state_idle);
	// -----------------------------------------------------------------------
	if( (rta::romIR.left    < wrt_irSwitchThreshold)  &&
		(rta::romIR.forward < wrt_irSwitchThreshold)  &&
		(rta::romIR.right   < wrt_irSwitchThreshold)   )
	{
		// -------------------------------------------------------------------
		rta::romIR.action = static_cast<int>(wrt_ir_state_irPulseForward);
		usrState = static_cast<uint16_t>(wrt_ir_state_irPulseForward);

		rta::dHerc.stepObstacleAvoidance( usrState );
	}
	// -----------------------------------------------------------------------
	else
	{
		usrState  = static_cast<uint16_t>(wrt_ir_state_idle);
		// -----------------------------------------------------------------------
		if( rta::romIR.left < wrt_irSwitchThreshold )
		{
			// -------------------------------------------------------------------
			rta::romIR.action = static_cast<int>(wrt_ir_state_irPulseLeft);
			usrState = static_cast<uint16_t>(wrt_ir_state_irPulseLeft);
		}
		// -----------------------------------------------------------------------
		rta::dHerc.stepObstacleAvoidance( usrState );

		// -----------------------------------------------------------------------
		usrState  = wrt_ir_state_idle;
		// -----------------------------------------------------------------------
		if( rta::romIR.forward < wrt_irSwitchThreshold )
		{
			// -------------------------------------------------------------------
			rta::romIR.action = static_cast<int>(wrt_ir_state_irPulseForward);
			usrState = static_cast<uint16_t>(wrt_ir_state_irPulseForward);
		}
		// -----------------------------------------------------------------------
		rta::dHerc.stepObstacleAvoidance( usrState );

		// -----------------------------------------------------------------------
		usrState  = wrt_ir_state_idle;
		// -----------------------------------------------------------------------
		if( rta::romIR.right < wrt_irSwitchThreshold )
		{
			// -------------------------------------------------------------------
			rta::romIR.action = static_cast<int>(wrt_ir_state_irPulseRight);
			usrState = static_cast<uint16_t>(wrt_ir_state_irPulseRight);
		}
		// -----------------------------------------------------------------------
		rta::dHerc.stepObstacleAvoidance( usrState );
	}

}  // done runObstackeAvoidance()

// --------------------------------------------------------------[ runRobotIR ]
void runRobotIR( void )
{
	//Serial.println( rta::cPad.ticks );

	// --- implement the command with action
	// --------------------------------------------
	if( true == rta::isGoodKeyIR )
	{
		rta::dHerc.stepIRC( rta::isRunVehicle );
	}
	// --------------------------------------------
	// PciManager.registerListener(wrt_pin_ir_sensorRomeo, &irListener);
	// --------------------------------------------
	rta::isGoodKeyIR = false;
	rta::isCaptureIR = false;

}	// done runRobotIR()

// -----------------------------------------------------------[ runRobotIdle ]
void runRobotIdle( void )
{
	// --- perform other fast tasks as needed
	if( rta::deltaStatusTime < rta::deltaStatus++ )
	{
		noInterrupts();	// Call to disable interrupts
		// ------------------------------------------------------------------
		rta::deltaStatus = 0;
		// ---------------------------------------------
		if( rta::verbose > 0 )
		{
			Serial.print("iddle: ");
			Serial.println( " " );

			Serial.print("currentBehavior: ");
			Serial.print( rta::currentBehavior, HEX );
			Serial.print( ", " );

			// -----------------------------------------------------------------------
			switch( rta::currentBehavior )
			{	default:
				{
					Serial.print("no behavior; ");
				}	break;
				// ------------------------------------------------------[ IR command ]
				case wrt_rc_cmd_behavior_irc:
				{
					Serial.print("IR command ");
				}	break;
				// ----------------------------------------------[ obstacle avoidance ]
				case wrt_rc_cmd_behavior_obstacleAvoidance:
				{
					Serial.print("obstacle avoidance ");
				}	break;
			}	// break
			// ---
			Serial.println( " " );

		}
		// ------------------------------------------------------------------
		interrupts();	// Call to enable interrupts

	} 	// if

}	// done runRobotIdle()

// ----------------------------------------------[ stepTaskObstackeAvoidance ]
void stepTaskObstackeAvoidance( void )
{
	// -----------------------------------------------------------------------
	switch( rta::currentBehavior )
	{	default:
		{	; // iddle
		}	break;
		// ----------------------------------------------[ obstacle avoidance ]
		case wrt_rc_cmd_behavior_obstacleAvoidance:
		{	if( true == rta::isRunVehicle )
			{
				runObstackeAvoidance();
			}
		}	break;
	}	// break

}  // done stepTaskObstackeAvoidance()

// -----------------------------------------------------------[ stepTaskMain ]
void stepTaskMain( void )
{
	// --- find out what the user wants to do via IR remote
	// ----------------------------------------------------
	rta::isGoodKeyIR = rta::dHerc.readIRC( rta::cPad );
	// ----------------------------------------------------
	rta::currentBehavior = rta::dHerc.getCurrentBehavior();

	// -----------------------------------------------------------------------
	switch( rta::currentBehavior )
	{	default:
		// ------------------------------------------------------[ IR command ]
		case wrt_rc_cmd_behavior_irc:
		{
			runRobotIR();
		}	break;
		// ----------------------------------------------[ obstacle avoidance ]
		case wrt_rc_cmd_behavior_obstacleAvoidance:
		{	if( true == rta::isRunVehicle )
			{
				runObstackeAvoidance();
			}
		}	break;
	}	// break

	// --- perform idle tasks
	runRobotIdle();

}  // done stepTaskMain()

// --- this is the callback for the Timer
// -----------------------------------------------------------[ timerCallback ]
//void timerCallback( byte changeKind )
void timerCallback( void )
{
#if defined(RT_TEST_HERCULES)


	rta::herc.setSpeedDirLeft(80, rt_robot_hercules_direction_forward);
	delay(2000);

	rta::herc.setStopLeft();
	delay(2000);

	rta::herc.setSpeedDirLeft(80, rt_robot_hercules_direction_reverse);
	delay(2000);

	rta::herc.setStopLeft();
	delay(2000);

	rta::herc.setSpeedDirRight(80, rt_robot_hercules_direction_forward);
	delay(2000);

	rta::herc.setStopRight();
	delay(2000);

	rta::herc.setSpeedDirRight(80, rt_robot_hercules_direction_reverse);
	delay(2000);

	rta::herc.setStopRight();
	delay(2000);

	rta::herc.setSpeedDir(80, rt_robot_hercules_direction_forward);
	delay(2000);

	rta::herc.stop();
	delay(2000);

	rta::herc.setSpeedDir(80, rt_robot_hercules_direction_reverse);
	delay(2000);

	rta::herc.stop();
	delay(2000);

#else

	rta::controller.run();

#endif

}	// done timerCallback()

// -------------------------------------------------------------------[ setup ]
void setup()
{
	// --- setup system wide settings and parameters
	// ----------------------------------------------------------
	rta::cPad.arch		= static_cast<uint16_t>(wrt_board_romeo_avr);
	rta::cPad.board	  	= static_cast<uint16_t>(wrt_board_hercules);

	// --- setup system wide settings and parameters
	// ----------------------------------------------------------
	rta::cPad.firstTime	  = true;
	rta::cPad.idx		  = 0;
	// ---

	// --- serial port start
	// ----------------------------------------------------------
	Serial.begin( wrt_io_serialComBaudRate );
	// ------
	delay(5);

	// ---
	rta::cPad.id = static_cast<uint16_t>( wrt_io_i2c_robot );
	rta::cPad.counter  = 0;
	rta::cPad.nbytesRx = 0;
	rta::cPad.nbytesTx = 0;
	// ---

    // --- IR proximity switches
    // ----------------------------------------------------------------
	pinMode( wrt_pin_analog_dIR1 , INPUT);
	pinMode( wrt_pin_analog_dIR2 , INPUT);
	pinMode( wrt_pin_analog_dIR3 , INPUT);

	// note: non the Arduino Duemilanove
	// -----------------------------------------------------------------
	//		External Interrupts: 2 and 3.
	//		These pins can be configured to trigger an interrupt on a
	//		low value, a rising or falling edge, or a change in value.
	//		See the attachInterrupt() function for details.
	// -----------------------------------------------------------------
	rta::dHerc.init( wrt_pin_ir_sensorHercules );


#if defined(RT_TEST_HERCULES)
    rta::herc.init();

    // --- electrical motor operation
    // ----------------------------------------------------------------
    rta::herc.setReverseMotorLeft(  false ); // left  motor is normal
    rta::herc.setReverseMotorRight( false  ); // right motor is reversed

#else

    rta::dHerc.init( wrt_pin_ir_sensorHercules );

    // --- electrical motor operation
    // ----------------------------------------------------------------
    rta::dHerc.setReverseMotorLeft(  false ); // left  motor is normal
    rta::dHerc.setReverseMotorRight( false  ); // right motor is reversed

#endif

	// ----------------------------------------------------[ tasks definition ]
	rta::taskHeartBit.onRun( stepTaskHeartBit );
	rta::taskHeartBit.setInterval(2000);

	// -- taskMain runs at 10Hz
	rta::taskMain.onRun( stepTaskMain );
	rta::taskMain.setInterval(100);

	// -- taskObstackeAvoidance runs at 10Hz
	rta::taskObstackeAvoidance.onRun( stepTaskObstackeAvoidance );
	rta::taskObstackeAvoidance.setInterval(1);
	rta::taskObstackeAvoidance.enabled = false;

	// --- adds both threads to the controller
	rta::controller.add( &rta::taskHeartBit );
	rta::controller.add( &rta::taskMain );
	rta::controller.add( &rta::taskObstackeAvoidance );


    delay(100);


}	// done setup()

// -------------------------------------------------------------------[ loop ]
void loop()
{
	delayMicroseconds(1667);

	timerCallback();




}
/*********************************************************************************************************
 * END FILE
 *********************************************************************************************************/


