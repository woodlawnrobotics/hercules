// *
// * WRTbot.cpp
// *
// *  Created on: Jul 21, 2013
// *      Author: simon
// *
// ---------------------------------------------------------------------[ BoF ]
#include "RTsys_inc.h"
// -------------------------------------------------
#include "WRTbot.h"

namespace WRT_ARDUINO_ROBOT
// -------------------------------------------------------[ WRT_ARDUINO_ROBOT ]
{

int wrt_romeoKeyVal[5] = { 30, 150, 360, 535, 760 };

// -------------------------------------------------------------[ constructor ]
WRTbot::WRTbot()
{
	WRTbot::isFirstPass = true;
	// ------------------------
	verbose = 0;
	ir.rc.isGoodKey  = false;
	// ---
	ir.rc.key = -1;
	ir.rc.nGoodKeys = 0;
	ir.rc.idx = 0;
	// ---
	ir.rc.command  = wrt_rc_cmd_reset;
	ir.rc.behavior = wrt_rc_cmd_behavior_pray;

	WRTbot::irRx    = new WRT_ARDUINO_IR_RECEIVER::WRTirReceiver();
	WRTbot::drv     = new WRT_ARDUINO_BOT_DRIVE::WRTbotDrive();
	WRTbot::irObAv  = new WRT_ARDUINO_IR_AVOIDANCE::WRTirObstacleAvoidance();
	WRTbot::lineTrk = new WRT_LINE_TRACK::WRTLineTrack();

	isButtonPressed = false;
	dKeyButtonIs = static_cast<uint16_t>( wrt_button_key_none );



}	// end constructor

// --------------------------------------------------------------[ destructor ]
WRTbot::~WRTbot()
{
	delete WRTbot::irRx;
	delete WRTbot::drv;
	delete WRTbot::irObAv;

}	// end destructor

// --------------------------------------------------------------------[ init ]
void WRTbot::init( const uint8_t usrPinForReceiverIR )
{
	WRTbot::isFirstPass = true;
	// ------------------------
	verbose = 0;
	ir.rc.isGoodKey     = false;
	ir.rc.isNewBehavior = false;
	// ---
	ir.rc.key = -1;
	ir.rc.nGoodKeys  = 0;
	ir.rc.idx = 0;
	// ---
	ir.rc.command  = wrt_rc_cmd_reset;
	ir.rc.behavior = wrt_rc_cmd_behavior_irc;	// infrared command by default

	// --- note: default behavior is to go forward and avoid obstacles.
	//			 this will be the behavior when the first command is 'power'

	// -------------------
	WRTbot::irRx->init( usrPinForReceiverIR );
	// -------------------

	// -------------------
	WRTbot::drv->init();

	// -------------------
	WRTbot::irObAv->init();

	// ---------------------
	WRTbot::lineTrk->init();

#if defined( WRT_ROBOT_USE_SERVO_LIB )

	//	Digital Pin 9: Romeo Vex Claw
	// -----------------------------------------------------------
	WRTbot::vexClaw.attach(wrt_pin_romeo_vexClaw);

	// --- test the Vex claw
	// ----------------------
	WRTbot::vexClaw.writeMicroseconds(1500);  // set servo to mid-point
	delay(200);

	WRTbot::vexClaw.writeMicroseconds(1000);  // set servo to low-point
	delay(400);

	WRTbot::vexClaw.writeMicroseconds(2000);  // set servo to high-point
	delay(400);

	WRTbot::vexClaw.writeMicroseconds(1500);  // set servo to mid-point
	delay(200);

#endif

}	// end init()

// --- this step() is called regularly regardless of any
//	   activity in the IR controller
// ------------------------------------------------------------[ stepBehavior ]
bool WRTbot::stepBehavior( void )
{
	bool isBehaviorOk = false;

	// -------------------------------------------------------------------
	if( 0 == (ir.rc.command & wrt_rc_cmd_toggle_power) )
	{
		motor.left.control  &= wrt_motorControl_power_OFF;
		motor.right.control &= wrt_motorControl_power_OFF;

		motor.left.speed  = wrt_code_motor_stop;
		motor.right.speed = wrt_code_motor_stop;
	}

	// ---------------------------------------------------[  key combination: ]
	switch( ir.rc.behavior )
	{	case wrt_rc_cmd_behavior_predator:
		case wrt_rc_cmd_behavior_pray:			// 		  [			func/stop ]
		case wrt_rc_cmd_behavior_lightHunter:	// 		  [				   EQ ]
		case wrt_rc_cmd_behavior_irc:			// 		  [ 			  >|| ]
		{
			isBehaviorOk = WRTbot::drv->driveBehavior( ir.rc, motor );

		}	break;
		// ---------------------------------------------------------[ default ]
		default:
		{	// --- lost in the woods: stop the robot
			WRTbot::drv->stop();

			// --- Indicate that we are lost somehow...


			// ---
			isBehaviorOk = false;

		}	break;
	}	// switch

	return( isBehaviorOk );

}	// done stepBehavior()

// -------------------------------------------------------------[ stepVexClaw ]
bool WRTbot::stepVexClaw( void )
{
	// ---------------------------------------------------[  auxiliary action ]
	switch( ir.rc.auxBehavior )
	{	// ---------------------------------------------------------[ default ]
		default:
		{
			;	// do nothing
		}	break;
		// -----------------------------------------------------[ claw action ]
		case wrt_rc_cmd_behavior_clawAction:
		{
			WRTbot::actionIRC();

		}	break;
	}	// switch

	return( true );

}	// done stepVexClaw()

// --- this stepIRC() is called only after a sucessful IR controller
//	   reading. It starts a new behavior or modifies the current
//	   behavior
// -----------------------------------------------------------------[ stepIRC ]
bool WRTbot::stepIRC( void )
{
	// -----------------------------------------------------[  driving action ]
	switch( ir.rc.behavior )
	{	default:
		case wrt_rc_cmd_behavior_predator:
		case wrt_rc_cmd_behavior_pray:			// 		  [ 		func/stop ]
		case wrt_rc_cmd_behavior_lightHunter:	// 		  [ 			   EQ ]
		{
			WRTbot::driveIRC();

		}	break;
		// ---------------------------------------------------------[ default ]
	}	// switch

	return( true );

}	// done stepIRC()

// ---------------------------------------------------------------[ actionIRC ]
bool WRTbot::actionIRC( void )
{
	bool isLinearCmd = false;
	uint16_t   delta = static_cast<uint16_t>( wrt_vexClaw_set_delta );

	// ---------------------------------------------------[  key combination: ]
	switch( ir.rc.auxCommand )
	{	case wrt_rc_cmd_claw_clooseFull:		// key: 3
		{
			vexClawPWM = static_cast<uint16_t>( wrt_vexClaw_set_min );

		}	break;
		case wrt_rc_cmd_claw_openFull:			// key: 1
		{
			vexClawPWM  = static_cast<uint16_t>( wrt_vexClaw_set_max );

		}	break;
		case wrt_rc_cmd_claw_linear:			// key: 2
		{
			isLinearCmd = true;
			delta = static_cast<uint16_t>( wrt_vexClaw_set_delta );

		}	break;
		case wrt_rc_cmd_claw_linear2:			// key: 5
		{
			isLinearCmd = true;
			delta = static_cast<uint16_t>( wrt_vexClaw_set_delta2 );

		}	break;
		case wrt_rc_cmd_claw_linear4:			// key: 8
		{
			isLinearCmd = true;
			delta = static_cast<uint16_t>( wrt_vexClaw_set_delta4 );

		}	break;
		// ---------------------------------------------------------[ default ]
		default:
		case wrt_rc_cmd_claw_nominal:			// key: 0
		{
			vexClawPWM = static_cast<uint16_t>( wrt_vexClaw_set_nominal );

		}	break;
	}	// switch

	if( true == isLinearCmd )
	{
		if( true == vexClawDirection)
		{
			vexClawPWM += delta;
			if( wrt_vexClaw_set_max < vexClawPWM )
			{
				vexClawDirection = false;	// change direction
				vexClawPWM = static_cast<uint16_t>( wrt_vexClaw_set_max );
			}
		}
		else
		{	vexClawPWM -= delta;
			if( vexClawPWM < wrt_vexClaw_set_min)
			{
				vexClawDirection = true;	// change direction
				vexClawPWM = static_cast<uint16_t>( wrt_vexClaw_set_min );
			}
		}
	}

#if defined( WRT_ROBOT_USE_SERVO_LIB )

	vexClaw.writeMicroseconds( vexClawPWM );

#endif


}	// done actionIRC()


// ----------------------------------------------------------------[ driveIRC ]
bool WRTbot::driveIRC( void )
{
	if( true == WRTbot::isFirstPass )
	{
		WRTbot::isFirstPass = false;

		// --- implement default behavior
		ir.rc.isNewBehavior = true;

		Serial.println("stepIRC: first time");
	}

	if( true == ir.rc.isNewBehavior )
	{
		Serial.println("stepIRC: isNewBehavior");

		ir.rc.isNewBehavior = false;
		// ---------------------------
		WRTbot::drv->set( ir.rc, motor );
	}

	// -------------------------------------------------------------------
	if( 0 == (ir.rc.command & wrt_rc_cmd_toggle_power) )
	{
		motor.left.control  &= wrt_motorControl_power_OFF;
		motor.right.control &= wrt_motorControl_power_OFF;

		motor.left.speed  = wrt_code_motor_stop;
		motor.right.speed = wrt_code_motor_stop;
	}

	// ---------------------------------------------------[  key combination: ]
	switch( ir.rc.behavior )
	{	case wrt_rc_cmd_behavior_predator:
		case wrt_rc_cmd_behavior_pray:			// 		  [ 		func/stop ]
		case wrt_rc_cmd_behavior_lightHunter:	// 		  [ 			   EQ ]
		{
			WRTbot::drv->driveBehavior( ir.rc, motor );

		}	break;
		// ---------------------------------------------------------[ default ]
		default:
		case wrt_rc_cmd_behavior_irc:			// 		  [ 			  >|| ]
		{
			WRTbot::drv->ircDrive( ir.rc, motor );
		}	break;
	}	// switch

	return( true );

}	// done driveIRC()

// -----------------------------------------------------------------[ readIRC ]
bool WRTbot::readIRC( rt_ioParam_t &iPad )
{

	ir.rc.key 	 	= wrt_ir_key_none;			// infrared remote control key
	ir.rc.isGoodKey =  false;
	isGoodKeyIR     =  false;

#if defined(ARDUINO_ARCH_AVR) && defined( WRT_ROBOT_USE_IR_REMOTE_LIB )

	iPad.ticks = irRx->getTicks();

	ir.rc.key = irRx->getKey38KHz();
	// -----------------------------
	if( ir.rc.key > 0 )
	{
		isGoodKeyIR = irRx->irResponse( ir.rc );
	}
#else

	isGoodKeyIR = readIRC();

#endif

	ir.rc.isGoodKey = isGoodKeyIR;
	// ---------------------------
	return( isGoodKeyIR );

}	 // done readIRC()

// -----------------------------------------------------------------[ readIRC ]
bool WRTbot::readIRC( void )
{
	ir.rc.key 	 	=  wrt_ir_key_none;			// infrared remote control key
	ir.rc.isGoodKey =  false;
	isGoodKeyIR     =  false;
	// ------------------------
	ir.rc.key = irRx->getKey();
	// ------------------------
	if( ir.rc.key > 0 )
	{
		isGoodKeyIR = irRx->irResponse( ir.rc );
	}

	ir.rc.isGoodKey = isGoodKeyIR;
	// ---------------------------
	return( isGoodKeyIR );

}	// end readIRC()


// ---------------------------------------------------------[ stepObjectTrack ]
void WRTbot::stepObjectTrack( const uint16_t usrState )
{
	switch( usrState )
	{	default:
		case wrt_ir_state_idle:
		{
			drive( wrt_forward_fast );

		}	break;
		case wrt_ir_state_irPulseLeft:
		{
			// choices: wrt_turn_left, 		wrt_turn_left_fast,
			//			wrt_turnLeftDiff, 	wrt_turnLeftDiff_fast
			// -------------------------------------------------------------
			drive( wrt_turn_left_fast );
			// -------------------------------------------------------------
			delay(150);//Delay for 0.5 second

			// -------------------------------------------------------------
			drive( wrt_turn_left );
			// -------------------------------------------------------------
			delay(150);//Delay for 0.5 second

			//drive( wrt_forward_slow );

		}	break;
		case wrt_ir_state_irPulseForward:
		{
			// -------------------------------------------------------------
			drive( wrt_forward_fast );
			// -------------------------------------------------------------
			delay(500);//delay for 0.5 second

		}	break;

		case wrt_ir_state_irPulseRight:
		{
			// choices: wrt_turn_right, 	wrt_turn_right_fast,
			//			wrt_turnRightDiff, 	wrt_turnRightDiff_fast
			// -------------------------------------------------------------
			drive( wrt_turn_right_fast );
			// -------------------------------------------------------------
			delay(150);//delay for 0.5 second

			// -------------------------------------------------------------
			drive( wrt_turn_right );
			// -------------------------------------------------------------
			delay(150);//delay for 0.5 second

			//drive( wrt_forward_slow );

		}	break;
	}	// done switch

}	// done stepObjectTrack


// ---------------------------------------------------[ stepObstacleAvoidance ]
void WRTbot::stepObstacleAvoidance( const uint16_t usrState )
{
	switch( usrState )
	{	default:
		case wrt_ir_state_idle:
		{
			drive( wrt_forward_fast );

		}	break;
		case wrt_ir_state_irPulseLeft:
		{	// --- response to positive left IR pulse
			// -------------------------------------------------------------
			drive( wrt_reverse );
			// -------------------------------------------------------------
			delay(500);//Delay for 0.5 second

			// -------------------------------------------------------------
			drive( wrt_turn_right );
			// -------------------------------------------------------------
			delay(500);//Delay for 0.5 second

		}	break;
		case wrt_ir_state_irPulseForward:
		{
			// --- right IR pulse
			// -------------------------------------------------------------
			drive( wrt_reverse );
			// -------------------------------------------------------------
			delay(700);//delay for 0.7 second

			// -------------------------------------------------------------
			drive( wrt_turn_left );
			// -------------------------------------------------------------
			delay(200);//delay for 0.5 second

			// -------------------------------------------------------------
			drive( wrt_reverse );
			// -------------------------------------------------------------
			delay(300);//delay for 0.7 second

			// -------------------------------------------------------------
			drive( wrt_turn_left );
			// -------------------------------------------------------------
			delay(200);//delay for 0.5 second

		}	break;

		case wrt_ir_state_irPulseRight:
		{
			// --- right IR pulse
			// -------------------------------------------------------------
			drive( wrt_reverse );
			// -------------------------------------------------------------
			delay(500);//delay for 0.5 second

			// -------------------------------------------------------------
			drive( wrt_turn_left );
			// -------------------------------------------------------------
			delay(500);//delay for 0.5 second

		}	break;
	}	// done switch

}	// done stepObstacleAvoidance()

// ---------------------------------------------------------------[ trackLine ]
void WRTbot::trackLine( bool isDriveNow )
{
	WRTbot::lineTrk->nextStepX( motor );
	// ---------------------------------------
	if( true == isDriveNow )
	{
		// --- continue
		WRTbot::drv->drive( motor );
	}
	else
	{
		WRTbot::lineTrk->printSensorValues();
	}

}	// done trackLine()

// ------------------------------------------------------[ readKeyButtonRomeo ]
bool WRTbot::readKeyButtonRomeo( void )
{
	int ii = 0;

	isButtonPressed = false;
	// ---
	dKeyButtonIs = static_cast<uint16_t>( wrt_button_key_none );

	// read the analog voltage value 7
	// ---------------------------------------------------------
	dataFromKeyButton = analogRead ( wrt_pin_analogKey_romeo );
	// ---------------------------------------------------------
	if( dataFromKeyButton <  wrt_romeoKeyVal[wrt_button_key_number_romeo-1] )
	{
		for (ii = 0; ii < wrt_button_key_number_romeo; ii++)
		{
			if (dataFromKeyButton < wrt_romeoKeyVal[ii] )
			{
				isButtonPressed = true;
				dKeyButtonIs = static_cast<uint16_t>( ii ) + 1;

				break;
			}
		}
	}
	// ---------------------------------------------------------

	return( isButtonPressed );

}	// done readKeyButtonRomeo()

// -----------------------------------------------------------[ readKeyButton ]
bool WRTbot::readKeyButton( void )
{
	float data = 0.0;

	// read the analog voltage value 5
	dataFromKeyButton = analogRead ( wrt_pin_analogKey_mQ4wd );


	// digital value into an analog value
	data = ((static_cast<float>(dataFromKeyButton) * 5.0F) / 1024);

	isButtonPressed = false;
	dKeyButtonIs = static_cast<uint16_t>( wrt_button_key_none );
	// ----------------------------------------------------------------------
	if( (data > 4.50) && (data < 6.00) ) // no button is pressed
	{
		return( isButtonPressed );
	}
	else
	{	if( verbose > 0 )
		{
			Serial.print("data = ");
			Serial.println( data );
		}

		if( (data >= 0.00) && (data < 0.50) ) // Button 1 press
		{
			delay (180); // debounce delay
			if( (data >= 0.00) && (data < 0.50) )// press button 1 does
			{
				isButtonPressed = true;
				dKeyButtonIs = static_cast<uint16_t>( wrt_button_key_mQ4wd_1 );
			}
		}
		else if( (data >= 0.50) && (data <1.5) )
		{
			delay (180);
			if( (data >= 0.50) && (data <1.5) )
			{
				isButtonPressed = true;
				dKeyButtonIs = static_cast<uint16_t>( wrt_button_key_mQ4wd_2 );
			}
		}
		else if( (data >= 1.5) && (data <2.5) )
		{
			delay (180);
			if( (data >= 1.5) && (data <2.5) )
			{
				isButtonPressed = true;
				dKeyButtonIs = static_cast<uint16_t>( wrt_button_key_mQ4wd_3 );
			}
		}
	}

	return( isButtonPressed );

} // done readKeyButton()

// -------------------------------------------------------[ WRT_ARDUINO_ROBOT ]
} /* namespace WRT_ARDUINO_ROBOT */
// ---------------------------------------------------------------------[ EoF ]
