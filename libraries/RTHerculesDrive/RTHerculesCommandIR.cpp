/*
 * RTHerculesCommandIR.cpp
 *
 *  Created on: Oct 24, 2014
 *      Author: simon
 */
// ---------------------------------------------------------------------[ BoF ]
#include "RTHerculesCommandIR.h"

#if !defined(ARDUINO_ARCH_AVR)
#error "class RTHerculesCommandIR is not compatible with this architecture"
#endif
// ---
#if !defined( WRT_ROBOT_USE_IR_REMOTE_LIB )
#error "class RTHerculesCommandIR requires IRremote library"
#endif

namespace RT_HERCULES_IR_RECEIVER
{

RTHerculesCommandIR::RTHerculesCommandIR()
{
	// --- by default
	RTHerculesCommandIR::irPin = static_cast<uint8_t>(wrt_pin_ir_sensor);
	// ---
	isInit 		  = false;	// IR receiver has not been initialized
	isGoodKey     = false;	// no key detected by default
	debug 		  = false;  // flag as true to output raw IR pulse data stream length in microseconds
	output_verify = false;  // flag as true to print decoded verification integers. same number for all buttons
	output_key 	  = false;  // flag as true to print decoded key integers
	remote_verify = 16128;  // verifies first bits are 11111100000000
						    // different remotes may have different start codes
	verbose = 0;
	nTicks  = 0;

}	// end constructor

RTHerculesCommandIR::~RTHerculesCommandIR()
{
	if( true == isInit )
	{
		delete irRcv38KHz;
	}

}	// end destructor


void RTHerculesCommandIR::init( const uint8_t usrPinForReceiverIR )
{
	isGoodKey     = false;	// no key detected by default
	debug 		  = false;  // flag as true to output raw IR pulse data stream length in microseconds
	output_verify = false;  // flag as true to print decoded verification integers. same number for all buttons
	output_key 	  = false;  // flag as true to print decoded key integers
	remote_verify = 16128;  // verifies first bits are 11111100000000
						    // different remotes may have different start codes
	verbose = 0;
	// ----
	setReceiverPinIR( usrPinForReceiverIR );

	// --- init the buzzer pin to output mode
	pinMode (wrt_bot_minQ4wd_pin_buzzer, OUTPUT);

	irRcv38KHz = new IRrecv( RTHerculesCommandIR::irPin );

	// --- start the receiver
	 irRcv38KHz->enableIRIn();

	 isInit = true;

}	// end init()

// ----------------------------------------------------------------------------
//  respond to specific remote-control keys with different behaviors
// --------------------------------------------------------------[ irResponse ]
bool RTHerculesCommandIR::irResponse( wrt_ir_rc_t &rc )
{
	isGoodKey = true;
	// ------------------------------
	if( true == output_key )
	{
		Serial.print("Key ");
		Serial.println(rc.key, HEX);
	}

	// --- first: determine if the key is good, and if it is, take action:
	switch( rc.key )
	{
		case wrt_ir_key_power:  	// turns all motion on/off (toggles)
		case wrt_ir_key_funcStop:  	// FUNC/STOP ir rc hand control behavior
		case wrt_ir_key_fbkp:  		// |<< left command
		case wrt_ir_key_fwrdEnd:  	// >|| avoid obstacles command
		case wrt_ir_key_ffwrd:  	// >>| right command
		case wrt_ir_key_volUp:  	// VOL+ turns on individual test beeper
		case wrt_ir_key_volDown:  	// VOL- turns off individual test beeper
		case wrt_ir_key_down: 		// v reverse robot motion
		case wrt_ir_key_up:  		// ^ forward motion
		case wrt_ir_key_eq:  		// EQ negative tests internal setup
		case wrt_ir_key_srt:  		// ST/REPT
		// ----------------------------
		case wrt_ir_key38KHz_power:
		case wrt_ir_key38KHz_funcStop:
		case wrt_ir_key38KHz_fbkp:
		case wrt_ir_key38KHz_fwrdEnd:
		case wrt_ir_key38KHz_ffwrd:
		case wrt_ir_key38KHz_volUp:
		case wrt_ir_key38KHz_volDown:
		case wrt_ir_key38KHz_down:
		case wrt_ir_key38KHz_up:
		case wrt_ir_key38KHz_eq:
		case wrt_ir_key38KHz_srt:

		{
			rc.isNewBehavior = false;

			rc.idx += ( rc.idx < wrt_bot_ir_nkeys ) ? 1 : (-rc.idx);

			// --- keep command history by one step
			rc.commandLast = rc.command;

			// --- reset command
			rc.command = wrt_rc_cmd_reset;

			// --- preserve power status
			rc.command |= (rc.commandLast & wrt_rc_cmd_toggle_power);

		}	break;

		case wrt_ir_key_0:  		// 0
		case wrt_ir_key_1:  		// 1
		case wrt_ir_key_2:  		// 2
		case wrt_ir_key_3:  		// 3
		case wrt_ir_key_4:  		// 4
		case wrt_ir_key_5:  		// 5
		case wrt_ir_key_6: 			// 6
		case wrt_ir_key_7:  		// 7
		case wrt_ir_key_8:  		// 8
		case wrt_ir_key_9:  		// 9
		// -------------------------------
		case wrt_ir_key38KHz_0:
		case wrt_ir_key38KHz_1:
		case wrt_ir_key38KHz_2:
		case wrt_ir_key38KHz_3:
		case wrt_ir_key38KHz_4:
		case wrt_ir_key38KHz_5:
		case wrt_ir_key38KHz_6:
		case wrt_ir_key38KHz_7:
		case wrt_ir_key38KHz_8:
		case wrt_ir_key38KHz_9:
		{
			;
		} 	break;
		// ------------------------------
		case wrt_ir_key_none:
		default:
		{
			isGoodKey = false;

		} 	break;
	}	// switch

	// --- map keys to commands
	// -----------------------------------------------------------------------
	switch( rc.key )
	{	case wrt_ir_key_none:
		default:
		{
			isGoodKey = false;

		} 	break;
		// --------------------------------[ infrared remote control commands ]
		case wrt_ir_key38KHz_power:
		case wrt_ir_key_power:  			// power button, not a behavior
		{
			rc.command 	   ^= static_cast<uint32_t>(wrt_rc_cmd_toggle_power);
			rc.cmd[rc.idx]  = rc.command;
			rc.ncmd++;

			// --- reset direction history
			rc.commandDirection = 0;

		}	break;

		case wrt_ir_key38KHz_fbkp:
		case wrt_ir_key_fbkp:  				// (key: |<< ) left command
		{
			rc.command 	   |= static_cast<uint32_t>(wrt_rc_cmd_left);
			rc.cmd[rc.idx]  = rc.command;
			rc.ncmd++;

		}	break;

		case wrt_ir_key38KHz_ffwrd:
		case wrt_ir_key_ffwrd:  			// (key: >>| ) right command
		{
			rc.command 	   |= static_cast<uint32_t>(wrt_rc_cmd_right);
			rc.cmd[rc.idx]  = rc.command;
			rc.ncmd++;

		}	break;

		case wrt_ir_key38KHz_down:
		case wrt_ir_key_down:  				// (key: v ) reverse command
		{
			rc.command 	   |= static_cast<uint32_t>(wrt_rc_cmd_reverse);
			rc.cmd[rc.idx]  = rc.command;
			rc.ncmd++;

			rc.commandDirection &= wrt_rc_cmd_mask_svClear;

		}	break;

		case wrt_ir_key38KHz_up:
		case wrt_ir_key_up:  				// (key: ^ ) forward command
		{
			rc.command 	   |= static_cast<uint32_t>(wrt_rc_cmd_forward);
			rc.cmd[rc.idx]  = rc.command;
			rc.ncmd++;

			rc.commandDirection &= wrt_rc_cmd_mask_svClear;

		}	break;

		case wrt_ir_key38KHz_volUp:
		case wrt_ir_key_volUp:  			// VOL+  spin to the right
		{
			rc.command 	   |= wrt_rc_cmd_spin_right;
			rc.cmd[rc.idx]  = rc.command;
			rc.ncmd++;

			rc.commandDirection &= wrt_rc_cmd_mask_fvclear;

		}	break;

		case wrt_ir_key38KHz_volDown:
		case wrt_ir_key_volDown:  			// VOL- spin to the left
		{
			rc.command 	   |= wrt_rc_cmd_spin_left;
			rc.cmd[rc.idx]  = rc.command;
			rc.ncmd++;

			rc.commandDirection &= wrt_rc_cmd_mask_fvclear;

		}	break;

		// -------------------------------------------------------[ behaviors ]
		case wrt_ir_key38KHz_fwrdEnd:
		case wrt_ir_key_fwrdEnd:  			// (key:  >|| ) behavior
		{
			rc.isNewBehavior = true;
			rc.behavior &= wrt_rc_cmd_behavior_clear;

			// --- infrared remote command mode
			// ---------------------------------------------------------------
			rc.behavior = static_cast<uint32_t>(wrt_rc_cmd_behavior_irc);
			// ---------------------------------------------------------------

			// note: a behavior is not a command, but we need to store it
			// ---------------------------------------------------------------
			rc.command 	   |= static_cast<uint32_t>(wrt_rc_cmd_toggle_power);
			rc.command 	   |= static_cast<uint32_t>(wrt_rc_cmd_behavior_irc);
			rc.cmd[rc.idx]  = rc.command;
			rc.ncmd++;

		}	break;

		case wrt_ir_key38KHz_funcStop:
		case wrt_ir_key_funcStop:  			// (key: FUNC/STOP) behavior
		{
			rc.isNewBehavior = true;
			rc.behavior &= wrt_rc_cmd_behavior_clear;

			// --- follow the leader behavior or predator behavior (follow obstacles)
			//	   note: how to avoid fixed obstacles?
			// ---------------------------------------------------------------
			rc.behavior = wrt_rc_cmd_behavior_pray;	// toggle the behavior
			// ---------------------------------------------------------------

			// note: a behavior is not a command, but we need to store it
			// ---------------------------------------------------------------
			rc.command 	   |= wrt_rc_cmd_behavior_pray;
			rc.cmd[rc.idx]  = wrt_rc_cmd_behavior_pray;
			rc.ncmd++;

		}	break;

		case wrt_ir_key38KHz_eq:
		case wrt_ir_key_eq:  			// (key:  EQ ) behavior
		{
			rc.isNewBehavior = true;
			rc.behavior &= wrt_rc_cmd_behavior_clear;

			// --- infrared remote command mode
			// ---------------------------------------------------------------
			rc.behavior = static_cast<uint32_t>(wrt_rc_cmd_behavior_obstacleAvoidance);
			// ---------------------------------------------------------------

			// note: a behavior is not a command, but we need to store it
			// ---------------------------------------------------------------
			rc.command 	   |= static_cast<uint32_t>(wrt_rc_cmd_toggle_power);
			rc.command 	   |= static_cast<uint32_t>(wrt_rc_cmd_behavior_obstacleAvoidance);
			rc.cmd[rc.idx]  = rc.command;
			rc.ncmd++;

		}	break;

		case wrt_ir_key38KHz_srt:
		case wrt_ir_key_srt:  // ST/REPT Positive tests Select Test and Repeat Test
			break;

		case wrt_ir_key38KHz_0:
		case wrt_ir_key_0:  // 0
		{
			// --- infrared remote command mode
			// ---------------------------------------------------------------
			rc.auxBehavior = static_cast<uint32_t>(wrt_rc_cmd_behavior_clawAction);
			// ---------------------------------------------------------------

			// note: a behavior is not a command, but we need to store it
			// ---------------------------------------------------------------
			rc.auxCommand = static_cast<uint32_t>(wrt_rc_cmd_claw_nominal);

		}	break;

		case wrt_ir_key38KHz_1:
		case wrt_ir_key_1:  // 1
		{
			// --- infrared remote command mode
			// ---------------------------------------------------------------
			rc.auxBehavior = static_cast<uint32_t>(wrt_rc_cmd_behavior_clawAction);
			// ---------------------------------------------------------------

			// note: a behavior is not a command, but we need to store it
			// ---------------------------------------------------------------
			rc.auxCommand = static_cast<uint32_t>(wrt_rc_cmd_claw_openFull);

		}	break;

		case wrt_ir_key38KHz_2:
		case wrt_ir_key_2:  // 2
		{
			// --- infrared remote command mode
			// ---------------------------------------------------------------
			rc.auxBehavior = static_cast<uint32_t>(wrt_rc_cmd_behavior_clawAction);
			// ---------------------------------------------------------------

			// note: a behavior is not a command, but we need to store it
			// ---------------------------------------------------------------
			rc.auxCommand = static_cast<uint32_t>(wrt_rc_cmd_claw_linear);

		}	break;

		case wrt_ir_key38KHz_3:
		case wrt_ir_key_3:  // 3
		{
			// --- infrared remote command mode
			// ---------------------------------------------------------------
			rc.auxBehavior = static_cast<uint32_t>(wrt_rc_cmd_behavior_clawAction);
			// ---------------------------------------------------------------

			// note: a behavior is not a command, but we need to store it
			// ---------------------------------------------------------------
			rc.auxCommand  = static_cast<uint32_t>(wrt_rc_cmd_claw_clooseFull);

		}	break;

		case wrt_ir_key38KHz_4:
		case wrt_ir_key_4:  // 4
			break;

		case wrt_ir_key38KHz_5:
		case wrt_ir_key_5:  // 5
		{
			// --- infrared remote command mode
			// ---------------------------------------------------------------
			rc.auxBehavior = static_cast<uint32_t>(wrt_rc_cmd_behavior_clawAction);
			// ---------------------------------------------------------------

			// note: a behavior is not a command, but we need to store it
			// ---------------------------------------------------------------
			rc.auxCommand  = static_cast<uint32_t>(wrt_rc_cmd_claw_linear2);

		}	break;

		case wrt_ir_key38KHz_6:
		case wrt_ir_key_6:  // 6
			break;

		case wrt_ir_key38KHz_7:
		case wrt_ir_key_7:  // 7
			break;

		case wrt_ir_key38KHz_8:
		case wrt_ir_key_8:  // 8
		{
			// --- infrared remote command mode
			// ---------------------------------------------------------------
			rc.auxBehavior = static_cast<uint32_t>(wrt_rc_cmd_behavior_clawAction);
			// ---------------------------------------------------------------

			// note: a behavior is not a command, but we need to store it
			// ---------------------------------------------------------------
			rc.auxCommand  = static_cast<uint32_t>(wrt_rc_cmd_claw_linear4);

		}	break;

		case wrt_ir_key38KHz_9:
		case wrt_ir_key_9:  // 9
		{
		}	break;
	}	// switch
	// --------------------------------------------------------------------
	rc.nGoodKeys += ( true == isGoodKey ) ? 1 : (-rc.nGoodKeys);
	// --------------------------------------------------------------------
	rc.isGoodKey  = isGoodKey;
	// --------------------------------------------------------------------
	if( verbose > 0 )
	{
		RTHerculesCommandIR::printKey( rc.key );

	}	// if verbose
	// ------------------------------
	return( isGoodKey );

} // done irResponse()

// ----------------------------------------------------------------[ printKey ]
void RTHerculesCommandIR::printKey( const uint16_t irKey )
{
	switch( irKey )
	{
		case wrt_ir_key38KHz_power:
		case wrt_ir_key_power:  			// turns on UUT power
			Serial.println("POWER");
			break;

		case wrt_ir_key38KHz_funcStop:
		case wrt_ir_key_funcStop:  			// FUNC/STOP turns off UUT power
			Serial.println("FUNC/STOP");
			break;

		case wrt_ir_key38KHz_fbkp:
		case wrt_ir_key_fbkp:  				// |<< ReTest failed Test
			Serial.println("|<<");
			break;

		case wrt_ir_key38KHz_fwrdEnd:
		case wrt_ir_key_fwrdEnd:  			// >|| Test
			Serial.println(">||");
			break;

		// >>| perform selected test number
		case wrt_ir_key38KHz_ffwrd:
		case wrt_ir_key_ffwrd:
			Serial.println(">>|");
			break;

		// VOL+ turns on individual test beeper
		case wrt_ir_key38KHz_volUp:
		case wrt_ir_key_volUp:
			Serial.println("VOL+");
			break;

		// VOL- turns off individual test beeper
		case wrt_ir_key38KHz_volDown:
		case wrt_ir_key_volDown:
			Serial.println("VOL-");
			break;

		case wrt_ir_key38KHz_down:
		case wrt_ir_key_down:  // v scroll down tests
			Serial.println("v");
			break;

		case wrt_ir_key38KHz_up:
		case wrt_ir_key_up:  // ^ scroll up tests
			Serial.println("^");
			break;

		case wrt_ir_key38KHz_eq:
		case wrt_ir_key_eq:  // EQ negative tests internal setup
			Serial.println("EQ");
			break;

		// ST/REPT Positive tests Select Test and Repeat Test
		case wrt_ir_key38KHz_srt:
		case wrt_ir_key_srt:
			Serial.println("ST/REPT");
			break;

		case wrt_ir_key38KHz_0:
		case wrt_ir_key_0:  // 0
			Serial.println("0");
			break;

		case wrt_ir_key38KHz_1:
		case wrt_ir_key_1:  // 1
			Serial.println("1");
			break;

		case wrt_ir_key38KHz_2:
		case wrt_ir_key_2:  // 2
			Serial.println("2");
			break;

		case wrt_ir_key38KHz_3:
		case wrt_ir_key_3:  // 3
			Serial.println("3");
			break;

		case wrt_ir_key38KHz_4:
		case wrt_ir_key_4:  // 4
			Serial.println("4");
			break;

		case wrt_ir_key38KHz_5:
		case wrt_ir_key_5:  // 5
			Serial.println("5");
			break;

		case wrt_ir_key38KHz_6:
		case wrt_ir_key_6:  // 6
			Serial.println("6");
			break;

		case wrt_ir_key38KHz_7:
		case wrt_ir_key_7:  // 7
			Serial.println("7");
			break;

		case wrt_ir_key38KHz_8:
		case wrt_ir_key_8:  // 8
			Serial.println("8");
			break;

		case wrt_ir_key38KHz_9:
		case wrt_ir_key_9:  // 9
			Serial.println("9");
			break;

		case wrt_ir_key_none:
		default:
		{
			isGoodKey = false;

			Serial.print("Key ");
			Serial.print( irKey);
			Serial.println(" not programmed");

			Serial.print("compiled: ");
			Serial.print(  __TIME__  );
			Serial.print(  ", "  );
			Serial.println( __DATE__ );

		} 	break;
	}	// switch

}	// done printKey()

// ------------------------------------------------------------------[ getKey ]
uint16_t RTHerculesCommandIR::getKey( void )
{
	uint16_t rVal = 0;

	uint16_t dVal = 0;
	unsigned char  header = 0;

	// ---  Decodes the received IR message
	// ---  returns 0 if no data ready, 1 if data ready.
	// --------------------------------------------------
	rVal = irRcv38KHz->decode( &irResults );
	// ---
	RTHerculesCommandIR::nTicks = static_cast<unsigned short>(rVal);
	// --------------------------------------------------

	if( rVal > 0 )
	{
		rVal   = 0;
		header = ((irResults.value & 0xFF0000) >> 16) ;
		// ----------------------------------------------
	    if( 0xFD == header )
	    {
	    	dVal = (irResults.value & 0xFFFF);

			if( verbose > 0 )
			{
				Serial.print( header  , HEX);
				Serial.print( ", " );
				Serial.print( dVal  , HEX);
				Serial.print( ", " );
				Serial.println(irResults.value, HEX);
			}

			// --- reset output
			rVal   = dVal;
	    }

	    irRcv38KHz->resume(); // Receive the next value
	}

	return( rVal );

}	// --- done getKey()


// ---------------------------------------------------------------------[ EoF ]
} /* namespace RT_HERCULES_IR_RECEIVER */
