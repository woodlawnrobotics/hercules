/*
 * RTsys_def.h
 *
 *  Created on: May 19, 2014
 *      Author: simon
 */

#ifndef RTSYS_DEF_H_
#define RTSYS_DEF_H_


// --- specify if Servo library should be used
// --- note: Servo library uses timer.1
// --------------------------------------------------------
#if defined( WRT_ROBOT_USE_SERVO_LIB )
#undef WRT_ROBOT_USE_SERVO_LIB
#endif
//#define WRT_ROBOT_USE_SERVO_LIB	1

// --- specify if IRremote library should be used
// --- note: IRremote library uses timer.4 in Leonardo, and timer.2 in Uno
// -------------------------------------------------------------------------
#if defined( WRT_ROBOT_USE_IR_REMOTE_LIB )
#undef WRT_ROBOT_USE_IR_REMOTE_LIB
#endif
#define WRT_ROBOT_USE_IR_REMOTE_LIB	1


// CPU Type and How to Determine Arduino Bord
// According to selected target processor, compiler defined constant by processor name.
// ---------------------------------------------------------------------------
// Constant					CPU							Board
//__AVR_ATmega168__		ATmega 168			Arduino Decimilia and older
//__AVR_ATmega328P__	ATmega 328P			Arduino Duemilanove and Uno
//__AVR_ATmega1280__	ATmega 1280			Arduino Mega
//__AVR_ATmega2560__	ATmega 2560			Arduino Mega 2560
//__AVR_ATmega32U4__	ATmega 32U4			Arduino Leonardo
//__SAM3X8E__			AT91SAM3X8E			Arduino Due
// ---------------------------------------------------------------------------

// --- define active robot:
// ---------------------------------------------[ Arduino Duemilanove and Uno ]
#if defined(__AVR_ATmega328P__)  //

// -DARDUINO_AVR_DUEMILANOVE

#if defined( WRT_ROBOT_MinQ4wd )
#undef WRT_ROBOT_MinQ4wd
#endif
// #define WRT_ROBOT_MinQ4wd 1

// ---------------------------------------------------------------------------
// note: hercules board: Atmege328 controller
//		 Tools->Board->Arduino Duemilanove/ATmega328.
// ---------------------------------------------------------------------------
#if defined( WRT_ROBOT_Hercules )
#undef WRT_ROBOT_Hercules
#endif
#define WRT_ROBOT_Hercules 1
// ---------------------------------------------------------------------------
#endif	// __AVR_ATmega328P__

// --------------------------------------------------------[ Arduino Leonardo ]
#if defined(__AVR_ATmega32U4__)

#if defined( WRT_ROBOT_LEONARDO )
#undef WRT_ROBOT_LEONARDO
#endif
#define WRT_ROBOT_LEONARDO	1

// --- the Romeo v.2.1 is a Leonardo board
#if defined( WRT_ROBOT_Romeo )
#undef WRT_ROBOT_Romeo
#endif
#define WRT_ROBOT_Romeo 1

#endif	// __AVR_ATmega32U4__

// -------------------------------------------------------------[ Arduino Due ]
#if defined( __SAM3X8E__ )

#if defined( WRT_ROBOT_SAM )
#undef WRT_ROBOT_SAM
#endif
#define WRT_ROBOT_SAM 1

#endif 	// __SAM3X8E__


// Architecture specific include
#if defined(ARDUINO_ARCH_AVR)
// ---------------------------------------------------------------------[ AVR ]
// -DARDUINO_ARCH_AVR

#if defined( WRT_ROBOT_AVR )
#undef WRT_ROBOT_AVR
#endif
#define WRT_ROBOT_AVR 1

// ---------------------------------------------------------------------[ AVR ]
#elif defined(ARDUINO_ARCH_SAM)
// ---------------------------------------------------------------------[ SAM ]

#if defined( WRT_ROBOT_USE_IR_REMOTE_LIB )
#undef WRT_ROBOT_USE_IR_REMOTE_LIB
#error "IRremote lib can only be used in the AVR architecture for now"
#endif

#if defined( WRT_ROBOT_Romeo ) || defined(WRT_ROBOT_MinQ4wd) || defined( WRT_ROBOT_LEONARDO ) || defined( WRT_ROBOT_Hercules )
#error "no Romeo or Uno or Leonardo or Duemilanove boards for SAM arch"
#endif


// ---------------------------------------------------------------------[ SAM ]
#else
#error "This program only supports boards with an AVR or SAM processor"
#endif

// -------------------------------------------------------------[ error catch ]
#if defined( WRT_ROBOT_SAM ) && defined( WRT_ROBOT_AVR )
#error "cannot have AVR and SAM arch simultaneously"
#endif

#if defined( WRT_ROBOT_SAM ) && defined( WRT_ROBOT_Romeo )
#error "the Romeo board is not compatible with SAM arch"
#endif

#if defined( WRT_ROBOT_SAM ) && defined( WRT_ROBOT_MinQ4wd )
#error "the UNO board is not compatible with SAM arch"
#endif

#if defined( WRT_ROBOT_SAM ) && defined( WRT_ROBOT_Hercules )
#error "the Duemilanove board is not compatible with SAM arch"
#endif


#endif /* RTSYS_DEF_H_ */
