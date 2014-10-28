// *
// * RTblueTooth.cpp
// *
// *  Created on: Feb 9, 2014
// *      Author: simon
// *
// ---------------------------------------------------------------------[ BoF ]

#include "RTblueTooth.h"

namespace RT_BLUETOOTH
{


const int RTblueTooth::SerialBufferSize = 0x1000; // one page
// ------------------------------------------------------------ [ constructor ]
RTblueTooth::RTblueTooth()
{
	btBuffer = new uint8_t[RTblueTooth::SerialBufferSize];
	memset( (char *) btBuffer, 0, RTblueTooth::SerialBufferSize);
	// ---
	memset( (char *) &pad, 0, sizeof( struct RT_IO_Param_tag ) );

}	// done constructor

RTblueTooth::~RTblueTooth()
{
	delete this->btBuffer;

}	// done destructor

// --------------------------------------------------------------------[ init ]
void RTblueTooth::init( void )
{
	pad.mode = static_cast<uint16_t>(wrt_due_io_slave);

}	// done init()

// ----------------------------------------------------------------[ initMode ]
void RTblueTooth::initMode( void )
{
	int dVal = 0;

	dVal = digitalRead( wrt_due_digitalPin_id_master );
	// -----------------------------------------------------------------
	if( HIGH == dVal )
	{
		pad.mode = static_cast<uint16_t>( wrt_due_io_master );
		pad.id   = pad.mode;
	}
	else
	{	pad.mode = static_cast<uint16_t>( wrt_due_io_slave );

	//			4	3  	2  	1 	 0       	<-- ID pin nomenclature
	// --- 	35 	34	33 	32 	31 	30			<-- digital pin
	//		1  	x	x  	x   x	x			ID is master
	//		0  	0	0  	0   0	0				id 0
	//		0  	0	0  	0   0	1				id 1
	//				...
	// ---------------------------------------------------------------
		dVal = digitalRead( wrt_due_digitalPin_id_3 );
		pad.id = (static_cast<uint16_t>(dVal) << 3);

		dVal = digitalRead( wrt_due_digitalPin_id_2 );
		pad.id |= (static_cast<uint16_t>(dVal) << 2);

		dVal = digitalRead( wrt_due_digitalPin_id_1 );
		pad.id |= (static_cast<uint16_t>(dVal) << 1);

		dVal = digitalRead( wrt_due_digitalPin_id_0 );
		pad.id |= static_cast<uint16_t>(dVal) ;

	} // else

}	// done initMode()

// --------------------------------------------------------------[ readSerial ]
int RTblueTooth::readSerial( void )
{
	rt_ioSerialData_t *dat = (rt_ioSerialData_t *) btBuffer;

	pad.nbytesRx = 0;

	// --- bluetooth i/o
	// ----------------------------------------------------
	pad.sizeRx = Serial1.available();
	// ----------------------------------------------------
	if( pad.sizeRx > 0)
	{
		// read the incoming byte:
		pad.nbytesRx = Serial.readBytes( (char *) btBuffer, pad.sizeRx );

		// say what you got:
		Serial.print("bt received: ");
		Serial.print( pad.nbytesRx , DEC );
		Serial.print(", synch: ");
		Serial.println( dat->hdr.synch , HEX );

	}



	return( pad.nbytesRx );

}	// done readSerial()

// ------------------------------------------------------------[ writeSerial ]
int RTblueTooth::writeSerial( void )
{
	memset( &txData, 0, sizeof(struct RT_IO_SerialData_tag) );
	// --------------------------------------------------------------------
	txData.hdr.synch 	= static_cast<uint16_t>(wrt_due_io_com_synch);
	txData.hdr.id 		= pad.id;	// --- unique unit id
	txData.hdr.counter  = pad.counter++;	// --- message counter
	txData.hdr.size     = 0; 		// --- size of body data
	txData.hdr.cmnd     = 0;		// --- command (from master), ack (from slave)
	txData.hdr.flag     = 0;		// --- context flag (depends on command or status)
	txData.hdr.chkSumPLoad = 0;		// --- data payload checksum
	txData.hdr.chkSum   = 0;		// --- header checksum
	// --------------------------------------------------------------------
	switch( pad.mode )
	{	default:
		case wrt_due_io_slave:
		{
			txData.hdr.cmnd = static_cast<uint16_t>(wrt_due_io_com_ack_OK );

		}	break;
		case wrt_due_io_master:
		{
			txData.hdr.cmnd = static_cast<uint16_t>(wrt_due_io_com_command_OK);

		}	break;
	}
	// --------------------------------------------------------------------
	pad.nbytesTx  = sizeof(struct RT_IO_Header_tag);
	pad.nbytesTx += txData.hdr.size;
	// ---
	Serial1.write( pad.nbytesTx );

}	// done writeSerial()

// ---------------------------------------------------------------------[ EoF ]
} /* namespace RT_BLUETOOTH */
