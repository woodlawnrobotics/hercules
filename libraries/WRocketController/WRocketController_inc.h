/*
 * WRocketController_inc.h
 *
 *  Created on: May 12, 2013
 *      Author: simon
 */

#ifndef WROCKETCONTROLLER_INC_H_
#define WROCKETCONTROLLER_INC_H_


// ------------------------------------------
typedef struct  WR_Pad_tag
{
	short logSerialPortBaudRate; //Set this value equal to the baud rate of your GPS

	short humiditySensorPin; 	// #define DHT11PIN 2;

} wr_pad_t;



#endif /* WROCKETCONTROLLER_INC_H_ */
