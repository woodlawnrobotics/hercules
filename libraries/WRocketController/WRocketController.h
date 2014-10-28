/*
 * WRocketController.h
 *
 *  Created on: May 12, 2013
 *      Author: simon
 */

#ifndef WROCKETCONTROLLER_H_
#define WROCKETCONTROLLER_H_

#include <Arduino.h>

// --- Humidity Sensor Package
#include <dht11.h>

// In order for this sketch to work, you will need to download
// TinyGPS library from arduiniana.org and put them
// into the hardware->libraries folder in your ardiuno directory.
//#include <SoftwareSerial.h>
#include <TinyGPS.h>

#include <WRocketController_inc.h>

class WRocketController
{
public:
	WRocketController();
	~WRocketController();

	int init( wr_pad_t & );

	// --- gps
	int readGPS( void );


	// --- humidity sensor
	int        readHumiditySensor( void );
	int	readHumiditySensorVerbose( void );


private:

	// --- instance of the TinyGPS object
	TinyGPS gps;

	// --- instance of DHT11 humidity and temperature sensor
	dht11 humidtySensor;

	wr_pad_t iPad;

	// --- gps sensor
	// -----------------------------------------
	int logGPS( void );
	int logVerboseGPS( void );

	// --- humidity sensor
	// -----------------------------------------
	double Fahrenheit(double );
	double Kelvin(double );
	double dewPoint(double , double );
	double dewPointFast(double , double  );
	// -----------------------------------------


};

#endif /* WROCKETCONTROLLER_H_ */
