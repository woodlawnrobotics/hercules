/*
 * WRocketController.cpp
 *
 *  Created on: May 12, 2013
 *      Author: simon
 */


#include <stdlib.h>
#include <stdio.h>

#include <Arduino.h>
#include <dht11.h>


#include "WRocketController.h"

WRocketController::WRocketController()
{
	iPad.humiditySensorPin = 2;

}

WRocketController::~WRocketController()
{
	// TODO Auto-generated destructor stub
}


int WRocketController::init( wr_pad_t &userPad )
{
	memcpy( (char *) &iPad, (char *) &userPad,  sizeof(struct  WR_Pad_tag) );
	// ---------------------
	iPad.humiditySensorPin 		= userPad.humiditySensorPin;
	iPad.logSerialPortBaudRate  = userPad.logSerialPortBaudRate;

	return( 1 );

}	// done init()

// --- the readGPS function will get and print the values we want.
int WRocketController::readGPS( void )
{
	int c = 0;

//  while(uart_gps.available())     // While there is data on the RX pin...
  while(Serial2.available())     // While there is data on the RX pin...
  {
      c = Serial2.read();    // load the data into a variable...
      if(gps.encode(c))      // if there is a new valid sentence...

      if( c > 0 )
      {
    	  logGPS();         // then grab the data.

      }
      else if( c == -1 )
      {
        Serial2.flush();



        break;
      }

  } // while

	return( 1 );

}	// done readGPS()

// --- the logGPS function will get and print the values we want.
int WRocketController::logGPS( void )
{
  // To get all of the data into varialbes that you can use in your code,
  // all you need to do is define variables and query the object for the
  // data. To see the complete list of functions see keywords.txt file in
  // the TinyGPS and NewSoftSerial libs.

  // Define the variables that will be used
  float latitude, longitude;

  // Same goes for date and time
  int year;
  byte month, day, hour, minute, second, hundredths;


	  gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths);
	//  // Print data and time
	//  Serial.print("Date: "); Serial.print(month, DEC); Serial.print("/");
	//  Serial.print(day, DEC); Serial.print("/"); Serial.print(year);
	//  Serial.print("  Time: "); Serial.print(hour, DEC); Serial.print(":");
	//  Serial.print(minute, DEC); Serial.print(":"); Serial.print(second, DEC);
	//  Serial.print("."); Serial.println(hundredths, DEC);
	//  //Since month, day, hour, minute, second, and hundr

	Serial.print(year);         Serial.print(" ");
	Serial.print(month, DEC);   Serial.print(" ");
	Serial.print(day, DEC);     Serial.print(" ");

	Serial.print(hour, DEC);    Serial.print(" ");
	Serial.print(minute, DEC);  Serial.print(" ");
	Serial.print(second, DEC);  Serial.print(" ");

	//  Serial.print("."); Serial.println(hundredths, DEC);
	//  //Since month, day, hour, minute, second, and hundr

	  // Then call this function
	  gps.f_get_position(&latitude, &longitude);

	  // You can now print variables latitude and longitude
	//  Serial.print("Lat/Long: ");
	//  Serial.print(latitude,5);
	//  Serial.print(", ");
	//  Serial.println(longitude,5);

	Serial.print(latitude,7);  Serial.print(" ");
	Serial.print(longitude,7); Serial.print(" ");



	  // Here you can print the altitude and course values directly since
	  // there is only one value for the function
	//  Serial.print("Altitude (meters): "); Serial.println(gps.f_altitude());
	//  // Same goes for course
	//  Serial.print("Course (degrees): "); Serial.println(gps.f_course());
	//  // And same goes for speed
	//  Serial.print("Speed(kmph): "); Serial.println(gps.f_speed_kmph());
	//  Serial.println();

	Serial.print(gps.f_altitude());    Serial.print(" ");
	Serial.print(gps.f_course());      Serial.print(" ");
	Serial.print(gps.f_speed_kmph());  Serial.print(" ");



	  // Here you can print statistics on the sentences.
	//  unsigned long chars;
	//  unsigned short sentences, failed_checksum;
	//  gps.stats(&chars, &sentences, &failed_checksum);
	  //Serial.print("Failed Checksums: ");Serial.print(failed_checksum);
	  //Serial.println(); Serial.println();

	return( 1 );

}	// done logGPS()

// --- the readGPS function will get and print the values we want.
int WRocketController::logVerboseGPS( void )
{
  // To get all of the data into varialbes that you can use in your code,
  // all you need to do is define variables and query the object for the
  // data. To see the complete list of functions see keywords.txt file in
  // the TinyGPS and NewSoftSerial libs.

  // Define the variables that will be used
  float latitude, longitude;

  // Same goes for date and time
  int year;
  byte month, day, hour, minute, second, hundredths;


	  gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths);
	  // Print data and time
	  Serial.print("Date: "); Serial.print(month, DEC); Serial.print("/");
	  Serial.print(day, DEC); Serial.print("/"); Serial.print(year);
	  Serial.print("  Time: "); Serial.print(hour, DEC); Serial.print(":");
	  Serial.print(minute, DEC); Serial.print(":"); Serial.print(second, DEC);
	  Serial.print("."); Serial.println(hundredths, DEC);
	  //Since month, day, hour, minute, second, and hundr



	//  Serial.print("."); Serial.println(hundredths, DEC);
	//  //Since month, day, hour, minute, second, and hundr

	  // Then call this function
	  gps.f_get_position(&latitude, &longitude);

	  // You can now print variables latitude and longitude
	  Serial.print("Lat/Long: ");
	  Serial.print(latitude,5);
	  Serial.print(", ");
	  Serial.println(longitude,5);



	  // Here you can print the altitude and course values directly since
	  // there is only one value for the function
	  Serial.print("Altitude (meters): "); Serial.println(gps.f_altitude());
	  // Same goes for course
	  Serial.print("Course (degrees): "); Serial.println(gps.f_course());
	  // And same goes for speed
	  Serial.print("Speed(kmph): "); Serial.println(gps.f_speed_kmph());
	  Serial.println();


	  // Here you can print statistics on the sentences.
	//  unsigned long chars;
	//  unsigned short sentences, failed_checksum;
	//  gps.stats(&chars, &sentences, &failed_checksum);
	  //Serial.print("Failed Checksums: ");Serial.print(failed_checksum);
	  //Serial.println(); Serial.println();

	return( 1 );

}	// done logVerboseGPS()


int WRocketController::readHumiditySensor( void )
{
	  int chk = 0;

	  chk = humidtySensor.read( WRocketController::iPad.humiditySensorPin );

	  // --- error
	  Serial.print (chk);
	  Serial.print(" ");

	  // --- Humidity (%):;
	  Serial.print((float)humidtySensor.humidity, 2);
	  Serial.print(" ");

	  // --- Temperature (oC):
	  Serial.print((float)humidtySensor.temperature, 2);
	  Serial.print(" ");

	  // --- Temperature (oF):
	  Serial.print(Fahrenheit(humidtySensor.temperature), 2);
	  Serial.print(" ");

	  // --- Temperature (K):
	  Serial.print(Kelvin(humidtySensor.temperature), 2);
	  Serial.print(" ");

	  // --- Dew Point (oC): ");
	  Serial.print(dewPoint(humidtySensor.temperature, humidtySensor.humidity));
	  Serial.print(" ");

	  // --- Dew PointFast (oC):
	  Serial.print(dewPointFast(humidtySensor.temperature, humidtySensor.humidity));
	  Serial.print(" ");

	  return( chk );

}	// done readHumiditySensor()


int WRocketController::readHumiditySensorVerbose( void )
{

	  int chk = humidtySensor.read( WRocketController::iPad.humiditySensorPin );

	  Serial.print("Read sensor: ");
	  switch (chk)
	  {
	    case DHTLIB_OK:
			Serial.println("OK");
			break;
	    case DHTLIB_ERROR_CHECKSUM:
			Serial.println("Checksum error");
			break;
	    case DHTLIB_ERROR_TIMEOUT:
			Serial.println("Time out error");
			break;
	    default:
			Serial.println("Unknown error");
			break;
	  }

	  Serial.print("Humidity (%): ");
	  Serial.println((float)humidtySensor.humidity, 2);

	  Serial.print("Temperature (oC): ");
	  Serial.println((float)humidtySensor.temperature, 2);

	  Serial.print("Temperature (oF): ");
	  Serial.println(Fahrenheit(humidtySensor.temperature), 2);

	  Serial.print("Temperature (K): ");
	  Serial.println(Kelvin(humidtySensor.temperature), 2);

	  Serial.print("Dew Point (oC): ");
	  Serial.println(dewPoint(humidtySensor.temperature, humidtySensor.humidity));

	  Serial.print("Dew PointFast (oC): ");
	  Serial.println(dewPointFast(humidtySensor.temperature, humidtySensor.humidity));

}	// done readHumiditySensorVerbose()

// --- Celsius to Kelvin conversion
double WRocketController::Kelvin(double celsius)
{
	return celsius + 273.15;
}

//Celsius to Fahrenheit conversion
double WRocketController::Fahrenheit(double celsius)
{
	return 1.8 * celsius + 32;
}

// dewPoint function NOAA
// reference: http://wahiduddin.net/calc/density_algorithms.htm
double WRocketController::dewPoint(double celsius, double humidity)
{
	double RATIO = 373.15 / (273.15 + celsius);  // RATIO was originally named A0, possibly confusing in Arduino context
	double SUM = -7.90298 * (RATIO - 1);
	SUM += 5.02808 * log10(RATIO);
	SUM += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
	SUM += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
	SUM += log10(1013.246);
	double VP = pow(10, SUM - 3) * humidity;
	double T = log(VP/0.61078);   // temp var
	return (241.88 * T) / (17.558 - T);
}

// delta max = 0.6544 wrt dewPoint()
// 5x faster than dewPoint()
// reference: http://en.wikipedia.org/wiki/Dew_point
double WRocketController::dewPointFast(double celsius, double humidity)
{
	double a = 17.271;
	double b = 237.7;
	double temp = (a * celsius) / (b + celsius) + log(humidity/100);
	double Td = (b * temp) / (a - temp);
	return Td;
}


