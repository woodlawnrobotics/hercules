/**
 * File: WRobotController.h
 * Description:
 * WRobotController library is an easy to use robot controller for Arduino.
 *
 * Author: Miguel Simon
 * Contact: simon  at  ou punto edu
 * Copyright: Woodlawn School, Moresville, NC
 * Copying permission statement:
    This file is part of WRobotController.

    WRobotController is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
/*
 * WRobotController.h
 *
 *  Created on: May 5, 2013
 *      Author: simon
 */

#ifndef WROBOTCONTROLLER_H_
#define WROBOTCONTROLLER_H_

#include "Arduino.h"
#include "WRobotController_inc.h"

class WRobotController
{
public:
	WRobotController( void );
	~WRobotController( void );

	int init( const wr_pad_t & );
	int getState(   wr_pad_t & );

	int stepBehavior( const wr_mode_t );

	inline short setMotorDigitalPinLeft(  const short inMotorLeftPin )
	{
		WRobotController::iPad.pinMotorLeft = inMotorLeftPin;
	}

	inline short setMotorDigitalPinRight( const short inMotorRightPin )
	{
		WRobotController::iPad.pinMotorRight = inMotorRightPin;
	}

	inline short ping( void )
	{

	}
private:

	int moveForward( bool );
	int	moveToLeft(  void );
	int moveToRight( void );
	// ---
	int readInfraRedSensors( void );
	int getRobotCommand( void );


	// --- input pad
	// ---------------
	wr_pad_t iPad;

	static const short infraRedSensorThreshold;

	// --- other
	short verbose;

	int	  taskState;


};

#endif /* WROBOTCONTROLLER_H_ */
