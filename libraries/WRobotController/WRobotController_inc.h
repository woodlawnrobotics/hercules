/*
 * WRobotController_inc.h
 *
 *  Created on: May 5, 2013
 *      Author: simon
 */

#ifndef WROBOTCONTROLLER_INC_H_
#define WROBOTCONTROLLER_INC_H_


// ------------------------------------------
typedef enum  WR_TaskState_tag
{

  wr_taskState_iddle   	= 0,
  wr_taskState_step1   	= 1,
  wr_taskState_step2  	= 2,
  wr_taskState_step3 	= 3,
  wr_taskState_step4  	= 4,
  wr_taskState_step5 	= 5,

}  wr_taskState_t;

// ------------------------------------------
typedef enum  WR_IR_tag
{

  wr_ir_stop   = 0,
  wr_ir_left   = 0x0001,
  wr_ir_right  = 0x0002,
  wr_ir_foward = 0x0004,

}  wr_ir_t;

// ------------------------------------------
typedef enum  WR_Command_tag
{

  wr_command_stop   = 0,
  wr_command_left   = 0x0001,
  wr_command_right  = 0x0002,
  wr_command_foward = 0x0004,

}  wr_command_t;

// ------------------------------------------
typedef enum  WR_Mode_tag
{
  wr_mode_none     = 0,
  wr_mode_predator = 0x0001,
  wr_mode_pray     = 0x0002,
  wr_mode_swarm    = 0x0004,
  wr_mode_leader   = 0x0008,
  wr_mode_follower = 0x0010,

}  wr_mode_t;


// ------------------------------------------
typedef struct  WR_Pad_tag
{
	wr_ir_t   		irState;
	wr_command_t  	dRobotCommand;
	wr_mode_t  		dRobotMode;

	short dMotorLeft;
	short dMotorRight;

	short pinMotorLeft;
	short pinMotorRight;

	short irRightChannel;
	short irLeftChannel;

	short irRightVal;        // variable to store the values from sensor(initially zero)
	short irLeftVal;         // variable to store the values from sensor(initially zero)

} wr_pad_t;


#endif /* WROBOTCONTROLLER_INC_H_ */
