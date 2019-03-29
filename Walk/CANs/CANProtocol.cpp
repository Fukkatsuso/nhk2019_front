/*
 * CANProtocol.cpp
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */


#include "CANProtocol.h"


/********
 * CANID
 ********/
unsigned int CANID::generate(CANID::From from, CANID::To to, CANID::DataType type)
{
	return ((from&0xf00) | (to&0x0f0) | (type&0x00f));
}

bool CANID::is_from(unsigned int can_id, CANID::From from)
{
	return ((can_id&0xf00) == (from&0xf00));
}

bool CANID::is_to(unsigned int can_id, CANID::To to)
{
	return ((can_id&0x0f0) == (to&0x0f0));
}

bool CANID::is_type(unsigned int can_id, CANID::DataType type)
{
	return ((can_id&0x00f) == (type&0x00f));
}


/**************
 * CANProtocol
 **************/
CANProtocol::CANProtocol(CAN *can)
{
	this->can = can;

	data.direction.value 					= 0;
	data.speed.value 						= 0;
	data.velocity_vector.direction.value	= 0;
	data.velocity_vector.speed.value 		= 0;
	data.timer_reset.value 					= 0;
	data.leg_up.FR 							= 0;
	data.leg_up.FL 							= 0;
	data.leg_up.RR 							= 0;
	data.leg_up.RL 							= 0;
	data.move_dist_front.value 				= 0;
	data.move_dist_rear.value 				= 0;
}


float CANProtocol::get_direction()
{
	return data.direction.value;
}

float CANProtocol::get_speed()
{
	return data.speed.value;
}

unsigned short CANProtocol::get_timer_reset()
{
	return (unsigned short)data.timer_reset.value;
}

unsigned char CANProtocol::get_area()
{
	return data.area.value;
}

unsigned short CANProtocol::get_area_change()
{
	return (unsigned short)data.area_change.value;
}

bool CANProtocol::get_leg_up_FR()
{
	return (bool)data.leg_up.FR;
}

bool CANProtocol::get_leg_up_FL()
{
	return (bool)data.leg_up.FL;
}

bool CANProtocol::get_leg_up_RR()
{
	return (bool)data.leg_up.RR;
}

bool CANProtocol::get_leg_up_RL()
{
	return (bool)data.leg_up.RL;
}

short CANProtocol::get_move_dist_front()
{
	return data.move_dist_front.value;
}

short CANProtocol::get_move_dist_rear()
{
	return data.move_dist_rear.value;
}
