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
}
