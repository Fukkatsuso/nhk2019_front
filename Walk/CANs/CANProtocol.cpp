/*
 * CANProtocol.cpp
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */


#include "CANProtocol.h"


const short CANProtocol::CANFormats[CANID::DataType_end][FormatType::FormatType_end] =
{		//ID,				Len_integer,	Len_fraction
		{CANID::Period,		1,	4},	//Period
		{CANID::Duty,		1,	4},	//Duty
		{CANID::Speed,		3,	4},	//Speed
		{CANID::Direction,	2,	4},	//Direction
		{CANID::TimerReset,	1,	0},	//TimerReset
		{CANID::Area,		2,	0},	//Area
		{CANID::Gait,		1,	0},	//Gait
		{CANID::LegState,	1,	0}	//LegState
};


int CANID_generate(CANID::From from, CANID::To to)
{
	return ((from&0xf00) | (to&0x0f0));
}

int CANID_generate(CANID::From from, CANID::To to, CANID::DataType type)
{
	return ((from&0xf00) | (to&0x0f0) | (type&0x00f));
}

bool CANID_is_from(int id, CANID::From from)
{
	return ((id&0xf00) == (from&0xf00));
}

bool CANID_is_to(int id, CANID::To to)
{
	return ((id&0x0f0) == (to&0x0f0));
}

bool CANID_is_type(int id, CANID::DataType type)
{
	return ((id&0x00f) == (type&0x00f));
}
