/*
 * CANCommand.cpp
 *
 *  Created on: 2019/02/18
 *      Author: mutsuro
 */

#include "CANCommand.h"


short CANReceiveFormats[CANCommand::ReceiveDataType_end][CANCommand::ReceiveFormatType_end] =
{		//ID,	Len_integer,	Len_fraction
		{CANCommand::Period,	1,	4},	//Period
		{CANCommand::Duty,		1,	4},	//Duty
		{CANCommand::Speed,		3,	4},	//Speed
		{CANCommand::Direction,	2,	4},	//Direction
		{CANCommand::Time,		2,	4},	//Time
		{CANCommand::Area,		2,	0}	//Area
};

CANCommand::CANCommand(CAN *can)
{
	this->can = can;
}


/*
 * example:
 * CANCommand cancmd(&can);
 * void can_receive(){
 * 		if(can.read(rcvMsg)){
 * 			if((rcvMsg.id&0xff0)==ID_sndAll){
 * 				cancmd.recerive(rcvMsg.id, rcvMsg.data);
 * 			}
 * 		}
 * }
 */
void CANCommand::receive(unsigned int id, unsigned char data[])
{
	id = (id)&(0x00f);//下一桁のみ取り出す
	if(0<=id && id<ReceiveDataType_end)
		rcvData[id] = decode_from_array(data, CANReceiveFormats[id][Len_integer], CANReceiveFormats[id][Len_fraction]);
}


float CANCommand::get(ReceiveDataType type){
	return rcvData[type];
}


float CANCommand::decode_from_array(unsigned char array[], int len_i, int len_f)
{
	float value=0;
	for(int i=1; i<=len_i; i++)
		value += array[i]*(int)pow(10, len_i-i);
	for(int i=len_i+1; i<=len_i+len_f; i++)
		value += (float)array[i]*(float)pow(0.1, i-len_i);
	if(array[0])value *= -1;
	return value;
}
