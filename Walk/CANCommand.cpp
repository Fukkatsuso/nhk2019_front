/*
 * CANCommand.cpp
 *
 *  Created on: 2019/02/18
 *      Author: mutsuro
 */

#include "CANCommand.h"


short CANFormats[CANID::DataType_end][CANCommand::FormatType::FormatType_end] =
{		//ID,				Len_integer,	Len_fraction
		{CANID::Period,		1,	4},	//Period
		{CANID::Duty,		1,	4},	//Duty
		{CANID::Speed,		3,	4},	//Speed
		{CANID::Direction,	2,	4},	//Direction
		{CANID::Time,		2,	4},	//Time
		{CANID::Area,		2,	0},	//Area
		{CANID::Gait,		1,	0}	//Gait
};

int CANID_generate(CANID::From from, CANID::To to)
{
	return ((from&0x100) | (to&0x010));
}

int CANID_generate(CANID::From from, CANID::To to, CANID::DataType type)
{
	return ((from&0x100) | (to&0x010) | (type&0x001));
}

bool CANID_is_from(int id, CANID::From from)
{
	return ((id&0x100) == (from&0x100));
}

bool CANID_is_to(int id, CANID::To to)
{
	return ((id&0x010) == (to&0x010));
}

CANCommand::CANCommand(CAN *can)
{
	this->can = can;
}


void CANCommand::send(int id, float data)
{
	const char dammy=0;
	int type = id & 0x001; //idの下1桁が送信データの種類を表す
	int len_i = CANFormats[type][FormatType::Len_integer];
	int len_f = CANFormats[type][FormatType::Len_fraction];
	CANMessage msg(id, &dammy, len_i+len_f+1); //+1:符号用
	store_in_data(data, len_i, len_f);
	copy_data(&msg, len_i+len_f+1);
	if(can->write(msg));
	else if(can->write(msg));
	else can->write(msg);
}


/*
 * example:
 * CANCommand cancmd(&can);
 * void can_receive(){
 * 		if(can.read(rcvMsg)){
 * 			if((rcvMsg.id&0x110)==(CANID::From::Master | CANID::To::SlaveAll)){
 * 				cancmd.recerive(rcvMsg.id, rcvMsg.data);
 * 			}
 * 		}
 * }
 */
void CANCommand::receive(unsigned int id, unsigned char data[])
{
	id = (id)&(0x001);//下一桁のみ取り出す
	if(0<=id && id<CANID::DataType_end)
		rcvData[id] = decode_from_array(
				data,
				CANFormats[id][FormatType::Len_integer],
				CANFormats[id][FormatType::Len_fraction]
									  );
}


float CANCommand::get(CANID::DataType type){
	return rcvData[type];
}

int CANCommand::get_area(){
	return (int)rcvData[CANID::Area];
}

int CANCommand::get_gait(){
	return (int)rcvData[CANID::Gait];
}


/************************
 * 		protected		*
 ************************/
void CANCommand::store_in_data(float f, int len_i, int len_f)
{
	for(int i=0; i<8; i++){
		sndData[i] = 0; //初期化
	}
	sndData[0] = (f>=0? 1:-1);
	f = fabs(f);
	for(int i=1; i<=len_i; i++)
		sndData[i] = (int)(f/pow(10, len_i-i))%10;
	for(int i=1+len_i; i<=1+len_i+len_f; i++)
		sndData[i] = (int)(f*pow(10, i-len_i))%10;
}


void CANCommand::copy_data(CANMessage *msg, int len)
{
	for(int i=0; i<len; i++)
		msg->data[i] = sndData[i] & 0x11;
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
