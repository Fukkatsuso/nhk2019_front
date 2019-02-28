/*
 * CANProtocol.h
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

#ifndef WALK_CANS_CANPROTOCOL_H_
#define WALK_CANS_CANPROTOCOL_H_

#include "mbed.h"

struct CANID{
	enum From{
		FromMaster = 0x000,
		FromSlave = 0x100,
		FromFront = 0x200,
		FromRear = 0x300,
		FromFR = 0x400,
		FromFL = 0x500,
		FromRR = 0x600,
		FromRL = 0x700
	};
	enum To{
		ToMaster = 0x000,
		ToSlaveAll = 0x010,
		ToFront = 0x020,
		ToRear = 0x030,
		ToFR = 0x040,
		ToFL = 0x050,
		ToRR = 0x060,
		ToRL = 0x070
	};
	enum DataType{
		Period=0,
		Duty,
		Speed,
		Direction,
		TimerReset,
		Area,
		Gait,//番号で歩容を見る	//保留
		LegState,//Slaveから送信	//保留
		DataType_end//<=0x00f=15 に制限（仕様上）
	};
};

int CANID_generate(CANID::From from, CANID::To to);
int CANID_generate(CANID::From from, CANID::To to, CANID::DataType type);
bool CANID_is_from(int id, CANID::From from);
bool CANID_is_to(int id, CANID::To to);
bool CANID_is_type(int id, CANID::DataType type);


class CANProtocol{
protected:
	CAN *can;

	static struct FormatType{
		enum{
			ID=0, //ID
			Len_integer, //整数部分長
			Len_fraction, //小数部分長
			FormatType_end
		};
	};

	static short CANFormats[CANID::DataType_end][FormatType::FormatType_end] =
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
};

/*
CANCommand::CANCommand(CAN *can)
{
	this->can = can;
	//一応初期化
	rcvData[CANID::Period] = 0;
	rcvData[CANID::Duty] = 1;
	rcvData[CANID::Speed] = 0;
	rcvData[CANID::Direction] = 0;
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
*/

#endif /* WALK_CANS_CANPROTOCOL_H_ */
