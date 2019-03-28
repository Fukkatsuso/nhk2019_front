/*
 * CANSender.cpp
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

#include "CANSender.h"


CANSender::CANSender(CAN *can)
{
	this->can = can;
}


void CANSender::send(int id_can, float value_f)
{
	const char dammy=0;
	CANID::DataType type = (CANID::DataType)(id_can & 0x00f); //idの下1桁が送信データの種類を表す
	int len_i = CANFormats[type][FormatType::Len_integer];
	int len_f = CANFormats[type][FormatType::Len_fraction];
	store_value_in_data(type, value_f, len_i, len_f);
	CANMessage msg(id_can, &dammy, len_i+len_f+1); //+1:符号用
	copy_data_in_msg(&msg, type, len_i+len_f+1);
	if(can->write(msg));
	else if(can->write(msg));
	else can->write(msg);
}


/************************
 * 		protected		*
 ************************/
void CANSender::store_value_in_data(enum CANID::DataType type, float value_f, int len_i, int len_f)
{
	for(int i=0; i<8; i++){
		data[type][i] = 0; //初期化
	}
	data[type][0] = (value_f>=0? 0:1);
	value_f = fabs(value_f);
	for(int i=1; i<=len_i; i++)
		data[type][i] = (int)(value_f/pow(10, len_i-i))%10;
	for(int i=1+len_i; i<=1+len_i+len_f; i++)
		data[type][i] = (int)(value_f*pow(10, i-len_i))%10;
}


void CANSender::copy_data_in_msg(CANMessage *msg, enum CANID::DataType type, int len)
{
	for(int i=0; i<len; i++)
		msg->data[i] = data[type][i] & 0xff;
}
