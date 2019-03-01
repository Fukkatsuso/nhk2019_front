/*
 * CANReceiver.cpp
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

#include "CANReceiver.h"


CANReceiver::CANReceiver(CAN *can)
{
	this->can = can;
}


/*
 * example:
 * CANReceiver canrcv(&can);
 * void can_receive(){
 * 		if(can.read(rcvMsg)){
 * 			if((rcvMsg.id&0x110)==(CANID::From::Master | CANID::To::SlaveAll)){
 * 				canrcv.recerive(rcvMsg.id, rcvMsg.data);
 * 			}
 * 		}
 * }
 */
void CANReceiver::receive(unsigned int id_can, unsigned char data_can[])
{
	id_can = (id_can)&(0x001);//下一桁のみ取り出す
	if(id_can<0 || CANID::DataType_end<=id_can)return;
	data[id_can] = decode_from_candata(data_can,
			CANFormats[id_can][FormatType::Len_integer], CANFormats[id_can][FormatType::Len_fraction]);
}


float CANReceiver::get_data(enum CANID::DataType type)
{
	return data[type];
}

int CANReceiver::get_area()
{
	return data[CANID::Area];
}


/************************
 * 		protected		*
 ************************/
float CANReceiver::decode_from_candata(unsigned char data_can[], int len_i, int len_f)
{
	float value=0;
	for(int i=1; i<=len_i; i++)
		value += data_can[i]*(int)pow(10, len_i-i);
	for(int i=len_i+1; i<=len_i+len_f; i++)
		value += (float)data_can[i]*(float)pow(0.1, i-len_i);
	if(data_can[0])value *= -1;
	return value;
}
