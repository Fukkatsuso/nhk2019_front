/*
 * CANSender.h
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

#ifndef WALK_CANS_CANSENDER_H_
#define WALK_CANS_CANSENDER_H_

#include "mbed.h"
#include "CANProtocol.h"


class CANSender : public CANProtocol
{
public:
	CANSender(CAN *can);
	void send(int id_can, float value_f);

protected:
	void store_value_in_data(enum CANID::DataType type, float value_f, int len_i, int len_f);
	void copy_data_in_msg(CANMessage *msg, enum CANID::DataType type, int len);

private:
	int data[CANID::DataType_end][8];//データを送信用に変換して一時的に保存しておく配列
};


#endif /* WALK_CANS_CANSENDER_H_ */
