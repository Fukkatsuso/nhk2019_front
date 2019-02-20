/*
 * CANCommand.h
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

//新たに受信する変数を追加するときには
//ReceiveDataType の末尾(_endの前)に変数名を追加すること
//CANReceiveFormats[][]の末尾に要素を追加すること


#ifndef WALK_CANCOMMAND_H_
#define WALK_CANCOMMAND_H_

#include "mbed.h"


/*
 * 中央制御用マイコンとCAN通信
 */
class CANCommand
{
public:
	enum ReceiveDataType{
		Period=0,
		Duty,
		Speed,
		Direction,
		Time,
		Area,
		Gait,
		ReceiveDataType_end//<=0x00f=15 に制限（仕様上）
	};

	enum ReceiveFormatType{
		ID=0, //ID
		Len_integer, //整数部分長
		Len_fraction, //小数部分長
		ReceiveFormatType_end
	};

	CANCommand(CAN *can);
	void receive(unsigned int id, unsigned char data[]);
	//void send(int id, float data);後回し

	float get(enum ReceiveDataType type);
	int get_area();
	int get_gait();

protected:
	float decode_from_array(unsigned char array[], int len_i, int len_f);
	void store_in_data(float f, int len_i, int len_f);
	void copy_data(CANMessage *msg, int len);

private:
	CAN *can;
	float rcvData[CANCommand::ReceiveDataType_end];
};

//extern short CANReceiveFormats[CANCommand::ReceiveDataType_end][CANCommand::ReceiveFormatType_end];


#endif /* WALK_CANCOMMAND_H_ */
