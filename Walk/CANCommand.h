/*
 * CANCommand.h
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */
//新たに受信する変数を追加するときには
//DataType, FormatType の末尾(_endの前)に変数名を追加すること
//CANformats[][]の末尾に要素を追加すること


#ifndef WALK_CANCOMMAND_H_
#define WALK_CANCOMMAND_H_

#include "mbed.h"


enum DataType{
	Period=0,
	Duty,
	Speed,
	Direction,
	Time,
	Area,
	DataType_end//<=0x00f=15 に制限（仕様上）
};

enum FormatType{
	ID=0,
	Len_integer,
	Len_fraction,
	FormatType_end
};

extern short CANformats[DataType_end][FormatType_end];


/*
 * 中央制御用マイコンとCAN通信
 */
class CANCommand
{
public:
	CANCommand(CAN *can);
	void receive(unsigned int id, unsigned char data[]);
	//void send(int ID, int , float data);後回し

	float get(enum DataType type);

protected:
	float decode_from_array(unsigned char array[], int len_i, int len_f);
	void store_in_data(float f, int len_i, int len_f);
	void copy_data(CANMessage *msg, int len);

private:
	CAN *can;
	float rcvData[DataType_end];
};


#endif /* WALK_CANCOMMAND_H_ */
