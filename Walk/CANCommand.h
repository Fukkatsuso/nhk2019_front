/*
 * CANCommand.h
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

//新たに受信する変数を追加するときには
//DataType の末尾(_endの前)に変数名を追加すること
//CANFormats[][]の末尾に要素を追加すること


#ifndef WALK_CANCOMMAND_H_
#define WALK_CANCOMMAND_H_

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
		Gait,
		DataType_end//<=0x00f=15 に制限（仕様上）
	};
};

int CANID_generate(CANID::From from, CANID::To to);
int CANID_generate(CANID::From from, CANID::To to, CANID::DataType type);
bool CANID_is_from(int id, CANID::From from);
bool CANID_is_to(int id, CANID::To to);
bool CANID_is_type(int id, CANID::DataType type);


/*
 * 中央制御用マイコンとCAN通信
 */
class CANCommand
{
public:
	struct FormatType{
		enum{
			ID=0, //ID
			Len_integer, //整数部分長
			Len_fraction, //小数部分長
			FormatType_end
		};
	};

	CANCommand(CAN *can);
	void send(int id, float data);
	void receive(unsigned int id, unsigned char data[]);

	float get(CANID::DataType type);
	int get_area();
	int get_gait();

protected:
	//send()にて
	void store_in_data(float f, int len_i, int len_f);
	void copy_data(CANMessage *msg, int len);
	//receive()にて
	float decode_from_array(unsigned char array[], int len_i, int len_f);

private:
	CAN *can;
	float rcvData[CANID::DataType_end];
	unsigned int sndData[8];
};

#endif /* WALK_CANCOMMAND_H_ */
