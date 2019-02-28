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

extern int CANID_generate(CANID::From from, CANID::To to);
extern int CANID_generate(CANID::From from, CANID::To to, CANID::DataType type);
extern bool CANID_is_from(int id, CANID::From from);
extern bool CANID_is_to(int id, CANID::To to);
extern bool CANID_is_type(int id, CANID::DataType type);


class CANProtocol{
protected:
	CAN *can;

	struct FormatType{
		enum{
			ID=0, //ID
			Len_integer, //整数部分長
			Len_fraction, //小数部分長
			FormatType_end
		};
	};

	static const short CANFormats[CANID::DataType_end][FormatType::FormatType_end];
};


#endif /* WALK_CANS_CANPROTOCOL_H_ */
