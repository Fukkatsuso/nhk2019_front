/*
 * CANProtocol.h
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

#ifndef WALK_CANS_CANPROTOCOL_H_
#define WALK_CANS_CANPROTOCOL_H_

#include "mbed.h"


//改善
//というか修正
//CANFormatsは何バイト送るかのみ
//送受信データはunionでまとめる
//union CANData{
//	unsigned char byte[??];
//	struct{
//		unsigned float period;
//		unsigned float duty;
//		unsigned float speed;
//		...
//	};
//};

//[float(4) Speed][float(4) Direction]
//[unsigned char(1) TimerReset]
//[unsigned char(1) Area]
//[unsigned char(1) LegUp]
//[unsigned char(1) AreaChange]
//[short(2) MoveDistFront]
//[short(2) MoveDistRear]
//[short(2) MoveDistCentroid]

struct CANID{
	enum From{
		FromMaster 		= 0x000,
		FromController 	= 0x100,
		FromSlave 		= 0x200,
		FromFront 		= 0x300,
		FromRear 		= 0x400
	};
	enum To{
		ToMaster 		= 0x000,
		ToController 	= 0x010,
		ToSlaveAll 		= 0x020,
		ToFront 		= 0x030,
		ToRear 			= 0x040
	};
	enum DataType{
		Period = 0,			//たぶん送らない
		Duty,				//たぶん送らない
		Speed,				//Controller->Slave
		Direction,			//Master->Controller->Slave
		TimerReset,			//Controller->Slave
		Area,				//Master->Controller-中間処理->Slave
		LegUp,				//Controller->Slave
		AreaChange,			//Slave->Controller : Area変更の要請を送るためだけ
		MoveDistFront,		//Slave->Controller
		MoveDistRear,		//Slave->Controller
		MoveDistCentroid,	//Controller->Master : 各脚歩いた量を送信 : これで送信元識別
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
