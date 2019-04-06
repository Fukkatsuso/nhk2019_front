/*
 * CANProtocol.h
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

#ifndef WALK_CANS_CANPROTOCOL_H_
#define WALK_CANS_CANPROTOCOL_H_

#include "mbed.h"


/***************
 * CAN通信のID管理
 ***************/
//Master->Controller : angle, status 					: 0x013
//Master<-Controller : dist, kouden_front, kouden_rear	: 0x102
class CANID{
public:
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
		WalkCommand,			//Controller->Slave
		TimerReset,				//Controller->Slave

		MRInfo,					//Master<-Controller
		Navi,					//Master->Controller


		LegInfo,				//Controller<-Slave
		AreaChange,				//Controller<-Slave

		DataType_end
	};

	static unsigned int generate(CANID::From from, CANID::To to, CANID::DataType type);
	static bool is_from(unsigned int can_id, CANID::From from);
	static bool is_to(unsigned int can_id, CANID::To to);
	static bool is_type(unsigned int can_id, CANID::DataType type);
};


/*****************
 * CAN送受信データの型
 *****************/
//受信データのみ末尾に読取可否フラグ追加。
//Navi			: status, angle, (pitch)
//MRInfo		: dist, kouden

//WalkCommand	: area, speed, direction, (pitch), leg_up
//TimerReset	: reset
//LegInfo		: dist, leg_state, kouden
//AreaChange	: area

#define BYTE_WALK_COMMAND 8
union can_WalkCommand{
	unsigned char byte[BYTE_WALK_COMMAND];
	struct{
		unsigned char area;
		signed short speed;
		signed short direction;
		signed short pitch;	//未実装
		union{
			unsigned char byte[1];
			struct{
				unsigned char FR : 1;
				unsigned char FL : 1;
				unsigned char RR : 1;
				unsigned char RL : 1;
			};
		}leg_up;
	};
};

#define BYTE_TIMER_RESET 1
union can_TimerReset{
	unsigned char byte[BYTE_TIMER_RESET];
	unsigned char value;
};

#define BYTE_LEG_INFO 4
union can_LegState{
	unsigned char byte[1];
	struct{
		unsigned char right : 4;
		unsigned char left 	: 4;
	};
};

union can_Kouden{
	unsigned char byte[1];
	struct{
		unsigned char sand_dune : 1;
	};
};

union can_LegInfo{
	unsigned char byte[BYTE_LEG_INFO];
	struct{
		signed short dist;
		union can_LegState state;
		union can_Kouden kouden;
	};
};

#define BYTE_AREA 1
union can_Area{
	unsigned char byte[BYTE_AREA];
	unsigned char area;
};


/**********************
 * CAN通信
 * Controller--Slaveのみ
 **********************/
class CANProtocol{
public:
	CANProtocol(CAN *can);

protected:
	CAN *can;
};


#endif /* WALK_CANS_CANPROTOCOL_H_ */
