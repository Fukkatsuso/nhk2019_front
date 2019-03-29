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
		Direction,			//Master->Controller->Slave
		Area,				//Master->Controller-中間処理->Slave
		MoveDistCentroid,	//Master<-Controller : 重心の移動量を送信

		Speed,				//Controller->Slave
		VelocityVector,		//Controller->Slave
		TimerReset,			//Controller->Slave
		LegUp,				//Controller->Slave

		AreaChange,			//Controller<-Slave : Area変更の要請を送るためだけ
		MoveDistFront,		//Controller<-Slave
		MoveDistRear,		//Controller<-Slave

		DataType_end		//<=0x00f=15 に制限
	};

	static unsigned int generate(CANID::From from, CANID::To to, CANID::DataType type);
	static bool is_from(unsigned int can_id, CANID::From from);
	static bool is_to(unsigned int can_id, CANID::To to);
	static bool is_type(unsigned int can_id, CANID::DataType type);
};


/*****************
 * CAN送受信データの型
 *****************/
//Direction 	: float(4B)
//Speed 		: float(4B)
//TimerReset 	: unsigned char(1B)
//Area 			: unsigned char(1B)
//LegUp 		: unsigned char(1B)
//MoveDist 		: short(2B)

#define BYTE_DIRECTION 4
union can_Direction{
	unsigned char byte[BYTE_DIRECTION];
	float value;
};

#define BYTE_SPEED 4
union can_Speed{
	unsigned char byte[BYTE_SPEED];
	float value;
};

#define BYTE_VELOCITYVECTOR (BYTE_SPEED + BYTE_DIRECTION)
union can_VelocityVector{
	unsigned char byte[BYTE_VELOCITYVECTOR];
	struct{
		can_Direction direction;
		can_Speed speed;
	};
};

#define BYTE_AREA 1
union can_Area{
	unsigned char byte[BYTE_AREA];
	unsigned char value;
};

#define BYTE_TIMERRESET 1
union can_TimerReset{
	unsigned char byte[BYTE_TIMERRESET];
	unsigned char value;
};

#define BYTE_LEGUP 1
union can_LegUp{
	unsigned char byte[BYTE_LEGUP];
	struct{
		unsigned char FR : 1;
		unsigned char FL : 1;
		unsigned char RR : 1;
		unsigned char RL : 1;
	};
};

#define BYTE_DIST 2
union can_MoveDist{
	unsigned char byte[BYTE_DIST];
	signed short value;
};


/**********************
 * CAN通信
 * Controller--Slaveのみ
 **********************/
class CANProtocol{
public:
	CANProtocol(CAN *can);

	float get_direction();
	float get_speed();
	unsigned short get_timer_reset();
	unsigned char get_area();
	unsigned short get_area_change();
	bool get_leg_up_FR();
	bool get_leg_up_FL();
	bool get_leg_up_RR();
	bool get_leg_up_RL();
	short get_move_dist_front();
	short get_move_dist_rear();

protected:
	CAN *can;
	struct CANData{
		union can_Direction 		direction;			//方向
		union can_Speed 			speed;				//速さ
		union can_VelocityVector 	velocity_vector;	//速度ベクトル
		union can_TimerReset 		timer_reset;		//足の同期用タイマーリセット
		union can_Area 				area;				//現在のArea
		union can_Area 				area_change; 		//エリア変更要請
		union can_LegUp 			leg_up;				//足上げフラグ
		union can_MoveDist 			move_dist_front;	//前脚移動平均
		union can_MoveDist 			move_dist_rear;		//後脚移動平均
	}data;
};


#endif /* WALK_CANS_CANPROTOCOL_H_ */
