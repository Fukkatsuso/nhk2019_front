/*
 * MRMode.h
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#ifndef WALK_MRMODE_H_
#define WALK_MRMODE_H_

#include "mbed.h"
#include "CANs/CANReceiver.h"
#include "CANs/CANSender.h"


/*stay
#define DUTY_MAX (0.65)//(0.8)
#define DUTY_MIN (0.35)//(0.2)
*/


enum LegPosition{
	Front = 1,
	Rear = -1,
	Right = 1,
	Left = -1
};

enum LegMode{
	Move = 1,
	Up = 2,
	Down = 3,
	Stay = 4,
	//障害物用
	StableUp = 5,
	StableSlide = 6,
	StableDown = 7,
	StableMove = 8,
	StableWait = 9
};

struct Gait{
	enum Mode{
		NormalGait,
		StableGait,
		ActiveStableGait
	};
};

struct Recovery{
	enum Mode{
		Cycloid,
		Quadrangle
	};
};


struct Limits{
	struct{
		float max;
		float min;
	}x, y, angle, duty;
};

struct Orbits{
	float gradient; 	//フィールド勾配
	float init_x; 		//足先の初期位置(x)
	float init_y; 		//(y)
	float height; 		//足先を上げるときの最大高さ
	float time_change;	//Areaの変更に伴う初期設定が完了するまでの時間
};


class MRMode
{
public:
	enum Area{
		WaitGobiUrtuu = 0,//待機
		GetGerege,//ゲルゲ受け取り検知
		PrepareWalking,
		Start1,//歩行開始
		GobiArea,//直進
		SandDuneFront,//段差1
		SandDuneRear,//段差2
		ReadyForTussock,//ここに何か入れるべき
		Tussock,//紐
		Finish1,//到着
		WaitMountainUrtuu,//待機
		GetSign,//非接触の合図
		Start2,//歩行開始
		StartClimb1,//Front登山開始
		StartClimb2,//Rear
		MountainArea,//登山
		UukhaiZone,//ウーハイゾーン
		Uukhai,//ウーハイ
		Finish2,//終了
		Area_end,
	};
	enum Reference{
		Initial,
		Prev,
		Now,
		Next,
		Reference_end
	};

	MRMode(CANReceiver *rcv, CANSender *snd, enum Area init_area, bool operate);
	void update();
	bool is_switched();
	bool is_changing_area();
	void request_to_change_area(enum Area area_req, CANID::From can_from);

	Area get_area(enum Reference ref);
	Area get_now();
	Limits *get_limits(enum Area area);
	Orbits *get_orbits (enum Area area);
	float get_x_dif_change_init();
	float get_y_dif_change_init();

private:
	Timer timer_changing_area;
	CANReceiver *can_receiver;
	CANSender *can_sender;
	Area area[MRMode::Reference_end];
	Area roop_prev, roop_now;//前回と今回のモード
	struct{
		bool operate;		//手動
		bool switched;		//モードが切り替わった直後
		bool changing_area;	//Areaの変更動作中
	}flag;

	float timeslice_changing_area;
	float time_changing_area_prev;
	float x_vel_change_init, y_vel_change_init;	//Areaの変更動作速度
};

extern Limits limits[MRMode::Area_end];
extern Orbits orbits[MRMode::Area_end];

/*
 * example:
 * MRMode mode(&cancommand, MRMode::WaitGobiUrtuu, false);
 * FR.set_x_lim(mode.get_limits(mode.get_area(Now))->x.max, mode.get_limits(mode.get_area(Now))->x.min);
 */


#endif /* WALK_MRMODE_H_ */
