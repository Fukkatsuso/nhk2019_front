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

//脚持ち上げ:脚下げ = LEGUP:LEGDOWN
#define LEGUP_MOVE 4.0f
#define LEGDOWN_MOVE (1.0f)
#define LEGUP_TIME 1.0f
#define LEGDOWN_TIME 1.0f//4.0f

#define X_STAY_MARGIN 2.0f

//normal
#define DUTY_MAX (0.8)///0.8
#define DUTY_MIN (0.2)//0.2

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
	Down = 3
};

struct Limits{
	struct{
		float max;
		float min;
	}x, y, angle, duty;
};

struct Orbits{
	float gradient; //フィールド勾配
	float init; //足先の初期位置
	float high; //足先を上げるときの最大高さ
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
		SandDune,//段差
		ReadyForTussock,//ここに何か入れるべき
		Tussock1,//紐1
		Tussock2,//紐2
		Finish1,//到着
		WaitMountainUrtuu,//待機
		GetSign,//非接触の合図
		Start2,//歩行開始
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

	MRMode(CANReceiver *rcv, enum Area init_area, bool operate);
	void update();
	bool is_switched();

	Area get_area(enum Reference ref);
//	float get_lim_x_max(enum Reference ref);
//	float get_lim_x_min(enum Reference ref);
//	float get_lim_y_max(enum Reference ref);
//	float get_lim_y_min(enum Reference ref);
//	float get_lim_angle_max(enum Reference ref);
//	float get_lim_angle_min(enum Reference ref);
//	float get_lim_duty_max(enum Reference ref);
//	float get_lim_duty_min(enum Reference ref);
	Limits *get_limits(enum Area area);

	float get_gradient(enum Reference ref);
	float get_init(enum Reference ref);
	float get_high(enum Reference ref);
	Orbits *get_orbits (enum Area area);

private:
	CANReceiver *can_receiver;
	Area area[MRMode::Reference_end];
	Area roop_prev, roop_now;//前回と今回のループでのモード
	struct{
		bool operate;//手動
		bool switched;//モードが切り替わった直後
	}flag;
};

/*
 * example:
 * MRMode mode(&cancommand, MRMode::WaitGobiUrtuu, false);
 * FR.set_x_lim(mode.get_limits(mode.get_area(Now))->x.max, mode.get_limits(mode.get_area(Now))->x.min);
 */


#endif /* WALK_MRMODE_H_ */
