/*
 * MRMode.h
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#ifndef WALK_MRMODE_H_
#define WALK_MRMODE_H_

#include "mbed.h"
#include "CANCommand.h"

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

enum Mode{
	WaitGobiUrtuu = 0,//待機
	GetGerege,//ゲルゲ受け取り検知
	PrepareWalking,
	Start1,//歩行開始
	GobiArea,//直進
	SandDune,//段差
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
/////////////////////////////
	Operate//PSコン操作
};


class MRMode
{
public:
	MRMode();
private:
};


#endif /* WALK_MRMODE_H_ */
