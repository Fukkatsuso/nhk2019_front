/*
 * CANSynchronizer.cpp
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

//Front:送信
//Rear:受信
#include "CANSynchronizer.h"


CANSynchronizer::CANSynchronizer(CAN *can, void (*fptr_timerreset)(void)):
	CANSender(can)
{
	this->fptr_timerreset = fptr_timerreset;
}


void CANSynchronizer::set_period(float period)
{
	if(this->period == period)return;
	this->period = period;
	ticker.attach(fptr_timerreset, period);//変更があれば実行
}


void CANSynchronizer::set_duty(float duty)
{
	if(this->duty == duty)return;
	this->duty = duty;
}


void CANSynchronizer::timer_reset(bool allReset)
{
	//ToSlaveAllにして自分自身も受信・タイマーリセットする
	send_timer_reset(CANID::generate(CANID::FromController, CANID::ToRear, CANID::TimerReset));
	//歩き出しの瞬間など
	if(allReset) ticker.attach(fptr_timerreset, period);//変更があれば実行
}


float CANSynchronizer::get_period()
{
	return period;
}

float CANSynchronizer::get_duty()
{
	return duty;
}
