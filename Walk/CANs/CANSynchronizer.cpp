/*
 * CANSynchronizer.cpp
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

//Front:送信
//Rear:受信
#include "CANSynchronizer.h"


CANSynchronizer::CANSynchronizer(CAN *can):
	CANSender(can){}


void CANSynchronizer::set_period(float period, bool send=true)
{
	this->period = period;
	if(!send)return;
	//CAN送信
	this->send(CANID_generate(CANID::FromFront, CANID::ToRear, CANID::Period), period);
}


void CANSynchronizer::set_duty(float duty, bool send=true)
{
	this->duty = duty;
	if(!send)return;
	//CAN送信
	this->send(CANID_generate(CANID::FromFront, CANID::ToRear, CANID::Duty), duty);
}
