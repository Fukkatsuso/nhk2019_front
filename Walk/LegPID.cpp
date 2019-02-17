/*
 * LegPID.cpp
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#include "LegPID.h"


LegPID::LegPID()
{}


void LegPID::set_PID(SingleLegQEI *enc, float Kp, float Ki, float Kd)
{
	this->enc = enc;
	PID::set_PID(Kp, Ki, Kd);
	PID::start(enc->getAngle(), 0);
}


float LegPID::calc_duty(float tgt_angle)
{
	calc(enc->getAngle(), tgt_angle);
	return PID::get_opr_nxt();//符号つき
}

float LegPID::get_enc()
{
	return enc->getAngle();
}

void LegPID::reset_duty()
{
	PID::reset_opr_nxt(0);
}
