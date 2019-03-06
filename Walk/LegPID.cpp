/*
 * LegPID.cpp
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#include "LegPID.h"


extern Serial pc;//DEBUG


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
//	pc.printf("tgt[%f]  ", tgt_angle);
//	pc.printf("enc:%3.2f  ", enc->getAngle());
//	pc.printf("d[%1.3f]  ", get_opr_nxt());
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
