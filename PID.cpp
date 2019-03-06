/*
 * PID.cpp
 *
 *  Created on: 2018/10/31
 *      Author: mutsuro
 */

#include "PID.h"


extern Serial pc;

/*PID::PID(float Kp, float Ki, float Kd):
	Kp(Kp), Ki(Ki), Kd(Kd)
{}*/


PID::PID(){}


PID::~PID(){}


void PID::set_timer(Timer *timer)
{
	this->timer = timer;
}

//事前にset_timer()を実行すること!
void PID::set_PID(float Kp, float Ki, float Kd)
{
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	start();
}


//必要最低限の初期化
void PID::start(float obs_init, float opr_init)
{
	//積分値ゼロ初期化
	I = 0;

	//観測値の過去値を初期値に設定
	obs.now = obs_init;
	obs.prv = obs_init;
	obs.prv_prv = obs_init;

	//最初は停止しているとする
	obs.dif = 0;
	obs.dif_prv = 0;

	//後でコレが_opr.prvの値になる
	opr.nxt = opr_init;

	timer->reset();
	timer->start();

	time.now = 0;//タイマーが足りないので全脚で共通のタイマーを使うことに
}


void PID::param_update(float obs_now, float obs_tgt)
{
	opr.prv = opr.nxt;

	//観測値更新
	obs.prv_prv_prv = obs.prv_prv;
	obs.prv_prv = obs.prv;
	obs.prv = obs.now;
	obs.now = obs_now;

	obs.tgt_prv = obs.tgt;
	obs.tgt = obs_tgt;
	if(obs_tgt==obs_now)start(obs_now, opr.prv);//積分値初期化

	//観測値と目標値の偏差
	obs.dif_prv_prv = obs.dif_prv;
	obs.dif_prv = obs.dif;
	obs.dif = obs.tgt - obs.now;

	//ループ時間
	time.prv = time.now;
	time.now = timer->read();
	time.dif = time.now - time.prv;
}

void PID::calc(float obs_now, float obs_tgt)
{
	param_update(obs_now, obs_tgt);

	calc_P();
	calc_I();
	calc_D();

	opr.dif = Kp*P + Ki*I + Kd*D;
	opr.nxt = opr.prv + opr.dif;
	param_limit();

//	pc.printf("[%1.3f,%1.3f] ", opr.max, opr.min);
}


void PID::calc_P()
{
	P = obs.dif;
}


void PID::calc_I()
{
	I += (obs.dif) * time.dif;
	if(fabs(obs.dif)>MARGIN_I) I = 0;
}


void PID::calc_D()
{
	D = (obs.dif - obs.dif_prv) / time.dif;
	if(fabs(obs.dif)>MARGIN_D) D = 0;
}


void PID::param_set_limit(float max, float min)
{
	opr.max = max;
	opr.min = min;
}


void PID::param_limit()
{
	if(opr.nxt > opr.max)opr.nxt = opr.max;
	else if(opr.nxt < opr.min)opr.nxt = opr.min;
}


float PID::get_opr_nxt()
{
	return opr.nxt;
}

float PID::get_obs_dif()
{
	return obs.dif;
}


float PID::get_P()
{
	return Kp*P;
}
float PID::get_I()
{
	return Ki*I;
}
float PID::get_D()
{
	return Kd*D;
}

void PID::reset_opr_nxt(float opr_init)
{
	opr.nxt = opr_init;
	P = I = D = 0;
}
