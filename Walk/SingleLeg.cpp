/*
 * SingleLeg.cpp
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#include "SingleLeg.h"


//π = 180°
float rad_to_degree(float rad)
{
	return rad*180.0/M_PI;
}


//アーム根元の水平座標、鉛直座標
SingleLeg::SingleLeg(LegPosition arg_fr, LegPosition arg_rl, float hrz_base, float vrt_base):
	InverseKinematics(vrt_base, arg_fr*hrz_base),
	fr(arg_fr), rl(arg_rl)
{
	set_angle_limit(ANGLE_MAX, ANGLE_MIN);
	legPID.param_set_limit(DUTY_MAX-0.5, DUTY_MIN-0.5);
}


void SingleLeg::unitize(PwmOut *motor, SingleLegQEI *enc, InitSwitch *sw)
{
	this->motor = motor;
	this->enc = enc;
	this->sw = sw;
}


//目標x,y->PIDでduty計算
/* rotation(angle+に対するブラシレスモータの回転方向)
 *|-------------------------|
 *| 		Front	Rear	|
 *| Right	CW		CCW		|
 *| Left	CCW		CW		|
 *|-------------------------|
 */
/*DC:duty
 *|-------------------------|
 *| 		Front	Rear	|
 *| Right	+		-		|
 *| Left	-		+		|
 *|-------------------------|
 */
void SingleLeg::move_to(float arg_x, float arg_y)
{
	float angle = InverseKinematics::move_to(arg_y, fr*arg_x);//xy反転
	angle = rad_to_degree(angle);//degree

	duty = 0.5 + (fr*rl)*legPID.calc_duty(angle);
}


short SingleLeg::get_rl()
{
	return rl;
}

float SingleLeg::get_duty()
{
	return duty;
}

float SingleLeg::get_x()
{
	return y.hand;
}

float SingleLeg::get_y()
{
	return fr*x.hand;	//move_toでfr補正したのでfrをかけてもとの値に戻してやる。これで共通のyが得られる
}

//動作目標角度[degree]
float SingleLeg::get_angle()
{
	return rad_to_degree(InverseKinematics::get_angle());
}

float SingleLeg::get_P()
{
	return legPID.get_P();
}
float SingleLeg::get_I()
{
	return legPID.get_I();
}
float SingleLeg::get_D()
{
	return legPID.get_D();
}

float SingleLeg::get_enc(){
	return enc->getAngle();
}
int SingleLeg::get_sw(){
	return sw->read();
}


//エンコーダーの値を格納する変数のアドレス, PID係数
void SingleLeg::set_PID(float Kp, float Ki, float Kd)
{
	legPID.set_PID(enc, Kp, Ki, Kd);
}


void SingleLeg::set_limit(int d_max, int d_min)
{
	legPID.param_set_limit(d_max, d_min);
}

void SingleLeg::reset_duty()
{
	legPID.reset_duty();
}

void SingleLeg::reset_duty(float reset)
{
	legPID.reset_duty();
	duty = reset;
}
