/*
 * SingleLeg.cpp
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#include "SingleLeg.h"

//絶対固定
#define BASE_X 20


extern Serial pc;//DEBUG


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
	status.duty = 0.5;
}


//legPID.set_PID()よりも先に実行すること! timerの初期化ができなくなる
void SingleLeg::unitize(PwmOut *motor, SingleLegQEI *enc, InitSwitch *sw, Timer *timer)
{
	this->motor = motor;
	this->enc = enc;
	this->sw = sw;
	legPID.set_timer(timer);
}

void SingleLeg::set_dependencies(MRMode *mode)
{
	MRmode = mode;
	set_limits();
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

	state_update();
	status.duty = 0.5 + (fr*rl)*legPID.calc_duty(angle);
	motor->write(status.duty);

//	pc.printf("d[%1.3f]  ", status.duty);
}

void SingleLeg::move_to(float arg_x, float arg_y, float duty_max, float duty_min)
{
	legPID.param_set_limit(duty_max-0.5, duty_min-0.5);
	SingleLeg::move_to(arg_x, arg_y);
}

//センサー更新	//スイッチONでエンコーダーリセットの機能だけでも可?
void SingleLeg::state_update()
{
	status.sw = sw->read();
	if(status.sw)enc->reset();
//	status.enc = enc->getAngle();//[degree]	//不要な気がする
//	motor->write(status.duty);	//move_to()で実行
}


short SingleLeg::get_rl()
{
	return rl;
}

float SingleLeg::get_duty()
{
	return status.duty;
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


void SingleLeg::set_PID_from_file(const char *fileName)
{
	float gain[3] = {0, 0, 0};
	FILE *fp = fopen(fileName, "r");
	if(fp){
		for(int i=0; i<3; i++){
			fscanf(fp, "%f", &gain[i]);
		}
		fclose(fp);
	}
	set_PID(gain[0], gain[1], gain[2]);
}

//PID係数
void SingleLeg::set_PID(float Kp, float Ki, float Kd)
{
	legPID.set_PID(enc, Kp, Ki, Kd);
}


void SingleLeg::set_limits()
{
	MRMode::Area mode = MRmode->get_area(MRMode::Now);
	Limits *limits = MRmode->get_limits(mode);
	set_angle_limit(limits->angle.max, limits->angle.min);
	set_duty_limit(limits->duty.max, limits->duty.min);
}

void SingleLeg::set_duty_limit(float d_max, float d_min)
{
	legPID.param_set_limit(d_max-0.5, d_min-0.5);
}

void SingleLeg::reset_duty()
{
	legPID.reset_duty();
}

void SingleLeg::reset_duty(float reset)
{
	legPID.reset_duty();
	status.duty = reset;
}
