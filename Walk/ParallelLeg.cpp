/*
 * ParallelLeg.cpp
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#include "ParallelLeg.h"
#include "functions.h"

#define FACTOR_Y (4.0*M_PI*(high-y.pos.init)/(Ty*(4.0+M_PI)))//坂道の傾斜はまだ考慮していない


ParallelLeg::ParallelLeg(int fr, int rl, float pos_x, float pos_y):
	fr(fr), rl(rl)
{
	speed = 0;

	flag.recovery = true;
	flag.stay_command = true;
	flag.stay = true;
	flag.first_cycle = true;
}

void ParallelLeg::set_dependencies(MRMode *mode, CANCommand *command)
{
	this->MRmode = mode;
	this->CANcmd = command;
	set_limits();
}


void ParallelLeg::set_x_lim(float xmax, float xmin)
{
	x.pos.max = xmax;
	x.pos.min = xmin;
}

void ParallelLeg::set_y_lim(float ymax, float ymin)
{
	y.pos.max = ymax;
	y.pos.min = ymin;
}

void ParallelLeg::set_limits()
{
	MRMode::Area mode = MRmode->get_area(MRMode::Now);
	Limits *limits = MRmode->get_limits(mode);
	set_x_lim(limits->x.max, limits->x.min);
	set_y_lim(limits->y.max, limits->y.min);
}

void ParallelLeg::set_x_initial(float x_initial){
	x.pos.init = x_initial;
}

void ParallelLeg::set_y_initial(float y_initial){
	y.pos.init = y_initial;
}

void ParallelLeg::set_initial(float x_initial, float y_initial)
{
	x.pos.init = x_initial;
	y.pos.init = y_initial;
}

void ParallelLeg::set_high(float high)
{
	this->high = high;
}

void ParallelLeg::set_gradient(float grad)//引数[°], 結果[rad]
{
	gradient = M_PI * grad / 180.0;
}

void ParallelLeg::set_period(float p)
{
	period = (p>0? p:0);
}

void ParallelLeg::set_duty(float d)
{
	duty = ((0.5<=d && d<=1.0)? d:1.0);
}


void ParallelLeg::walk(float spd, float dir, float tm)
{
	direction = dir;
	speed = curve_adjust(spd);
	x.pos.now = x.pos.next;
	y.pos.now = y.pos.next;
	calc_dt(tm);//時刻更新
	////////////////////////////////
	set_timing();//足上げタイミング等計算
	walk_mode();//足のモード
	check_flag();//if(flag.stay)mode = Move; を含める?
	calc_velocity();//vel計算
	calc_position();//pos計算
}

void ParallelLeg::walk()
{
	set_period(CANcmd->get(CANID::Period)); set_duty(CANcmd->get(CANID::Duty));
	walk(CANcmd->get(CANID::Speed), CANcmd->get(CANID::Direction), CANcmd->get(CANID::Time));
}

//斜め方向に歩くとき
float ParallelLeg::curve_adjust(float value)
{
	if((rl*direction)>0)	//その足の方に歩く->小股で
		value *= fabs(cos(direction));
	return value;
}

void ParallelLeg::calc_dt(float tm)
{
	time.prv = time.now;
	time.now = tm;
	if(time.now < time.prv)time.prv = 0.0;//タイマーリセットの次の瞬間
	time.dif = time.now - time.prv;
}

void ParallelLeg::set_timing()
{
	timing[0] = 0;//時刻ゼロ
	//復帰開始
	if(speed>=0){//前進: RL->FL->RR->FR	//FR,RL,FL,RRだった
		if(rl==Left){
			if(fr==Rear)timing[1] = 0;//RL
			else timing[1] = period * 1.0/4.0;//FL
		}
		else{
			if(fr==Rear)timing[1] = period * 2.0/4.0;//RR
			else timing[1] = period * 3.0/4.0;//FR
		}
	}
	else{//後退: FR->RR->FL->RL	//RL,FR,RR,FLだった
		if(rl==Right){
			if(fr==Front)timing[1] = 0;//FR
			else timing[1] = period * 1.0/4.0;//RR
		}
		else{
			if(fr==Front)timing[1] = period * 2.0/4.0;//FL
			else timing[1] = period * 3.0/4.0;//RL
		}
	}
	timing[2] = timing[1] + (1.0-duty)*period;//復帰完了
	timing[3] = period;//1周期完了
}

void ParallelLeg::walk_mode()
{
	mode_prv = mode;

	if(timing[0]<=time.now && time.now<timing[1]){
		if(timing[2]<=timing[3])mode = Move;//case:beta>=0.75
		else if(time.now<(timing[2]-timing[3]) && !flag.first_cycle)mode = Down;
		else mode = Move;
		//mode = Move;
	}
	//timing[1]とtiming[2]をLEGUP_TIME:LEGDOWN_TIMEに内分
	else if(timing[1]<=time.now && time.now<(LEGDOWN_TIME*timing[1]+LEGUP_TIME*timing[2])/(LEGUP_TIME+LEGDOWN_TIME))mode = Up;
	else if((LEGDOWN_TIME*timing[1]+LEGUP_TIME*timing[2])/(LEGUP_TIME+LEGDOWN_TIME)<=time.now && time.now<timing[2])mode = Down;
	else if(time.now>=timing[2])mode = Move;
	else mode = Move;
}

void ParallelLeg::check_flag()
{
	//復帰完了フラグ
	if(mode==Move && mode_prv==Down)
		flag.recovery = true;
	else flag.recovery = false;

	//静止コマンドフラグ
	if(fabs(speed)==0 && fabs(direction)==0)//コマンドが「停止」のとき
		flag.stay_command = true;
	else flag.stay_command = false;

	//最初のサイクルかどうか判断する(現状では最初の踏み出しが完了していないことを示す)フラグ:first_cycle
	if(flag.stay)
		flag.first_cycle = true;
	else if(flag.recovery)
		flag.first_cycle = false;
}

//送り時
//足の根元の、地面に対する相対移動速度
void ParallelLeg::calc_velocity()
{
	switch(mode){
	case Move:
		x.vel = -speed;
		y.vel = 0.0;//speed * tan(gradient)	//(y.pos.init-y.pos.now)/(duty*period/4.0)
		break;
	case Up:
		calc_step();//復帰の着地点座標(x)
		calc_vel_recovery();
		break;
	case Down:
		calc_step();//復帰の着地点座標(x)
		calc_vel_recovery();
		break;
	}
	if(flag.stay_command && fabs(x.pos.now-x.pos.init)<X_STAY_MARGIN){
		x.vel = 0.0;
		y.vel = 0.0;
		flag.stay = true;//停止コマンドかつ安定動作完了->停止完了
	}
	else flag.stay = false;
}


//復帰時
//x-原点から次の着地点までの距離
void ParallelLeg::calc_step()
{
	if(mode==Up && mode_prv==Move){
		x.pos.recover_start = x.pos.now;
		y.pos.recover_start = y.pos.now;
	}
	step = speed * period * duty / 2.0 + x.pos.init;//復帰完了地点
}

void ParallelLeg::calc_vel_recovery()
{
	float tm = time.now - timing[1];
	float Ty = (1.0-duty)*period;//遊脚時間
	//x速度計算
	x.vel = ((step-x.pos.recover_start)/Ty) * (1.0 - cos(2.0*M_PI*tm/Ty)/duty);//計算改良1
	//y速度計算
	if(0<=tm && tm<=Ty/4.0)
		y.vel = FACTOR_Y * (1.0 - cos(4.0*M_PI*tm/Ty));
	else if(Ty/4.0<tm && tm<Ty*3.0/4.0)
		y.vel = 2.0 * FACTOR_Y * cos(2.0*M_PI*(tm/Ty-1.0/4.0));
	else if(Ty*3.0/4.0<=tm && tm<=Ty)
		y.vel = -FACTOR_Y * (1.0 - cos(4.0*M_PI*(1.0-tm/Ty)));
}

void ParallelLeg::calc_position()
{
	x.pos.dif += x.vel*time.dif;
	y.pos.dif += y.vel*time.dif;
	if(fabs(y.vel)==0)y.pos.dif = 0;
	x.pos.next = limit(x.pos.init + x.pos.dif, x.pos.max, x.pos.min);
	y.pos.next = limit(y.pos.init + y.pos.dif, y.pos.max, y.pos.min);
}


float ParallelLeg::get_x()
{
	return x.pos.next;
}

float ParallelLeg::get_y()
{
	return y.pos.next;
}

float ParallelLeg::get_x_initial()
{
	return x.pos.init;
}

float ParallelLeg::get_y_initial()
{
	return y.pos.init;
}

float ParallelLeg::get_x_vel()
{
	return x.vel;
}
float ParallelLeg::get_y_vel()
{
	return y.vel;
}
int ParallelLeg::get_mode()
{
	return mode;
}

bool ParallelLeg::is_recovery()
{
	return flag.recovery;
}

bool ParallelLeg::is_stay(){
	return flag.stay;
}

bool ParallelLeg::is_climb()
{
	return flag.climb;
}

