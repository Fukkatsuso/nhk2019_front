/*
 * ParallelLeg.cpp
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#include "ParallelLeg.h"
#include "functions.h"

//#define FACTOR_Y (4.0*M_PI*(height-y.pos.init)/(Ty*(4.0+M_PI)))//坂道の傾斜はまだ考慮していない
#define FACTOR_Y (4.0*M_PI*(-height)/(Ty*(4.0+M_PI)))

//extern Serial pc;


ParallelLeg::ParallelLeg(int fr, int rl, float pos_x, float pos_y):
	fr(fr), rl(rl)
{
	//適当に初期化
	area = MRMode::PrepareWalking;
	area_prv = MRMode::Area_end;
	speed = 0;
	timing[0] = 0;//時刻ゼロ
	timing_stable[0] = 0;

	flag.timer_reset = true;
	flag.recovery = true;
	flag.stay_command = true;
	flag.stay = true;
	flag.first_cycle = true;
}

void ParallelLeg::set_dependencies(ClockTimer *tm_period, MRMode *mode, CANReceiver *can_rcv, CANSynchronizer *can_syn)
{
	this->timer_period = tm_period;
	this->MRmode = mode;
	this->can_receiver = can_rcv;
	this->can_synchronizer = can_syn;
	set_limits();
	set_orbits();
}


void ParallelLeg::mrmode_update()
{
	area_prv = area;
	area = MRmode->get_now();//->get_area(MRMode::Now);
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
//	if(area==area_prv)return;
	Limits *limits = MRmode->get_limits(area);
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

void ParallelLeg::set_height(float height)
{
	this->height = height;
}

void ParallelLeg::set_gradient(float grad)//引数[°], 結果[rad]
{
	gradient = M_PI * grad / 180.0;
}

void ParallelLeg::set_orbits()
{
//	if(area==area_prv)return;
	Orbits *orbits = MRmode->get_orbits(area);
	set_initial(orbits->init_x, orbits->init_y);
	set_height(orbits->height);
	set_gradient(orbits->gradient);
}

void ParallelLeg::set_period(float period)
{
	if(this->period==period)return;
	if(period < 0)period = 0;
	this->period = period;
}

void ParallelLeg::set_duty(float duty)
{
	if(this->duty==duty)return;
	if(duty < 0.5)duty = 0.5;
	else if(duty > 1.0)duty = 1.0;
	this->duty = duty;
}


void ParallelLeg::walk(float spd, float dir)
{
	direction = dir;
	speed = curve_adjust(spd);
	x.pos.now = x.pos.next;
	y.pos.now = y.pos.next;
	timer_update();
	set_timing();//足上げタイミング等計算
	walk_mode();//足のモード
	check_flag();
	calc_velocity();//vel計算
	calc_position();//pos計算
}

void ParallelLeg::walk()
{
//	set_period(CANcmd->get(CANID::Period)); set_duty(CANcmd->get(CANID::Duty));
	walk(can_receiver->get_data(CANID::Speed), can_receiver->get_data(CANID::Direction));
}

void ParallelLeg::walk_stable(float spd, float dir, float rate_stablemove_time)
{
	time_stablemove = period * rate_stablemove_time;
	direction = dir;
	speed = curve_adjust(spd);
	x.pos.now = x.pos.next;
	y.pos.now = y.pos.next;
	timer_update();
	set_stable_timing();
	walk_stable_mode();
	check_stable_flag();
	calc_stable_velocity();
	calc_stable_position();
}

//斜め方向に歩くとき
float ParallelLeg::curve_adjust(float value)
{
	if((rl*direction)>0)	//その足の方に歩く->小股で
		value *= /*fabs*/(cos(2.0*direction));///////////////////////////////////////////
	return value;
}

//void ParallelLeg::calc_dt(float tm)
//{
//	time.prv = time.now;
//	time.now = tm;
//	if(time.now < time.prv)time.prv = 0.0;//タイマーリセットの次の瞬間
//	time.dif = time.now - time.prv;
//}

void ParallelLeg::timer_update()
{	//歩き始めたらタイマーリセット->その瞬間tickerセット
	if(timer_period->read()>period)timer_period->reset();
	else if(mode==Stay)timer_period->reset();
	timer_period->calc_dt(); //	calc_dt(tm);//時刻更新
}


void ParallelLeg::set_timing()
{
	//timing[0]はコンストラクタで0に初期化
	//復帰開始
	if(speed>=0){//前進: FR,RL,FL,RR
		if(fr==Front){
			if(rl==Right) timing[1] = 0;//FR
			else timing[1] = period / 2.0;//FL
		}
		else{//fr==Rear
			if(rl==Left) timing[1] = period * (duty - 0.5);//RL	//period/2 - period*(1-duty)
			else timing[1] = period * duty;//RR	//period - period*(1-duty)
		}
	}
	else{//後退（前後左右反転）: RL,FR,RR,FLだった
		if(fr==Rear){
			if(rl==Left) timing[1] = 0;//RL
			else timing[1] = period / 2.0;//RR
		}
		else{//fr==Front
			if(rl==Right) timing[1] = period * (duty - 0.5);//FR
			else timing[1] = period * duty;//FL
		}
	}
	timing[2] = timing[1] + (1.0-duty)*period;//復帰完了
	timing[3] = period;//1周期完了
}

void ParallelLeg::walk_mode()
{
	mode_prv = mode;
	float now = timer_period->read();

	if(timing[0]<=now && now<timing[1]){
		if(timing[2]<=timing[3])mode = Move;//case:beta>=0.75
		else if(now<(timing[2]-timing[3]) && !flag.first_cycle)mode = Down;
		else mode = Move;
		//mode = Move;
	}
	//timing[1]とtiming[2]をLEGUP_TIME:LEGDOWN_TIMEに内分
	else if(timing[1]<=now && now<(LEGDOWN_TIME*timing[1]+LEGUP_TIME*timing[2])/(LEGUP_TIME+LEGDOWN_TIME))mode = Up;
	else if((LEGDOWN_TIME*timing[1]+LEGUP_TIME*timing[2])/(LEGUP_TIME+LEGDOWN_TIME)<=now && now<timing[2])mode = Down;
	else if(now>=timing[2])mode = Move;
	else mode = Move;
}

void ParallelLeg::check_flag()
{
	//復帰完了フラグ
	if(mode==Move && mode_prv==Down)
		flag.recovery = true;
	else flag.recovery = false;

	//静止コマンドフラグ
	if(fabs(speed)==0 /*&& fabs(direction)==0*/)//コマンドが「停止」のとき
		flag.stay_command = true;
	else flag.stay_command = false;

//要変更/////////////////////////////
	//最初のサイクルかどうか判断する(現状では最初の踏み出しが完了していないことを示す)フラグ:first_cycle
	if(flag.stay)
		flag.first_cycle = true;
	else if(flag.recovery)
		flag.first_cycle = false;
///////////////////////////////////

	if(flag.stay_command && fabs(x.pos.now-x.pos.init)<X_STAY_MARGIN){
		flag.stay = true;//停止コマンドかつ安定動作完了->停止完了
		mode = Stay;
	}
	else flag.stay = false;
}

//送り時
//足の根元の、地面に対する相対移動速度
void ParallelLeg::calc_velocity()
{
	switch(mode){
	case Move:
		x.vel = -speed;
		y.vel = speed * tan(gradient);	//(y.pos.init-y.pos.now)/(duty*period/4.0)
		break;
	case Up:
		calc_step();//復帰の着地点座標(x)
		//上げ高さ=?
		calc_vel_recovery();
		break;
	case Down:
		calc_step();//復帰の着地点座標(x)
		//着地点y=step*tan(gradient)
		//下げ高さ=最下点y-着地点y
//		height -= 2.0*(step-x.pos.init)*tan(gradient);//毎ループでheight更新するので
		calc_vel_recovery();
		break;
	case Stay:
		x.vel = 0.0;
		y.vel = 0.0;
		break;
	}
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
	float tm = timer_period->read() - timing[1];
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
//	x.vel *= cos(gradient); //坂道用
//	y.vel *= cos(gradient); //坂道用
}

void ParallelLeg::calc_position()
{
	x.pos.dif += x.vel*timer_period->get_dt();
	y.pos.dif += y.vel*timer_period->get_dt();
	if(fabs(y.vel)==0)y.pos.dif = 0;
	x.pos.next = limit(x.pos.init + x.pos.dif, x.pos.max, x.pos.min);
	y.pos.next = limit(y.pos.init + y.pos.dif, y.pos.max, y.pos.min);
}


//障害物用歩行計算
void ParallelLeg::set_stable_timing()
{
	float period_recover = period - time_stablemove; //periodはMove時間も含むため
	//復帰開始
	if(speed>=0){//前進:FR->FL->RR->RL
		if(fr==Front){
			if(rl==Right)timing_stable[1] = 0;
			else timing_stable[1] = period_recover * 1.0/4.0;
		}
		else{
			if(rl==Right)timing_stable[1] = period_recover * 2.0/4.0;
			else timing_stable[1] = period_recover * 3.0/4.0;
		}
	}
	else{
		if(fr==Front){
			if(rl==Right)timing_stable[1] = 0;
			else timing_stable[1] = period_recover * 1.0/4.0;
		}
		else{
			if(rl==Right)timing_stable[1] = period_recover * 2.0/4.0;
			else timing_stable[1] = period_recover * 3.0/4.0;
		}
	}
	if(timing_stable[1]>=period_recover*0.5)timing_stable[1] += (time_stablemove + period_recover*(duty-0.75))/2.0;
	timing_stable[2] = timing_stable[1] + period_recover/4.0; //復帰完了
	timing_stable[3] = period_recover*2.0/4.0; //Move開始
	timing_stable[4] = timing_stable[3] + time_stablemove; //Move完了
	timing_stable[5] = period; //1周期
}

void ParallelLeg::walk_stable_mode()
{
	mode_prv = mode;
	float now = timer_period->read();
	double period_recover = (double)timing_stable[2] - (double)timing_stable[1]; //periodはMove時間も含むため

	if(timing_stable[1]<=now && now<timing_stable[2]){//復帰中
		now -= timing_stable[1]; //復帰開始を基準にとる
//		period_recover *= (1.0-duty); //復帰に要する時間
		if(now<period_recover*(LEGUP_STABLE_TIME/(LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME+LEGDOWN_STABLE_TIME)))
			mode = StableUp;
		else if(now<period_recover*((LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME)/(LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME+LEGDOWN_STABLE_TIME)))
			mode = StableSlide;
		else mode = StableDown;
	}
	else if(timing_stable[3]<=now && now<timing_stable[4])mode = StableMove;
	else mode = StableWait;//他の脚の復帰を待機
}

void ParallelLeg::check_stable_flag()
{
	//復帰完了フラグ
	if(mode!=StableDown && mode_prv==StableDown)
		flag.recovery = true;
	else flag.recovery = false;

	//静止コマンドフラグ
	if(fabs(speed)==0 /*&& fabs(direction)==0*/)//コマンドが「停止」のとき
		flag.stay_command = true;
	else flag.stay_command = false;

//要変更/////////////////////////////
	//最初のサイクルかどうか判断する(現状では最初の踏み出しが完了していないことを示す)フラグ:first_cycle
	if(flag.stay)
		flag.first_cycle = true;
	else if(flag.recovery)
		flag.first_cycle = false;
///////////////////////////////////

	if(flag.stay_command && fabs(x.pos.now-x.pos.init)<X_STAY_MARGIN && fabs(y.pos.now-y.pos.init)<Y_STAY_MARGIN){
		flag.stay = true;//停止コマンドかつ安定動作完了->停止完了
		mode = Stay;
	}
	else flag.stay = false;
}

void ParallelLeg::calc_stable_velocity()
{
	if(mode==StableUp || mode==StableSlide || mode==StableDown || mode==StableMove){
		calc_stable_step();//復帰の着地点座標(x)
		calc_stable_vel_recovery();
	}
	else{
		x.vel = 0.0;
		y.vel = 0.0;
	}
}

void ParallelLeg::calc_stable_step()
{
	if(mode==StableUp && mode_prv!=StableUp){
		x.pos.recover_start = x.pos.now;
		y.pos.recover_start = y.pos.now;
	}
	step = (time_stablemove * speed / 2.0) + x.pos.init;//復帰完了地点
}

void ParallelLeg::calc_stable_vel_recovery()
{
	float tm = timer_period->read() - timing_stable[1];
	float Ty = (1.0-duty)*(period - time_stablemove);//遊脚時間
	float start_time;
	float finish_time; //その動作が終わるまでにかける時間(start_timeが基準)
	switch(mode){
	case StableUp:
		//start_time = 0; //なので省略
		finish_time = Ty*((LEGUP_STABLE_TIME)/(LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME+LEGDOWN_STABLE_TIME));
		x.vel = 0;
		y.vel = -height/finish_time;// * (1.0/2.0) * sin(tm*M_PI/finish_time);// / (M_PI/finish_time);
		break;
	case StableSlide:
		start_time = Ty*((LEGUP_STABLE_TIME)/(LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME+LEGDOWN_STABLE_TIME));
		finish_time = Ty*((LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME)/(LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME+LEGDOWN_STABLE_TIME));
		x.vel = (step-x.pos.recover_start)/(finish_time-start_time);// * (1.0/2.0) * sin((tm-start_time)*M_PI/(finish_time-start_time));// / (M_PI/(finish_time-start_time));
		y.vel = 0;
		break;
	case StableDown:
		start_time = Ty*((LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME)/(LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME+LEGDOWN_STABLE_TIME));
		finish_time = Ty;
		x.vel = 0;
		y.vel = height/(finish_time-start_time);// * (1.0/2.0) * sin((tm-start_time)*M_PI/(finish_time-start_time));// / (M_PI/(finish_time-start_time));
		break;
	case StableMove:
		x.vel = -(step - x.pos.init)/(time_stablemove);// * sin((timer_period->read()-timing_stable[3])*M_PI/time_stablemove);// / (M_PI/time_stablemove);
		y.vel = x.vel * tan(gradient);	//(y.pos.init-y.pos.now)/(duty*period/4.0)
		break;
	}
//	x.vel *= cos(gradient); //坂道用
//	y.vel *= cos(gradient); //坂道用
}

//void ParallelLeg::calc_stable_vel_recovery()
//{
//	float tm = timer_period->read();// - timing_stable[1];
//	float Ty = (1.0-duty)*(period - time_stablemove);//遊脚時間
//	float start_time; //遊脚開始時間
//	float finish_time; //動作が終わる時間
//	float required_time;
//	switch(mode){
//	case StableUp:
//		start_time = timing_stable[1];
//		finish_time = start_time + Ty*((LEGUP_STABLE_TIME)/(LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME+LEGDOWN_STABLE_TIME));
//		required_time = finish_time;
//		x.vel = 0;
//		y.vel = -height/required_time; //-height * (M_PI/required_time) * sin((tm-start_time)*(M_PI/required_time)) / 2.0;
//		break;
//	case StableSlide:
//		start_time = timing_stable[1] + Ty*((LEGUP_STABLE_TIME)/(LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME+LEGDOWN_STABLE_TIME));
//		finish_time = start_time + Ty*((LEGSLIDE_STABLE_TIME)/(LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME+LEGDOWN_STABLE_TIME));
//		required_time = finish_time-start_time;
//		x.vel = (step-x.pos.recover_start)/required_time; //(step-x.pos.recover_start) * (M_PI/required_time) * sin((tm-start_time)*(M_PI/required_time)) / 2.0;
//		y.vel = 0;
//		break;
//	case StableDown:
//		start_time = timing_stable[1] + Ty*((LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME)/(LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME+LEGDOWN_STABLE_TIME));
//		finish_time = start_time + Ty*((LEGDOWN_STABLE_TIME)/(LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME+LEGDOWN_STABLE_TIME));;
//		required_time = finish_time-start_time;
//		x.vel = 0;
//		y.vel = height/required_time; //height * (M_PI/required_time) * sin((tm-start_time)*(M_PI/required_time)) / 2.0;
//		break;
//	case StableMove:
//		x.vel = -/*2.0**/(step - x.pos.init)/(time_stablemove); //-(step - x.pos.init) * (M_PI/time_stablemove) * sin((tm-timing_stable[3])*(M_PI/time_stablemove));
//		y.vel = x.vel * tan(gradient);	//(y.pos.init-y.pos.now)/(duty*period/4.0)
//		break;
//	}
////	x.vel *= cos(gradient); //坂道用
////	y.vel *= cos(gradient); //坂道用
//}


void ParallelLeg::calc_stable_position()
{
	x.pos.dif += x.vel*timer_period->get_dt();
	y.pos.dif += y.vel*timer_period->get_dt();
	if(fabs(y.vel)==0 && mode!=StableSlide)y.pos.dif = 0;
//	if(mode==StableDown)x.pos.dif = 0;
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

