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
	gait_mode = Gait::NormalGait;
	recovery_mode = Recovery::Cycloid;
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
	flag.sanddune = false;
	flag.tussock = false;

	counter.walk_on_dune = 0;
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
	area = MRmode->get_now();
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


// 通常/安定, 復帰軌道, 4足移動総時間(1周期あたり)
void ParallelLeg::set_walkmode(Gait::Mode gait, Recovery::Mode recovery, float time_stablemove_rate=0)
{
	gait_mode = gait;
	recovery_mode = recovery;
	this->time_stablemove = period*time_stablemove_rate/2.0; //１度の４足移動にかける時間
}

void ParallelLeg::trigger_sanddune(int trigger)
{
	if(!(area==MRMode::SandDuneFront || area==MRMode::SandDuneRear)){
		flag.sanddune = false;
		return;
	}
//	if(mode==StableDown){
//		if(counter.walk_on_dune>2){//3歩目の着地動作はSandDuneを越えているとする
//			flag.sanddune = false;
//			counter.walk_on_dune = 0;
//			return;
//		}
//		if(flag.sanddune && mode_prv==StableSlide)counter.walk_on_dune++; //復帰着地の瞬間、1歩カウント
//	}
//	if(!trigger)return; //そのまま
	//Quadrangle
//	if(mode==StableDown)flag.sanddune = (bool)trigger;
	if(!trigger && mode==StableDown)flag.sanddune = false;
	if(trigger && mode==StableSlide)flag.sanddune = true; //復帰スライド中にy初期位置を設定
	//Cycloid
	//	if(mode==Down)flag.sanddune = (bool)trigger;
	if(!trigger && mode==Down)flag.sanddune = false;
	if(trigger && mode==Down)flag.sanddune = true;
}

void ParallelLeg::trigger_tussock(int trigger)
{
	if(area!=MRMode::Tussock){
		flag.tussock = false;
		return;
	}
	if(!trigger)return; //そのまま
	if(mode==Move)flag.tussock = true; //支持脚中に足上げ最高点を設定
}

void ParallelLeg::over_obstacle()
{
	if(flag.sanddune){
		set_y_initial(260-100);
		set_height(y.pos.init-y.pos.min);
		return;
	}
	if(flag.tussock){
		if(mode==Up)set_height(360);
		else if(mode==Down){
			set_height(y.pos.init - y.pos.min);
		}
		else if(mode_prv==Down)flag.tussock = false; //着地完了
		return;
	}
}

//walk//////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParallelLeg::walk(float spd, float dir)
{
	direction = dir;
	speed = curve_adjust(spd);
	x.pos.now = x.pos.next;
	y.pos.now = y.pos.next;
	over_obstacle();
	timer_update();

	switch(gait_mode){
	case Gait::NormalGait:
		set_timing();//足上げタイミング等計算
		walk_mode();//足のモード
		check_flag();
		calc_velocity();//vel計算
		calc_position();//pos計算
		break;
	case Gait::StableGait:
		set_timing_stable();//
		walk_mode_stable();//
		check_flag_stable();//
		calc_velocity_stable();//
		calc_position_stable();//
		break;
	case Gait::ActiveStableGait:
		set_timing_activestable();//
		walk_mode_activestable();//
		check_flag_stable();//
		calc_velocity_activestable();//
		calc_position_stable();//
		break;
	}
}

void ParallelLeg::walk()
{
//	set_period(CANcmd->get(CANID::Period)); set_duty(CANcmd->get(CANID::Duty));
	walk(can_receiver->get_data(CANID::Speed), can_receiver->get_data(CANID::Direction));
}


//斜め方向に歩くとき
float ParallelLeg::curve_adjust(float value)
{
	if((rl*direction)>0)	//その足の方に歩く->小股で
		value *= (cos(2.0*direction));
	return value;
}

void ParallelLeg::timer_update()
{	//歩き始めたらタイマーリセット->その瞬間tickerセット
	if(timer_period->read()>period)timer_period->reset();
	else if(mode==Stay)timer_period->reset();
	timer_period->calc_dt(); //時刻更新
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
	}
	//timing[1]とtiming[2]を1:1に内分
	else if(timing[1]<=now && now<(timing[1]+timing[2])/2.0)mode = Up;
	else if(((timing[1]+timing[2])/2.0)<=now && now<timing[2])mode = Down;
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

	if(flag.stay_command){
		if(fabs(x.pos.now-x.pos.init)<X_STAY_MARGIN && fabs(y.pos.now-y.pos.init)<Y_STAY_MARGIN){
			flag.stay = true;//停止コマンドかつ安定動作完了->停止完了
			mode = Stay;
		}
	}
	else flag.stay = false;
}

//足の根元の、地面に対する相対移動速度
void ParallelLeg::calc_velocity()
{
	//静止
	if(mode==Stay){
		x.vel = 0;
		y.vel = 0;
		return;
	}
	//支持脚
	if(mode==Move){
		calc_vel_move();
		return;
	}
	//復帰
	calc_step();
	calc_vel_recovery_cycloid(timing[1], (1.0-duty)*period);
}

void ParallelLeg::calc_vel_move()
{
	x.vel = -speed;
	y.vel = speed * tan(gradient);	//(y.pos.init-y.pos.now)/(duty*period/4.0)
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

void ParallelLeg::calc_vel_recovery_cycloid(float timing_start, float Ty)
{
	float tm = timer_period->read() - timing_start;
	//x速度計算
	x.vel = ((step-x.pos.recover_start)/Ty) * (1.0 - cos(2.0*M_PI*tm/Ty)/duty);//計算改良1
	//y速度計算
	if(!flag.sanddune && mode==Down)height += y.pos.init - y.pos.recover_start;
	if(0<=tm && tm<=Ty/4.0)
		y.vel = FACTOR_Y * (1.0 - cos(4.0*M_PI*tm/Ty));
	else if(Ty/4.0<tm && tm<Ty*3.0/4.0)
		y.vel = 2.0 * FACTOR_Y * cos(2.0*M_PI*(tm/Ty-1.0/4.0));
	else if(Ty*3.0/4.0<=tm && tm<=Ty)
		y.vel = -FACTOR_Y * (1.0 - cos(4.0*M_PI*(1.0-tm/Ty)));
	y.vel += -(step-x.pos.recover_start)*tan(gradient)/Ty;
//	x.vel *= cos(gradient); //坂道用
//	y.vel *= cos(gradient); //坂道用
}

void ParallelLeg::calc_position()
{
	x.pos.dif = limit(x.pos.dif + x.vel*timer_period->get_dt(), x.pos.max-x.pos.init, x.pos.min-x.pos.init);
	y.pos.dif = limit(y.pos.dif + y.vel*timer_period->get_dt(), y.pos.max-y.pos.init, y.pos.min-y.pos.init);
	if(fabs(y.vel)==0)y.pos.dif = 0;
	x.pos.next = x.pos.init + x.pos.dif;
	y.pos.next = y.pos.init + y.pos.dif;
}


//障害物用歩行計算
void ParallelLeg::set_timing_stable()
{
	float period_recover = period - time_stablemove*2.0; //periodはMove時間も含むため
	//復帰開始
	if(speed>=0){//前進:FR&RL->FL&RR
		if(fr==Front){
			if(rl==Right)timing_stable[1] = 0;
			else timing_stable[1] = period * 0.5;
		}
		else{
			//period/2 - time_stablemove - (1-duty)*period_recover
			//= period_recover/2 - (1-duty)*period_recover
			//= (duty-0.5)*period_recover
			if(rl==Left)timing_stable[1] = (duty-0.5)*period_recover;
			//period - time_stablemove - (1-duty)*period_recover
			//= period_recover + time_stablemove - (1-duty)*period_recover
			//= duty*period_recover + time_stablemove
			else timing_stable[1] = duty*period_recover + time_stablemove;
		}
	}
	else{
		if(fr==Rear){
			if(rl==Left)timing_stable[1] = 0;
			else timing_stable[1] = period * 0.5;
		}
		else{
			if(rl==Right)timing_stable[1] = (duty-0.5)*period_recover;
			else timing_stable[1] = duty*period_recover + time_stablemove;
		}
	}
	timing_stable[2] = timing_stable[1] + period_recover*(1.0-duty); //復帰完了
	timing_stable[3] = period_recover/2.0; //Move1開始
	timing_stable[4] = timing_stable[3] + time_stablemove; //Move1完了
	timing_stable[5] = period - time_stablemove; //Move2開始
	timing_stable[6] = timing_stable[7] = period; //Move2終了・1周期
}

void ParallelLeg::walk_mode_stable()
{
	mode_prv = mode;
	double period_recover = (double)timing_stable[2] - (double)timing_stable[1]; //復帰に要する時間。periodはMove時間も含むため
	float now = timer_period->read();

	switch(recovery_mode){
	case Recovery::Quadrangle:
		if(timing_stable[1]<=now && now<timing_stable[2]){//復帰中
			now -= timing_stable[1]; //復帰開始を基準にとる
			if(now<period_recover*(LEGUP_STABLE_TIME/(LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME+LEGDOWN_STABLE_TIME)))
				mode = StableUp;
			else if(now<period_recover*((LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME)/(LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME+LEGDOWN_STABLE_TIME)))
				mode = StableSlide;
			else mode = StableDown;
		}
		else if(timing_stable[3]<=now && now<timing_stable[4])mode = StableMove;
		else if(timing_stable[5]<=now && now<timing_stable[6])mode = StableMove;
		else mode = StableWait;//他の脚の復帰を待機するなど
		break;

	case Recovery::Cycloid:
		//timing[1]とtiming[2]を1:1に内分
		if(timing_stable[1]<=now && now<(timing_stable[1]+timing_stable[2])/2.0)mode = Up;
		else if(((timing_stable[1]+timing_stable[2])/2.0)<=now && now<timing_stable[2])mode = Down;
		else if(timing_stable[3]<=now && now<timing_stable[4])mode = StableMove;
		else if(timing_stable[5]<=now && now<timing_stable[6])mode = StableMove;
		else mode = StableWait;//他の脚の復帰を待機するなど
		break;
	}
}

void ParallelLeg::check_flag_stable()
{
	//復帰完了フラグ
	switch(recovery_mode){
	case Recovery::Quadrangle:
		if(mode!=StableDown && mode_prv==StableDown)
			flag.recovery = true;
		else flag.recovery = false;
		break;
	case Recovery::Cycloid:
		if(mode!=Down && mode_prv==Down)
			flag.recovery = true;
		else flag.recovery = false;
		break;
	}

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

void ParallelLeg::calc_velocity_stable()
{
	if(mode==Stay || mode==StableWait){
		x.vel = y.vel = 0;
		return;
	}

	calc_step_stable();

	if(mode==StableMove){
		calc_vel_move_stable();
		return;
	}

	switch(recovery_mode){
	case Recovery::Cycloid:
		calc_vel_recovery_cycloid(timing_stable[1], (1.0-duty)*(period - time_stablemove*2.0));
		break;
	case Recovery::Quadrangle:
		calc_vel_recovery_quadrangle(timing_stable[1], (1.0-duty)*(period - time_stablemove*2.0));
		break;
	}
}

void ParallelLeg::calc_step_stable()
{
	switch(recovery_mode){
	case Recovery::Quadrangle:
		if(mode==StableUp && mode_prv!=StableUp){
			x.pos.recover_start = x.pos.now;
			y.pos.recover_start = y.pos.now;
		}
		break;
	case Recovery::Cycloid:
		if(mode==Up && mode_prv!=Up){
			x.pos.recover_start = x.pos.now;
			y.pos.recover_start = y.pos.now;
		}
		break;
	}
	step = (time_stablemove * speed/* / 2.0*/) + x.pos.init;//復帰完了地点
}

void ParallelLeg::calc_vel_move_stable()
{
	x.vel = -(step - x.pos.init)/(time_stablemove);// * sin((timer_period->read()-timing_stable[3])*M_PI/time_stablemove);// / (M_PI/time_stablemove);
	y.vel = x.vel * tan(gradient);	//(y.pos.init-y.pos.now)/(duty*period/4.0)
}

void ParallelLeg::calc_vel_recovery_quadrangle(float timing_start, float Ty)
{
	float required_time;
	float distance_x, distance_y;

	switch(mode){
	case StableUp:
		required_time = Ty*((LEGUP_STABLE_TIME)/(LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME+LEGDOWN_STABLE_TIME));
		distance_x = 0;
		distance_y = -height;
		break;
	case StableSlide:
		required_time = Ty*((LEGSLIDE_STABLE_TIME)/(LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME+LEGDOWN_STABLE_TIME));
		distance_x = step - x.pos.recover_start;
		distance_y = 0;
		break;
	case StableDown:
		required_time = Ty*((LEGDOWN_STABLE_TIME)/(LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME+LEGDOWN_STABLE_TIME));
		distance_x = 0;
		distance_y = height;
		break;
	default:
		return;
	}
	x.vel = distance_x / required_time;
	y.vel = distance_y / required_time;
//	x.vel *= cos(gradient); //坂道用
//	y.vel *= cos(gradient); //坂道用
}

void ParallelLeg::calc_position_stable()
{
	x.pos.dif += limit(x.vel*timer_period->get_dt(), x.pos.max-x.pos.now, x.pos.min-x.pos.now);
	y.pos.dif += limit(y.vel*timer_period->get_dt(), y.pos.max-y.pos.now, y.pos.min-y.pos.now);
	if(recovery_mode==Recovery::Quadrangle){
		if(fabs(y.vel)==0 && mode!=StableSlide)y.pos.dif = 0;
		if(fabs(x.vel)==0 && mode==StableDown)x.pos.dif = step - x.pos.init;
	}

	x.pos.next = x.pos.init + x.pos.dif;
	y.pos.next = y.pos.init + y.pos.dif;
}


void ParallelLeg::set_timing_activestable()
{
	float period_recover = period - time_stablemove*2.0; //periodはMove時間も含むため
	//復帰開始
	if(speed>=0){//前進:FR&RL->FL&RR
		if(fr==Front){
			if(rl==Right)timing[1] = 0;
			else timing[1] = period * 0.5;
		}
		else{
			if(rl==Left)timing[1] = (duty-0.5)*period_recover;
			else timing[1] = duty*period_recover + time_stablemove;
		}
	}
	else{
		if(fr==Rear){
			if(rl==Left)timing[1] = 0;
			else timing[1] = period * 0.5;
		}
		else{
			if(rl==Right)timing[1] = (duty-0.5)*period_recover;
			else timing[1] = duty*period_recover + time_stablemove;
		}
	}
	timing[2] = timing[1] + period_recover*(1.0-duty);//復帰完了
	timing[3] = period;//1周期完了
}

void ParallelLeg::walk_mode_activestable()
{
	mode_prv = mode;
	double period_recover = (double)timing[2] - (double)timing[1]; //復帰に要する時間。periodはMove時間も含むため
	float now = timer_period->read();

	switch(recovery_mode){
	case Recovery::Quadrangle:
		if(timing[1]<=now && now<timing[2]){//復帰中
			now -= timing[1]; //復帰開始を基準にとる
			if(now<period_recover*(LEGUP_STABLE_TIME/(LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME+LEGDOWN_STABLE_TIME)))
				mode = StableUp;
			else if(now<period_recover*((LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME)/(LEGUP_STABLE_TIME+LEGSLIDE_STABLE_TIME+LEGDOWN_STABLE_TIME)))
				mode = StableSlide;
			else mode = StableDown;
		}
		else mode = Move;
		break;
	case Recovery::Cycloid:
		//timing[1]とtiming[2]を1:1に内分
		if(timing[1]<=now && now<(timing[1]+timing[2])/2.0)mode = Up;
		else if(((timing[1]+timing[2])/2.0)<=now && now<timing[2])mode = Down;
		else mode = Move;
		break;
	}
}

void ParallelLeg::calc_velocity_activestable()
{
	//静止
	if(mode==Stay){
		x.vel = 0;
		y.vel = 0;
		return;
	}
	//支持脚
	if(mode==Move){
		calc_vel_move();
		return;
	}
	//復帰
	calc_step_stable();
	switch(recovery_mode){
	case Recovery::Cycloid:
		calc_vel_recovery_cycloid(timing[1], (1.0-duty)*(period - time_stablemove*2.0));
		break;
	case Recovery::Quadrangle:
		calc_vel_recovery_quadrangle(timing[1], (1.0-duty)*(period - time_stablemove*2.0));
		break;
	}
}


//DEBUG用///////////////////////////////////////////////////////////////////////////////////////


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

