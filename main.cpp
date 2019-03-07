#include "mbed.h"
#include "Pins.h"
#include "functions.h"
#include "Walk/CANs/CANReceiver.h"
#include "Walk/CANs/CANSynchronizer.h"
#include "Walk/ClockTimer.h"
#include "Walk/SingleLeg.h"
#include "Walk/ParallelLeg.h"
#include "Walk/MRMode.h"
#include "Walk/ForwardKinematics.h"

//タイマー辺りに注意
/*
 * 歩行計画
 * デューティー比をベースにして歩行速度を決定する
 */

LocalFileSystem local("local");//PIDゲイン調整に使用

Timer timer_PID;

ClockTimer timer_FR;
ClockTimer timer_FL;
SingleLeg FRf(Front, Right, BASE_X, 0);
SingleLeg FRr(Rear, Right, -BASE_X, 0);
ParallelLeg FR(Front, Right, 200, 200);
SingleLeg FLf(Front, Left, BASE_X, 0);
SingleLeg FLr(Rear, Left, -BASE_X, 0);
ParallelLeg FL(Front, Left, -200, 200);

ForwardKinematics fw_FR(BASE_X, 0, &enc_FRf, -BASE_X, 0, &enc_FRr);
ForwardKinematics fw_FL(BASE_X, 0, &enc_FLf, -BASE_X, 0, &enc_FLr);


void setLegs();
void set_limits();
void CANrcv();
void CANsnd_TimerReset();

struct InitLegInfo{
	bool enc_reset;
	bool finish_init;
	float angle_target;
	float x_target;
	float y_target;
};
void initLegs(SingleLeg *leg_f, InitLegInfo *info_f,
			  SingleLeg *leg_r, InitLegInfo *info_r,
			  ForwardKinematics *fw);
void autoInit();

CANMessage rcvMsg;
CANReceiver can_receiver(&can);
CANSynchronizer can_synchronizer(&can, &CANsnd_TimerReset);

MRMode MRmode(&can_receiver, MRMode::GobiArea, true);//実行の度に要確認


/******************
 * 		main	  *
 ******************/
int main(){
	float walk_period = 1;//2;
	float walk_duty = 0.55;//0.80;
	can.frequency(1000000);
	can.attach(&CANrcv, CAN::RxIrq);
	wait_ms(300); //全ての基板の電源が入るまで待つ
	pc.baud(921600);

	initParts();//センサー・モーター初期化
	setLegs();//不変的設定

	autoInit();//自動キャリブレーション

	set_limits();

	while(1){
		AdjustCycle(1000);//min:0.0008[sec]=800[us](?)

		MRmode.update();
		if(MRmode.is_switched())set_limits();

		FR.set_period(walk_period);
		FR.set_duty(walk_duty);
		FL.set_period(walk_period);
		FL.set_duty(walk_duty);
		can_synchronizer.set_period(walk_period);

		//脚固定系座標での目標位置計算
		FR.walk();
		FL.walk();

		//駆動
		FRf.move_to(FR.get_x(), FR.get_y());
		FRr.move_to(FR.get_x(), FR.get_y());
		FLf.move_to(FL.get_x(), FL.get_y());
		FLr.move_to(FL.get_x(), FL.get_y());

		//DEBUG
		pc.printf("mode:%d  ", FL.get_mode());
		pc.printf("timer:%1.4f  ", timer_FL.read());
		pc.printf("speed:%3.4f  dir:%1.3f  ", can_receiver.get_data(CANID::Speed), can_receiver.get_data(CANID::Direction));
		pc.printf("x:%3.3f  y:%3.3f  ", FL.get_x(), FL.get_y());
		pc.printf("enc:%3.2f  ", enc_FLf.getAngle());
		pc.printf("angle:%3.2f  duty:%1.4f  ", FLf.get_angle(), FLf.get_duty());

		pc.printf("\r\n");
	}
}


void setLegs(){
	FRf.unitize(&motor_FRf, &enc_FRf, &sw_FRf, &timer_PID);
	FRr.unitize(&motor_FRr, &enc_FRr, &sw_FRr, &timer_PID);
	FLf.unitize(&motor_FLf, &enc_FLf, &sw_FLf, &timer_PID);
	FLr.unitize(&motor_FLr, &enc_FLr, &sw_FLr, &timer_PID);
	FRf.set_PID_from_file("/local/PID_FRf.txt");
	FRr.set_PID_from_file("/local/PID_FRr.txt");
	FLf.set_PID_from_file("/local/PID_FLf.txt");
	FLr.set_PID_from_file("/local/PID_FLr.txt");

	FRf.set_dependencies(&MRmode);
	FRr.set_dependencies(&MRmode);
	FLf.set_dependencies(&MRmode);
	FLr.set_dependencies(&MRmode);
	FR.set_dependencies(&timer_FR, &MRmode, &can_receiver, &can_synchronizer);
	FL.set_dependencies(&timer_FL, &MRmode, &can_receiver, &can_synchronizer);
}


void set_limits(){
	FRf.set_limits();
	FRr.set_limits();
	FLf.set_limits();
	FLr.set_limits();
	FR.set_limits();
	FL.set_limits();
	FR.set_orbits();
	FL.set_orbits();
}


void CANrcv(){
	if(can.read(rcvMsg)){
		unsigned int id = rcvMsg.id;
		if(CANID_is_from(id, CANID::FromMaster)){
			if(CANID_is_type(id, CANID::TimerReset))return;
			if(CANID_is_to(id, CANID::ToSlaveAll)){
				//歩行パラメータ取得
				can_receiver.receive(id, rcvMsg.data);
			}
		}
	}
}


//Timer同期用
void CANsnd_TimerReset(){
	timer_FR.reset();
	timer_FL.reset();
	can_synchronizer.timer_reset(false);
}


void initLegs(SingleLeg *leg_f, InitLegInfo *info_f,
		  	  SingleLeg *leg_r, InitLegInfo *info_r,
			  ForwardKinematics *fw){
	//ゆっくりスイッチに近づける
	if(!info_f->enc_reset){
		info_f->angle_target = leg_f->get_enc() + 0.2;//200[roop/sec], 20[degree/sec] -> 0.1[degree/roop]
		info_r->angle_target = leg_r->get_enc() - 0.1;
		if(leg_f->get_sw()){
			leg_f->reset_duty();leg_r->reset_duty();
			leg_f->reset_enc();
			info_f->enc_reset = true;
		}
	}
	else if(!info_r->enc_reset){
		info_f->angle_target = 30;
		info_r->angle_target = leg_r->get_enc() + 0.2;
		if(leg_r->get_sw()){
			leg_f->reset_duty();leg_r->reset_duty();
			leg_r->reset_enc();
			info_r->enc_reset = true;
		}
	}
	else{//enc両方リセット完了
		if((fabs(0.0f - fw->get_x()) < 1.0) && (fabs(250.0f - fw->get_y()) < 1.0))
			info_f->finish_init = info_r->finish_init = true;
		fw->estimate();
		info_f->x_target = info_r->x_target = 0.0f;
		info_f->y_target = info_r->y_target = 250.0f;
	}

	//駆動
	if(!info_f->enc_reset){
		leg_f->set_duty_limit(0.55, 0.45);
		leg_r->set_duty_limit(0.55, 0.45);
		leg_f->move_to_angle(info_f->angle_target);
		leg_r->move_to_angle(info_r->angle_target);
	}
	else if(!info_r->enc_reset){
		leg_f->set_duty_limit(0.575, 0.4);
		leg_r->set_duty_limit(0.575, 0.4);
		leg_f->move_to_angle(info_f->angle_target);
		leg_r->move_to_angle(info_r->angle_target);
	}
	else{
		leg_f->set_duty_limit(0.6, 0.4);
		leg_r->set_duty_limit(0.6, 0.4);
		leg_f->move_to(info_f->x_target, info_f->y_target);
		leg_r->move_to(info_r->x_target, info_r->y_target);
	}
}

void autoInit(){
	InitLegInfo frf, frr, flf, flr;
	frf.enc_reset = frr.enc_reset = flf.enc_reset = flr.enc_reset = false;
	frf.finish_init = frr.finish_init = flf.finish_init = flr.finish_init = false;
	while(!(frf.finish_init && frr.finish_init && flf.finish_init && flr.finish_init)){
		AdjustCycle(5000);
		initLegs(&FRf, &frf, &FRr, &frr, &fw_FR);
		initLegs(&FLf, &flf, &FLr, &flr, &fw_FL);
		pc.printf("enc");
		pc.printf("[%3.2f][%3.2f]", enc_FRf.getAngle(), enc_FRr.getAngle());
		pc.printf("[%3.2f][%3.2f]", enc_FLf.getAngle(), enc_FLr.getAngle());
		pc.printf("  target");
		if(!frf.enc_reset || !frr.enc_reset)
			pc.printf("ang[%3.2f][%3.2f]", frf.angle_target, frr.angle_target);
		else{
			pc.printf("xy[%3.2f][%3.2f]", frf.x_target, frf.y_target);
			pc.printf("est[%3.2f][%3.2f]", fw_FR.get_x(), fw_FR.get_y());
		}
		if(!flf.enc_reset || !flr.enc_reset)
			pc.printf("ang[%3.2f][%3.2f]", flf.angle_target, flr.angle_target);
		else{
			pc.printf("xy[%3.2f][%3.2f]", flf.x_target, flr.y_target);
			pc.printf("est[%3.2f][%3.2f]", fw_FL.get_x(), fw_FL.get_y());
		}
//		pc.printf("  sw");
//		pc.printf("[%d][%d]", sw_FRf.read(), sw_FRr.read());
//		pc.printf("[%d][%d]", sw_FLf.read(), sw_FLr.read());
		pc.printf("  duty");
		pc.printf("[%1.3f][%1.3f]", FRf.get_duty(), FRr.get_duty());
		pc.printf("[%1.3f][%1.3f]", FLf.get_duty(), FLr.get_duty());
		pc.printf("\r\n");
	}
}
