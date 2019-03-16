#include "mbed.h"
#include "Pins.h"
#include "functions.h"
#include "LegFunctions.h"
#include "Walk/CANs/CANReceiver.h"
#include "Walk/CANs/CANSynchronizer.h"
#include "Walk/ClockTimer.h"
#include "Walk/SingleLeg.h"
#include "Walk/ParallelLeg.h"
#include "Walk/MRMode.h"
#include "Walk/ForwardKinematics.h"

/*
 * 予定
 * 障害物用の軌道を模索
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

CANMessage rcvMsg;
CANReceiver can_receiver(&can);
void CANsnd_TimerReset(); //Ticker割り込み用関数
CANSynchronizer can_synchronizer(&can, &CANsnd_TimerReset);

MRMode MRmode(&can_receiver, MRMode::GobiArea, true);//実行の度に要確認

void set_cycle(float *period, float *duty);
void CANrcv();


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
		set_limits();

		set_cycle(&walk_period, &walk_duty);

		FR.set_period(walk_period);
		FR.set_duty(walk_duty);
		FL.set_period(walk_period);
		FL.set_duty(walk_duty);
		can_synchronizer.set_period(walk_period);

		//腰固定座標系での目標位置計算
		if(MRmode.get_now()==MRMode::SandDuneFront || MRmode.get_now()==MRMode::SandDuneRear){
			if((int)can_receiver.get_data(CANID::LegUp)&0x1)FR.set_y_initial(280-100);
			if((int)can_receiver.get_data(CANID::LegUp)&0x4)FL.set_y_initial(280-100);
		}

		if(MRMode::StartClimb1<=MRmode.get_now() && MRmode.get_now()<=MRMode::MountainArea){
			//1歩1歩進めていく歩容
			//ただし、復帰幅と送り幅が違って徐々に足が前にいってしまう
			//軌道計算を要再考
			FR.walk_stable(can_receiver.get_data(CANID::Speed), can_receiver.get_data(CANID::Direction), 0.1);
			moveLeg(&FRf, &FRr, FR.get_x(), FR.get_y());
			FL.walk_stable(can_receiver.get_data(CANID::Speed), can_receiver.get_data(CANID::Direction), 0.1);
			moveLeg(&FLf, &FLr, FL.get_x(), FL.get_y());
		}
		else{
			FR.walk();
			moveLeg(&FRf, &FRr, FR.get_x(), FR.get_y());
			FL.walk();
			moveLeg(&FLf, &FLr, FL.get_x(), FL.get_y());
		}

		//DEBUG
		if(pc.readable()){
//			pc.printf("mode:%d  ", FL.get_mode());
//			pc.printf("timer:%1.4f  ", timer_FL.read());
//			pc.printf("speed:%3.4f  dir:%1.3f  ", can_receiver.get_data(CANID::Speed), can_receiver.get_data(CANID::Direction));
//			pc.printf("x:%3.3f  y:%3.3f  ", FL.get_x(), FL.get_y());
//			pc.printf("enc:%3.2f  ", enc_FLf.getAngle());
//			pc.printf("angle:%3.2f  duty:%1.4f  ", FLf.get_angle(), FLf.get_duty());
//
//			pc.printf("vel[%3.2f][%3.2f]  ", FL.get_x_vel(), FL.get_y_vel());

			pc.printf("%5.3f\t%5.3f\t", FRf.get_D(), FRr.get_D());
			orbit_log(&FR, &fw_FR);
			pc.printf("\r\n");
		}
	}
}


//Timer同期用
void CANsnd_TimerReset(){
	timer_FR.reset();
	timer_FL.reset();
	can_synchronizer.timer_reset(false);
}


void set_cycle(float *period, float *duty){
	switch((int)MRmode.get_now()){
	case MRMode::GobiArea:
		*period = 2;//1.5;//1;
		*duty = 0.55;
		break;
	case MRMode::SandDuneFront:
		*period = 1.6;//5;//1;
		*duty = 0.55;//0.8;//0.55;
		break;
	case MRMode::SandDuneRear:
		*period = 1.6;//5;//1;
		*duty = 0.55;//0.8;//0.55;
		break;
	case MRMode::Tussock1:
		*period = 1;
		*duty = 0.55;
		break;
	case MRMode::Start2:
		*period = 2;
		*duty = 0.8;
		break;
	case MRMode::StartClimb1:
		*period = 5;
		*duty = 0.8;
		break;
	case MRMode::StartClimb2:
		*period = 5;
		*duty = 0.8;
	}
}


void CANrcv(){
	if(can.read(rcvMsg)){
		unsigned int id = rcvMsg.id;
		if(!CANID_is_from(id, CANID::FromMaster))return;
		if(CANID_is_type(id, CANID::TimerReset))return;
		if(CANID_is_to(id, CANID::ToSlaveAll)){
			//歩行パラメータ取得
			can_receiver.receive(id, rcvMsg.data);
		}
	}
}
