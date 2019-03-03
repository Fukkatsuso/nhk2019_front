#include "mbed.h"
#include "Pins.h"
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

ClockTimer timer_FR;
ClockTimer timer_FL;
SingleLeg FRf(Front, Right, BASE_X, 0);
SingleLeg FRr(Rear, Right, -BASE_X, 0);
ParallelLeg FR(Front, Right, 200, 200);
SingleLeg FLf(Front, Left, BASE_X, 0);
SingleLeg FLr(Rear, Left, -BASE_X, 0);
ParallelLeg FL(Front, Left, -200, 200);


void initLegs();
void set_limits();
void CANrcv();
void CANsnd_TimerReset();

CANMessage rcvMsg;
CANReceiver can_receiver(&can);
CANSynchronizer can_synchronizer(&can, &CANsnd_TimerReset);

MRMode MRmode(&can_receiver, MRMode::GobiArea, true);//実行の度に要確認


/******************
 * 		main	  *
 ******************/
int main(){
	float walk_period = 2;
	float walk_duty = 0.80;
	can.frequency(1000000);
	can.attach(&CANrcv, CAN::RxIrq);
	wait_ms(300); //全ての基板の電源が入るまで待つ
	pc.baud(921600);

	initParts();
	initLegs();

	while(1){
		AdjustCycle(1000);//min:0.0008[sec]=800[us]

		MRmode.update();
		if(MRmode.is_switched())set_limits();

		FR.set_period(walk_period);
		FR.set_duty(walk_duty);
		FL.set_period(walk_period);
		FL.set_duty(walk_duty);

		//脚固定系座標での目標位置計算
		FR.walk();
		FL.walk();

		//駆動
		FRf.move_to(FR.get_x(), FR.get_y());
		FRr.move_to(FR.get_x(), FR.get_y());
		FLf.move_to(FL.get_x(), FL.get_y());
		FLr.move_to(FL.get_x(), FL.get_y());

		//DEBUG
		pc.printf("mode:%d  ", FR.get_mode());
		pc.printf("timer:%1.4f  ", timer_FR.read());
		pc.printf("speed:%3.4f  dir:%1.3f  ", can_receiver.get_data(CANID::Speed), can_receiver.get_data(CANID::Direction));
		pc.printf("x:%3.3f  y:%3.3f  ", FR.get_x(), FR.get_y());
		pc.printf("y_vel:%3.3f  ", FR.get_y_vel());
		pc.printf("sw[%d][%d][%d][%d]  ", sw_FRf.read(), sw_FRr.read(), sw_FLf.read(), sw_FLr.read());
		pc.printf("enc[%2.2f][%2.2f][%2.2f][%2.2f]  ", enc_FRf.getAngle(), enc_FRr.getAngle(), enc_FLf.getAngle(), enc_FLr.getAngle());

		pc.printf("\r\n");
	}
}


void initLegs(){
	FRf.unitize(&motor_FRf, &enc_FRf, &sw_FRf);
	FRr.unitize(&motor_FRr, &enc_FRr, &sw_FRr);
	FLf.unitize(&motor_FLf, &enc_FLf, &sw_FLf);
	FLr.unitize(&motor_FLr, &enc_FLr, &sw_FLr);
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
