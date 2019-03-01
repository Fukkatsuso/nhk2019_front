#include "mbed.h"
#include "Pins.h"
#include "Walk/CANs/CANReceiver.h"
#include "Walk/CANs/CANSynchronizer.h"
#include "Walk/ClockTimer.h"
#include "Walk/SingleLeg.h"
#include "Walk/ParallelLeg.h"
#include "Walk/MRMode.h"
#include "Walk/ForwardKinematics.h"

/*
 * 歩行計画
 * デューティー比をベースにして歩行速度を決定する
 */

LocalFileSystem local("local");//PIDゲイン調整に使用

CANMessage rcvMsg;
CANReceiver can_receiver(&can);
CANSynchronizer can_synchronizer(&can);

MRMode MRmode(&can_receiver, MRMode::GobiArea, true);//実行の度に要確認

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


/******************
 * 		main	  *
 ******************/
int main(){
	can.frequency(1000000);
	can.attach(&CANrcv, CAN::RxIrq);
	wait_ms(300); //全ての基板の電源が入るまで待つ
	pc.baud(921600);

	initParts();
	initLegs();

	while(1){
		AdjustCycle(5000);

		MRmode.update();
		if(MRmode.is_switched())set_limits();

		//脚固定系座標での目標位置計算
		FR.walk();
		FL.walk();

		//駆動
		FRf.state_update();
		FRr.state_update();
		FLf.state_update();
		FLr.state_update();
		FRf.move_to(FR.get_x(), FR.get_y());
		FRr.move_to(FR.get_x(), FR.get_y());
		FLf.move_to(FL.get_x(), FL.get_y());
		FLr.move_to(FL.get_x(), FL.get_y());
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
}


void CANrcv(){
	if(can.read(rcvMsg)){
		if(CANID_is_from(rcvMsg.id, CANID::FromMaster) && CANID_is_to(rcvMsg.id, CANID::ToSlaveAll)){
			//歩行パラメータ取得
			can_receiver.receive(rcvMsg.id, rcvMsg.data);
//			if(CANID_is_type(rcvMsg.id, CANID::TimerReset) && CANcmd.get(CANID::TimerReset)){
//				//タイマーリセット
//				timer_FR.reset();
//				timer_FL.reset();
//			}
		}
	}
}
