#include "mbed.h"
#include "Pins.h"
#include "functions.h"
#include "LegFunctions.h"
#include "Walk/CANs/CANReceiver.h"
#include "Walk/CANs/CANSender.h"
#include "Walk/CANs/CANSynchronizer.h"
#include "Walk/ClockTimer.h"
#include "Walk/SingleLeg.h"
#include "Walk/ParallelLeg.h"
#include "Walk/MRMode.h"
#include "Walk/ForwardKinematics.h"


LocalFileSystem local("local");//PIDゲイン調整に使用

Timer timer_PID;

ClockTimer timer_FR;
ClockTimer timer_FL;
SingleLeg FRf(Front, Right, BASE_X, 0);
SingleLeg FRr(Rear, Right, -BASE_X, 0);
ParallelLeg FR(Front, Right, 225, 200);
SingleLeg FLf(Front, Left, BASE_X, 0);
SingleLeg FLr(Rear, Left, -BASE_X, 0);
ParallelLeg FL(Front, Left, -225, 200);

ForwardKinematics fw_FR(BASE_X, 0, &enc_FRf, -BASE_X, 0, &enc_FRr);
ForwardKinematics fw_FL(BASE_X, 0, &enc_FLf, -BASE_X, 0, &enc_FLr);

CANMessage rcvMsg;
CANReceiver can_receiver(&can);
CANSender can_sender(&can);
void CANsnd_TimerReset(); //Ticker割り込み用関数
CANSynchronizer can_synchronizer(&can, &CANsnd_TimerReset);

MRMode MRmode(&can_receiver, &can_sender, MRMode::GobiArea, true);//実行の度に要確認

void set_cycle(float *period, float *duty);
void send_leg_info(enum CANID::From from, short dist, unsigned char state, unsigned char kouden);
void CANrcv();


/******************
 * 		main	  *
 ******************/
int main(){
	//歩行に必要な情報
	float walk_period = 1;
	float walk_duty = 0.5;
	float walk_speed = 0;
	float walk_direction = 0;
	float walk_pitch = 0; //radに変換して格納すること
	//歩いた結果の情報
	float walk_dist_right = 0;
	float walk_dist_left = 0;
	float walk_dist_front = 0;
	unsigned int kouden_SandDuneFront_max = 0;
	union can_LegState state_front = {};
	state_front.right = state_front.left = Stay;
	union can_Kouden kouden_front = {};

	int mrmode = (int)MRmode.get_now();

	can.frequency(1000000);
	can.attach(&CANrcv, CAN::RxIrq);
	wait_ms(300); //全ての基板の電源が入るまで待つ
	pc.baud(921600);

	initParts();//センサー・モーター初期化
	setLegs();//不変的設定

	send_leg_info(CANID::FromFront,	0, state_front.byte[0], 0);

	autoInit();//自動キャリブレーション

	set_limits();

	walk_duty = 0.5;
	while(1){
		AdjustCycle(1000);

		MRmode.update();
		set_limits();

		kouden_SandDuneFront.sensing();
		mrmode = MRmode.get_now();

		//基本:脚下げ時に光電センサの値が閾値以下(カウンタ最大値の一定以下?)であれば一番下まで下げる
		//段差開始:Down時[flag==0 かつ 光電センサのカウンタ > ## かつ walk_on_dune==0] であれば、flagを1にする
		//段差終了:Down時[flag==1 かつ 光電センサのカウンタ < カウンタの最大値-??] であれば、flagを0にする
		//指定の歩数以上歩いたら強制的に一番下まで下げる(脚下げ時のみ判断)
		//trigger_sanddune(kouden.get_counter(0), cnt_max, ##, cnt_max-??, max_on_dune, area);
		FR.trigger_sanddune(
				kouden_SandDuneFront.get_counter(0), kouden_SandDuneFront_max, 120, uint_cut(kouden_SandDuneFront_max, 250),
				3-1, MRMode::SandDuneFront);
		FL.trigger_sanddune(
				kouden_SandDuneFront.get_counter(0), kouden_SandDuneFront_max, 120, uint_cut(kouden_SandDuneFront_max, 250),
				3-1, MRMode::SandDuneFront);

		if(mrmode==MRMode::SandDuneFront || mrmode==MRMode::SandDuneRear){
			FR.set_walkmode(Gait::ActiveStableGait, Recovery::Quadrangle, 0);
			FL.set_walkmode(Gait::ActiveStableGait, Recovery::Quadrangle, 0);
			//trigger_sanddune実行後にこれを行う
			if(mrmode==MRMode::SandDuneFront){
				//最大値更新
				kouden_SandDuneFront_max = max_uint(kouden_SandDuneFront_max, kouden_SandDuneFront.get_counter(0));
				//段差に乗っていない かつ 段差上を歩いた回数(各脚)>0 -> モード切替
				if(FR.get_count_walk_on_dune()>0 && FL.get_count_walk_on_dune()>0){
					if(!FR.is_on_dune() && !FL.is_on_dune()){
						if(FR.get_y()>200 && FL.get_y()>200) //ちゃんと下げきってからモード変更
							MRmode.request_to_change_area(MRMode::SandDuneRear, CANID::FromFront);
					}
				}
			}
		}
		else if(mrmode==MRMode::Tussock){
			FR.trigger_tussock(
//					(int)can_receiver.get_leg_up_FR()
					1
					);
			FL.trigger_tussock(
//					(int)can_receiver.get_leg_up_FL()
					1
					);
			FR.set_walkmode(Gait::NormalGait, Recovery::Cycloid, 0);
			FL.set_walkmode(Gait::NormalGait, Recovery::Cycloid, 0);
		}
		else if(MRMode::StartClimb1<=mrmode && mrmode<=MRMode::UukhaiZone){
			FR.set_walkmode(Gait::ActiveStableGait, Recovery::Cycloid, 0);
			FL.set_walkmode(Gait::ActiveStableGait, Recovery::Cycloid, 0);
		}
		else{
			kouden_SandDuneFront.reset_counter();
			FR.set_walkmode(Gait::NormalGait, Recovery::Cycloid, 0);
			FL.set_walkmode(Gait::NormalGait, Recovery::Cycloid, 0);
		}

		//period, duty計算
		set_cycle(&walk_period, &walk_duty);
		FR.set_period_duty(walk_period, walk_duty);
		FL.set_period_duty(walk_period, walk_duty);
		can_synchronizer.set_period(walk_period);

		walk_speed = (float)can_receiver.get_speed();
		walk_direction = - ((float)can_receiver.get_direction()) * M_PI / 180.0; //変換
		walk_pitch = ((float)can_receiver.get_pitch()) * M_PI / 180.0;

		if(mrmode==MRMode::ReadyForTussock)walk_direction *= 2.0;

		//腰固定座標系での目標位置計算+動作
		FR.walk(walk_speed, walk_direction);
		moveLeg(&FRf, &FRr, FR.get_x(), FR.get_y());
		FL.walk(walk_speed, walk_direction);
		moveLeg(&FLf, &FLr, FL.get_x(), FL.get_y());

		//歩行量+足状態+光電センサ送信
		walk_dist_right += FR.get_x_distance_move();
		walk_dist_left += FL.get_x_distance_move();

		walk_dist_front = (walk_dist_right + walk_dist_left) / 2.0;
		kouden_front.sand_dune = (unsigned char)kouden_SandDuneFront.read();
		state_front.right = FR.get_mode();
		state_front.left = FL.get_mode();
		send_leg_info(CANID::FromFront, (short)walk_dist_front, state_front.byte[0], kouden_front.byte[0]);

		//DEBUG
		if(pc.readable()){
			pc.printf("[Front]");
			pc.printf("mode:%d  ", MRmode.get_now());
			pc.printf("kouden:%d  ", kouden_SandDuneFront.get_counter());
//			pc.printf("timer:%1.4f  ", timer_FR.read());
//			pc.printf("speed:%3.4f  dir:%1.3f  ", can_receiver.get_speed(), can_receiver.get_direction());
			pc.printf("FL:x:%3.3f  y:%3.3f  ", FL.get_x(), FL.get_y());
//			pc.printf("enc:%3.2f  ", enc_FLf.getAngle());
//			pc.printf("angle:%3.2f  duty:%1.4f  ", FLf.get_angle(), FLf.get_duty());
//
//			pc.printf("vel[%3.2f][%3.2f]  ", FL.get_x_vel(), FL.get_y_vel());

			pc.printf("spd:%d  dir:%f  ", (int)walk_speed, walk_direction);
			pc.printf("pos[%f][%1d]  ", walk_dist_front, kouden_SandDuneFront.read());

//			pc.printf("up:%d%d  ", (int)can_receiver.get_leg_up_FR(), (int)can_receiver.get_leg_up_FL());

//			orbit_log(&FR, &fw_FR);
//			orbit_log(&FL, &fw_FL);
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
//	*duty = 0.5;
	int mrmode = (int)MRmode.get_now();
	switch(mrmode){
	case MRMode::GobiArea:
//		*period = 200.0/240.0;
		*period = 320.0/350.0;
		break;

	case MRMode::SandDuneFront:
//		*period = 1.6;
		*period = 1.2;
		break;
	case MRMode::SandDuneRear:
//		*period = 1.6;
		*period = 1.2;
		break;

	case MRMode::ReadyForTussock:
		*period = 200.0/240.0;
		break;
	case MRMode::Tussock:
		*period = 1;
		break;

	case MRMode::Start2:
		*period = 1;
		break;
	case MRMode::StartClimb1:
//		*period = 1;
		*period = 0.88;
		break;
	case MRMode::StartClimb2:
//		*period = 1;
		*period = 0.88;
		break;
	case MRMode::MountainArea:
//		*period = 1;
		*period = 0.88;
		break;
	case MRMode::UukhaiZone:
//		*period = 1;
		*period = 0.88;
		break;
	}
}


void send_leg_info(enum CANID::From from, short dist, unsigned char state, unsigned char kouden){
	can_sender.send_leg_info(CANID::generate(from, CANID::ToController, CANID::LegInfo), dist, state, kouden);
}


void CANrcv(){
	if(can.read(rcvMsg)){
		unsigned int id = rcvMsg.id;
		//歩行パラメータ取得
		if(CANID::is_to(id, CANID::ToSlaveAll)){
			can_receiver.receive(rcvMsg);
		}
	}
}
