/*
 * LegFunctions.cpp
 *
 *  Created on: 2019/03/13
 *      Author: mutsuro
 */

#include "LegFunctions.h"


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
	FRf.mrmode_update();
	FRr.mrmode_update();
	FLf.mrmode_update();
	FLr.mrmode_update();
	FR.mrmode_update();
	FL.mrmode_update();
	FRf.set_limits();
	FRr.set_limits();
	FLf.set_limits();
	FLr.set_limits();
	FR.set_limits();
	FL.set_limits();
	FR.set_orbits();
	FL.set_orbits();
}


void moveLeg(SingleLeg *front, SingleLeg *rear, float x, float y){
	front->move_to(x, y);
	rear->move_to(x, y);
}


#define INITIAL_SET_X 0
#define INITIAL_SET_Y 260
void initLegs(SingleLeg *leg_f, InitLegInfo *info_f,
		  	  SingleLeg *leg_r, InitLegInfo *info_r,
			  ForwardKinematics *fw){
	//ゆっくりスイッチに近づける
	if(!info_f->enc_reset){
		info_f->angle_target = leg_f->get_enc() + 0.25;//200[roop/sec], 20[degree/sec] -> 0.1[degree/roop]
		info_r->angle_target = leg_r->get_enc() - 0.01;
		if(leg_f->get_sw()){
			leg_f->reset_duty();leg_r->reset_duty();
			leg_f->reset_enc();
			info_f->enc_reset = true;
		}
	}
	else if(!info_r->enc_reset){
		info_f->angle_target = 30;
		info_r->angle_target = leg_r->get_enc() + 0.25;
		if(leg_r->get_sw()){
			leg_f->reset_duty();leg_r->reset_duty();
			leg_r->reset_enc();
			info_r->enc_reset = true;
		}
	}
	else{//enc両方リセット完了
		if((fabs(INITIAL_SET_X - fw->get_x()) < 1.0) && (fabs(INITIAL_SET_Y - fw->get_y()) < 1.0))
			info_f->finish_init = info_r->finish_init = true;
		fw->estimate();
		info_f->x_target = info_r->x_target = INITIAL_SET_X;
		info_f->y_target = info_r->y_target = INITIAL_SET_Y;
	}

	//駆動
	if(!info_f->enc_reset){
		leg_f->set_duty_limit(0.575, 0.425);
		leg_r->set_duty_limit(0.51, 0.49);
		leg_f->move_to_angle(info_f->angle_target);
		leg_r->move_to_angle(info_r->angle_target);
	}
	else if(!info_r->enc_reset){
		leg_f->set_duty_limit(0.575, 0.425);
		leg_r->set_duty_limit(0.6, 0.4);
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
			//	右前,右後,左前,左後
	InitLegInfo Rf, Rr, Lf, Lr;
	Rf.enc_reset = Rr.enc_reset = Lf.enc_reset = Lr.enc_reset = false;
	Rf.finish_init = Rr.finish_init = Lf.finish_init = Lr.finish_init = false;
	while(!(Rf.finish_init && Rr.finish_init && Lf.finish_init && Lr.finish_init)){
		AdjustCycle(5000);
		led0 = 1;
		initLegs(&FRf, &Rf, &FRr, &Rr, &fw_FR);
		initLegs(&FLf, &Lf, &FLr, &Lr, &fw_FL);
		pc.printf("enc");
		pc.printf("[%3.2f][%3.2f]", enc_FRf.getAngle(), enc_FRr.getAngle());
		pc.printf("[%3.2f][%3.2f]", enc_FLf.getAngle(), enc_FLr.getAngle());
		pc.printf("  target");
		if(!Rf.enc_reset || !Rr.enc_reset)
			pc.printf("ang[%3.2f][%3.2f]", Rf.angle_target, Rr.angle_target);
		else{
			pc.printf("xy[%3.2f][%3.2f]", Rf.x_target, Rf.y_target);
			pc.printf("est[%3.2f][%3.2f]", fw_FR.get_x(), fw_FR.get_y());
		}
		if(!Lf.enc_reset || !Lr.enc_reset)
			pc.printf("ang[%3.2f][%3.2f]", Lf.angle_target, Lr.angle_target);
		else{
			pc.printf("xy[%3.2f][%3.2f]", Lf.x_target, Lr.y_target);
			pc.printf("est[%3.2f][%3.2f]", fw_FL.get_x(), fw_FL.get_y());
		}
//		pc.printf("  sw");
//		pc.printf("[%d][%d]", sw_FRf.read(), sw_FRr.read());
//		pc.printf("[%d][%d]", sw_FLf.read(), sw_FLr.read());
		pc.printf("  duty");
		pc.printf("[%1.3f][%1.3f]", FRf.get_duty(), FRr.get_duty());
		pc.printf("[%1.3f][%1.3f]", FLf.get_duty(), FLr.get_duty());
		pc.printf("\r\n");

		if(pc.readable())if(pc.getc()=='s')break;//"s"を押したら強制終了
	}
	led0 = 0;
}


void orbit_log(ParallelLeg *invLeg, ForwardKinematics *fwLeg){
	fwLeg->estimate();
	pc.printf("%3.4f\t%3.4f\t", invLeg->get_x(), invLeg->get_y());
	pc.printf("%3.4f\t%3.4f\t", fwLeg->get_x(), fwLeg->get_y());
}
