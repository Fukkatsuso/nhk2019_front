#include "mbed.h"
#include "Pins.h"
#include "Walk/SingleLeg.h"
#include "Walk/ParallelLeg.h"
#include "Walk/MRMode.h"
#include "Walk/CANCommand.h"
#include "Walk/ForwardKinematics.h"

LocalFileSystem local("local");

CANMessage rcvMsg;
CANCommand CANcmd(&can);
MRMode MRmode(&CANcmd, MRMode::GobiArea, true);//実行の度に要確認

SingleLeg FRf(Front, Right, BASE_X, 0);
SingleLeg FRr(Rear, Right, -BASE_X, 0);
ParallelLeg FR(200, 200);
SingleLeg FLf(Front, Left, BASE_X, 0);
SingleLeg FLr(Rear, Left, -BASE_X, 0);
ParallelLeg FL(-200, 200);


void initLegs();


/******************
 * 		main	  *
 ******************/
int main(){
	can.frequency(1000000);
	//can.attach(, CAN::RxIrq);
	wait_ms(300); //全ての基板の電源が入るまで待つ
	pc.baud(921600);

	initParts();
	initLegs();

	while(1){
		AdjustCycle(5000);

		FRf.state_update();
		FRr.state_update();
		FLf.state_update();
		FLr.state_update();
		
		//.walk()が入る

		FRf.move_to(FR.get_x(), FR.get_y()/*, FR.get_lim_duty_max(), FR.get_lim_duty_min()*/);
		FRr.move_to(FR.get_x(), FR.get_y()/*, FR.get_lim_duty_max(), FR.get_lim_duty_min()*/);
		FLf.move_to(FL.get_x(), FL.get_y()/*, FL.get_lim_duty_max(), FL.get_lim_duty_min()*/);
		FLr.move_to(FL.get_x(), FL.get_y()/*, FL.get_lim_duty_max(), FL.get_lim_duty_min()*/);
	}
}


void initLegs(){
	FRf.unitize(&motor_FRf, &enc_FRf, &sw_FRf);
	FRr.unitize(&motor_FRr, &enc_FRr, &sw_FRr);
	FLf.unitize(&motor_FLf, &enc_FLf, &sw_FLf);
	FLr.unitize(&motor_FLr, &enc_FLr, &sw_FLr);
	FRf.lengths(LEG_UPPER, LEG_FORE);
	FRr.lengths(LEG_UPPER, LEG_FORE);
	FLf.lengths(LEG_UPPER, LEG_FORE);
	FLr.lengths(LEG_UPPER, LEG_FORE);
	FRf.set_PID_from_file("/local/PID_FRf.txt");
	FRr.set_PID_from_file("/local/PID_FRr.txt");
	FLf.set_PID_from_file("/local/PID_FLf.txt");
	FLr.set_PID_from_file("/local/PID_FLr.txt");

	FR.set_dependencies(&MRmode, &CANcmd);
	FL.set_dependencies(&MRmode, &CANcmd);
}
