#include "mbed.h"
#include "Pins.h"
#include "Walk/SingleLeg.h"
#include "Walk/ParallelLeg.h"
#include "Walk/ForwardKinematics.h"

LocalFileSystem local("local");

SingleLeg FRf(Front, Right, BASE_X, 0);
SingleLeg FRr(Rear, Right, -BASE_X, 0);
SingleLeg FLf(Front, Left, BASE_X, 0);
SingleLeg FLr(Rear, Left, -BASE_X, 0);


int main(){
	wait_ms(300); //全ての基板の電源が入るまで待つ
	pc.baud(921600);

	initParts();

	FRf.unitize(&motor_FRf, &enc_FRf, &sw_FRf);
	FRr.unitize(&motor_FRr, &enc_FRr, &sw_FRr);
	FLf.unitize(&motor_FLf, &enc_FLf, &sw_FLf);
	FLr.unitize(&motor_FLr, &enc_FLr, &sw_FLr);


	while(1){
		AdjustCycle(5000);
	}
}
