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

void read_PIDgain(float gain[], const char *fileName);


/******************
 * 		main	  *
 ******************/
int main(){
	float gain[3] = {0, 0, 0};
	can.frequency(1000000);
	//can.attach(, CAN::RxIrq);
	wait_ms(300); //全ての基板の電源が入るまで待つ
	pc.baud(921600);

	initParts();

	FRf.unitize(&motor_FRf, &enc_FRf, &sw_FRf);
	FRr.unitize(&motor_FRr, &enc_FRr, &sw_FRr);
	FLf.unitize(&motor_FLf, &enc_FLf, &sw_FLf);
	FLr.unitize(&motor_FLr, &enc_FLr, &sw_FLr);
	FRf.lengths(LEG_UPPER, LEG_FORE);
	FRr.lengths(LEG_UPPER, LEG_FORE);
	FLf.lengths(LEG_UPPER, LEG_FORE);
	FLr.lengths(LEG_UPPER, LEG_FORE);
	read_PIDgain(gain, "/local/PID_FRf.txt");
	FRf.set_PID(gain[0], gain[1], gain[2]);
	read_PIDgain(gain, "/local/PID_FRr.txt");
	FRr.set_PID(gain[0], gain[1], gain[2]);
	read_PIDgain(gain, "/local/PID_FLf.txt");
	FLf.set_PID(gain[0], gain[1], gain[2]);
	read_PIDgain(gain, "/local/PID_FLr.txt");
	FLr.set_PID(gain[0], gain[1], gain[2]);

	while(1){
		AdjustCycle(5000);
	}
}


void read_PIDgain(float gain[], const char *fileName){
	FILE *fp = fopen(fileName, "r");
	pc.printf("\r\n");
	if(!fp){
		pc.printf("file open error!!\r\n");
		return;
	}
	for(int i=0; i<3; i++){
		fscanf(fp, "%f", &gain[i]);
		pc.printf("%f  ", gain[i]);
	}
	fclose(fp);
}
