/*
 * LegFunctions.h
 *
 *  Created on: 2019/03/13
 *      Author: mutsuro
 */

#ifndef LEGFUNCTIONS_H_
#define LEGFUNCTIONS_H_

#include "mbed.h"
#include "Pins.h"
#include "Walk/CANs/CANSynchronizer.h"
#include "Walk/ClockTimer.h"
#include "Walk/SingleLeg.h"
#include "Walk/ParallelLeg.h"
#include "Walk/MRMode.h"
#include "Walk/ForwardKinematics.h"


struct InitLegInfo{
	bool enc_reset;
	bool finish_init;
	float angle_target;
	float x_target;
	float y_target;
};


void setLegs();
void set_limits();
void moveLeg(SingleLeg *front, SingleLeg *rear, float x, float y);

void initLegs(SingleLeg *leg_f, InitLegInfo *info_f,
			  SingleLeg *leg_r, InitLegInfo *info_r,
			  ForwardKinematics *fw);
void autoInit();
void orbit_log(ParallelLeg *invLeg, ForwardKinematics *fwLeg);

float adjust_walk_direction(float direction);


extern Serial pc;

extern LocalFileSystem local;//PIDゲイン調整に使用

extern Timer timer_PID;

extern ClockTimer timer_FR;
extern ClockTimer timer_FL;
extern SingleLeg FRf;
extern SingleLeg FRr;
extern ParallelLeg FR;
extern SingleLeg FLf;
extern SingleLeg FLr;
extern ParallelLeg FL;

extern ForwardKinematics fw_FR;
extern ForwardKinematics fw_FL;

extern CANMessage rcvMsg;
extern CANReceiver can_receiver;
extern CANSynchronizer can_synchronizer;
extern MRMode MRmode;//実行の度に要確認


#endif /* LEGFUNCTIONS_H_ */
