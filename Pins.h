/*
 * init2.h
 *
 *  Created on: 2018/09
 *      Author: mutsuro
 */
#ifndef PINS_H_
#define PINS_H_

#include "mbed.h"
#include "QEI/SingleLegQEI.h"
#include "InitSwitch.h"

#define CYCLE 5000 //動作周期(us)


/*----------------------
 -----機能選択するピン-----
 ----------------------*/
extern CAN can;


/********************
 * 		enc			*
 ********************/
extern SingleLegQEI enc_FRf;
extern SingleLegQEI enc_FRr;
extern SingleLegQEI enc_FLf;
extern SingleLegQEI enc_FLr;


/********************
 * 		スイッチ		*
 ********************/
extern InitSwitch sw_FRf;
extern InitSwitch sw_FRr;
extern InitSwitch sw_FLf;
extern InitSwitch sw_FLr;


/********************
 * 		DCモータ		*
 ********************/
extern PwmOut motor_FRf;
extern PwmOut motor_FRr;
extern PwmOut motor_FLf;
extern PwmOut motor_FLr;


/*----------------------
 ----mbed本体上のピン-----
 ----------------------*/
extern Serial pc;
extern DigitalOut led0;
extern DigitalOut led1;
extern DigitalOut led2;
extern DigitalOut led3;
extern Timer AdCycle; //AdjustCycleで使うタイマ


/*----------------------
 ---------関数---------
 ----------------------*/
void AdjustCycle(int t_us);
void initParts();


#endif /* PINS_H_ */

