/*
 * Pins2.cpp
 *
 *  Created on: 2019/02
 *      Author: mutsuro
 */

#include "Pins.h"

/*----------------------
 -----機能選択するピン-----
 ----------------------*/
CAN can(p9, p10);


/********************
 * 		enc			*
 ********************/
SingleLegQEI enc_FRf(p12, p11);
SingleLegQEI enc_FRr(p27, p28);
SingleLegQEI enc_FLf(p14, p13);
SingleLegQEI enc_FLr(p29, p30);


/********************
 * 		スイッチ		*
 ********************/
InitSwitch sw_FRf(p22, 0);
InitSwitch sw_FRr(p21, 1);
InitSwitch sw_FLf(p6, 1);
InitSwitch sw_FLr(p5, 1);


/********************
 * 		DCモータ		*
 ********************/
PwmOut motor_FRf(p25);
PwmOut motor_FRr(p26);
PwmOut motor_FLf(p23);
PwmOut motor_FLr(p24);

/*----------------------
 ----mbed本体上のピン-----
 ----------------------*/
Serial pc(USBTX, USBRX);
DigitalOut led0(LED1);
DigitalOut led1(LED2);
DigitalOut led2(LED3);
DigitalOut led3(LED4);
Timer AdCycle;



/*
 * 関数名	AdjustCycle
 *
 * 用途		マイコンの動作周期を調整する
 *
 * 引数		int t_us:目的の動作周期(us)
 *
 * 戻り値		なし
 *
 * 備考		関数実行時、前の実行時からt_us経っていない場合、t_us経つまで待つ
 * 			すでにt_us経っている場合、led3を点灯する
 */
void AdjustCycle(int t_us){
    if(AdCycle.read_us() == 0) AdCycle.start();

    if(AdCycle.read_us()>t_us){
    	led3=1;
//    	pc.printf("AdCycle=%dus\r\n",AdCycle.read_us());
    }
    else {
    	led3=0;
//    	pc.printf("AdCycle=%dus\r\n",AdCycle.read_us());
    }
    while(AdCycle.read_us()<=t_us);
    AdCycle.reset();
}


void initParts(){
	sw_FRf.mode(PullDown);
	sw_FRr.mode(PullDown);
	sw_FLf.mode(PullDown);
	sw_FLr.mode(PullDown);
	enc_FRf.reset();
	enc_FRr.reset();
	enc_FLf.reset();
	enc_FLr.reset();
	motor_FRf.period_us(50);
	motor_FRr.period_us(50);
	motor_FLf.period_us(50);
	motor_FLr.period_us(50);
}
